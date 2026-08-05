import json
import re
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SuccorCommandScheduler(Node):
    """
    Receives JSON scheduling commands (from an MQTT bridge) and forwards
    raw serial command strings to the serial_parser node on a timer.
    Also subscribes to serial responses and re-publishes them as JSON for the MQTT bridge.

    Incoming JSON formats (on command_topic):
        {"command": "$B02LL", "repeating": true,  "period": 10.0}
        {"command": "$B02LL", "repeating": false}
        {"command": "$B02LL", "cancel": true}

    Outgoing commands (on serial_out_topic):
        std_msgs/String with msg.data = raw serial command, e.g. "$B02LL"

    Incoming responses (on serial_in_topic):
        std_msgs/String with msg.data = raw device response, e.g. "#B00107-0000.2"

    Outgoing feedback (on feedback_topic):
        {"response": "#B00107-0000.2"}

    Incoming friendly puffin commands (on uwcom_command_topic):
        {"puffin_id": 1, "mode": "none"}     # cancel all scheduled commands for puffin 1
        {"puffin_id": 1, "mode": "noise"}    # $U00102NM every 4s (8s if both puffins measure)
        {"puffin_id": 2, "mode": "sensors"}  # $U00202DE / $U00202TE alternating every 2s
        {"puffin_id": 1, "mode": "all"}      # noise + sensors combined
        {"puffin_id": 1, "mode": "surface"}  # cancel all, then $U00102CU every 2s, up to 5 sends

    puffin_id 1 → device $U001, puffin_id 2 → device $U002. If both puffins are in a
    measuring mode (noise/sensors/all) at the same time, each puffin's commands are sent
    at half the rate and staggered so the two puffins' transmissions interleave instead of
    colliding (e.g. solo noise every 4s becomes every 8s per puffin when both measure).
    A puffin in "surface" mode always keeps its 2s cadence (and stops after 5 sends); the
    other puffin's measuring schedule is phase-shifted by 1s to avoid landing on the same
    instant.

    Responses from puffins are parsed into friendly readings and published on
    uwcom_feedback_topic (unrecognised responses are dropped from this topic, logged only):
        "#B00108TE0024.6"  → {"puffin_id": 1, "temperature": 24.6}
        "#B00207DE004.6"   → {"puffin_id": 2, "depth": 4.6}
        "#B0010...#NR000181P..." → {"puffin_id": 1, "noise": 181}
    feedback_topic keeps publishing every response raw, unparsed, as {"response": ...}.
    """

    # mode -> (solo per-command period [s], ordered raw command suffixes)
    _MEASURE_MODES = {
        "noise":   {"period": 4.0, "commands": ["NM"]},
        "sensors": {"period": 4.0, "commands": ["DE", "TE"]},
        "all":     {"period": 9.0, "commands": ["NM", "DE", "TE"]},
    }
    _VALID_MODES = {"none", "surface"} | set(_MEASURE_MODES)
    _SURFACE_PERIOD = 2.0
    _SURFACE_MAX_SENDS = 10

    # "#B" response = sender(3) + length(2) + data; data is "#NR" + 6-digit noise
    # value (only the digits before the following "P" matter), a "DE"/"TE" marker
    # followed by the depth/temperature reading, or — until temperature responses
    # exist — a bare decimal, which can only be a depth reading.
    _B_RESPONSE_RE = re.compile(r"^#B(\d{3})\d{2}(.*)$")
    _NOISE_RE = re.compile(r"^#NR(\d{6})")
    _MEASURE_RE = re.compile(r"^(DE|TE)(-?[\d.]+)")
    _BARE_DECIMAL_RE = re.compile(r"^(-?[\d.]+)$")

    def __init__(self):
        super().__init__("succor_command_scheduler")

        self.declare_parameter("command_topic",  "/evolo/waraps/sensor/succor/command")
        self.declare_parameter("serial_out_topic", "/evolo/sensors/succor/to")
        self.declare_parameter("serial_in_topic",  "/evolo/sensors/succor/from")
        self.declare_parameter("feedback_topic", "/evolo/waraps/sensor/succor/feedback")
        self.declare_parameter("uwcom_command_topic", "/evolo/waraps/sensor/uwcom/command")
        self.declare_parameter("uwcom_feedback_topic", "/evolo/waraps/sensor/uwcom/feedback")
        self.declare_parameter("tick_rate", 10.0)  # Hz — scheduler resolution

        command_topic    = self.get_parameter("command_topic").get_parameter_value().string_value
        serial_out_topic = self.get_parameter("serial_out_topic").get_parameter_value().string_value
        serial_in_topic  = self.get_parameter("serial_in_topic").get_parameter_value().string_value
        feedback_topic   = self.get_parameter("feedback_topic").get_parameter_value().string_value
        uwcom_command_topic  = self.get_parameter("uwcom_command_topic").get_parameter_value().string_value
        uwcom_feedback_topic = self.get_parameter("uwcom_feedback_topic").get_parameter_value().string_value
        tick_rate        = self.get_parameter("tick_rate").get_parameter_value().double_value

        self._serial_pub  = self.create_publisher(String, serial_out_topic, 10)
        self._feedback_pub = self.create_publisher(String, feedback_topic, 10)
        self._uwcom_feedback_pub = self.create_publisher(String, uwcom_feedback_topic, 10)
        self.create_subscription(String, command_topic,   self._command_cb,  10)
        self.create_subscription(String, serial_in_topic, self._response_cb, 10)
        self.create_subscription(String, uwcom_command_topic, self._uwcom_command_cb, 10)
        self.create_timer(1.0 / tick_rate, self._tick)

        # {command_str: {"period": float, "next_at": float}}
        self._scheduled: dict = {}
        self._puffin_mode = {1: "none", 2: "none"}

        self.get_logger().info(
            f"Commands:  '{command_topic}' → '{serial_out_topic}'\n"
            f"Responses: '{serial_in_topic}' → '{feedback_topic}'\n"
            f"UW commands: '{uwcom_command_topic}' → '{serial_out_topic}'\n"
            f"UW feedback: '{serial_in_topic}' → '{uwcom_feedback_topic}'"
        )

    # ------------------------------------------------------------------

    def _command_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Invalid JSON: {e} — received: {msg.data!r}")
            return

        if data.get("cancel"):
            count = len(self._scheduled)
            self._scheduled.clear()
            self.get_logger().info(f"Cancelled all scheduled commands ({count})")
            return

        command = data.get("command", "")
        if not command:
            self.get_logger().warn("Message missing 'command' field")
            return

        if data.get("repeating", False):
            period = float(data.get("period", 10.0))
            self._scheduled[command] = {"period": period, "next_at": time.monotonic() + period}
            self.get_logger().info(f"Scheduled {command!r} every {period}s")

        else:
            self._send(command)

    def _response_cb(self, msg: String):
        # Ignore echoed TX commands ($...), only forward device responses (#...)
        if msg.data.startswith("$"):
            return

        out = String()
        out.data = json.dumps({"response": msg.data})
        self._feedback_pub.publish(out)

        parsed = self._parse_uwcom_response(msg.data)
        if parsed is not None:
            puffin_id, reading, value = parsed
            uwcom_out = String()
            uwcom_out.data = json.dumps({"puffin_id": puffin_id, reading: value})
            self._uwcom_feedback_pub.publish(uwcom_out)
        else:
            self.get_logger().debug(f"uwcom: unrecognised response {msg.data!r}")

        self.get_logger().info(f"Feedback: {msg.data!r}")

    def _parse_uwcom_response(self, raw: str):
        b_match = self._B_RESPONSE_RE.match(raw)
        if not b_match:
            return None

        sender_id, data = b_match.groups()
        puffin_id = int(sender_id)
        if puffin_id not in (1, 2):
            return None

        noise_match = self._NOISE_RE.match(data)
        if noise_match:
            return puffin_id, "noise", int(noise_match.group(1))

        measure_match = self._MEASURE_RE.match(data)
        if measure_match:
            marker, value_str = measure_match.groups()
            reading = "depth" if marker == "DE" else "temperature"
            return puffin_id, reading, float(value_str)

        bare_match = self._BARE_DECIMAL_RE.match(data)
        if bare_match:
            return puffin_id, "depth", float(bare_match.group(1))

        return None

    def _uwcom_command_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Invalid uwcom JSON: {e} — received: {msg.data!r}")
            return

        puffin_id = data.get("puffin_id")
        mode = data.get("mode")

        if puffin_id not in (1, 2):
            self.get_logger().error(f"uwcom command: puffin_id must be 1 or 2, got {puffin_id!r}")
            return
        if mode not in self._VALID_MODES:
            self.get_logger().error(f"uwcom command: unknown mode {mode!r} for puffin {puffin_id}")
            return

        self._puffin_mode[puffin_id] = mode
        self._apply_puffin_modes()
        self.get_logger().info(f"uwcom: puffin {puffin_id} mode set to {mode!r}")

    def _apply_puffin_modes(self):
        # Drop this puffin's previously scheduled commands (uwcom-managed or legacy
        # raw ones addressed to the same device) before rebuilding from current modes.
        for dev in (1, 2):
            device_prefix = f"$U{dev:03d}"
            for command in list(self._scheduled):
                if command.startswith(device_prefix):
                    del self._scheduled[command]

        modes = self._puffin_mode
        both_measuring = all(modes[d] in self._MEASURE_MODES for d in (1, 2))
        both_surfacing = all(modes[d] == "surface" for d in (1, 2))
        now = time.monotonic()

        for dev in (1, 2):
            mode = modes[dev]
            other = 2 if dev == 1 else 1
            prefix = f"$U{dev:03d}02"

            if mode in self._MEASURE_MODES:
                cfg = self._MEASURE_MODES[mode]
                solo_period = cfg["period"]
                commands = cfg["commands"]
                step = solo_period / len(commands)
                multiplier = 2.0 if both_measuring else 1.0
                period = solo_period * multiplier
                # Secondary puffin (2) is staggered half a (doubled) period behind
                # primary puffin (1) so simultaneous measuring never collides.
                secondary_shift = solo_period if (both_measuring and dev == 2) else 0.0
                # If the other puffin is surfacing, dodge its every-2s cadence.
                surface_shift = self._SURFACE_PERIOD / 2.0 if modes[other] == "surface" else 0.0
                for i, suffix in enumerate(commands):
                    phase = (i * step * multiplier + secondary_shift + surface_shift) % period
                    self._scheduled[f"{prefix}{suffix}"] = {
                        "period": period,
                        "next_at": now + phase,
                    }

            elif mode == "surface":
                phase = self._SURFACE_PERIOD / 2.0 if (both_surfacing and dev == 2) else 0.0
                self._scheduled[f"{prefix}CU"] = {
                    "period": self._SURFACE_PERIOD,
                    "next_at": now + phase,
                    "remaining": self._SURFACE_MAX_SENDS,
                }

    def _tick(self):
        now = time.monotonic()
        for command, entry in list(self._scheduled.items()):
            if now >= entry["next_at"]:
                self._send(command)
                remaining = entry.get("remaining")
                if remaining is not None:
                    remaining -= 1
                    if remaining <= 0:
                        del self._scheduled[command]
                        continue
                    entry["remaining"] = remaining
                entry["next_at"] = now + entry["period"]

    def _send(self, command: str):
        msg = String()
        msg.data = command
        self._serial_pub.publish(msg)
        self.get_logger().info(f"Sent: {command!r}")


def main(args=None):
    rclpy.init(args=args)
    node = SuccorCommandScheduler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
