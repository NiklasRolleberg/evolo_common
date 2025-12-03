#!/usr/bin/env python3

# General imports
import math

# Functions fo converting between ENU yaw and compass heading
def normalize_angle_rad(angle):
    """Normalize angle to [-π, π]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi


def normalize_angle_deg(angle):
    """Normalize angle to [0°, 360°)."""
    return angle % 360


# --- Conversion Functions ---
def heading_to_yaw(heading_deg):
    """Convert heading in degrees (North=0°, East=90°) to ENU yaw in radians."""
    yaw = math.radians(90 - heading_deg)
    return normalize_angle_rad(yaw)


def yaw_to_heading(yaw_rad):
    """Convert ENU yaw in radians to heading in degrees."""
    heading = 90 - math.degrees(yaw_rad)
    return normalize_angle_deg(heading)