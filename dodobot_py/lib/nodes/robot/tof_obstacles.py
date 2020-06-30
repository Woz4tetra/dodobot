import math
from collections import namedtuple

from lib.config import ConfigManager

rover_config = ConfigManager.get_robot_config()

TofThresholds = namedtuple('TofThresholds', 'front_lower front_upper front_servo back_lower back_upper back_servo')


def angle_rad_to_tof_servo_command(angle_rad):
    angle_rad = angle_rad + (math.pi * 2 if angle_rad <= 0.0 else 0)  # bound to 270...360 deg
    angle_deg = math.degrees(angle_rad)
    y0 = rover_config.tof_servo_lower_command
    y1 = rover_config.tof_servo_upper_command
    x0 = rover_config.tof_servo_lower_angle_deg
    x1 = rover_config.tof_servo_upper_angle_deg

    servo_command = (x1 - x0) / (y1 - y0) * (angle_deg - x0) + y0
    return int(servo_command)


def calculate_tof_thresholds(obstacle_x, ledge_y, buffer_x, wall_offset):
    gaze_x = obstacle_x + buffer_x + wall_offset  # X coordinate where the sensor is pointing at
    threshold_x = obstacle_x + wall_offset  # X coordinate where threshold starts
    offset_ledge_y = ledge_y + rover_config.tof_ground_dist_mm  # Y threshold relative to the axis of rotation
    sensor_angle = math.atan2(-rover_config.tof_ground_dist_mm, gaze_x)

    # sensor lower threshold accounting for sensor's distance away from the axis of rotation
    obstacle_threshold = threshold_x / math.cos(sensor_angle) - rover_config.tof_off_axis_mm

    # sensor upper threshold accounting for sensor's distance away from the axis of rotation
    ledge_threshold = abs(offset_ledge_y / math.sin(sensor_angle)) - rover_config.tof_off_axis_mm

    servo_command = angle_rad_to_tof_servo_command(sensor_angle)
    assert obstacle_threshold >= 0.0, obstacle_threshold
    assert ledge_threshold >= 0.0, ledge_threshold
    return obstacle_threshold, ledge_threshold, servo_command


def get_stopping_thresholds(obstacle_threshold_x_mm, ledge_threshold_y_mm, buffer_x_mm=10.0):
    # obstacle_threshold_x_mm: measured from the maximum point of the front or back of the robot
    # ledge_threshold_y_mm: measured from the ground plane where the robot sits flat on the ground
    # buffer_x_mm: x buffer distance to account for robot stopping time and update rate delay

    front_obstacle_threshold, front_ledge_threshold, front_servo_command = \
        calculate_tof_thresholds(obstacle_threshold_x_mm, ledge_threshold_y_mm, buffer_x_mm,
                                 rover_config.tof_front_wall_dist_mm)

    back_obstacle_threshold, back_ledge_threshold, back_servo_command = \
        calculate_tof_thresholds(obstacle_threshold_x_mm, ledge_threshold_y_mm, buffer_x_mm,
                                 rover_config.tof_back_wall_dist_mm)

    return TofThresholds(
        front_obstacle_threshold, front_ledge_threshold, front_servo_command,
        back_obstacle_threshold, back_ledge_threshold, back_servo_command
    )


if __name__ == "__main__":
    print(get_stopping_thresholds(50, 5, 100))
