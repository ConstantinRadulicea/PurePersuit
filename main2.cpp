#include "geometry2D.h"
#include <cstdint>
#include <vector>
#include <string.h>
#include <math.h>




float SteeringAngleToRawAngle_rad(float steering_angle) {
	Point2D servo_position = Point2D{ 0.0f, 0.0f };
	Point2D arm_wheel_position = Point2D{ 52.392, -6.0f };
	float servo_arm_circle_radius_mm = 24.0f;	// 24mm, 20mm
	float arm_wheel_circle_radius_mm = 25.547f;
	float arm_wheel_angle_rad = radians(16.46f);

	float servo_arm_to_arm_wheel_rod_length_mm = 45.6188202f;	// not necessary to calculate manually, already done automatically
	float arm_wheel_angle_position_rad = (M_PI_2 * 3.0f) - arm_wheel_angle_rad;
	float servo_rod_angle_position_rad = (M_PI_2 * 3.0f);
	float raw_angle = 0.0f;
	Point2D servo_rod_position;
	Point2D arm_wheel_rod_position;
	IntersectionPoints2D_2 temp_intersectionPoints;
	float temp_distance_1, temp_distance_2;
	float cmp_result_1;

	servo_rod_position = circleAngleToPoint2D(servo_position, servo_arm_circle_radius_mm, servo_rod_angle_position_rad);
	arm_wheel_rod_position = circleAngleToPoint2D(arm_wheel_position, arm_wheel_circle_radius_mm, arm_wheel_angle_position_rad);
	servo_arm_to_arm_wheel_rod_length_mm = euclidianDistance(servo_rod_position, arm_wheel_rod_position);

	arm_wheel_angle_position_rad = arm_wheel_angle_position_rad + steering_angle;
	arm_wheel_rod_position = circleAngleToPoint2D(arm_wheel_position, arm_wheel_circle_radius_mm, arm_wheel_angle_position_rad);

	temp_intersectionPoints = intersectionBwCircles(servo_position, servo_arm_circle_radius_mm, arm_wheel_rod_position, servo_arm_to_arm_wheel_rod_length_mm);

	temp_distance_1 = euclidianDistance(servo_rod_position, temp_intersectionPoints.point1);
	temp_distance_2 = euclidianDistance(servo_rod_position, temp_intersectionPoints.point2);

	if (temp_distance_1 < temp_distance_2) {
		servo_rod_position = temp_intersectionPoints.point1;
	}
	else {
		servo_rod_position = temp_intersectionPoints.point2;
	}

	raw_angle = circlePoint2DToAngle(servo_position, servo_arm_circle_radius_mm, servo_rod_position);

	return raw_angle;
}

int main() {
	SteeringAngleToRawAngle_rad(radians(30.0)); // -0.936093390
	return 0;
}