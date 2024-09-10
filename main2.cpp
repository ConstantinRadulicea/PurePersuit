#include "geometry2D.h"
#include <cstdint>
#include <vector>
#include <string.h>
#include <math.h>
#include <iostream>

// wheels angle: [-48.946945; 59.711018], servo angle: [-65; 41]
Point2D g_servo_position = Point2D{ 0.0f, 0.0f };
Point2D g_arm_wheel_position = Point2D{ 52.392, -6.0f };
float g_servo_arm_circle_radius_mm = 24.0f;	// 24mm, 20mm
float g_arm_wheel_circle_radius_mm = 25.547f;
float g_arm_wheel_angle_rad = NormalizePiToNegPi(radians(16.46f));		//16.46

float g_servo_arm_to_arm_wheel_rod_length_mm = 45.6188202f;	// not necessary to calculate manually, already done automatically
float g_arm_wheel_angle_position_rad = NormalizePiToNegPi((M_PI_2 * 3.0f)) - g_arm_wheel_angle_rad;
float g_servo_rod_forward_angle_position_rad = NormalizePiToNegPi((M_PI_2 * 3.0f));
float g_servo_rod_angle_position_rad = g_servo_rod_forward_angle_position_rad;
float g_arm_wheel_forward_angle_position_rad = NormalizePiToNegPi((M_PI_2 * 3.0f) - g_arm_wheel_angle_rad);




float SteeringAngleToRawAngle_rad(float steering_angle) {
	steering_angle = NormalizePiToNegPi(steering_angle);
	Point2D servo_position = g_servo_position;
	Point2D arm_wheel_position = g_arm_wheel_position;
	float servo_arm_circle_radius_mm = g_servo_arm_circle_radius_mm;
	float arm_wheel_circle_radius_mm = g_arm_wheel_circle_radius_mm;
	float arm_wheel_angle_rad = g_arm_wheel_angle_rad;

	float servo_arm_to_arm_wheel_rod_length_mm = g_servo_arm_to_arm_wheel_rod_length_mm;
	float arm_wheel_angle_position_rad = g_arm_wheel_angle_position_rad;
	float arm_wheel_forward_angle_position_rad = g_arm_wheel_forward_angle_position_rad;
	float servo_rod_forward_angle_position_rad = g_servo_rod_forward_angle_position_rad;
	float servo_rod_angle_position_rad = g_servo_rod_angle_position_rad;
	float raw_angle = 0.0f;
	Point2D servo_rod_position;
	Point2D arm_wheel_rod_position;
	IntersectionPoints2D_2 temp_intersectionPoints;
	float temp_distance_1, temp_distance_2;
	float cmp_result_1;

	servo_rod_position = circleAngleToPoint2D(servo_position, servo_arm_circle_radius_mm, servo_rod_angle_position_rad);
	arm_wheel_rod_position = circleAngleToPoint2D(arm_wheel_position, arm_wheel_circle_radius_mm, arm_wheel_angle_position_rad);
	servo_arm_to_arm_wheel_rod_length_mm = euclidianDistance(servo_rod_position, arm_wheel_rod_position);


	// the function main logic starts here
	arm_wheel_angle_position_rad = NormalizePiToNegPi(arm_wheel_angle_position_rad + steering_angle);
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
	servo_rod_angle_position_rad = circlePoint2DToAngle(servo_position, servo_rod_position);

	raw_angle = NormalizePiToNegPi(servo_rod_angle_position_rad - servo_rod_forward_angle_position_rad);

	return raw_angle;
}

float RawAngleToSteeringAngle_rad(float raw_angle) {
	raw_angle = NormalizePiToNegPi(raw_angle);
	Point2D servo_position = g_servo_position;
	Point2D arm_wheel_position = g_arm_wheel_position;
	float servo_arm_circle_radius_mm = g_servo_arm_circle_radius_mm;
	float arm_wheel_circle_radius_mm = g_arm_wheel_circle_radius_mm;
	float arm_wheel_angle_rad = g_arm_wheel_angle_rad;

	float servo_arm_to_arm_wheel_rod_length_mm = g_servo_arm_to_arm_wheel_rod_length_mm;
	float arm_wheel_angle_position_rad = g_arm_wheel_angle_position_rad;
	float arm_wheel_forward_angle_position_rad = g_arm_wheel_forward_angle_position_rad;
	float servo_rod_forward_angle_position_rad = g_servo_rod_forward_angle_position_rad;
	float servo_rod_angle_position_rad = g_servo_rod_angle_position_rad;
	float steering_angle = 0.0f;
	Point2D servo_rod_position;
	Point2D arm_wheel_rod_position;
	IntersectionPoints2D_2 temp_intersectionPoints;
	float temp_distance_1, temp_distance_2;
	float cmp_result_1;

	servo_rod_position = circleAngleToPoint2D(servo_position, servo_arm_circle_radius_mm, servo_rod_angle_position_rad);
	arm_wheel_rod_position = circleAngleToPoint2D(arm_wheel_position, arm_wheel_circle_radius_mm, arm_wheel_angle_position_rad);
	servo_arm_to_arm_wheel_rod_length_mm = euclidianDistance(servo_rod_position, arm_wheel_rod_position);

	// the function main logic starts here
	servo_rod_angle_position_rad = NormalizePiToNegPi(servo_rod_angle_position_rad + raw_angle);
	servo_rod_position = circleAngleToPoint2D(servo_position, servo_arm_circle_radius_mm, servo_rod_angle_position_rad);

	temp_intersectionPoints = intersectionBwCircles(arm_wheel_position, arm_wheel_circle_radius_mm, servo_rod_position, servo_arm_to_arm_wheel_rod_length_mm);

	temp_distance_1 = euclidianDistance(arm_wheel_rod_position, temp_intersectionPoints.point1);
	temp_distance_2 = euclidianDistance(arm_wheel_rod_position, temp_intersectionPoints.point2);

	if (temp_distance_1 < temp_distance_2) {
		arm_wheel_rod_position = temp_intersectionPoints.point1;
	}
	else {
		arm_wheel_rod_position = temp_intersectionPoints.point2;
	}

	arm_wheel_angle_position_rad = circlePoint2DToAngle(arm_wheel_position, arm_wheel_rod_position);
	steering_angle = NormalizePiToNegPi(arm_wheel_angle_position_rad - arm_wheel_forward_angle_position_rad);
	return steering_angle;
}

int main3() {
	float result_1, result_2, result_3;
	float servo, wheels;
	result_1 = degrees(SteeringAngleToRawAngle_rad(radians(40))); // -0.936093390
	//result_2 = degrees(RawAngleToSteeringAngle_rad(radians(result_1))); // 30
	result_1 = degrees(SteeringAngleToRawAngle_rad(radians(-40))); // -0.936093390
	//result_2 = degrees(RawAngleToSteeringAngle_rad(radians(result_1))); // 30
	for (int i = -90; i < 90; i++)
	{
		servo = degrees(SteeringAngleToRawAngle_rad(radians(i)));
		wheels = degrees(RawAngleToSteeringAngle_rad(radians(servo)));;
		printf("[%d] = servo: %f\t wheels: %f\n", i, servo, wheels);
	}

	return 0;
}