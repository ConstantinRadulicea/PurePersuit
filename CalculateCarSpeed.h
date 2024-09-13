#ifndef __CALCULATECARSPEED_H__
#define __CALCULATECARSPEED_H__

#include "geometry2D.h"
#include "PurePursuitGeometry.h"
#include <math.h>


static float calculateCarSpeed_old(float minSpeed, float maxSpeed, float maxSteeringWheelAngle, float steeringWheelAngle, LineABC laneMiddleLine, float ki, float kd, float kiMinMaxImpact) {
	float newCarSpeed_bySteeringAngle, speedSpan, angleCurrentTrajectoryAndMiddleLane, newCarSpeed_byTrajectoryAngle;
	static float sumSteeringWheelAngle = 0.0f;
	static float prevSteeringWheelAngleError = 0.0f;
	float derivativeSteeringError;
	LineABC currentTrajectory;

	kiMinMaxImpact = fabs(kiMinMaxImpact);

	speedSpan = maxSpeed - minSpeed;
	maxSteeringWheelAngle = fabsf(maxSteeringWheelAngle);
	steeringWheelAngle = fabsf(steeringWheelAngle);
	steeringWheelAngle = MIN(steeringWheelAngle, maxSteeringWheelAngle);

	derivativeSteeringError = fabs(steeringWheelAngle - prevSteeringWheelAngleError);

	currentTrajectory = yAxisABC();
	angleCurrentTrajectoryAndMiddleLane = fabsf(angleBetweenLinesABC(currentTrajectory, laneMiddleLine));

	newCarSpeed_byTrajectoryAngle = minSpeed + ((((float)M_PI_2 - angleCurrentTrajectoryAndMiddleLane) / (float)M_PI_2) * speedSpan) + (ki * sumSteeringWheelAngle) + (kd * derivativeSteeringError);

	newCarSpeed_byTrajectoryAngle = MAX(newCarSpeed_byTrajectoryAngle, minSpeed);
	newCarSpeed_byTrajectoryAngle = MIN(newCarSpeed_byTrajectoryAngle, maxSpeed);

	newCarSpeed_bySteeringAngle = minSpeed + (((maxSteeringWheelAngle - steeringWheelAngle) / maxSteeringWheelAngle) * speedSpan) + (ki * sumSteeringWheelAngle) + (kd * derivativeSteeringError);

	newCarSpeed_bySteeringAngle = MAX(newCarSpeed_bySteeringAngle, minSpeed);
	newCarSpeed_bySteeringAngle = MIN(newCarSpeed_bySteeringAngle, maxSpeed);

	sumSteeringWheelAngle += steeringWheelAngle;

	if (floatCmp(ki, 0.0f) != 0) {
		sumSteeringWheelAngle = MAX(sumSteeringWheelAngle, -kiMinMaxImpact / ki);
		sumSteeringWheelAngle = MIN(sumSteeringWheelAngle, kiMinMaxImpact / ki);
	}
	else {
		sumSteeringWheelAngle = 0.0f;
	}



	prevSteeringWheelAngleError = steeringWheelAngle;

	return (float)MIN(newCarSpeed_bySteeringAngle, newCarSpeed_byTrajectoryAngle);
}

static float CalculateCarSpeed(float _min_speed, float _max_speed, float _wheel_base, float _friction_coefficient, float _downward_acceleration, float _turn_angle) {
	float new_car_speed, turn_radius;
	turn_radius = turnRadius(_wheel_base, _turn_angle);
	new_car_speed = carTurnMaxSpeed(turn_radius, _friction_coefficient, _downward_acceleration);

	new_car_speed = MIN(_max_speed, new_car_speed);
	new_car_speed = MAX(_min_speed, new_car_speed);

	return new_car_speed;
}


#endif // !__CALCULATECARSPEED_H__
