#pragma once
#include <math.h>
#include <string.h>

typedef struct LineMQ {
	float m;
	float q;
}LineMQ;

typedef struct Point2D {
	float x;
	float y;
}Point2D;

typedef struct IntersectionPoints2D_2
{
	Point2D point1;
	Point2D point2;
	int numPoints;
}IntersectionPoints2D_2;

typedef struct PurePersuitInfo {
	Point2D carPos;
	Point2D nextWayPoint;
	Point2D turnPoint;
	float distanceToWayPoints;
	float lookAheadDistance;
	float wheelAngle;
	float steeringAngle;
	float carLength;
	float turnRadius;
}PurePersuitInfo;


static LineMQ points2line(Point2D point1, Point2D point2) {
	LineMQ line;
	line.m = (point1.y - point2.y) / (point1.x - point1.x);
	line.q = (line.m * (-point1.x)) + point1.y;
	return line;
}

static float euclidianDistance(Point2D point1, Point2D point2) {
	float distance;
	distance = sqrtf(((point2.x - point1.x) * (point2.x - point1.x)) + ((point2.y - point1.y)* (point2.y - point1.y)));
	return distance;
}

static float distance2line(Point2D point, LineMQ line) {
	float distance;
	distance = fabsf((line.m * point.x) + (-1.0f * point.y) + line.q) / sqrtf((line.m * line.m) + 1);
	return distance;
}

static IntersectionPoints2D_2 intersectionLineCircle(Point2D circleCenter, float circleRadius, LineMQ line) {
	IntersectionPoints2D_2 points;
	memset(&points, 0, sizeof(points));

	float a, b, c, delta;
	a = 1.0f + (line.m * line.m);
	b = (2.0f * line.m * line.q) + ((-2.0f * circleCenter.x) + ((-2.0f * circleCenter.y * line.m)));
	c = ((-2.0f) * circleCenter.y * line.q) + (line.q * line.q) + (circleCenter.x * circleCenter.x) + (circleCenter.y * circleCenter.y) - (circleRadius * circleRadius);

	delta = b * b + ((-4.0f) * a * c);

	if (delta < 0) {
		points.numPoints = 0;
	}
	else if (delta == 0) {
		points.numPoints = 1;
	}
	else {
		points.numPoints = 2;
	}

	points.point1.x = (-b + sqrtf(delta)) / (2.0f * a);
	points.point2.x = (-b - sqrtf(delta)) / (2.0f * a);
	points.point1.y = line.m * points.point1.x + line.q;
	points.point2.y = line.m * points.point2.x + line.q;

	return points;
}

static float triangleAngleA(float AC, float CB, float BA) {
	float angle;
	angle = acosf(((AC * AC) + (BA * BA) - (CB * CB)) / (2.0f * AC * BA));
	return angle;
}

static float purePursuitWheelAngle(Point2D carPos, Point2D nextWayPoint) {
	float lookAheadDistance, wheelAngle;
	Point2D temp;

	lookAheadDistance = euclidianDistance(carPos, nextWayPoint);
	temp = carPos;
	temp.y += lookAheadDistance;
	wheelAngle = triangleAngleA(lookAheadDistance, euclidianDistance(nextWayPoint, temp), lookAheadDistance);
	if (carPos.x < nextWayPoint.x) {
		wheelAngle = -wheelAngle;
	}
	return wheelAngle;
}

static float steeringWheelAngle(float wheelAngle, float carLength, float nextWayPointDistance) {
	float angle;
	angle = (2.0f * carLength * sinf(wheelAngle)) / nextWayPointDistance;
	return angle;
}

static float turnRadius(float wheelAngle, float carLength, float nextWayPointDistance) {
	float angle;
	angle = (nextWayPointDistance / (2.0f * sinf(wheelAngle)));
	return angle;
}

static float purePursuitComputeSteeringWheelAngle(Point2D carPos, LineMQ wayPoints, float carLength, float lookAheadDistance) {
	float temp;
	IntersectionPoints2D_2 intersectionPoints;
	Point2D nextWayPoint;

	temp = distance2line(carPos, wayPoints);
	if (temp >= lookAheadDistance) {
		lookAheadDistance = temp + (temp * 0.25f);
	}

	intersectionPoints = intersectionLineCircle(carPos, lookAheadDistance, wayPoints);
	if (intersectionPoints.point1.y > intersectionPoints.point2.y) {
		nextWayPoint = intersectionPoints.point1;
	}
	else {
		nextWayPoint = intersectionPoints.point2;
	}

	temp = purePursuitWheelAngle(carPos, nextWayPoint);
	return steeringWheelAngle(temp, carLength, lookAheadDistance);
}

static PurePersuitInfo purePursuitCompute(Point2D carPos, LineMQ wayPoints, float carLength, float lookAheadDistance) {
	float temp;
	PurePersuitInfo info;
	IntersectionPoints2D_2 intersectionPoints;
	Point2D nextWayPoint;

	temp = distance2line(carPos, wayPoints);
	info.distanceToWayPoints = temp;
	if (temp >= lookAheadDistance) {
		lookAheadDistance = temp + (temp * 0.25f);
	}

	intersectionPoints = intersectionLineCircle(carPos, lookAheadDistance, wayPoints);
	if (intersectionPoints.point1.y > intersectionPoints.point2.y) {
		nextWayPoint = intersectionPoints.point1;
	}
	else {
		nextWayPoint = intersectionPoints.point2;
	}
	info.wheelAngle = purePursuitWheelAngle(carPos, nextWayPoint);
	info.steeringAngle = steeringWheelAngle(info.wheelAngle, carLength, lookAheadDistance);

	info.carPos = carPos;
	info.nextWayPoint = nextWayPoint;
	info.lookAheadDistance = lookAheadDistance;
	info.carLength = carLength;
	info.turnRadius = turnRadius(info.wheelAngle, carLength, lookAheadDistance);
	info.turnPoint = carPos;
	if (info.wheelAngle < 0) {
		info.turnPoint.x += info.turnRadius;
	}
	else {
		info.turnPoint.x -= info.turnRadius;
	}
	
	return info;
}

