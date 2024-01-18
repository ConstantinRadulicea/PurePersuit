#ifndef __PUREPURSUITGEOMETRY_H__
#define __PUREPURSUITGEOMETRY_H__




#include <math.h>
#include <string.h>
#include <math.h>
#include <corecrt_math_defines.h>

typedef struct LineMQ {
	float m;
	float q;
}LineMQ;

typedef struct LineABC {
	float Ax;
	float By;
	float C;
}LineABC;

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
	float TrajectoryToWayPointAngle;
	float steeringAngle;
	float carLength;
	float turnRadius;
	float manouvreLength;
}PurePersuitInfo;


static LineABC xAxisABC() {
	LineABC line;
	line.Ax = 0.0f;
	line.By = 1.0f;
	line.C = 0.0f;
}

static LineABC yAxisABC() {
	LineABC line;
	line.Ax = 1.0f;
	line.By = 0.0f;
	line.C = 0.0f;
}

static LineABC normalizeLineABC2MQ(LineABC line) {
	if (line.By != 1.0f && line.By != 0.0f) {
		line.C = line.C / line.By;
		line.Ax = line.Ax / line.By;
		line.By = 1.0f;
	}
	return line;
}

static int isLineParallelToXaxis(LineABC line) {
	if (line.Ax == 0 && line.By != 0)
	{
		return 1;
	}
	else {
		return 0;
	}
}

static int isLineParallelToYaxis(LineABC line) {
	if (line.By == 0 && line.Ax != 0)
	{
		return 1;
	}
	else {
		return 0;
	}
}

static float angleBetweenLines(LineMQ line1, LineMQ line2) {
	float angle;
	angle = atanf(fabsf((line1.m - line2.m)) / (1.0f + (line1.m * line2.m)));
	return angle;
}

static LineMQ middleLine(LineMQ line1, LineMQ line2) {
	LineMQ middleLine_;
	middleLine_.m = (line1.m + line2.m) / 2.0f;
	middleLine_.q = (line1.q + line2.q) / 2.0f;
	return middleLine_;
}

static LineABC middleLineABC(LineABC line1, LineABC line2) {
	LineABC middleLine_;
	middleLine_.Ax = (line1.Ax + line2.Ax) / 2.0f;
	middleLine_.By = (line1.By + line2.By) / 2.0f;
	middleLine_.C = (line1.C + line2.C) / 2.0f;
	return middleLine_;
}

static LineMQ points2line(Point2D point1, Point2D point2) {
	LineMQ line;
	line.m = (point1.y - point2.y) / (point1.x - point1.x);
	line.q = (line.m * (-point1.x)) + point1.y;
	return line;
}

static LineABC lineMQ2ABC(LineMQ line){
	LineABC lineAbc;
	lineAbc.Ax = -line.m;
	lineAbc.By = 1.0f;
	lineAbc.C = -line.q;
	return lineAbc;
}

static LineMQ lineABC2MQ(LineABC line) {
	LineMQ lineMq;
	line = normalizeLineABC2MQ(line);
	lineMq.m = -line.Ax;
	lineMq.q = -line.C;
	return lineMq;
}

static float angleBetweenLinesABC(LineABC line1, LineABC line2) {
	float angle;
	LineMQ line1Mq, line2Mq;

	if (isLineParallelToYaxis(line1) && isLineParallelToYaxis(line2)) {
		return 0.0f;	/* line1 // line2 // Y */
	}

	if (isLineParallelToYaxis(line1))
	{
		line1Mq = lineABC2MQ(line2);
		line2Mq.m = 0;
		line2Mq.q = 0;
		angle = M_PI_2 - angleBetweenLines(line1Mq, line2Mq);
	}
	else if (isLineParallelToYaxis(line2)) {
		line1Mq = lineABC2MQ(line1);
		line2Mq.m = 0;
		line2Mq.q = 0;
		angle = M_PI_2 - angleBetweenLines(line1Mq, line2Mq);
	}
	else {
		line1Mq = lineABC2MQ(line1);
		line2Mq = lineABC2MQ(line2);
		angle = angleBetweenLines(line1Mq, line2Mq);
	}

	return angle;
}

static LineABC points2lineABC(Point2D point1, Point2D point2) {
	LineMQ lineMq;
	LineABC lineAbc;
	if (point1.x == point2.x) { // perpendicular to y axis
		lineAbc = yAxisABC();
		lineAbc.C = -point1.x;
		return lineAbc;
	}
	lineMq = points2line(point1, point2);
	lineAbc = lineMQ2ABC(lineMq);
	return lineAbc;
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

static float distance2lineABC(Point2D point, LineABC lineAbc) {
	float distance;
	LineMQ line;
	if (lineAbc.By == 0)
	{
		return fabsf(point.x - (-lineAbc.C));
	}
	line = lineABC2MQ(lineAbc);
	distance = fabsf((line.m * point.x) + (-1.0f * point.y) + line.q) / sqrtf((line.m * line.m) + 1);
	return distance;
}

static IntersectionPoints2D_2 intersectionLineCircle(Point2D circleCenter, float circleRadius, LineMQ line) {
	IntersectionPoints2D_2 points;
	float a, b, c, delta;

	memset(&points, 0, sizeof(points));

	// ax^2 + bx + c = 0
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

static IntersectionPoints2D_2 intersectionLineCircleABC(Point2D circleCenter, float circleRadius, LineABC lineAbc) {
	IntersectionPoints2D_2 points;
	LineMQ line;
	float a, b, c, x_, delta;

	memset(&points, 0, sizeof(points));

	if (!isLineParallelToYaxis(lineAbc))
	{
		line = lineABC2MQ(lineAbc);
		// ax^2 + bx + c = 0
		a = 1.0f + (line.m * line.m);
		b = (2.0f * line.m * line.q) + ((-2.0f * circleCenter.x) + ((-2.0f * circleCenter.y * line.m)));
		c = ((-2.0f) * circleCenter.y * line.q) + (line.q * line.q) + (circleCenter.x * circleCenter.x) + (circleCenter.y * circleCenter.y) - (circleRadius * circleRadius);
	}
	else {
		x_ = (- lineAbc.C) / lineAbc.Ax;
		a = 1.0f;
		b = -2.0f * circleCenter.y;
		c = (x_ * x_) + (-2.0f * circleCenter.x * x_)  + (circleCenter.x * circleCenter.x) - (circleRadius * circleRadius);
	}


	delta = b * b + ((-4.0f) * a * c);

	if (delta < 0) {
		points.numPoints = 0;
		return points;
	}
	else if (delta == 0) {
		points.numPoints = 1;
	}
	else {
		points.numPoints = 2;
	}


	if (!isLineParallelToYaxis(lineAbc))
	{
		points.point1.x = (-b + sqrtf(delta)) / (2.0f * a);
		points.point2.x = (-b - sqrtf(delta)) / (2.0f * a);
		points.point1.y = line.m * points.point1.x + line.q;
		points.point2.y = line.m * points.point2.x + line.q;
	}
	else {
		points.point1.y = (-b + sqrtf(delta)) / (2.0f * a);
		points.point2.y = (-b - sqrtf(delta)) / (2.0f * a);
		points.point1.x = x_;
		points.point2.x = x_;
	}

	return points;
}

static float triangleAngleA(float AC, float CB, float BA) {
	float angle;
	angle = acosf(((AC * AC) + (BA * BA) - (CB * CB)) / (2.0f * AC * BA));
	return angle;
}

static float carTrajectoryAndWayPointAngle(Point2D carPos, Point2D nextWayPoint) {
	float lookAheadDistance, TrajectoryToWayPointAngle;
	Point2D temp;

	lookAheadDistance = euclidianDistance(carPos, nextWayPoint);
	temp = carPos;
	temp.y += lookAheadDistance;
	TrajectoryToWayPointAngle = triangleAngleA(lookAheadDistance, euclidianDistance(nextWayPoint, temp), lookAheadDistance);
	if (carPos.x < nextWayPoint.x) {
		TrajectoryToWayPointAngle = -TrajectoryToWayPointAngle;
	}
	return TrajectoryToWayPointAngle;
}

static float steeringWheelAngle(float TrajectoryToWayPointAngle, float carLength, float nextWayPointDistance) {
	float angle;
	angle = atanf((2.0f * carLength * sinf(TrajectoryToWayPointAngle)) / nextWayPointDistance);
	return angle;
}

static float turnRadius(float TrajectoryToWayPointAngle, float carLength, float nextWayPointDistance) {
	float angle;
	angle = (nextWayPointDistance / (2.0f * sinf(TrajectoryToWayPointAngle)));
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

	temp = carTrajectoryAndWayPointAngle(carPos, nextWayPoint);
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
	info.TrajectoryToWayPointAngle = carTrajectoryAndWayPointAngle(carPos, nextWayPoint);
	info.steeringAngle = steeringWheelAngle(info.TrajectoryToWayPointAngle, carLength, lookAheadDistance);

	info.carPos = carPos;
	info.nextWayPoint = nextWayPoint;
	info.lookAheadDistance = lookAheadDistance;
	info.carLength = carLength;
	info.turnRadius = turnRadius(info.TrajectoryToWayPointAngle, carLength, lookAheadDistance);
	info.manouvreLength = fabsf(((2.0f * M_PI * info.turnRadius) * info.TrajectoryToWayPointAngle) / (2.0f * M_PI));
	info.turnPoint = carPos;
	if (info.TrajectoryToWayPointAngle < 0) {
		info.turnPoint.x += info.turnRadius;
	}
	else {
		info.turnPoint.x -= info.turnRadius;
	}
	
	return info;
}

static PurePersuitInfo purePursuitComputeABC(Point2D carPos, LineABC wayPoints, float carLength, float lookAheadDistance) {
	float temp;
	PurePersuitInfo info;
	IntersectionPoints2D_2 intersectionPoints;
	Point2D nextWayPoint;

	temp = distance2lineABC(carPos, wayPoints);
	info.distanceToWayPoints = temp;
	if (temp >= lookAheadDistance) {
		lookAheadDistance = temp + (temp * 0.25f);
	}

	intersectionPoints = intersectionLineCircleABC(carPos, lookAheadDistance, wayPoints);
	if (intersectionPoints.point1.y > intersectionPoints.point2.y) {
		nextWayPoint = intersectionPoints.point1;
	}
	else {
		nextWayPoint = intersectionPoints.point2;
	}
	info.TrajectoryToWayPointAngle = carTrajectoryAndWayPointAngle(carPos, nextWayPoint);
	info.steeringAngle = steeringWheelAngle(info.TrajectoryToWayPointAngle, carLength, lookAheadDistance);

	info.carPos = carPos;
	info.nextWayPoint = nextWayPoint;
	info.lookAheadDistance = lookAheadDistance;
	info.carLength = carLength;
	info.turnRadius = turnRadius(info.TrajectoryToWayPointAngle, carLength, lookAheadDistance);
	info.manouvreLength = fabsf(((2.0f * M_PI * info.turnRadius) * info.TrajectoryToWayPointAngle) / (2.0f * M_PI));
	info.turnPoint = carPos;
	if (info.TrajectoryToWayPointAngle < 0) {
		info.turnPoint.x += info.turnRadius;
	}
	else {
		info.turnPoint.x -= info.turnRadius;
	}

	return info;
}


#endif // !__PUREPURSUITGEOMETRY_H__

