#include "PurePursuitGeometry.h"
#include <cstdint>

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

#define MIN_SPEED (int)100
#define MAX_SPEED (int)150
#define SCREEN_CENTER_X (int)(78.0f / 2.0f)
#define IMAGE_MAX_X 78
#define IMAGE_MAX_Y 51
#define LANE_WIDTH_PIXELS 50

#define STEERING_SERVO_ANGLE_MIDDLE     85    // 90 middle
#define STEERING_SERVO_ANGLE_MAX_RIGHT  0    // 38 max right
#define STEERING_SERVO_ANGLE_MAX_LEFT   180   // 135 max left
#define STEERING_SERVO_MAX_ANGLE MAX(fabsf(STEERING_SERVO_ANGLE_MIDDLE - STEERING_SERVO_ANGLE_MAX_RIGHT), fabsf(STEERING_SERVO_ANGLE_MIDDLE - STEERING_SERVO_ANGLE_MAX_LEFT))





struct Vector
{
	uint8_t m_x0;
	uint8_t m_y0;
	uint8_t m_x1;
	uint8_t m_y1;
	uint8_t m_index;
	uint8_t m_flags;
};




static Point2D vectorMidPoint(Vector vec) {
	Point2D point1, point2;
	point1.x = (float)vec.m_x0;
	point1.y = (float)vec.m_y0;

	point2.x = (float)vec.m_x1;
	point2.y = (float)vec.m_y1;

	return midPoint(point1, point2);
}

static LineABC vectorToLineABC(Vector vec) {
	LineABC line2;
	Point2D point1, point2;

	point1.x = (float)vec.m_x0;
	point1.y = (float)vec.m_y0;

	point2.x = (float)vec.m_x1;
	point2.y = (float)vec.m_y1;

	line2 = points2lineABC(point1, point2);
	return line2;
}

static float calculateLookAheadDistance_noPID(float minDistance, float maxDistance, LineABC laneMiddleLine) {
	float angleCurrentTrajectoryAndMiddleLane, newLookAheadDistance, distanceSpan;
	LineABC currentTrajectory;
	distanceSpan = maxDistance - minDistance;

	currentTrajectory = yAxisABC();

	angleCurrentTrajectoryAndMiddleLane = angleBetweenLinesABC(currentTrajectory, laneMiddleLine);

	newLookAheadDistance = minDistance + ((((float)M_PI_2 - angleCurrentTrajectoryAndMiddleLane) / (float)M_PI_2) * distanceSpan);

	newLookAheadDistance = MAX(newLookAheadDistance, minDistance);
	newLookAheadDistance = MIN(newLookAheadDistance, maxDistance);

	return newLookAheadDistance;
}
/*
static float calculateLookAheadDistancePID(PID pid, float minDistance, float maxDistance, LineABC laneMiddleLine) {
	float angleCurrentTrajectoryAndMiddleLane, newLookAheadDistance, distanceSpan;
	float pidIn, pidOut;
	LineABC currentTrajectory;
	distanceSpan = maxDistance - minDistance;

	currentTrajectory = yAxisABC();

	angleCurrentTrajectoryAndMiddleLane = angleBetweenLinesABC(currentTrajectory, laneMiddleLine);

	pidOut = MAX(pidOut, 0.0f);
	pidOut = MIN(pidOut, 1.0f);

	newLookAheadDistance = minDistance + (pidOut * distanceSpan);

	newLookAheadDistance = MAX(newLookAheadDistance, minDistance);
	newLookAheadDistance = MIN(newLookAheadDistance, maxDistance);

	return newLookAheadDistance;
}
*/
static float calculateCarSpeed(float minSpeed, float maxSpeed, float maxSteeringWheelAngle, float steeringWheelAngle) {
	float newCarSpeed, speedSpan;

	speedSpan = maxSpeed - minSpeed;
	maxSteeringWheelAngle = fabsf(maxSteeringWheelAngle);
	steeringWheelAngle = fabsf(steeringWheelAngle);
	steeringWheelAngle = MIN(steeringWheelAngle, maxSteeringWheelAngle);
	
	newCarSpeed = minSpeed + (((maxSteeringWheelAngle - steeringWheelAngle) / maxSteeringWheelAngle) * speedSpan);

	newCarSpeed = MAX(newCarSpeed, minSpeed);
	newCarSpeed = MIN(newCarSpeed, maxSpeed);

	return newCarSpeed;
}


int main() {
	points2lineABC(Point2D{ 48, 36 }, Point2D{ 12, 10 });
	LineABC line1, line2, line3, line4;

	LineMQ wayPoints = { -3.5f, 625 };
	// LineABC wayPointsAbc = { 3.5f, 1, -625 };
	LineABC wayPointsAbc = { 2, 0, 0 };
	float carLength = 20.0f;
	float lookAheadDistance = 100.0f;
	Point2D carPos = { 200.0f, -carLength};
	PurePersuitInfo info, info2;

	float steeringAngle;

	line1 = points2lineABC(Point2D{ 36, 45 }, Point2D{ 53, 48 });
	line2 = parallelLineAtDistance(line1, 50, 1);
	bisectorsOfTwoLines(line1, line2, &line3, &line4);
	angleBetweenLinesABC(yAxisABC(), line3);
	angleBetweenLinesABC(yAxisABC(), line4);



	info = purePursuitCompute(carPos, wayPoints, carLength, lookAheadDistance);
	info2 = purePursuitComputeABC(carPos, line3, carLength, lookAheadDistance);

	carLength = angleBetweenLinesABC(points2lineABC(Point2D{ -1, -1 }, Point2D{-2, -2}), yAxisABC());
	
	wayPointsAbc = parallelLineAtDistance(LineABC{ 3.0f, 1.0f, -5.0f }, 0.9486f, 0);


	bisectorsOfTwoLines(LineABC{ 4.0f, -3.0f, 10.0f }, LineABC{ -6.0f, 8.0f, -5.0f }, &line1, &line2);
	bisectorsOfTwoLines(LineABC{ 0.0f, 1.0f, 10.0f }, LineABC{ 0.0f, 1.0f, 5.0f }, &line1, &line2);
	bisectorsOfTwoLines(LineABC{ 0.0f, 1.0f, 10.0f }, LineABC{ 0.0f, 1.0f, -5.0f }, &line1, &line2);
	bisectorsOfTwoLines(LineABC{ 2.0f, 1.0f, 10.0f }, LineABC{ 2.0f, 1.0f, -5.0f }, &line1, &line2);


	line1 = perpendicularToLinePassingThroughPointABC(xAxisABC(), Point2D{ 10, 12 });
	line1 = perpendicularToLinePassingThroughPointABC(yAxisABC(), Point2D{ 10, 12 });
	line1 = perpendicularToLinePassingThroughPointABC(LineABC{ 2.0f, 1.0f, -5.0f }, Point2D{ 10, 12 });	//-0.5x+1y-7=0
	line1 = perpendicularToLinePassingThroughPointABC(LineABC{ 2.0f, 1.0f, -5.0f }, Point2D{ -10, -12 });	//-0.5x+1y+7=0

	Vector vec, vecResult;


	calculateCarSpeed(100, 110, 90, 0);
	calculateCarSpeed(100, 110, 90, 20);
	calculateCarSpeed(100, 110, 90, 45);
	calculateCarSpeed(100, 110, 90, 60);
	calculateCarSpeed(100, 110, 90, 90);


	calculateLookAheadDistance(5, 30, LineABC{ 2, 1, -7 });
	calculateLookAheadDistance(5, 30, yAxisABC());
	calculateLookAheadDistance(5, 30, xAxisABC());


	return 0;
}






