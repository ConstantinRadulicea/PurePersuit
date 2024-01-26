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

// 
static Vector vectorApproximationSelectionLogic(Vector vec, Point2D approximatedPointFound, Point2D carPosition) {
	Point2D startVector, endVector, newVectorStart, newVectorEnd;
	float distanceStartVector, distanceEndVector, distanceApproximatedPointFound;
	Vector newVector;

	memset(&newVector, 0, sizeof(newVector));

	startVector.x = (float)vec.m_x0;
	startVector.y = (float)vec.m_y0;

	endVector.x = (float)vec.m_x1;
	endVector.y = (float)vec.m_y1;

	distanceStartVector = euclidianDistance(carPosition, startVector);
	distanceEndVector = euclidianDistance(carPosition, endVector);
	distanceApproximatedPointFound = euclidianDistance(carPosition, approximatedPointFound);

	if (distanceStartVector < distanceEndVector && distanceStartVector < distanceApproximatedPointFound){
		newVectorStart = startVector;
		newVectorEnd = approximatedPointFound;
	}
	else if (distanceEndVector < distanceStartVector && distanceEndVector < distanceApproximatedPointFound){
		newVectorStart = endVector;
		newVectorEnd = approximatedPointFound;
	}
	else{
		newVectorStart = approximatedPointFound;
		newVectorEnd = endVector;
	}

	newVector.m_x0 = newVectorStart.x;
	newVector.m_y0 = newVectorStart.y;
	newVector.m_x1 = newVectorEnd.x;
	newVector.m_y1 = newVectorEnd.y;

	return newVector;
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
	bisectorsOfTwoLines(line1, line2, &line3, NULL);



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
	//vec.m_x0 = 30;
	//vec.m_y0 = 10;
	//vec.m_x1 = 20;
	//vec.m_y1 = 30;

	//aproximateVector(vec, 0.5f);

	vec.m_x0 = 50;
	vec.m_y0 = 0;
	vec.m_x1 = 77;
	vec.m_y1 = 30;
	vecResult = vectorApproximationSelectionLogic(vec, Point2D{ 52.0f, 25.0f }, Point2D{ 39.0f, 0.0f });


	vec.m_x0 = 77;
	vec.m_y0 = 0;
	vec.m_x1 = 52;
	vec.m_y1 = 25;
	vecResult = vectorApproximationSelectionLogic(vec, Point2D{ 50.0f, 0.0f }, Point2D{ 39.0f, 0.0f });

	return 0;
}






