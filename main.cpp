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

// return 0 on success
static int aproximateVector(/*Pixy2& pixy,*/ Vector& vec, float blackTreshold) {
	int8_t res;
	Point2D midPoint_;
	LineABC perpendicularLine, vectorLine;
	int maxX, maxY, vectorMaxX, vectorMaxY;
	int xRight, xLeft, yUp, yDown, xFound, yFound;

	maxX = 315;
	maxY = 207;
	vectorMaxX = 78;
	vectorMaxY = 51;

	//res = pixy.changeProg("video");
	//if (res != PIXY_RESULT_OK) {
	//	return 1;
	//}
	//// write code here

	midPoint_ = vectorMidPoint(vec);
	midPoint_.x = (midPoint_.x / (float)vectorMaxX) * (float)maxX;
	midPoint_.y = (midPoint_.y / (float)vectorMaxY) * (float)maxY;

	vectorLine = vectorToLineABC(vec);
	perpendicularLine = perpendicularToLinePassingThroughPointABC(vectorLine, midPoint_);

	if (!isLineParallelToYaxis(perpendicularLine))
	{
		xRight = (int)roundf(midPoint_.x);
		xLeft = (int)roundf(midPoint_.x-1.0f);
		while (xRight <= maxX || xLeft >= 0) {
			if (xRight <= maxX)
			{
				yFound = (int)roundf(((-perpendicularLine.Ax) * (float)xRight) - perpendicularLine.C);
				if (yFound > maxY || yFound < 0) {
					xRight = maxX + 1;
					continue;
				}
				xFound = xRight;
				xRight++;
				// read pixel and do calculations
			}
			if (xLeft >= 0)
			{
				yFound = (int)roundf(((-perpendicularLine.Ax) * (float)xLeft) - perpendicularLine.C);
				if (yFound > maxY || yFound < 0) {
					xLeft = -1;
					continue;
				}
				xFound = xLeft;
				xLeft--;
				// read pixel and do calculations
			}
		}
	}
	else {
		yUp = (int)roundf(midPoint_.y);
		yDown = (int)roundf(midPoint_.y - 1.0f);
		xFound = midPoint_.x;
		while (yUp <= maxX || yDown >= 0) {
			if (yUp <= maxX)
			{
				yFound = yUp;
				yUp++;
				// read pixel and do calculations
			}
			if (yDown >= 0)
			{
				yFound = yDown;
				yDown--;
				// read pixel and do calculations
			}
		}
	}



	//res = pixy.changeProg("line");
	//if (res != PIXY_RESULT_OK) {
	//	return 1;
	//}
	//else {
	//	return 0;
	//}


	return 0;
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

	Vector vec;
	//vec.m_x0 = 30;
	//vec.m_y0 = 10;
	//vec.m_x1 = 20;
	//vec.m_y1 = 30;

	//aproximateVector(vec, 0.5f);

	vec.m_x0 = 84;
	vec.m_y0 = 28;
	vec.m_x1 = 117;
	vec.m_y1 = 6;

	aproximateVector(vec, 0.5f);
	return 0;
}






