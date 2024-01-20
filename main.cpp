#include "PurePursuitGeometry.h"




int main() {
	points2lineABC(Point2D{ 48, 36 }, Point2D{ 12, 10 });

	LineMQ wayPoints = { -3.5f, 625 };
	// LineABC wayPointsAbc = { 3.5f, 1, -625 };
	LineABC wayPointsAbc = { 2, 0, 0 };
	float carLength = 20.0f;
	float lookAheadDistance = 100.0f;
	Point2D carPos = { 200.0f, -carLength};
	PurePersuitInfo info, info2;

	float steeringAngle;


	info = purePursuitCompute(carPos, wayPoints, carLength, lookAheadDistance);
	info2 = purePursuitComputeABC(carPos, wayPointsAbc, carLength, lookAheadDistance);

	carLength = angleBetweenLinesABC(points2lineABC(Point2D{ 48, 36 }, Point2D{12, 10}), xAxisABC());
	
	wayPointsAbc = parallelLineAtDistance(LineABC{ 3.0f, 1.0f, -5.0f }, 0.9486f, 0);

	return 0;
}