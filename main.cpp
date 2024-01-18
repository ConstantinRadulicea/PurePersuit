#include "PurePursuitGeometry.h"




int main() {
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

	carLength = angleBetweenLinesABC(wayPointsAbc, lineMQ2ABC(wayPoints));
	return 0;
}