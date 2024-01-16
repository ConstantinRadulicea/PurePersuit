#include "PurePursuitGeometry.h"




int main() {
	LineMQ wayPoints = {-3.5f, 625};
	float carLength = 20.0f;
	float lookAheadDistance = 100.0f;
	Point2D carPos = { 200.0f, -carLength};
	PurePersuitInfo info;

	float steeringAngle;


	info = purePursuitCompute(carPos, wayPoints, carLength, lookAheadDistance);
	return 0;
}