#include "homography.h"


int main22() {
	float H[3][3] = { 0.0f };
	Point2D tem_point;
	Point2D imgPoints[4] = {
	Point2D{100, 200},   // Point 1
	Point2D{300, 200},   // Point 2
	Point2D{100, 400},   // Point 3
	Point2D{300, 400}    // Point 4
	};
	Point2D realWorldPoints[4] = {
	Point2D{0, 0},       // Point 1 corresponds to (100, 200) in image
	Point2D{200, 0},     // Point 2 corresponds to (300, 200) in image
	Point2D{0, 200},     // Point 3 corresponds to (100, 400) in image
	Point2D{200, 200}    // Point 4 corresponds to (300, 400) in image
	};
	calculateHomography(imgPoints, realWorldPoints, H);
	tem_point = applyHomography(Point2D{ 2, 2 }, H);
	return 0;
}