/*
* Copyright 2023 Constantin Dumitru Petre RĂDULICEA
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*   http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/


#include "PurePursuitGeometry.h"
#include <cstdint>
#include <vector>
#include <string.h>

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


static float lane_width_vector_unit_real;
static float lookahead_min_distance_cm;
static float lookahead_max_distance_cm;
static float lookahead_pid_kp;
static float emergency_break_distance_cm;
static float min_speed;
static float max_speed;
static float black_color_treshold; // 0=black, 1=white
static float car_length_cm;



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

/*
* str:
* C-string beginning with the representation of a floating-point number.
* endptr:
* Reference to an already allocated object of type char*, whose value is set by the function to the next character in str after the numerical value.
* This parameter can also be a null pointer, in which case it is not used.
*/
float parseNextFloat(char* str, size_t strSize, char variableTerminator, char** endptr, int* success) {
	char* nextTerminator;
	char* pEnd;
	float result;
	nextTerminator = (char*)memchr(str, (int)variableTerminator, strSize);
	if (str == nextTerminator && strSize == 1) {
		if (endptr) {
			*endptr = nextTerminator;
		}
		if (success) {
			*success = 0;
		}

		return 0.0f;
	}
	if (nextTerminator) {
		*nextTerminator = '\0';
	}
	else {
		nextTerminator = str + strSize;
	}
	
	result = strtof(str, &pEnd);

	if (pEnd != nextTerminator) {
		// handle incomplete parse
		pEnd += 1;
		*success = 0;
		if (endptr) {
			*endptr = pEnd;
		}
	}
	else {
		pEnd += 1;
		*success = 1;
		if (endptr) {
			*endptr = pEnd;
		}
	}
	return result;
}

// 60.0;16.0;30.0;4.0;70.0;96.0;112.0;0.2;17.5
void parseAndSetGlobalVariables(std::vector<char>& rawData, char variableTerminator = ';') {
	char* pEnd;
	int resultSuccess;
	pEnd = rawData.data();
	lane_width_vector_unit_real = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
	lookahead_min_distance_cm = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
	lookahead_max_distance_cm = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
	lookahead_pid_kp = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
	min_speed = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
	max_speed = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
	black_color_treshold = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);
	car_length_cm = parseNextFloat(pEnd, (rawData.size() + rawData.data()) - pEnd, variableTerminator, &pEnd, &resultSuccess);

}






//
//
//static float calculateLookAheadDistancePID(PID pid, float minDistance, float maxDistance, LineABC laneMiddleLine) {
//	float angleCurrentTrajectoryAndMiddleLane, newLookAheadDistance, distanceSpan;
//	float pidIn, pidOut;
//	LineABC currentTrajectory;
//
//	/*
//	* setpoint: M_PI_2
//	* PID input: angleCurrentTrajectoryAndMiddleLane
//	*/
//
//	distanceSpan = maxDistance - minDistance;
//	currentTrajectory = yAxisABC();
//	angleCurrentTrajectoryAndMiddleLane = angleBetweenLinesABC(currentTrajectory, laneMiddleLine);
//
//	pidOut = fabs(pidOut);
//	pidOut = pidOut / M_PI_2;
//	pidOut = MAX(pidOut, 0.0f);
//	pidOut = MIN(pidOut, 1.0f);
//
//	newLookAheadDistance = minDistance + (pidOut * distanceSpan);
//
//	newLookAheadDistance = MAX(newLookAheadDistance, minDistance);
//	newLookAheadDistance = MIN(newLookAheadDistance, maxDistance);
//
//	return newLookAheadDistance;
//}

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
	LineABC line1, line2, line3, line4, ottusangle, acutangle, midLine;
	float temp_float_1;

	LineMQ wayPoints = { -3.5f, 625 };
	// LineABC wayPointsAbc = { 3.5f, 1, -625 };
	LineABC wayPointsAbc = { 2, 0, 0 };
	float carLength = 20.0f;
	float lookAheadDistance = 100.0f;
	Point2D carPos = { 200.0f, -carLength};
	PurePersuitInfo info, info2;

	float steeringAngle;

	line1 = points2lineABC(Point2D{ 36, 45 }, Point2D{ 53, 48 });
	line2 = parallelLineAtDistanceABC(line1, 50, 1);
	bisectorsOfTwoLinesABC(line1, line2, &line3, &line4);
	angleBetweenLinesABC(yAxisABC(), line3);
	angleBetweenLinesABC(yAxisABC(), line4);



	info = purePursuitCompute(carPos, wayPoints, carLength, lookAheadDistance);
	info2 = purePursuitComputeABC(carPos, line3, carLength, lookAheadDistance);

	carLength = angleBetweenLinesABC(points2lineABC(Point2D{ -1, -1 }, Point2D{-2, -2}), yAxisABC());
	
	wayPointsAbc = parallelLineAtDistanceABC(LineABC{ 3.0f, 1.0f, -5.0f }, 0.9486f, 0);


	bisectorsOfTwoLinesABC(LineABC{ 4.0f, -3.0f, 10.0f }, LineABC{ -6.0f, 8.0f, -5.0f }, &line1, &line2);
	bisectorsOfTwoLinesABC(LineABC{ 0.0f, 1.0f, 10.0f }, LineABC{ 0.0f, 1.0f, 5.0f }, &line1, &line2);
	bisectorsOfTwoLinesABC(LineABC{ 0.0f, 1.0f, 10.0f }, LineABC{ 0.0f, 1.0f, -5.0f }, &line1, &line2);
	bisectorsOfTwoLinesABC(LineABC{ 2.0f, 1.0f, 10.0f }, LineABC{ 2.0f, 1.0f, -5.0f }, &line1, &line2);


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


	//calculateLookAheadDistance_noPID(5, 30, LineABC{ 2, 1, -7 });
	//calculateLookAheadDistance_noPID(5, 30, yAxisABC());
	//calculateLookAheadDistance_noPID(5, 30, xAxisABC());


	line1 = points2lineABC(Point2D{ 22, 20 }, Point2D{ 36, 34 });	// -1x + 1y + 2 = 0
	line2 = points2lineABC(Point2D{ 41, 7 }, Point2D{ 63, 21 });	// -0.636x + 1y + 19.0909 = 0

	bisectorsOfTwoLinesABC(line1, line2, &acutangle, &ottusangle);

	float angle1, angle2, angle3, angle4;
	angle1 = angleBetweenLinesABC(line1, acutangle);
	angle2 = angleBetweenLinesABC(line1, ottusangle);

	angle3 = angleBetweenLinesABC(yAxisABC(), acutangle);
	angle4 = angleBetweenLinesABC(yAxisABC(), ottusangle);
	

	char number[] = "60.0;16.0;30.0;4.0;70.0;96.0;112.0;0.2;17.5";
	std::vector<char> data;
	data.resize(strlen(number) + 1);
	memcpy(data.data(), number, strlen(number) + 1);
	data.pop_back();
	parseAndSetGlobalVariables(data, ';');

	line1 = points2lineABC(Point2D{ 0.0f, 0.0f }, Point2D{ 2.236f, 4.472f});
	line2 = points2lineABC(Point2D{ 0.0f, 0.0f }, Point2D{ 4.472f, 2.236f});
	temp_float_1 = distanceBwLinesABC(line1, line2, Point2D{ 2.236f, 4.472f });

	line1 = LineABC{ 1.0f, 1.0f, -2.0f };
	line2 = LineABC{ 1.0f, 1.0f, 0.0f };
	temp_float_1 = distanceBwLinesABC(line1, line2, Point2D{ 1.0f, 0.0f });

	line1 = LineABC{ 1.0f, 1.0f, -2.0f };
	line2 = LineABC{ 1.000001f, 1.0f, 0.0f };
	temp_float_1 = distanceBwLinesABC(line1, line2, Point2D{ 1.0f, 0.0f });

	line1 = LineABC{ 1.0f, 1.0f, -2.0f };
	line2 = LineABC{ 2.0f, 1.0f, 0.0f };
	temp_float_1 = distanceBwLinesABC(line1, line2, Point2D{ 1.0f, 0.0f });


	line1 = LineABC{ 99.0f, 0.0f, 0.0f };
	line2 = LineABC{ 3.0f, 0.0f, 3.0f };
	temp_float_1 = distanceBwParallelLinesABC(line1, line2);

	line1 = LineABC{ 3.0f, 20.0f, 0.0f };
	line2 = LineABC{ 3.0f, 20.0f, 3.0f };
	temp_float_1 = distanceBwParallelLinesABC(line1, line2);


	int int1 = isPointOnSegment(LineSegment{ Point2D{ -1.5f, 0.0f}, Point2D{ 0.0f, -3.0f} }, Point2D{ -0.75f, -1.5f });	// true
	int1 = isPointOnSegment(LineSegment{ Point2D{ -1.5f, 0.0f}, Point2D{ 0.0f, -3.0f} }, Point2D{ -0.76f, -1.5f });		// false

	LineSegmentsDistancePoints lineSegmentsDistances;
	lineSegmentsDistances = distancePointsBwSegments(LineSegment{ Point2D{ 0.0f, 1.0f}, Point2D{ 5.0f, 1.0f} }, LineSegment{ Point2D{ 0.0f, -5.0f}, Point2D{ 5.0f, -5.0f} });		// min = {A={x=0.00000000 y=1.00000000 } B={x=0.00000000 y=-5.00000000 } }		max = {A={x=0.00000000 y=1.00000000 } B={x=0.00000000 y=-5.00000000 } }
	lineSegmentsDistances = distancePointsBwSegments(LineSegment{ Point2D{ 1.0f, 1.0f}, Point2D{ 5.0f, 1.0f} }, LineSegment{ Point2D{ 0.0f, -5.0f}, Point2D{ 5.0f, -5.0f} });		// min = {A={x=1.00000000 y=1.00000000 } B={x=1.00000000 y=-5.00000000 } }		max = {A={x=1.00000000 y=1.00000000 } B={x=1.00000000 y=-5.00000000 } }
	lineSegmentsDistances = distancePointsBwSegments(LineSegment{ Point2D{ -1.0f, 2.0f}, Point2D{ 2.0f, -1.0f} }, LineSegment{ Point2D{ -2.0f, -5.0f}, Point2D{ 5.0f, -5.0f} });	// min = {A={x=2.00000000 y=-1.00000000 } B={x=2.00000000 y=-5.00000000 } }		max = {A={x=-1.00000000 y=2.00000000 } B={x=-1.00000000 y=-5.00000000 } }
	lineSegmentsDistances = distancePointsBwSegments(LineSegment{ Point2D{ -1.0f, 2.0f}, Point2D{ 2.0f, -1.0f} }, LineSegment{ Point2D{ 0.0f, -5.0f}, Point2D{ 5.0f, -5.0f} });		// min = {A={x=2.00000000 y=-1.00000000 } B={x=2.00000000 y=-5.00000000 } }		max = {A={x=2.00000000 y=-1.00000000 } B={x=2.00000000 y=-5.00000000 } }

	return 0;
}






