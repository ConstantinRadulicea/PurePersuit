#ifndef __HOMOGRAPHY_H__
#define __HOMOGRAPHY_H__

#include "geometry2D.h"

static void calculateHomography(Point2D imgPoints[4], Point2D worldPoints[4], float H[3][3]) {
    // A matrix (8x8) and b vector (8x1) to hold the linear equations
    float A[8][8+1] = { 0 };
    float b[8] = { 0 };
    float h[8];  // Vector for storing h11, h12, ..., h32 (we assume h33 = 1)

    // Populate A matrix and b vector using the point correspondences
    for (int i = 0; i < 4; i++) {
        float x = imgPoints[i].x;
        float y = imgPoints[i].y;
        float x_prime = worldPoints[i].x;
        float y_prime = worldPoints[i].y;

        // First row for this point correspondence
        A[2 * i][0] = x;
        A[2 * i][1] = y;
        A[2 * i][2] = 1;
        A[2 * i][6] = -x_prime * x;
        A[2 * i][7] = -x_prime * y;
        b[2 * i] = x_prime;

        // Second row for this point correspondence
        A[2 * i + 1][3] = x;
        A[2 * i + 1][4] = y;
        A[2 * i + 1][5] = 1;
        A[2 * i + 1][6] = -y_prime * x;
        A[2 * i + 1][7] = -y_prime * y;
        b[2 * i + 1] = y_prime;
    }

    for (size_t i = 0; i < 8; i++)
    {
        A[i][8] = b[i];
    }

    // Solve the system of equations A * h = b
    // This is typically done with Gaussian elimination or other methods. Here, let's use Gaussian elimination.
    gaussianElimination8(A, b, 8);

    // Now populate the H matrix with the result
    H[0][0] = b[0]; H[0][1] = b[1]; H[0][2] = b[2];
    H[1][0] = b[3]; H[1][1] = b[4]; H[1][2] = b[5];
    H[2][0] = b[6]; H[2][1] = b[7]; H[2][2] = 1.0f;  // h33 is assumed to be 1
}


// Function to apply homography to a point (x, y)
static Point2D applyHomography(Point2D original_point, float H[3][3]) {
    Point2D new_point;
    float z = H[2][0] * original_point.x + H[2][1] * original_point.y + H[2][2];
    new_point.x = (H[0][0] * original_point.x + H[0][1] * original_point.y + H[0][2]) / z;
    new_point.y = (H[1][0] * original_point.x + H[1][1] * original_point.y + H[1][2]) / z;
    return new_point;
}




#endif // !__HOMOGRAPHY_H__
