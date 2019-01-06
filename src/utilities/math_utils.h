//MIT License
//Copyright (c) 2018 Eotvos Lorand University, Budapest

/* vim:set ts=4 sw=4 sts=4 et: */

/*
 * Useful mathematical tools (e. g. linear algebra tools, randomizers, etc.)
 */

#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <ctype.h>

/* Randomizer functions */

/* Returns a random double value between minValue and maxValue (uniform distribution)
 */
double randomizeDouble(const double minValue, const double maxValue);

/* Returns a random double value (Gaussian distribution with given Mean and StdDev)
 * The function uses the so-called Box-Muller transformation rule
 * Mean defines the center of the Gaussian distribution
 * StdDev defines the "Sigma" value.
 */
double randomizeGaussDouble(const double Mean, const double StdDev);

/* Returns a random double value (Power-law distribution with given exponent in a given range)
 * http://mathworld.wolfram.com/RandomNumber.html
 * x0, x1: boundaries of the range
 * n is the exponent
 */
double randomizePowerLawDouble(const double x0, const double x1,
        const double n);

// integer power
int ipow(int base, int exp);

/* Linear algebra */

/* Fills a vector with the input values (x, y, z components)
 * VectToFill is the output vector
 * x, y and z are the 0th, 1st and 2nd components of the output vector, respectively
 */
void FillVect(double *VectToFill, const double x, const double y,
        const double z);

/* Returns the Magnitude of the input vector (3 dimensions)
 */
double VectAbs(double *InputVector);

/* Returns the XY-projected Magnitude of the input vector
 * Useful for calculating pair-potentials of 2 dimensional models
 */
double VectAbsXY(double *InputVector);

/* Difference of two vectors (3D)
 * "ABDiff" is the difference of the vectors "VectA" and "VectB"
 * ABDiff = VectA - VectB
 */
void VectDifference(double *ABDiff, double *VectA, double *VectB);

/* Sum of two vectors (3D)
 * "ABSum" is the sum of "VectA" and "VectB"
 * ABSum = VectA + VectB
 */
void VectSum(double *ABSum, double *VectA, double *VectB);

/* Fills up a vector with zero values
 * "VectorToNull" will be a vector filled with zeroes
 * "Dim" defines the number of components
 */
void NullVect(double *VectorToNull, const int Dim);

/* Fills up a matrix with zero values
 * "MatrixToNull" will be a matrix filled with zeroes
 * "rows" and "cols" define the number of components in 2D
 */
void NullMatrix(double **MatrixToNull, const int rows, const int cols);

/* Multiplicates a vector with a scalar value (3D)
 * "OutputVector" will be "VectorToMultiplicate" Multiplicated by "Scalar"
 * "Dim" defines the number of components
 */
void MultiplicateWithScalar(double *OutputVector,
        double *VectorToMultiplicate, const double Scalar, const int Dim);

/* Returns the scalar product of two vectors
 * The return value will be the inner product of the vectors "A" and "B"
 * "Dim" defines the number of components
 */
double ScalarProduct(double *A, double *B, const int Dim);

/* Outer product of two vectors (3D)
 * "Result" will be the vectorial product of the vectors "Vector1" and "Vector2"
 * Only works fine in 3 dimensions
 */
void VectorialProduct(double *Result, double *Vector1, double *Vector2);

/* Creates a unit vector (parallel with the input vector)
 * "OutputVector" will be "InputVector" divided by the Length of "InputVector"
 * If the Length of "InputVector" is 0, then OutputVector will be filled with zeroes
 * Works fine in 3 dimensions
 */
void UnitVect(double *OutputVector, double *InputVector);

/* Same as UnitVect, but it doesn't give back a 1 long vector, but a value long one
*/

void NormalizeVector(double *OutputVector, double *InputVector, double value);

/* Rotates vector around Z axis
 * "OutputVector" will be the "InputVector" rotated with "Angle" around Z axis
 * "Angle" should be in radians
 */
void RotateVectXY(double *OutputVector, double *InputVector,
        const double Angle);

/* Rotates vector around X axis
 * "OutputVector" will be the "InputVector" rotated with "Angle" around x axis
 * "Angle" should be in radians
 */
void RotateVectZY(double *OutputVector, double *InputVector,
        const double Angle);

/* Rotates vector around Y axis
 * "OutputVector" will be the "InputVector" rotated with "Angle" around Y axis
 * "Angle" should be in radians
 */
void RotateVectZX(double *OutputVector, double *InputVector,
        const double Angle);

/* Rodrigues formula (for rotating vectors around a specific axis)
 * "OutputVector will be the "InputVector" rotated around a specific axis defined by "Axis"
 * angle determines the angle of the rotation.
 */
void RotateVectAroundSpecificAxis(double *OutputVector, double *InputVector,
        double *Axis, const double Angle);

/* Gives the angle of two 3D vectors in
*/
double AngleOfTwoVectors(double *V, double *W, int dim);
// Gives the distance of two points in 2D;
double DistanceOfTwoPoints2D(double *P1, double *P2);

/* Projects vectors onto a specific line
 * "LineDirectionVector" defines the orientation of the line
 */

void ProjectVectOntoLine(double *OutputVector, double *InputVector,
        double *LineDirectionVector);

/* Projects vectors onto a specific plane
 * "PlaneDirectionVector" is a normal vector which gives the direction perpendicular to the plane.
 */
void ProjectVectOntoPlane(double *OutputVector, double *InputVector,
        double *PlaneNormalVector);

/* Calculating distance between a point and a line
 * SegmentEndPoint1 and SegmentEndPoint2 are two vectors defining the line
 */
double DistanceFromLine(double *Point, double *SegmentEndpoint1,
        double *SegmentEndPoint2);
double DistanceFromLineXY(double *Point, double *SegmentEndPoint1,
        double *SegmentEndPoint2);

/* Checking that the position of "WhichPoint" is inside the "shadow" of x_1 and x_2 (in 2D)
 * Returns 1 if CW-outside, -1 if CW-inside and 0 if not in shadow at all.
 */
int AtShadow(double *x1, double *x2, double *WhichPoint);

/* Checking that the position of "WhichPoint" is inside a polygon given by the vertex-set "Polygon"
 */
bool IsInsidePolygon(double *WhichPoint, double *Polygon,
        const int NumberOfVertices);

/* Multiplicate two N-dimensional square matrices
 */
void MultiplicateSquareMatrices(double **Res, double **Mat1, double **Mat2,
        const int N);

/* Calculates a power of an N-dimensional square matrix
 */
void PowerFuncMatrix(double **Res, double **Mat, const int N,
        const int Exponent);

/* Other useful tools */

double ClampScalar(const double x, const double x_min, const double x_max);

/*
 * Sinusoidal sigmoid curve
 *
 * 1 ----
 *     g \
 *       r0---> x
 */
double Sigmoid(const double x, const double gamma, const double r0);

/*
 * Linear-gain thresholded sigmoid v(x) curve
 *
 * v_max ----
 *         p \
 *           r0---> x
 */
double SigmoidLin(const double x, const double p, const double v_max,
        const double r0);

/*
 * Square root - linear combined velocity decay curve
 * The final value never goes over the "reference" square root function determined by "r0" and "acc"
 *
 *  v(x)
 *  ^
 *  |         /---- v0
 *  |        / p,a
 *  |------r0-----------> x
 */
double VelDecayLinSqrt(const double x, const double p, const double acc,
        const double v_max, const double r0);

/*
 * The inverse of the VelDecayLinSqrt function
 */
double StoppingDistanceLinSqrt(double v, double a, double p);

/* Calculating optimal size of an arena for "NumberOfAgents" agents
 */
double RadiusOfWayPointAreaFromNumberOfAgents(const int NumberOfAgents,
        const double SizeOfAgent, const double Gamma);

/* Returns the tangent points ("TangentPoint1" and "TangentPoint2")
 * and number of target points (return value)
 * of a circle relative to "Point".
 *
 *     tangent line:         __  _ _
 *              _______------   /   \
 *     point: *                |    |
 *                              \__/
 *
 */
int TangentsOfCircle(double *TangentPoint1, double *TangentPoint2,
        double *Point, double *CentreOfCircle, const double Radius);
int TangentsOfSphereSlice(double *TangentPoint1, double *TangentPoint2,
        double *Point, double *CentreOfCircle,
        double *NormalVect, const double Radius);

/* Intersection of the line segments A and B.
 * Line segment A is given by its two endpoints (A1, A2)
 * Half-line B is given by its endpoint B and the vector of the the half-line
 * The coordinates of the intersection points will be in the "Intersection" dynamic array
 * If the return value is "-1", then the intersection point doesn't exist.
 */
// !!! TODO: What if a line segment contains the other line segment?
bool IntersectionOfLineSegmentAndHalfLine(double *Intersection, double *A1,
        double *A2, double *B, double *VB);
bool IntersectionOfLineSegments(double *Intersection, double *A1, double *A2,
        double *B1, double *B2);
double IntersectionOfLines2D(double *Intersection, double *RefPointA1,
        double *RefPointA2, double *RefPointB1, double *RefpointB2);
double IntersectionOfLines2D_Dir(double *Intersection, double *RefPointA,
        double *DirectionA, double *RefPointB, double *DirectionB);
double IntersectionOfLineSegmentAndLine2D(double *Intersection,
        double *RefPointOfLine1,
        double *RefPoinrOfLine2, double *EndPoint1, double *EndPoint2);

/* Gives back the coordinates of two points on a line that are exactly "radius" far from the origo */
/* Also returns the number of valid points, and two NULLs if no intersection */
/* The two output vectors come in the order: the further in the direction of "DirectionOfLine", and the other */
/* You don't have to create an empty vector for the intersection you don't care about, just simply give a NULL instead */

int PointsOnLineAtAGivenDistanceFromOrigo3D(double *intersection1,
        double *intersection2,
        double radius, double *PointOnLine, double *DirectionOfLine);

/* Gives back the coordinates of two points ("Point1" and "Point2") on a line segment (defined by "EndPoint1" and "EndPoint2")
 * that are exactly "Distance" cm far from a reference point ("RefPoint"). Returns the number of valid points (0, 1 or 2)
 * This function also can be interpreted as the intersection points of a sphere and a line
 */
int PointsOnLineSegmentAtAGivenDistance3D(double *Point1, double *Point2,
        double *EndPoint1,
        double *EndPoint2, double *RefPoint, const double Distance);

/* Calculates the closest points on two lines
 * For algo details see: http://geomalgorithms.com/a07-_distance.html#dist3D_Segment_to_Segment
 * Line A is P0 + s*u, line B is Q0 + tv, s and t are calculated as scalars for closest point.
 * return value is true if lines are parallel
 */
bool ClosestPointOfLines3D(double *P0, double *u, double *Q0, double *v,
        double *s, double *t);

/* Generates a random vector on a unit halph-sphere
 * Coordination of the halph-sphere is given by an axis ("Axis"), which is the normal vector of the base circle
 * Result vector will be placed into "OutputVector".
 */
void GenerateVectOnHalfSphere(double *OutputVector, double *Axis);

/* Is there any intersection between two polygons?
 * "Polygon1" and "Polygon2" are 2D arrays with NumberOfVertices(1/2) * 2 components.
 */
bool IntersectingPolygons(double **Polygon1, const int NumberOfVertices1,
        double **Polygon2, const int NumberOfVertices2);

/* Calculating the CoM of polygon points in 2D
 * "Polygon" is the set of vertices, it has "NumberOfVertices*2" components.
 * "CentrePoint" will contain the output centre of mass
 */
void CentreOfPolygon2D(double *CentrePoint, double *Polygon,
        const int NumberOfVertices);

/* Calculating the CoM (centroid) of a polygon in 2D
 * "Polygon" is the set of vertices, it has "NumberOfVertices*2" components.
 * "CentroidPoint" will contain the centroid point of the polygon
 */
void CentroidOfPolygon2D(double *CentroidPoint, double *Polygon,
        const int NumberOfVertices);

/* Calculating the intersection points of a polygon and a line.
 * Also returns the number of intersections.
 * Line is given by the reference points "RefPoint1" and "RefPoint2"
 */
void IntersectionOfLineAndPolygon2D(double **IntersectionPoints,
        int *NumberOfIntersections,
        double *RefPoint1, double *RefPoint2,
        double *Polygon, const int NumberOfVertices);

/* Creating envelope polygon around set of polygons (in 2D, XY coordinate system)
 * "EnvelopeSquareCoords" is a 4*2 matrix, contains the output coordinates of the envelope square
 * "Polygons" contains the coordinates of all polygons. It has "NumberOfPolygons" rows,
 * and "NumberOfVertices[i]*2" columns at the ith row (for XY coordinates placed).
 */
void CreateEnvelopeSquareAroundPolygons(double **EnvelopeSquareCoords,
        double **Polygons, int *NumberOfVertices, const int NumberOfPolygons);

/*
Point1 and Point2 are on the same side of ReferencePoint if:
        x-2                                   x-2
         |                                     |
x-1      |             and not if              |
    \_   |                                     x-Ref
      \_ |                                   _/
         x-Ref                              x-1
*/
bool TwoPointsOnSameSideOfAPoint(double *Point1, double *Point2,
        double *ReferencePoint);

#endif
