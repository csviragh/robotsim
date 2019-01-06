//MIT License
//Copyright (c) 2018 Eotvos Lorand University, Budapest

/* vim:set ts=4 sw=4 sts=4 et: */

/*
 * Useful mathematical tools (e. g. linear algebra tools, randomizers, etc.)
 */

#include <math.h>
#include "math_utils.h"
#include "datastructs.h"

#define MAX(a,b) ((a>b)?a:b)
#define MIN(a,b) ((a<b)?a:b)

/* Randomizer functions */

/* Returns a random double between minValue and maxValue (uniform distribution) */
double randomizeDouble(const double MinValue, const double MaxValue) {

    if (MinValue > MaxValue) {

        fprintf(stderr,
                "The maximal value has to be larger than the minimal value... \n");
        exit(-3);

    }

    return MinValue + (double) rand() * (MaxValue - MinValue) / RAND_MAX;

}

/* Returns a random double value (Gaussian distribution with given Mean and StdDev) */
double randomizeGaussDouble(const double Mean, const double StdDev) {

    if (StdDev < 0) {

        fprintf(stderr, "StDev has to be larger than 0...");
        exit(-3);

    }

    /* Box-Muller transformation */
    static double uniRand;
    static double uniRand2;
    static double Theta;
    static double Rho;

    do {
        uniRand = randomizeDouble(0, 1);
    }
    while (uniRand == 1.0);

    uniRand2 = randomizeDouble(0, 1);
    Theta = 2 * M_PI * uniRand2;
    Rho = sqrt(-2 * log(1 - uniRand));

    return Mean + StdDev * Rho * cos(Theta);

}

/* Returns a random double value (Power-law distribution with given exponent in a given range) */
double randomizePowerLawDouble(const double x0, const double x1, const double n) {

    if (n == -1) {

        printf("Exponent of power-law distribution cannot be -1!\n");
        exit(-3);

    }

    static double uniRand;
    uniRand = randomizeDouble(0, 1);
    return pow((pow(x1, n + 1) - pow(x0, n + 1)) * uniRand + pow(x0, n + 1),
            1 / (1 + n));

}

// integer power
int ipow(int base, int exp) {
    int result = 1;
    while (exp) {
        if (exp & 1)
            result *= base;
        exp >>= 1;
        base *= base;
    }

    return result;
}

/* Linear algebra */

/* Fills a vector with the input values (x, y, z components) */
void FillVect(double *VectToFill, const double x, const double y,
        const double z) {

    VectToFill[0] = x;
    VectToFill[1] = y;
    VectToFill[2] = z;

}

/* Returns the Length of the input vector (3 dimensions) */
double VectAbs(double *InputVector) {
    return sqrt(InputVector[0] * InputVector[0] +
            InputVector[1] * InputVector[1] + InputVector[2] * InputVector[2]);
}

/* Returns the XY-projected Length of the input vector */
double VectAbsXY(double *InputVector) {

    return hypot(InputVector[0], InputVector[1]);

}

/* Difference of two vectors (3D) */
void VectDifference(double *ABDiff, double *VectA, double *VectB) {

    FillVect(ABDiff, VectA[0] - VectB[0], VectA[1] - VectB[1],
            VectA[2] - VectB[2]);

}

void VectDifference2D(double *ABDiff, double *VectA, double *VectB) {
    ABDiff[0] = VectA[0] - VectB[0];
    ABDiff[1] = VectA[1] - VectB[1];
}

/* Sum of two vectors (3D) */
void VectSum(double *ABSum, double *VectA, double *VectB) {

    FillVect(ABSum, VectA[0] + VectB[0], VectA[1] + VectB[1],
            VectA[2] + VectB[2]);

}

/* Fills up a vector with zero values */
void NullVect(double *VectorToNull, const int Dim) {

    memset(VectorToNull, 0, Dim * sizeof(double));

}

void NullMatrix(double **MatrixToNull, const int rows, const int cols) {
    int i;
    for (i = 0; i < rows; i++) {
        memset(MatrixToNull[i], 0, cols * sizeof(double));
    }
}

/* Multiplicates a vector with a scalar value (3D) */
void MultiplicateWithScalar(double *OutputVector, double *VectorToMultiplicate,
        const double Scalar, const int Dim) {

    int i;

    for (i = 0; i < Dim; i++) {

        OutputVector[i] = Scalar * VectorToMultiplicate[i];

    }

}

/* Returns the scalar product of two vectors */
double ScalarProduct(double *VectA, double *VectB, const int Dim) {

    int i;
    static double Product;
    Product = 0.0;

    for (i = 0; i < Dim; i++) {

        Product += VectA[i] * VectB[i];

    }

    return Product;

}

/* Outer product of two vectors (3D) */
void VectorialProduct(double *Result, double *Vector1, double *Vector2) {

    static double Temp1[3];
    static double Temp2[3];

    memcpy(Temp1, Vector1, 3 * sizeof(double));
    memcpy(Temp2, Vector2, 3 * sizeof(double));

    Result[0] = Temp1[1] * Temp2[2] - Temp1[2] * Temp2[1];
    Result[1] = Temp1[2] * Temp2[0] - Temp1[0] * Temp2[2];
    Result[2] = Temp1[0] * Temp2[1] - Temp1[1] * Temp2[0];

}

/* Creates a unit vector (parallel to the input vector) */
void UnitVect(double *OutputVector, double *InputVector) {

    int k;
    static double Abs;
    Abs = VectAbs(InputVector);

    /* If input vector is (0, 0, 0), the "unit" vect will be (0, 0, 0) */
    if (Abs < 0.0000000001) {

        NullVect(OutputVector, 3);

    } else {

        for (k = 0; k < 3; k++) {

            OutputVector[k] = InputVector[k] / Abs;

        }

    }

}

// normalize a vector to a given value
void NormalizeVector(double *OutputVector, double *InputVector, double value) {
    UnitVect(OutputVector, InputVector);
    MultiplicateWithScalar(OutputVector, OutputVector, value, 3);
    return;
}

/* Rotates vector around Z axis */
void RotateVectXY(double *OutputVector, double *InputVector, const double Angle) {

    static double Temp[3];
    FillVect(Temp, InputVector[0], InputVector[1], InputVector[2]);

    static double cosAngle;
    static double sinAngle;
    cosAngle = cos(Angle);
    sinAngle = sin(Angle);

    OutputVector[0] = Temp[0] * cosAngle - Temp[1] * sinAngle;
    OutputVector[1] = Temp[0] * sinAngle + Temp[1] * cosAngle;
    OutputVector[2] = Temp[2];

}

/* Rotates vector around X axis */
void RotateVectZY(double *OutputVector, double *InputVector, const double Angle) {

    static double Temp[3];
    FillVect(Temp, InputVector[0], InputVector[1], InputVector[2]);

    static double cosAngle;
    static double sinAngle;
    cosAngle = cos(Angle);
    sinAngle = sin(Angle);

    OutputVector[0] = Temp[0];
    OutputVector[1] = Temp[1] * cosAngle - Temp[2] * sinAngle;
    OutputVector[2] = Temp[1] * sinAngle + Temp[2] * cosAngle;

}

/* Rotates vector around Y axis */
void RotateVectZX(double *OutputVector, double *InputVector, const double Angle) {

    static double Temp[3];
    FillVect(Temp, InputVector[0], InputVector[1], InputVector[2]);

    static double cosAngle;
    static double sinAngle;
    cosAngle = cos(Angle);
    sinAngle = sin(Angle);

    OutputVector[0] = Temp[0] * cosAngle - Temp[2] * sinAngle;
    OutputVector[1] = Temp[1];
    OutputVector[2] = Temp[0] * sinAngle + Temp[2] * cosAngle;

}

/* Rodrigues formula (for rotating vectors around a specific axis) */
/* See http://aries.ektf.hu/~hz/pdf-tamop/pdf-01/html/ch05.html#id677681 (in Hungarian) */
void RotateVectAroundSpecificAxis(double *OutputVector, double *InputVector,
        double *Axis, const double Angle) {

    static double Temp[3];
    FillVect(Temp, InputVector[0], InputVector[1], InputVector[2]);

    static double cosAngle;
    static double sinAngle;
    static double AxisUnitVect[3];
    UnitVect(AxisUnitVect, Axis);

    sinAngle = sin(Angle);
    cosAngle = cos(Angle);

    OutputVector[0] =
            (cosAngle +
            AxisUnitVect[0] * AxisUnitVect[0] * (1 - cosAngle)) * Temp[0] +
            (AxisUnitVect[0] * AxisUnitVect[1] * (1 - cosAngle) -
            AxisUnitVect[2] * sinAngle) * Temp[1] +
            (AxisUnitVect[0] * AxisUnitVect[2] * (1 - cosAngle) +
            AxisUnitVect[1] * sinAngle) * Temp[2];
    OutputVector[1] =
            (AxisUnitVect[0] * AxisUnitVect[1] * (1 - cosAngle) +
            AxisUnitVect[2] * sinAngle) * Temp[0] + (cosAngle +
            AxisUnitVect[1] *
            AxisUnitVect[1] * (1 -
                    cosAngle)) *
            Temp[1] + (AxisUnitVect[1] * AxisUnitVect[2] * (1 - cosAngle) -
            AxisUnitVect[0] * sinAngle) * Temp[2];
    OutputVector[2] =
            (AxisUnitVect[2] * AxisUnitVect[0] * (1 - cosAngle) -
            AxisUnitVect[1] * sinAngle) * Temp[0] +
            (AxisUnitVect[2] * AxisUnitVect[1] * (1 - cosAngle) +
            AxisUnitVect[0] * sinAngle) * Temp[1] + (cosAngle +
            AxisUnitVect[2] * AxisUnitVect[2] * (1 - cosAngle)) * Temp[2];

}

/* Gives the angle of two vectors in radian*/
double AngleOfTwoVectors(double *V, double *W, int dim) {
    double V_1[3];
    UnitVect(V_1, V);
    double W_1[3];
    UnitVect(W_1, W);
    return acos(ScalarProduct(V_1, W_1, dim));
}

double DistanceOfTwoPoints2D(double *P1, double *P2) {
    double DifferenceVector[3];
    VectDifference(DifferenceVector, P1, P2);
    return VectAbsXY(DifferenceVector);
}

/* Projects vector orthogonally onto a specific line in 3D */
void ProjectVectOntoLine(double *OutputVector, double *InputVector,
        double *LineDirectionVector) {

    /* "u" is the unit vector of the line,
     * multiplication with P = u * u^T is a projection onto that line
     */

    static double Temp[3];
    FillVect(Temp, InputVector[0], InputVector[1], InputVector[2]);

    static double u[3];
    UnitVect(u, LineDirectionVector);

    OutputVector[0] =
            u[0] * u[0] * Temp[0] + u[0] * u[1] * Temp[1] +
            u[0] * u[2] * Temp[2];
    OutputVector[1] =
            u[1] * u[0] * Temp[0] + u[1] * u[1] * Temp[1] +
            u[1] * u[2] * Temp[2];
    OutputVector[2] =
            u[2] * u[0] * Temp[0] + u[2] * u[1] * Temp[1] +
            u[2] * u[2] * Temp[2];
}

/* Projects vectors into a specific plane in 3D */
void ProjectVectOntoPlane(double *OutputVector, double *InputVector,
        double *PlaneNormalVector) {

    static double Temp[3];
    NullVect(Temp, 3);
    ProjectVectOntoLine(Temp, InputVector, PlaneNormalVector);
    VectDifference(OutputVector, InputVector, Temp);
}

/* Distance from line in 3D */
double DistanceFromLine(double *Point, double *SegmentEndPoint1,
        double *SegmentEndPoint2) {

    static double Temp1[3];
    VectDifference(Temp1, SegmentEndPoint1, SegmentEndPoint2);
    UnitVect(Temp1, Temp1);

    static double Temp2[3];
    VectDifference(Temp2, SegmentEndPoint1, Point);

    static double Temp3[3];
    MultiplicateWithScalar(Temp3, Temp1, ScalarProduct(Temp2, Temp1, 3), 3);
    VectDifference(Temp3, Temp2, Temp3);

    return (VectAbs(Temp3));

}

/* Distance from line on the xy plane */
double DistanceFromLineXY(double *Point, double *SegmentEndPoint1,
        double *SegmentEndPoint2) {

    static double Temp1[3];
    VectDifference(Temp1, SegmentEndPoint1, SegmentEndPoint2);
    Temp1[2] = 0.0;
    UnitVect(Temp1, Temp1);

    static double Temp2[3];
    VectDifference(Temp2, SegmentEndPoint1, Point);
    Temp2[2] = 0.0;

    static double Temp3[3];
    MultiplicateWithScalar(Temp3, Temp1, ScalarProduct(Temp2, Temp1, 3), 3);
    VectDifference(Temp3, Temp2, Temp3);

    return (VectAbsXY(Temp3));

}

/* Checking that the position of "WhichPoint" is inside the "shadow" of x_1 and x_2 (in 2d) */
/*
       |           |
       |           |
       x1----------x2
       |           |
       |           |
 not   |  shadow   |  not
shadow |           | shadow

*/
int AtShadow(double *x1, double *x2, double *WhichPoint) {

    static double Temp1[3];
    static double Temp2[3];
    static double Temp3[3];
    static double Temp4[3];

    VectDifference(Temp1, WhichPoint, x1);
    VectDifference(Temp2, WhichPoint, x2);
    VectDifference(Temp3, x1, x2);
    VectDifference(Temp4, x2, x1);
    // we are working in 2D only
    Temp1[2] = Temp2[2] = Temp3[2] = Temp4[2] = 0;

    // in shadow
    if ((ScalarProduct(Temp1, Temp4, 2) >= 0)
            && (ScalarProduct(Temp2, Temp3, 2) >= 0)) {
        UnitVect(Temp4, Temp4);
        UnitVect(Temp1, Temp1);
        VectorialProduct(Temp2, Temp4, Temp1);
        // CW-outside
        if (Temp2[2] >= 0)
            return 1;
        // CW-inside
        else
            return -1;
    }
    // Not in shadow
    return 0;

}

/* Checking that the position of "WhichPoint" is inside a polygon given by the vertex-set "Polygon" */
bool IsInsidePolygon(double *WhichPoint, double *Polygon,
        const int NumberOfVertices) {

    if (NumberOfVertices < 3)
        return false;

    int i;

    static bool Inside;
    Inside = false;

    static double p1x;
    p1x = Polygon[0];
    static double p1y;
    p1y = Polygon[1];

    static double p2x;
    static double p2y;

    static double max;
    static double min;

    static double xints;
    xints = 0.0;

    for (i = 0; i <= NumberOfVertices; i++) {

        p2x = Polygon[(2 * i) % (2 * NumberOfVertices)];
        p2y = Polygon[(2 * i + 1) % (2 * NumberOfVertices)];

        if (p2y > p1y) {
            max = p2y;
            min = p1y;
        } else {
            max = p1y;
            min = p2y;
        }
        if (WhichPoint[1] > min) {
            if (WhichPoint[1] <= max) {
                if (p2x > p1x) {
                    max = p2x;
                    min = p1x;
                } else {
                    max = p1x;
                    min = p2x;
                }
                if (WhichPoint[0] <= max) {
                    if (p1y != p2y) {
                        xints = (WhichPoint[1] - p1y) * (p2x - p1x) / (p2y -
                                p1y) + p1x;
                    }
                    if (p1x == p2x || WhichPoint[0] <= xints) {
                        Inside = !Inside;
                    }
                }
            }
        }

        p1x = p2x;
        p1y = p2y;

    }

    return Inside;
}

/* Multiplicate two N-dimensional square matrices */
void MultiplicateSquareMatrices(double **Res, double **Mat1, double **Mat2,
        const int N) {

    int c, d, k;
    static double sum;
    sum = 0.0;

    double **Temp;
    Temp = doubleMatrix(N, N);

    for (c = 0; c < N; c++) {
        for (d = 0; d < N; d++) {
            for (k = 0; k < N; k++) {
                sum = sum + Mat1[c][k] * Mat2[k][d];
            }

            Temp[c][d] = sum;
            sum = 0;
        }
    }

    for (c = 0; c < N; c++) {
        for (d = 0; d < N; d++) {
            Res[c][d] = Temp[c][d];
        }
    }

    freeMatrix(Temp, N, N);

}

/* Calculates a power of an N-dimensional square matrix
 */
void PowerFuncMatrix(double **Res, double **Mat, const int N,
        const int Exponent) {

    int i, j;

    for (i = 0; i < N; i++) {
        for (j = 0; j < N; j++) {
            Res[i][j] = Mat[i][j];
        }
    }

    for (i = 0; i < Exponent - 1; i++) {

        MultiplicateSquareMatrices(Res, Res, Mat, N);

    }

}

/* Other useful tools */

double ClampScalar(const double x, const double x_min, const double x_max) {
    if (x <= x_min)
        return x_min;
    if (x >= x_max)
        return x_max;
    return x;
}

/* Sinusoidal sigmoid curve */
double Sigmoid(const double x, const double gamma, const double r0) {
    if (x > r0) {
        return 0.0;
    } else if (x > r0 - gamma) {
        return 0.5 * (sin((M_PI * pow(gamma, -1.0)) * (x - r0) - M_PI_2) + 1.0);
    } else {
        return 1.0;
    }
}

/* linear "sigmoid" v(x) curve */
double SigmoidLin(const double x, const double p, const double v_max,
        const double r0) {
    static double vel;
    vel = (r0 - x) * p;
    if (p <= 0 || vel <= 0)
        return 0;
    if (vel >= v_max)
        return v_max;
    return vel;
}

/* Square root - linear combined vel decay (-Sigmoid) curve */
double VelDecayLinSqrt(const double x, const double p, const double acc,
        const double v_max, const double r0) {
    //v_max could be named v_diff
    static double vel;
    /* linear v(x) phase */
    vel = (x - r0) * p;
    if (acc <= 0 || p <= 0 || vel <= 0)
        return 0;
    if (vel < acc / p) {
        if (vel >= v_max)
            return v_max;
        return vel;
    }
    /* const acceleration, i.e. sqrt v(x) phase */
    vel = sqrt(2 * acc * (x - r0) - acc * acc / p / p);
    if (vel >= v_max)
        return v_max;
    return vel;
}

double StoppingDistanceLinSqrt(double v, double a, double p) {
    if (v < a / p)
        return v / p;
    return (v * v / a + a / p / p) / 2;
}

/* Calculating size of an arena for "NumberOfAgents" agents (up to N = 20) */
double RadiusOfWayPointAreaFromNumberOfAgents(const int NumberOfAgents,
        const double SizeOfAgent, const double Gamma) {

    static const double Cinc[] = {
        1,                      /* 0 */
        1,                      /* 1 */
        2,                      /* 2 */
        2.1547005383792515290182975610, /* 3 */
        2.4142135623730950488016887242, /* 4 */
        2.7013016167040798643630809941, /* 5 */
        3,                      /* 6 */
        3,                      /* 7 */
        3.3047648709624865052411502235, /* 8 */
        3.6131259297527530557132863469, /* 9 */
        3.8130256313981243982516251560, /* 10 */
        3.9238044001630872522327544134, /* 11 */
        4.0296019301161834974827410413, /* 12 */
        4.2360679774997896964091736687, /* 13 */
        4.3284285548608366814039093675, /* 14 */
        4.5213569647061642409073640084, /* 15 */
        4.6154255948731939855392441620, /* 16 */
        4.7920337483105791701491239533, /* 17 */
        4.837033051562731469989727989,  /* 18 */
        4.8637033051562731469989727989, /* 19 */
        5.1223207369915283214476857922  /* 20 */
    };

    if (NumberOfAgents < 20 && SizeOfAgent >= 0) {

        return ((SizeOfAgent / 2)) * Cinc[NumberOfAgents + 1] -
                SizeOfAgent / 2 + Gamma;

    } else {

        return 100000000;

    }

}

/* Returns the tangent points and number of tangent points of a circle relative to "Point". */
int TangentsOfCircle(double *TangentPoint1, double *TangentPoint2,
        double *Point, double *CentreOfCircle, const double Radius) {

    static double TempVect[3];
    VectDifference(TempVect, CentreOfCircle, Point);
    static double d;
    d = VectAbsXY(TempVect);

    if (d < Radius) {           // What if "Point" is inside the circle?
        return 0;
    } else if (d > Radius) {
        UnitVect(TempVect, TempVect);
        static double h;
        h = sqrt(d * d - Radius * Radius);
        MultiplicateWithScalar(TempVect, TempVect, h, 2);

        static double angle;
        angle = asin(Radius / d);

        RotateVectXY(TangentPoint1, TempVect, angle);
        RotateVectXY(TangentPoint2, TempVect, -angle);
        VectSum(TangentPoint1, TangentPoint1, Point);
        VectSum(TangentPoint2, TangentPoint2, Point);
        return 2;
    } else {                    // What if "Point" is on the circle?
        FillVect(TangentPoint1, Point[0], Point[1], 0.0);
        return 1;
    }

}

int TangentsOfSphereSlice(double *TangentPoint1, double *TangentPoint2,
        double *Point, double *CentreOfCircle,
        double *NormalVect, const double Radius) {

    /* Check if Point is far enough from the centre */

    static double FromCentre[3];
    VectDifference(FromCentre, CentreOfCircle, Point);

    /* NormalVect should be perpendicular with "Centre" minus "Point" */
    if (fabs(ScalarProduct(FromCentre, NormalVect, 3)) > 0.0000000001) {
        return 0;

    }

    static double d;
    d = VectAbs(FromCentre);

    if (d > Radius) {

        UnitVect(FromCentre, FromCentre);
        static double YAxis[3];
        VectorialProduct(YAxis, FromCentre, NormalVect);

        // Now FromCentre, YAxis and NormalVect give us a Descartes system

        static double h;
        h = sqrt(d * d - Radius * Radius);
        MultiplicateWithScalar(FromCentre, FromCentre, h, 2);

        static double angle;
        angle = asin(Radius / d);

        RotateVectAroundSpecificAxis(TangentPoint1, FromCentre, NormalVect,
                angle);
        RotateVectAroundSpecificAxis(TangentPoint2, FromCentre, NormalVect,
                -angle);
        VectSum(TangentPoint1, TangentPoint1, Point);
        VectSum(TangentPoint2, TangentPoint2, Point);

        return 2;

    } else if (Radius == d) {   // What if "Point" is on the slice?
        FillVect(TangentPoint1, Point[0], Point[1], 0.0);
        return 1;
    } else {
        return 0;
    }
}

/* Intersection of a line segment and a half-line */
bool IntersectionOfLineSegmentAndHalfLine(double *Intersection, double *A1,
        double *A2, double *B, double *VB) {

    static double SA[3];
    VectDifference(SA, A2, A1);

    static double t;
    t = (SA[0] * VB[1] - SA[1] * VB[0]);

    if (fabs(t) < 0.0000000001) {

        return false;

    }

    static double s;
    s = (-SA[1] * (A1[0] - B[0]) + SA[0] * (A1[1] - B[1])) / t;
    t = (VB[0] * (A1[1] - B[1]) - VB[1] * (A1[0] - B[0])) / t;

    if (s >= 0 && t >= 0 && t <= 1) {

        FillVect(Intersection, A1[0] + t * SA[0], A1[1] + t * SA[1], 0);
        return true;

    }

    return false;

}

/* Intersection of two line segments */
bool IntersectionOfLineSegments(double *Intersection, double *A1, double *A2,
        double *B1, double *B2) {
    static double SA[3];
    VectDifference(SA, A2, A1);
    static double SB[3];
    VectDifference(SB, B2, B1);

    static double t;
    t = (SA[0] * SB[1] - SA[1] * SB[0]);

    if (fabs(t) < 0.0000000001) {

        return false;

    }

    static double s;
    s = (-SA[1] * (A1[0] - B1[0]) + SA[0] * (A1[1] - B1[1])) / t;
    t = (SB[0] * (A1[1] - B1[1]) - SB[1] * (A1[0] - B1[0])) / t;

    if (s >= 0 && t >= 0 && t <= 1 && s <= 1) {

        FillVect(Intersection, A1[0] + t * SA[0], A1[1] + t * SA[1], 0);
        return true;

    }

    return false;

}

/* Intersection of two lines (2D) */
double IntersectionOfLines2D(double *Intersection, double *RefPointA1,
        double *RefPointA2, double *RefPointB1, double *RefPointB2) {

    static double det;          // If det = 0, the lines are parallel

    det = (RefPointA1[0] - RefPointA2[0]) * (RefPointB1[1] - RefPointB2[1]) -
            (RefPointA1[1] - RefPointA2[1]) * (RefPointB1[0] - RefPointB2[0]);

    if (fabs(det) < 0.0000000001) {
        return -1.0;
    } else {
        static double temp_coeff1, temp_coeff2;
        temp_coeff1 =
                RefPointA1[0] * RefPointA2[1] - RefPointA1[1] * RefPointA2[0];
        temp_coeff2 =
                RefPointB1[0] * RefPointB2[1] - RefPointB1[1] * RefPointB2[0];
        Intersection[0] =
                ((RefPointB1[0] - RefPointB2[0]) * temp_coeff1 -
                (RefPointA1[0] - RefPointA2[0]) * temp_coeff2) / det;
        Intersection[0] =
                ((RefPointB1[1] - RefPointB2[1]) * temp_coeff1 -
                (RefPointA1[1] - RefPointA2[1]) * temp_coeff2) / det;
        return 0.0;
    }

}

double IntersectionOfLines2D_Dir(double *Intersection, double *RefPointA,
        double *DirectionA, double *RefPointB, double *DirectionB) {

    double OtherPointOnA[3], OtherPointOnB[3];
    VectSum(OtherPointOnA, RefPointA, DirectionA);
    VectSum(OtherPointOnB, RefPointB, DirectionB);
    return IntersectionOfLines2D(Intersection, RefPointA, OtherPointOnA,
            RefPointB, OtherPointOnB);
}

/* Intersection of a line segment and a line (2D) */
double IntersectionOfLineSegmentAndLine2D(double *Intersection,
        double *RefPointOfLine1,
        double *RefPointOfLine2, double *EndPoint1, double *EndPoint2) {

    if (-1.0 ==
            IntersectionOfLines2D(Intersection, RefPointOfLine1,
                    RefPointOfLine2, EndPoint1, EndPoint2)) {
        return -1.0;
    } else {
        /* If the intersection point is between the endpoints, then it's a valid int. point */
        if ((fabs(EndPoint1[0] - EndPoint2[0]) >
                        fabs(Intersection[0] - EndPoint2[0]))
                && (fabs(EndPoint1[1] - EndPoint2[1]) >
                        fabs(Intersection[1] - EndPoint2[1]))) {
            return 0.0;
            /* Otherwise, intersection point have to be reseted to NULL */
        } else {
            return -1.0;
        }
    }
}

/* Gives back the coordinates of two points on a line that are exactly "radius" far from the origo */
/* Also returns the number of valid points, and two NULLs if no intersection */
/* The two output vectors come in the order: the further in the direction of "DirectionOfLine", and the other */
/* You don't have to create an empty vector for the intersection you don't care about, just simply give a NULL instead */

int PointsOnLineAtAGivenDistanceFromOrigo3D(double *intersection1,
        double *intersection2, double radius,
        double *PointOnLine, double *DirectionOfLine) {

    double n_dir[3];
    UnitVect(n_dir, DirectionOfLine);
    double b, c, lambda1, lambda2;
    b = 2 * ScalarProduct(PointOnLine, n_dir, 3);
    c = VectAbs(PointOnLine) * VectAbs(PointOnLine) - radius * radius;
    if (b * b - 4 * c < 0) {
        return 0;
    }
    // just calculate the intersection the user is intrested in
    if (intersection1 != NULL) {
        lambda1 = (-b + sqrt(b * b - 4 * c)) / 2;
        MultiplicateWithScalar(intersection1, n_dir, lambda1, 3);
        VectSum(intersection1, intersection1, PointOnLine);
    }
    if (intersection2 != NULL) {
        lambda2 = (-b - sqrt(b * b - 4 * c)) / 2;
        MultiplicateWithScalar(intersection2, n_dir, lambda2, 3);
        VectSum(intersection2, intersection2, PointOnLine);
    }
    if (b * b - 4 * c == 0) {
        // in this case the two output vectors are the same if both were asked
        return 1;
    } else {
        return 2;
    }
}

/* Gives back the coordinates of two points on a line segment that are exactly "Distance" cm far from a reference point */
/* Also returns the number of valid points */
int PointsOnLineSegmentAtAGivenDistance3D(double *Point1, double *Point2,
        double *EndPoint1, double *EndPoint2,
        double *RefPoint, const double Distance) {

    /* Calculating direction of line */
    static double DirectionOfLine[3];
    VectDifference(DirectionOfLine, EndPoint2, EndPoint1);
    UnitVect(DirectionOfLine, DirectionOfLine);

    static double Temp[3];
    VectDifference(Temp, RefPoint, EndPoint1);

    static double ProjDistOfEndPointFromRef;
    ProjDistOfEndPointFromRef = ScalarProduct(DirectionOfLine, Temp, 3);

    static double Temp2[3];
    static double Temp3[3];
    MultiplicateWithScalar(Temp2, DirectionOfLine, ProjDistOfEndPointFromRef,
            3);
    VectSum(Temp2, Temp2, EndPoint1);
    VectDifference(Temp3, Temp2, RefPoint);

    static double DistanceFromLine;
    DistanceFromLine = VectAbs(Temp3);

    if (DistanceFromLine > Distance) {
        /* There is no intersection point */
        return 0;
    } else if (DistanceFromLine == Distance) {
        /* Distance of "RefPoint" and the line is "Distance" */
        FillVect(Point1, Temp2[0], Temp2[1], Temp2[2]);
        return 1;
    } else {
        /* There are two intersection points */
        static double AdditiveTerm;
        AdditiveTerm =
                sqrt(Distance * Distance - DistanceFromLine * DistanceFromLine);

        MultiplicateWithScalar(Point1, DirectionOfLine,
                ProjDistOfEndPointFromRef + AdditiveTerm, 3);
        MultiplicateWithScalar(Point2, DirectionOfLine,
                ProjDistOfEndPointFromRef - AdditiveTerm, 3);
        VectSum(Point1, Point1, EndPoint1);
        VectSum(Point2, Point2, EndPoint1);
        return 2;
    }
}

bool ClosestPointOfLines3D(double *P0, double *u, double *Q0, double *v,
        double *s, double *t) {
    double w0[3];
    double a, b, c, d, e;
    bool is_parallel = false;
    // initialize variables
    VectDifference(w0, P0, Q0);
    a = ScalarProduct(u, u, 3);
    b = ScalarProduct(u, v, 3);
    c = ScalarProduct(v, v, 3);
    d = ScalarProduct(u, w0, 3);
    e = ScalarProduct(v, w0, 3);
    // get denominator
    (*s) = a * c - b * b;
    // if lines are parallel
    if ((*s) == 0) {
        (*t) = d / b;
        is_parallel = true;
        // if lines are not parallel
    } else {
        (*t) = (a * e - b * d) / (*s);
        (*s) = (b * e - c * d) / (*s);
    }
    return is_parallel;
}

/* Generates a random vector on a unit halph-sphere
 * Coordination of the halph-sphere is given by an axis, which is the normal vector of the base circle */
void GenerateVectOnHalfSphere(double *OutputVector, double *Axis) {

    //Setting up a random vector on a sphere.
    UnitVect(Axis, Axis);
    static double Temp[3];
    NullVect(Temp, 3);
    Temp[0] = 1.0;
    RotateVectZX(Temp, Temp, randomizeDouble(-M_PI, M_PI));
    RotateVectXY(Temp, Temp, randomizeDouble(-M_PI, M_PI));

    //Setting up its reflection, if it is necessary
    static double Product;
    Product = ScalarProduct(Axis, Temp, 3);
    if (Product < 0.0) {
        static double MirroredTemp[3];
        NullVect(MirroredTemp, 3);
        VectSum(MirroredTemp, MirroredTemp, Axis);
        MultiplicateWithScalar(MirroredTemp, MirroredTemp, 2 * Product, 3);
        VectDifference(Temp, Temp, MirroredTemp);
    }

    NullVect(OutputVector, 3);
    VectSum(OutputVector, Temp, OutputVector);

}

// SANDBOX SANDBOX SANDBOX //

/* Tools for cell decomposition (these can be useful for path planning) */

/* Is there any intersection of two polygons? */
bool IntersectingPolygons(double **Polygon1, const int NumberOfVertices1,
        double **Polygon2, const int NumberOfVertices2) {

    int i, j;
    static double TempIntersection[3];

    for (i = 0; i < NumberOfVertices1; i++) {

        for (j = 0; j < NumberOfVertices2; j++) {

            if (IntersectionOfLineSegments(TempIntersection,
                            Polygon1[i],
                            Polygon1[(i + 1) % NumberOfVertices1],
                            Polygon2[j],
                            Polygon2[(j + 1) % NumberOfVertices2])) {

                return true;

            }

        }

    }

    return false;

}

/* Calculating the CoM of polygon points in 2D */
void CentreOfPolygon2D(double *CentrePoint, double *Polygon,
        const int NumberOfVertices) {

    NullVect(CentrePoint, 3);

    int i;
    int n = 0;

    for (i = 0; i < NumberOfVertices * 2; i += 2) {

        CentrePoint[0] += Polygon[i];
        CentrePoint[1] += Polygon[i + 1];

        n++;

    }

    MultiplicateWithScalar(CentrePoint, CentrePoint, 1.0 / n, 2);

}

void CentroidOfPolygon2D(double *CentroidPoint, double *Polygon,
        const int NumberOfVertices) {
    CentroidPoint[0] = 0;
    CentroidPoint[1] = 0;
    double signedArea = 0.0;
    double x0 = 0.0;            // Current vertex X
    double y0 = 0.0;            // Current vertex Y
    double x1 = 0.0;            // Next vertex X
    double y1 = 0.0;            // Next vertex Y
    double a = 0.0;             // Partial signed area

    // For all vertices except last
    int i = 0;
    for (i = 0; i < (NumberOfVertices - 1) * 2; i += 2) {
        x0 = Polygon[i];
        y0 = Polygon[i + 1];
        x1 = Polygon[i + 2];
        y1 = Polygon[i + 3];
        a = x0 * y1 - x1 * y0;
        signedArea += a;
        CentroidPoint[0] += (x0 + x1) * a;
        CentroidPoint[1] += (y0 + y1) * a;
    }
    // Do last vertex
    x0 = Polygon[i];
    y0 = Polygon[i + 1];
    x1 = Polygon[0];
    y1 = Polygon[1];
    a = x0 * y1 - x1 * y0;
    signedArea += a;
    CentroidPoint[0] += (x0 + x1) * a;
    CentroidPoint[1] += (y0 + y1) * a;
    // norm with 6 * area
    signedArea *= 0.5;
    CentroidPoint[0] /= (6.0 * signedArea);
    CentroidPoint[1] /= (6.0 * signedArea);
}

/* Returns the intersection points of a polygon and a line */
void IntersectionOfLineAndPolygon2D(double **IntersectionPoints,
        int *NumberOfIntersections, double *RefPoint1,
        double *RefPoint2, double *Polygon, const int NumberOfVertices) {

    int i;

    static double TempVect1[3];
    static double TempVect2[3];
    static double TempIntersect[3];

    for (i = 0; i < NumberOfVertices * 2; i += 2) {

        FillVect(TempVect1, Polygon[i], Polygon[i + 1], 0.0);
        FillVect(TempVect2, Polygon[(i + 2) % (2 * NumberOfVertices)],
                Polygon[(i + 4) % (2 * NumberOfVertices)], 0.0);
        // Still not complete

    }

}

/* Creating envelope polygon around set of polygons (in 2D, XY coordinate system) */
/* The envelope square contains all of the polygons */
void CreateEnvelopeSquareAroundPolygons(double **EnvelopeSquareCoords,
        double **Polygons, int *NumberOfVertices, const int NumberOfPolygons) {

    /* Searching for largest and smallest XY coordinates */
    int i, j;
    static double smallestx;
    smallestx = 2e222;
    static double largestx;
    largestx = -2e222;
    static double smallesty;
    smallesty = 2e222;
    static double largesty;
    largesty = -2e222;

    for (i = 0; i < NumberOfPolygons; i++) {

        for (j = 0; j < NumberOfVertices[i] * 2; j += 2) {

            if (Polygons[i][j] < smallestx) {
                smallestx = Polygons[i][j];
            }
            if (Polygons[i][j] > largestx) {
                largestx = Polygons[i][j];
            }

            if (Polygons[i][j + 1] < smallesty) {
                smallesty = Polygons[i][j];
            }
            if (Polygons[i][j + 1] > largesty) {
                largesty = Polygons[i][j];
            }

        }

    }

    /* Craeting envelope square */// Cutting trees, sorry!
    static double plussize_x;
    plussize_x = fabs(largestx - smallestx) * 0.5;
    static double plussize_y;
    plussize_y = fabs(largesty - smallesty) * 0.5;

    EnvelopeSquareCoords[0][0] = smallestx - plussize_x;
    EnvelopeSquareCoords[0][1] = smallesty - plussize_y;

    EnvelopeSquareCoords[1][0] = largestx + plussize_x;
    EnvelopeSquareCoords[1][1] = smallesty - plussize_y;

    EnvelopeSquareCoords[2][0] = largestx + plussize_x;
    EnvelopeSquareCoords[2][1] = largesty + plussize_y;

    EnvelopeSquareCoords[3][0] = smallestx - plussize_x;
    EnvelopeSquareCoords[3][1] = largesty - plussize_y;

}

/* Creating set of convex polygons for simple cell decomposition */
int CreateCellDecompositionPolygons(double **OutputConvexPolygonSet,
        double **EnvelopeSquare, double **Polygons,
        int *NumberOfVertices, const int NumberOfPolygons) {

    int i, j, k;
    int n = 0;

    static double TempCoord_1[3];
    static double TempCoord_2[3];

    for (i = 0; i < NumberOfPolygons; i++) {

        for (j = 0; j < NumberOfVertices[i] * 2; j += 2) {

            OutputConvexPolygonSet[n][j] = Polygons[i][j];
            OutputConvexPolygonSet[n][j + 1] = Polygons[i][j + 1];

            /* Creating a vertical line */
            FillVect(TempCoord_1, Polygons[i][j], Polygons[i][j + 1] - 2e222,
                    0.0);
            FillVect(TempCoord_2, Polygons[i][j], Polygons[i][j + 1] + 2e222,
                    0.0);

            /* Is there any intersections of this line and a polygon? */
            // TODO

            /* This intersection is the other point */
            // TODO

        }

    }

    // temp
    return 1;

}

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
        double *ReferencePoint) {
    double Temp1[3];
    double Temp2[3];
    VectDifference(Temp1, Point1, ReferencePoint);
    VectDifference(Temp2, Point2, ReferencePoint);
    return (ScalarProduct(Temp1, Temp2, 3) >= 0);
}

