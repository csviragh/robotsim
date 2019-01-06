//MIT License
//Copyright (c) 2018 Eotvos Lorand University, Budapest

/* vim:set ts=4 sw=4 sts=4 et: */

/* Tools for calculating basic statistical quantities */

#include "stat.h"

/* Global "temporary" variables */
static double Avg = 0.0;
static double StDev = 0.0;
static double Min = 2e222;
static double Max = 0.0;

/* Setting up initial values for all (algo-independent) statistical properties */
void ResetStatistics(statistics_t * Statistics) {

    int i;

    for (i = 0; i < 8; i++) {
        Statistics->Data_Acceleration_Sum[i] = 0.0;
        Statistics->Data_Acceleration_StDev[i] = 0.0;
        Statistics->Data_Velocity_Sum[i] = 0.0;
        Statistics->Data_Velocity_StDev[i] = 0.0;
        if (3 < i)
            continue;
        Statistics->Data_DistanceBetweenUnits_Sum[i] = 0.0;
        Statistics->Data_DistanceBetweenUnits_StDev[i] = 0.0;
        Statistics->Data_Correlation_Sum[i] = 0.0;
        Statistics->Data_Correlation_StDev[i] = 0.0;
        if (2 < i)
            continue;
        Statistics->Data_DistanceBetweenNeighbours_Sum[i] = 0.0;
        Statistics->Data_DistanceBetweenNeighbours_StDev[i] = 0.0;
        Statistics->Data_CoM_Sum[i] = 0.0;
        Statistics->Data_CoM_StDev[i] = 0.0;
    }

    Statistics->Data_CollisionRatio_Sum = 0.0;
    Statistics->Data_CollisionRatio_StDev = 0.0;

}

/* Returns an array that contains the average, deviation, minimum and maximum of
 * distance between units
 */
double *StatOfDistanceBetweenUnits(phase_t * Phase) {

    int i, j;

    Avg = 0.0;
    StDev = 0.0;
    Min = 2e222;
    Max = 0.0;

    static double Dist_1_2 = 0.0;

    double *Coord1;
    double *Coord2;
    static double CoordDiff[3];

    static double StatData[4];

    for (i = 0; i < Phase->NumberOfAgents - 1; i++) {

        for (j = i + 1; j < Phase->NumberOfAgents; j++) {

            Coord1 = Phase->Coordinates[i];
            Coord2 = Phase->Coordinates[j];
            VectDifference(CoordDiff, Coord1, Coord2);
            Dist_1_2 = VectAbs(CoordDiff);

            if (Dist_1_2 > Max) {
                Max = Dist_1_2;
            }
            if (Dist_1_2 < Min) {
                Min = Dist_1_2;
            }

            Avg += Dist_1_2;
            StDev += Dist_1_2 * Dist_1_2;

        }

    }

    Avg *= 2.0 / (Phase->NumberOfAgents * (Phase->NumberOfAgents - 1));
    StDev *= 2.0 / (Phase->NumberOfAgents * (Phase->NumberOfAgents - 1));
    StDev -= Avg * Avg;

    if (StDev < 0.0) {
        StDev = 0.0;
    }

    StatData[0] = Avg;
    StatData[1] = sqrt(StDev);
    StatData[2] = Min;
    StatData[3] = Max;

    return StatData;

}

/* Returns an array that contains the average, deviation, minimum and maximum of
 * distance between nearest neighbours
 */
double *StatOfDistanceBetweenNearestNeighbours(phase_t * Phase) {

    int i, j;

    Avg = 0.0;
    StDev = 0.0;
    Max = 0.0;

    static double Min_i;
    Min_i = 2e222;
    static int Min_i_ID;
    Min_i_ID = Phase->NumberOfAgents + 10;

    static double Dist_1_2 = 0.0;

    double *Coord1;
    double *Coord2;
    static double CoordDiff[3];

    static double StatData[3];

    for (i = 0; i < Phase->NumberOfAgents; i++) {

        for (j = 0; j < Phase->NumberOfAgents; j++) {
            if (i != j) {

                Coord1 = Phase->Coordinates[i];
                Coord2 = Phase->Coordinates[j];
                VectDifference(CoordDiff, Coord1, Coord2);
                Dist_1_2 = VectAbs(CoordDiff);

                if (Dist_1_2 < Min_i) {
                    Min_i = Dist_1_2;
                    Min_i_ID = j;
                }
            }
        }

        Avg += Min_i;
        StDev += Min_i * Min_i;
        if (Min_i > Max) {
            Max = Min_i;
        }

        Min_i = 2e222;

    }

    Avg /= Phase->NumberOfAgents;
    StDev /= Phase->NumberOfAgents;
    StDev -= Avg * Avg;

    if (StDev < 0.0) {
        StDev = 0.0;
    }

    StatData[0] = Avg;
    StatData[1] = sqrt(StDev);
    StatData[2] = Max;

    return StatData;

}

/* Returns an array that contains the average, deviation, minimum and maximum of
 * velocity Length
 * and the Length and XYZ components of the average velocity
 */
double *StatOfVelocity(phase_t * Phase) {

    int i;
    double *Velocity;
    static double AvgVelocity[3];
    NullVect(AvgVelocity, 3);

    Avg = 0.0;
    StDev = 0.0;
    Min = 2e222;
    Max = 0.0;

    static double velLength;
    velLength = 0.0;

    /* Output */
    static double StatVel[8];

    for (i = 0; i < Phase->NumberOfAgents; i++) {

        Velocity = Phase->Velocities[i];

        VectSum(AvgVelocity, AvgVelocity, Velocity);

        velLength = VectAbs(Velocity);
        Avg += velLength;
        StDev += velLength * velLength;

        if (velLength > Max) {
            Max = velLength;
        }
        if (velLength < Min) {
            Min = velLength;
        }

    }

    Avg /= Phase->NumberOfAgents;
    StDev /= Phase->NumberOfAgents;
    StDev -= Avg * Avg;

    if (StDev < 0.0) {
        StDev = 0.0;
    }

    MultiplicateWithScalar(AvgVelocity, AvgVelocity,
            1.0 / (Phase->NumberOfAgents), 3);

    StatVel[0] = Avg;
    StatVel[1] = sqrt(StDev);
    StatVel[2] = Min;
    StatVel[3] = Max;
    StatVel[4] = VectAbs(AvgVelocity);
    StatVel[5] = AvgVelocity[0];
    StatVel[6] = AvgVelocity[1];
    StatVel[7] = AvgVelocity[2];

    return StatVel;

}

/* Returns an array that contains the average, deviation, minimum and maximum of
 * velocity scalar products
 */
double *StatOfCorrelation(phase_t * Phase) {

    int i, j;

    Avg = 0.0;
    StDev = 0.0;
    Min = 2e222;
    Max = 0.0;

    static double Corr_1_2;
    Corr_1_2 = 0.0;

    double *Vel1;
    double *Vel2;

    static double velLength1;
    static double velLength2;
    velLength1 = 0.0;
    velLength2 = 0.0;

    static double StatData[4];

    for (i = 0; i < Phase->NumberOfAgents - 1; i++) {

        for (j = i + 1; j < Phase->NumberOfAgents; j++) {

            Vel1 = Phase->Velocities[i];
            Vel2 = Phase->Velocities[j];

            velLength1 = VectAbs(Vel1);
            velLength2 = VectAbs(Vel2);

            if (velLength1 != 0 && velLength2 != 0) {
                Corr_1_2 =
                        ScalarProduct(Vel1, Vel2,
                        3) / (VectAbs(Vel1) * VectAbs(Vel2));
            } else {
                Corr_1_2 = 0.0;
            }

            if (Corr_1_2 > Max) {
                Max = Corr_1_2;
            }
            if (Corr_1_2 < Min) {
                Min = Corr_1_2;
            }

            Avg += Corr_1_2;
            StDev += Corr_1_2 * Corr_1_2;

        }

    }

    Avg *= 2.0 / (Phase->NumberOfAgents * (Phase->NumberOfAgents - 1));
    StDev *= 2.0 / (Phase->NumberOfAgents * (Phase->NumberOfAgents - 1));
    StDev -= Avg * Avg;

    if (StDev < 0.0) {
        StDev = 0.0;
    }

    StatData[0] = Avg;
    StatData[1] = sqrt(StDev);
    StatData[2] = Min;
    StatData[3] = Max;

    return StatData;

}

/* Returns an array that contains the average, deviation, minimum and maximum of
 * accelerations
 */
double *StatOfAcceleration(double *Accelerations, const double NumberOfAgents) {

    int i;

    double Avg = 0.0;
    double StDev = 0.0;
    double Min = 2e222;
    double Max = 0.0;

    static double accLength;

    /* Output */
    static double StatAcc[4];

    for (i = 0; i < NumberOfAgents; i++) {

        accLength = Accelerations[i];
        Avg += accLength;
        StDev += accLength * accLength;

        if (accLength > Max) {
            Max = accLength;
        }
        if (accLength < Min) {
            Min = accLength;
        }

    }

    Avg /= NumberOfAgents;
    StDev /= NumberOfAgents;
    StDev -= Avg * Avg;

    if (StDev < 0.0) {
        StDev = 0.0;
    }

    StatAcc[0] = Avg;
    StatAcc[1] = sqrt(StDev);
    StatAcc[2] = Min;
    StatAcc[3] = Max;

    return StatAcc;

}

/* Calculating a parameter that is proportional with
 * the number of collisions and dangerous situations.
 */
double RatioOfDangerousSituations(phase_t * Phase, const double RadiusOfCopter) {

    /* Vectors to store positions */
    double *x_i;
    static double x_j[3];

    static int NumberOfCollisions;
    NumberOfCollisions = 0;

    int i, j;

    /* Is there any collision? */
    for (i = 0; i < Phase->NumberOfAgents; i++) {

        x_i = Phase->Coordinates[i];

        for (j = 0; j < i; j++) {

            GetAgentsCoordinates(x_j, Phase, j);
            VectDifference(x_j, x_j, x_i);

            if (VectAbs(x_j) <= RadiusOfCopter) {

                NumberOfCollisions++;

            }

        }

    }

    /* Final result has to be divided with the number of all possible collisions */
    return NumberOfCollisions * 2.0 / (Phase->NumberOfAgents *
            (Phase->NumberOfAgents - 1));

}
