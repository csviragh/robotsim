/* vim:set ts=4 sw=4 sts=4 et: */

/*
 * This file contains unversal interaction terms.
 */

#include <limits.h>
#include "interactions.h"

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

void FrictionLinSqrt(double *OutputVelocity, phase_t * Phase,
        const double C_Frict_l, const double V_Frict_l,
        const double Acc_l, const double p_l, const double R_0_l,
        const int WhichAgent, const int Dim_l) {

    NullVect(OutputVelocity, 3);

    int i;
    int n = 0;

    double *AgentsCoordinates;
    double *AgentsVelocity;
    double *NeighboursCoordinates;
    double *NeighboursVelocity;

    AgentsCoordinates = Phase->Coordinates[WhichAgent];
    AgentsVelocity = Phase->Velocities[WhichAgent];

    static double DifferenceVector[3];
    static double DistanceFromNeighbour;
    static double VelDiff;
    static double MaxVelDiff;

    /* Friction-like term */
    for (i = 0; i < Phase->NumberOfAgents; i++) {

        if (i == WhichAgent)
            continue;
        /* Get distance from neighbor */
        NeighboursCoordinates = Phase->Coordinates[i];
        NullVect(DifferenceVector, 3);
        VectDifference(DifferenceVector, NeighboursCoordinates,
                AgentsCoordinates);
        DistanceFromNeighbour = VectAbs(DifferenceVector);
        /* Get velocity difference from neighbor */
        NeighboursVelocity = Phase->Velocities[i];
        VectDifference(DifferenceVector, NeighboursVelocity, AgentsVelocity);
        if (2 == Dim_l) {
            DifferenceVector[2] = 0.0;
        }
        VelDiff = VectAbs(DifferenceVector);
        UnitVect(DifferenceVector, DifferenceVector);
        // calculate max allowed velocity difference at a given distance based
        // on an optimal linsqrt breaking curve and allow for V_Frict slack
        MaxVelDiff =
                MAX(V_Frict_l,
                VelDecayLinSqrt(DistanceFromNeighbour, p_l, Acc_l, VelDiff,
                        R_0_l));
        // if velocity difference is larger than allowed, we compensate it
        if (VelDiff > MaxVelDiff) {
            MultiplicateWithScalar(DifferenceVector, DifferenceVector,
                    C_Frict_l * (VelDiff - MaxVelDiff), Dim_l);
            VectSum(OutputVelocity, OutputVelocity, DifferenceVector);
        }
    }
}

void RepulsionLin(double *OutputVelocity,
        phase_t * Phase, const double V_Rep_l, const double p_l,
        const double R_0_l, const int WhichAgent, const int Dim_l,
        const bool normalize) {

    NullVect(OutputVelocity, 3);

    int i;
    int n = 0;

    double *AgentsCoordinates;
    double *NeighboursCoordinates;

    AgentsCoordinates = Phase->Coordinates[WhichAgent];

    static double DifferenceVector[3];
    static double DistanceFromNeighbour;
    /* Repulsive interaction term */
    for (i = 0; i < Phase->NumberOfAgents; i++) {
        if (i == WhichAgent)
            continue;
        NeighboursCoordinates = Phase->Coordinates[i];
        VectDifference(DifferenceVector, AgentsCoordinates,
                NeighboursCoordinates);
        if (2 == Dim_l) {
            DifferenceVector[2] = 0.0;
        }
        DistanceFromNeighbour = VectAbs(DifferenceVector);
        /* Check if we interact at all */
        if (DistanceFromNeighbour >= R_0_l)
            continue;
        n += 1;

        UnitVect(DifferenceVector, DifferenceVector);

        MultiplicateWithScalar(DifferenceVector, DifferenceVector,
                SigmoidLin(DistanceFromNeighbour, p_l, V_Rep_l, R_0_l), Dim_l);

        VectSum(OutputVelocity, OutputVelocity, DifferenceVector);
    }

    /* divide result by number of interacting units */
    if (normalize && n > 1) {
        double length = VectAbs(OutputVelocity) / n;
        UnitVect(OutputVelocity, OutputVelocity);
        MultiplicateWithScalar(OutputVelocity, OutputVelocity, length, Dim_l);
    }
    //printf("Number of Repulsive neighbours: %d Norm of repulsive term relative to max repulsion velocity: %f\n", n, VectAbs (OutputVelocity)/V_Rep_l);
}


