//MIT License
//Copyright (c) 2018 Eotvos Lorand University, Budapest

/* vim:set ts=4 sw=4 sts=4 et: */

#include "dynamics_utils.h"

/*
    Tools for manipulating phase space
*/

void AllocatePhase(phase_t * Phase, const int NumberOfAgents,
        const int NumberOfInnerStates) {
    int i;

    Phase->NumberOfAgents = NumberOfAgents;
    Phase->Coordinates = doubleMatrix(NumberOfAgents, 3);
    Phase->Velocities = doubleMatrix(NumberOfAgents, 3);
    Phase->InnerStates = doubleMatrix(NumberOfAgents, NumberOfInnerStates);
    Phase->RealIDs = intData(NumberOfAgents);
    Phase->NumberOfInnerStates = NumberOfInnerStates;

    /* Initialize RealIDs */
    for (i = 0; i < NumberOfAgents; i++) {
        Phase->RealIDs[i] = i;
    }
}

void freePhase(phase_t * Phase) {

    freeMatrix(Phase->Coordinates, Phase->NumberOfAgents, 3);
    freeMatrix(Phase->Velocities, Phase->NumberOfAgents, 3);
    freeMatrix(Phase->InnerStates, Phase->NumberOfAgents,
            Phase->NumberOfInnerStates);
    free(Phase->RealIDs);

}

/* Inserts the "WhichAgent"th agent's position and velocity into "Phase" */
void InsertAgentPhase(phase_t * Phase, double *Agent, const int WhichAgent) {

    int i;
    for (i = 0; i < 3; i++) {
        Phase->Coordinates[WhichAgent][i] = Agent[i];
        Phase->Velocities[WhichAgent][i] = Agent[i + 3];
    }

}

/* Gets the "WhichAgent"th agent's position and velocity from "Phase" */
void GetAgentPhase(double *Agent, phase_t * Phase, const int WhichAgent) {

    int j;
    for (j = 0; j < 3; j++) {
        Agent[j] = Phase->Coordinates[WhichAgent][j];
        Agent[j + 3] = Phase->Velocities[WhichAgent][j];
    }

}

/* Gets the "WhichAgent"th agent's velocity from "Phase" */
void GetAgentsVelocity(double *Velocity, phase_t * Phase, const int WhichAgent) {

    int j;
    for (j = 0; j < 3; j++) {
        Velocity[j] = Phase->Velocities[WhichAgent][j];
    }

}

/* Inserts the "WhichAgent"th agent's velocity into "Phase" */
void
InsertAgentsVelocity(phase_t * Phase, double *Velocity, const int WhichAgent) {

    int j;
    for (j = 0; j < 3; j++) {
        Phase->Velocities[WhichAgent][j] = Velocity[j];
    }

}

/* Gets the "WhichAgent"th agent's position from "Phase" */
void GetAgentsCoordinates(double *Coords, phase_t * Phase, const int WhichAgent) {

    int j;
    for (j = 0; j < 3; j++) {
        Coords[j] = Phase->Coordinates[WhichAgent][j];
    }

}

/* Gets the "WhichAgent"th agent's extrapolated position from "Phase" */
void GetAgentsCoordinatesEp(double *Coords, phase_t * Phase,
        const int WhichAgent, const double Delay) {
    double vel[3];
    GetAgentsCoordinates(Coords, Phase, WhichAgent);
    // we use linear extrapolation with velocity
    // TODO: calculate acceleration as well for second order extrapolation
    GetAgentsVelocity(vel, Phase, WhichAgent);
    MultiplicateWithScalar(vel, vel, Delay, 3);
    VectSum(Coords, Coords, vel);
}

/* Inserts the "WhichAgent"th agent's position into "Phase" */
void InsertAgentsCoordinates(phase_t * Phase, double *Coords,
        const int WhichAgent) {

    int j;
    for (j = 0; j < 3; j++) {
        Phase->Coordinates[WhichAgent][j] = Coords[j];
    }

}

/* Calculates euclidean distance between two agents */
double DistanceBetweenAgents(phase_t * Phase, const int A1, const int A2,
        const int Dim) {

    if ((A1 < Phase->NumberOfAgents) &&
            (A2 < Phase->NumberOfAgents) && (A1 > -1) && (A2 > -1)) {

        static double Dist;
        Dist = 0.0;
        int i;

        for (i = 0; i < Dim; i++) {
            Dist += pow(Phase->Coordinates[A1][i] - Phase->Coordinates[A2][i],
                    2);
        }

        return sqrt(Dist);

    }

    printf("Error: index of agent must be larger than -1 and less than NumberOfAgents!\n");
    exit(-1);

}

/* Gets centre of mass of the agents with "unit mass" from "Phase" */
void GetCoM(double *CoMCoord, phase_t * Phase) {

    int i, j;

    /* Konvektor... :) */
    double *CacheCoMVector;

    for (j = 0; j < Phase->NumberOfAgents; j++) {

        CacheCoMVector = Phase->Coordinates[j];

        for (i = 0; i < 3; i++) {

            CoMCoord[i] += CacheCoMVector[i];

        }

    }

    for (i = 0; i < 3; i++) {

        CoMCoord[i] = CoMCoord[i] * pow(Phase->NumberOfAgents, -1);

    }

}

/* Gets the number of nearby units */
int NumberOfNearbyUnits(phase_t * Phase, const int WhichAgent,
        const double RadiusOfLocalArea) {

    int i;

    static int NumberOfNeighbours;
    NumberOfNeighbours = 0;

    double *Temp1;
    Temp1 = Phase->Coordinates[WhichAgent];

    for (i = 0; i < Phase->NumberOfAgents; i++) {

        static double Temp2[3];
        GetAgentsCoordinates(Temp2, Phase, i);
        VectDifference(Temp2, Temp2, Temp1);

        if (VectAbs(Temp2) <= RadiusOfLocalArea) {
            NumberOfNeighbours++;
        }

    }

    return NumberOfNeighbours;

}

/* Gets the local centre of mass from the viewpoint of the "WhichAgent"th agent from "Phase"
 * The CoM should be calculated using the positions of the nearby agents */
void GetLocalCoM(double *CoMCoord, phase_t * Phase, const int WhichAgent,
        const double RadiusOfLocalArea) {

    NullVect(CoMCoord, 3);

    int i, j;
    static int HowManyAgentsNearby;
    HowManyAgentsNearby = 0;
    static double NeighboursPosition[3];
    static double ActualAgentsPosition[3];
    static double DifferenceOfPositions[3];

    GetAgentsCoordinates(ActualAgentsPosition, Phase, WhichAgent);

    for (j = 0; j < Phase->NumberOfAgents; j++) {

        GetAgentsCoordinates(NeighboursPosition, Phase, j);

        VectDifference(DifferenceOfPositions, ActualAgentsPosition,
                NeighboursPosition);

        /* If the jth agent is closer than "RadiusOfLocalArea", then it is a "nearby" agent */

        if (VectAbs(DifferenceOfPositions) <= RadiusOfLocalArea) {
            for (i = 0; i < 3; i++) {

                CoMCoord[i] += NeighboursPosition[i];

            }
            HowManyAgentsNearby++;
        }

    }

    /* Division by the number of nearby agents */
    MultiplicateWithScalar(CoMCoord, CoMCoord, pow(HowManyAgentsNearby, -1), 3);

}

/* Calculates CoM of specific agents */
void GetAgentSpecificCoM(double *CoMCoord, phase_t * Phase,
        int *ExceptWhichAgents) {

    int i, j;
    static int HowManyAgents;
    HowManyAgents = 0;
    double *CacheVector;

    static bool IsItAnException;
    IsItAnException = false;

    for (j = 0; j < Phase->NumberOfAgents; j++) {

        CacheVector = Phase->Coordinates[j];

        i = 0;
        while (i < Phase->NumberOfAgents && IsItAnException == false) {
            if (ExceptWhichAgents[i] == j) {
                IsItAnException = true;
            }
            i++;
        }

        if (IsItAnException == false) {
            for (i = 0; i < 3; i++) {

                CoMCoord[i] += CacheVector[i];

            }

            HowManyAgents++;
        }
        IsItAnException = false;

    }

    for (i = 0; i < 3; i++) {

        CoMCoord[i] = CoMCoord[i] * pow(HowManyAgents, -1);

    }

}

/* Calculates local CoM.
 * "ExceptWhichAgents" contains and array that determines e.g. target point agents...
 */
void GetAgentSpecificLocalCoM(double *CoMCoord, phase_t * Phase,
        const int AroundWhichAgent,
        int *ExceptWhichAgents, const double NeighbourhoodRange) {

    int i, j;
    static int HowManyAgents;
    HowManyAgents = 0;

    double *ActualAgentsCoordinates;
    double *NeighboursCoordinates;
    static double CoordinateDifference[3];
    NullVect(CoordinateDifference, 3);

    static bool IsItAnException;
    IsItAnException = false;

    ActualAgentsCoordinates = Phase->Coordinates[AroundWhichAgent];

    for (j = 0; j < Phase->NumberOfAgents; j++) {

        NeighboursCoordinates = Phase->Coordinates[j];
        VectDifference(CoordinateDifference, ActualAgentsCoordinates,
                NeighboursCoordinates);

        i = 0;
        while (i < Phase->NumberOfAgents && IsItAnException == false) {
            if (ExceptWhichAgents[i] == j) {
                IsItAnException = true;
            }
            i++;
        }

        if (VectAbs(CoordinateDifference) <= NeighbourhoodRange
                && IsItAnException == false) {
            for (i = 0; i < 3; i++) {

                CoMCoord[i] += NeighboursCoordinates[i];

            }

            HowManyAgents++;
        }

        IsItAnException = false;
    }

    for (i = 0; i < 3; i++) {

        CoMCoord[i] = CoMCoord[i] * pow(HowManyAgents, -1);

    }

}

/* Average velocity vector */
void GetAvgOfVelocity(double *CoMVel, phase_t * Phase) {

    int i, j;
    double *CacheVector;

    for (j = 0; j < Phase->NumberOfAgents; j++) {

        CacheVector = Phase->Coordinates[j];

        for (i = 0; i < 3; i++) {

            CoMVel[i] += CacheVector[i];
        }

    }

    for (i = 0; i < 3; i++) {

        CoMVel[i] = CoMVel[i] * pow(Phase->NumberOfAgents, -1);

    }

}

/* Local average velocity vector */
void GetLocalAverageOfXYVelocity(double *CoMVel, phase_t * Phase,
        const int WhichAgent, const double AreaRadius) {

    int n = 0;
    int i, j;

    double *CacheVelocityVector;
    double *CacheCoordinateVector;
    double *CacheActualCoordinateVector;
    CacheActualCoordinateVector = Phase->Coordinates[WhichAgent];
    static double CacheDifferenceVector[3];

    for (j = 0; j < Phase->NumberOfAgents; j++) {

        if (j != WhichAgent) {

            CacheVelocityVector = Phase->Velocities[j];
            CacheCoordinateVector = Phase->Coordinates[j];

            VectDifference(CacheDifferenceVector, CacheActualCoordinateVector,
                    CacheCoordinateVector);
            if (VectAbs(CacheDifferenceVector) <= AreaRadius) {

                for (i = 0; i < 2; i++) {

                    CoMVel[i] += CacheVelocityVector[i];

                }

                n++;

            }

        }

    }

    CacheVelocityVector = Phase->Velocities[WhichAgent];

    for (i = 0; i < 2; i++) {

        CoMVel[i] += CacheVelocityVector[i];

    }

    n++;

    for (i = 0; i < 2; i++) {

        CoMVel[i] = CoMVel[i] * pow(n, -1);

    }

    CoMVel[2] = 0;              /* Only the XY plane is interesting... */

}

/* Average of tangencial velocity (reference point is "RefPoint", e. g. CoM) in the XY plane */
void GetAverageOfXYTangentialVelocity(double *OutputLAPVel,
        phase_t * Phase, double *RefPoint, const int WhichAgent) {

    NullVect(OutputLAPVel, 3);

    static double x[3];
    GetAgentsCoordinates(x, Phase, WhichAgent);
    static double v[3];
    GetAgentsVelocity(v, Phase, WhichAgent);

    static double ProjectedMagn;

    static double v_i_LAP[3];
    static double v_j_LAP[3];
    static double to_ref[3];

    VectDifference(to_ref, x, RefPoint);

    RotateVectXY(v_i_LAP, to_ref, M_PI_2);
    UnitVect(v_i_LAP, v_i_LAP);
    ProjectedMagn = ScalarProduct(v_i_LAP, v, 2);

    int j;

    for (j = 0; j < Phase->NumberOfAgents; j++) {

        if (j == WhichAgent)
            continue;

        GetAgentsCoordinates(x, Phase, j);
        GetAgentsVelocity(v, Phase, j);
        VectDifference(to_ref, x, RefPoint);
        RotateVectXY(v_j_LAP, to_ref, M_PI_2);
        UnitVect(v_j_LAP, v_j_LAP);
        ProjectedMagn += ScalarProduct(v_j_LAP, v, 2);

    }

    FillVect(OutputLAPVel, v_i_LAP[0], v_i_LAP[1], 0.0);
    MultiplicateWithScalar(OutputLAPVel, OutputLAPVel,
            ProjectedMagn / Phase->NumberOfAgents, 2);

}

/* Local average of tangencial velocity (reference point is "RefPoint", e. g. CoM) in the XY plane */
void GetLocalAverageOfXYTangentialVelocity(double *OutputLAPVel,
        phase_t * Phase,
        double *RefPoint, const int WhichAgent, const double AreaSize) {

    NullVect(OutputLAPVel, 3);

    static int n;
    n = 1;

    static double x[3];
    GetAgentsCoordinates(x, Phase, WhichAgent);
    static double v[3];
    GetAgentsVelocity(v, Phase, WhichAgent);

    static double ProjectedMagn;

    static double v_i_LAP[3];
    static double v_j_LAP[3];
    static double to_ref[3];

    VectDifference(to_ref, x, RefPoint);

    RotateVectXY(v_i_LAP, to_ref, M_PI_2);
    UnitVect(v_i_LAP, v_i_LAP);
    ProjectedMagn = ScalarProduct(v_i_LAP, v, 2);

    int j;

    for (j = 0; j < Phase->NumberOfAgents; j++) {

        if (j == WhichAgent)
            continue;

        static double x_neighbour[3];
        static double DistanceVector[3];

        GetAgentsCoordinates(x_neighbour, Phase, j);
        VectDifference(DistanceVector, x_neighbour, x);

        if (VectAbsXY(DistanceVector) <= AreaSize) {

            GetAgentsVelocity(v, Phase, j);
            VectDifference(to_ref, x_neighbour, RefPoint);
            RotateVectXY(v_j_LAP, to_ref, M_PI_2);
            UnitVect(v_j_LAP, v_j_LAP);
            ProjectedMagn += ScalarProduct(v_j_LAP, v, 2);
            n++;

        }

    }

    FillVect(OutputLAPVel, v_i_LAP[0], v_i_LAP[1], 0.0);
    MultiplicateWithScalar(OutputLAPVel, OutputLAPVel, ProjectedMagn / n, 2);

}

/* Local average of tangencial velocity (reference point is "RefPoint", e. g. CoM) in the XY plane */
void GetAgentSpecificLocalAverageOfXYTangentialVelocity(double *OutputLAPVel,
        phase_t * Phase,
        double *RefPoint,
        int *ExceptWhichAgents, const int WhichAgent, const double AreaSize) {

    static bool IsItAnException;
    IsItAnException = false;

    NullVect(OutputLAPVel, 3);

    static int n;
    n = 1;

    static double x[3];
    GetAgentsCoordinates(x, Phase, WhichAgent);
    static double v[3];
    GetAgentsVelocity(v, Phase, WhichAgent);

    static double ProjectedMagn;

    static double v_i_LAP[3];
    static double v_j_LAP[3];
    static double to_ref[3];

    VectDifference(to_ref, x, RefPoint);

    RotateVectXY(v_i_LAP, to_ref, M_PI_2);
    UnitVect(v_i_LAP, v_i_LAP);
    ProjectedMagn = ScalarProduct(v_i_LAP, v, 2);

    int i, j;

    for (j = 0; j < Phase->NumberOfAgents; j++) {

        if (j == WhichAgent)
            continue;

        static double x_neighbour[3];
        static double DistanceVector[3];

        i = 0;
        while (i < Phase->NumberOfAgents && IsItAnException == false) {
            if (ExceptWhichAgents[i] == j) {
                IsItAnException = true;
            }
            i++;
        }

        if (true == IsItAnException) {
            continue;
        }

        GetAgentsCoordinates(x_neighbour, Phase, j);
        VectDifference(DistanceVector, x_neighbour, x);

        if (VectAbsXY(DistanceVector) <= AreaSize) {

            GetAgentsVelocity(v, Phase, j);
            VectDifference(to_ref, x_neighbour, RefPoint);
            RotateVectXY(v_j_LAP, to_ref, M_PI_2);
            UnitVect(v_j_LAP, v_j_LAP);
            ProjectedMagn += ScalarProduct(v_j_LAP, v, 2);
            n++;

        }

    }

    FillVect(OutputLAPVel, v_i_LAP[0], v_i_LAP[1], 0.0);
    MultiplicateWithScalar(OutputLAPVel, OutputLAPVel, ProjectedMagn / n, 2);

}

/* Local average of tangencial velocity (reference point is "RefPoint", e. g. CoM) */
void GetLocalAverageOfTangentialVelocity(double *OutputLAPVel,
        phase_t * Phase,
        double *RefPoint,
        double *Axis, const int WhichAgent, const double AreaSize) {

    NullVect(OutputLAPVel, 3);

    static int n;
    n = 1;

    static double x[3];
    GetAgentsCoordinates(x, Phase, WhichAgent);
    static double v[3];
    GetAgentsVelocity(v, Phase, WhichAgent);

    static double ProjectedMagn;

    static double v_i_LAP[3];
    static double v_j_LAP[3];
    static double to_ref[3];

    VectDifference(to_ref, x, RefPoint);

    RotateVectAroundSpecificAxis(v_i_LAP, to_ref, Axis, M_PI_2);
    UnitVect(v_i_LAP, v_i_LAP);
    ProjectedMagn = ScalarProduct(v_i_LAP, v, 3);

    int j;

    for (j = 0; j < Phase->NumberOfAgents; j++) {

        if (j == WhichAgent)
            continue;

        static double x_neighbour[3];
        static double DistanceVector[3];

        GetAgentsCoordinates(x_neighbour, Phase, j);
        VectDifference(DistanceVector, x_neighbour, x);

        if (VectAbs(DistanceVector) <= AreaSize) {

            GetAgentsVelocity(v, Phase, j);
            VectDifference(to_ref, x_neighbour, RefPoint);
            RotateVectAroundSpecificAxis(v_j_LAP, to_ref, Axis, M_PI_2);
            UnitVect(v_j_LAP, v_j_LAP);
            ProjectedMagn += ScalarProduct(v_j_LAP, v, 3);
            n++;

        }

    }

    FillVect(OutputLAPVel, v_i_LAP[0], v_i_LAP[1], v_i_LAP[2]);
    MultiplicateWithScalar(OutputLAPVel, OutputLAPVel,
            ProjectedMagn / ((double) n), 3);

}

/* Gets the "WhichAgent"th position at the "WhichStep"th timestep from "PhaseData"  */
void GetAgentsCoordinatesFromTimeLine(double *Coords, phase_t * PhaseData,
        const int WhichAgent, const int WhichStep) {

    int j;
    for (j = 0; j < 3; j++) {

        Coords[j] = PhaseData[WhichStep].Coordinates[WhichAgent][j];

    }

}

/* Gets the "WhichAgent"th velocity at the "WhichStep"th timestep from "PhaseData"  */
void GetAgentsVelocityFromTimeLine(double *Velocity, phase_t * PhaseData,
        const int WhichAgent, const int WhichStep) {

    int j;
    for (j = 0; j < 3; j++) {

        Velocity[j] = PhaseData[WhichStep].Velocities[WhichAgent][j];

    }

}

/* Inserts the actual position and velocity of the agents into "PhaseData" */
void InsertPhaseToDataLine(phase_t * PhaseData, phase_t * Phase,
        const int WhichStep) {

    int i, j;
    for (j = 0; j < Phase->NumberOfAgents; j++) {
        for (i = 0; i < 3; i++) {
            PhaseData[WhichStep].Coordinates[j][i] = Phase->Coordinates[j][i];
            PhaseData[WhichStep].Velocities[j][i] = Phase->Velocities[j][i];
        }
    }

}

/* Inserting inner states into inner state timeline */
void InsertInnerStatesToDataLine(phase_t * PhaseData, phase_t * Phase,
        const int WhichStep) {

    int j, k;

    for (k = 0; k < Phase->NumberOfInnerStates; k++) {
        for (j = 0; j < Phase->NumberOfAgents; j++) {

            PhaseData[WhichStep].InnerStates[j][k] = Phase->InnerStates[j][k];

        }
    }

}

/* Shifting "PhaseData" to maintain an endless loop of simulated experiment */
void ShiftDataLine(phase_t * PhaseData, const int HowManyRows,
        const int HowManyRowsToSave) {

    int i, j, k;

    for (i = 0; i < HowManyRowsToSave; i++) {
        for (j = 0; j < PhaseData[0].NumberOfAgents; j++) {
            for (k = 0; k < 3; k++) {
                PhaseData[i].Coordinates[j][k] =
                        PhaseData[HowManyRows - HowManyRowsToSave +
                        i].Coordinates[j][k];
                PhaseData[i].Velocities[j][k] =
                        PhaseData[HowManyRows - HowManyRowsToSave +
                        i].Velocities[j][k];
            }
        }
    }

}

/* Inserting inner states into inner state timeline */
void ShiftInnerStateDataLine(phase_t * PhaseData,
        const int HowManyRows, const int HowManyRowsToSave) {

    int i, j, k;

    for (i = 0; i < HowManyRowsToSave; i++) {
        for (j = 0; j < PhaseData[0].NumberOfAgents; j++) {
            for (k = 0; k < PhaseData[0].NumberOfInnerStates; k++) {
                PhaseData[i].InnerStates[j][k] =
                        PhaseData[HowManyRows - HowManyRowsToSave +
                        i].InnerStates[j][k];
            }
        }
    }

}

/* Waiting - Filling up first rows of phase data timeline */
void Wait(phase_t * PhaseData, const double TimeToWait, const double h) {

    int i, j, k;

    /* Positions and velocities */
    for (i = 1; i < ((int) ((1 + TimeToWait) / h)); i++) {
        for (j = 0; j < PhaseData[0].NumberOfAgents; j++) {
            for (k = 0; k < 3; k++) {
                PhaseData[i].Coordinates[j][k] =
                        PhaseData[i - 1].Coordinates[j][k];
                PhaseData[i].Velocities[j][k] = 0.0;
            }
            for (k = 0; k < PhaseData[0].NumberOfInnerStates; k++) {
                PhaseData[i].InnerStates[j][k] =
                        PhaseData[i - 1].InnerStates[j][k];
            }
        }
    }

}

/* Randomizing phase of agents (with zero velocities) */
void RandomizePhase(phase_t * Phase,
        const double XSize,
        const double YSize,
        const double ZSize,
        const double XCenter,
        const double YCenter,
        const double ZCenter,
        const int fromAgent, const int toAgent, const double RadiusOfCopter) {

    int i, j, StepCount;
    static int MaxStep;
    MaxStep = 100 * Phase->NumberOfAgents;

    static double RandomPlaceVector[3];
    static double ActualAgentsVelocity[3];

    static double AgentjsCoords[3];
    static double DiffCoords[3];

    static bool isArrangementCorrect;
    isArrangementCorrect = false;

    for (i = fromAgent; i < toAgent; i++) {
        FillVect(RandomPlaceVector, 2e22, 2e22, 2e22);
        InsertAgentsCoordinates(Phase, RandomPlaceVector, i);
        NullVect(RandomPlaceVector, 3);
        InsertAgentsVelocity(Phase, RandomPlaceVector, i);
    }

    NullVect(ActualAgentsVelocity, 3);
    StepCount = 0;

    for (i = fromAgent; i < toAgent; i++) {
        while (false == isArrangementCorrect) {
            isArrangementCorrect = true;

            RandomPlaceVector[0] =
                    randomizeDouble(XCenter - XSize / 2.0,
                    XCenter + XSize / 2.0);
            RandomPlaceVector[1] =
                    randomizeDouble(YCenter - YSize / 2.0,
                    YCenter + YSize / 2.0);
            RandomPlaceVector[2] =
                    randomizeDouble(ZCenter - ZSize / 2.0,
                    ZCenter + ZSize / 2.0);

            for (j = 0; j < Phase->NumberOfAgents; j++) {
                /* Skip positions of "unplaced" agents */
                if (i == j) {
                    j = toAgent - 1;
                    continue;
                };

                GetAgentsCoordinates(AgentjsCoords, Phase, j);
                VectDifference(DiffCoords, AgentjsCoords, RandomPlaceVector);
                if (VectAbs(DiffCoords) <= 4 * RadiusOfCopter) {
                    isArrangementCorrect = false;
                    break;
                }
            }

            StepCount++;

            if (StepCount > MaxStep) {
                fprintf(stderr, "Please, increase the initial area sizes!\n");
                exit(-1);
            }
        }

        InsertAgentsVelocity(Phase, ActualAgentsVelocity, i);
        InsertAgentsCoordinates(Phase, RandomPlaceVector, i);
        isArrangementCorrect = false;
    }
}

/* Setting up random positions inside a ring */
void PlaceAgentsInsideARing(phase_t * Phase, const double SizeOfRing,
        const int fromAgent, const int toAgent,
        const double XCenter, const double YCenter,
        const double ZCenter, const double ZSize, const double RadiusOfCopter) {

    int i, j, StepCount;
    static int MaxStep;
    MaxStep = 100 * Phase->NumberOfAgents;

    static double RandomPlaceVector[3];
    static double ActualAgentsVelocity[3];

    static double AgentjsCoords[3];
    static double DiffCoords[3];

    static double temp;

    static bool isArrangementCorrect;
    isArrangementCorrect = false;

    for (i = fromAgent; i < toAgent; i++) {

        FillVect(RandomPlaceVector, 2e22, 2e22, 2e22);
        InsertAgentsCoordinates(Phase, RandomPlaceVector, i);
        NullVect(RandomPlaceVector, 3);
        InsertAgentsVelocity(Phase, RandomPlaceVector, i);

    }

    NullVect(ActualAgentsVelocity, 3);
    StepCount = 0;

    for (i = fromAgent; i < toAgent; i++) {

        while (false == isArrangementCorrect) {

            isArrangementCorrect = true;

            RandomPlaceVector[0] = randomizeDouble(0, 2.0 * M_PI);
            temp = randomizeDouble(0, SizeOfRing);
            RandomPlaceVector[1] = YCenter + temp * cos(RandomPlaceVector[0]);
            RandomPlaceVector[0] = XCenter + temp * sin(RandomPlaceVector[0]);
            RandomPlaceVector[2] =
                    randomizeDouble(ZCenter - ZSize / 2.0,
                    ZCenter + ZSize / 2.0);

            for (j = 0; j < Phase->NumberOfAgents; j++) {

                /* Skip positions of "unplaced" agents */
                if (i == j) {
                    j = toAgent - 1;
                    continue;
                };

                GetAgentsCoordinates(AgentjsCoords, Phase, j);
                VectDifference(DiffCoords, AgentjsCoords, RandomPlaceVector);
                if (VectAbs(DiffCoords) <= RadiusOfCopter) {

                    isArrangementCorrect = false;

                }

            }

            StepCount++;

            if (StepCount > MaxStep) {
                fprintf(stderr, "Please, increase the initial area sizes!\n");
                exit(-1);
            }

        }

        InsertAgentsVelocity(Phase, ActualAgentsVelocity, i);
        InsertAgentsCoordinates(Phase, RandomPlaceVector, i);
        isArrangementCorrect = false;

    }

}

/* Setting up random positions inside a sphere */
void PlaceAgentsInsideASphere(phase_t * Phase, const double SizeOfSphere,
        const int fromAgent, const int toAgent,
        const double XCenter, const double YCenter,
        const double ZCenter, const double RadiusOfCopter) {

    int i, j, StepCount;
    static int MaxStep;
    MaxStep = 100 * Phase->NumberOfAgents;

    static double RandomPlaceVector[3];
    static double ActualAgentsVelocity[3];

    static double AgentjsCoords[3];
    static double DiffCoords[3];

    static double temp;
    static double Phi;
    static double Theta;

    static bool isArrangementCorrect;
    isArrangementCorrect = false;

    for (i = fromAgent; i < toAgent; i++) {

        FillVect(RandomPlaceVector, 2e22, 2e22, 2e22);
        InsertAgentsCoordinates(Phase, RandomPlaceVector, i);
        NullVect(RandomPlaceVector, 3);
        InsertAgentsVelocity(Phase, RandomPlaceVector, i);

    }

    NullVect(ActualAgentsVelocity, 3);
    StepCount = 0;

    for (i = fromAgent; i < toAgent; i++) {

        while (false == isArrangementCorrect) {

            isArrangementCorrect = true;

            Phi = randomizeDouble(0, 2.0 * M_PI);
            Theta = randomizeDouble(0, M_PI);
            temp = randomizeDouble(0, SizeOfSphere);

            RandomPlaceVector[0] = XCenter + temp * cos(Phi) * sin(Theta);
            RandomPlaceVector[1] = YCenter + temp * sin(Phi) * sin(Theta);
            RandomPlaceVector[2] = ZCenter + temp * cos(Theta);

            for (j = 0; j < Phase->NumberOfAgents; j++) {

                /* Skip positions of "unplaced" agents */
                if (i == j) {
                    j = toAgent - 1;
                    continue;
                }

                GetAgentsCoordinates(AgentjsCoords, Phase, j);
                VectDifference(DiffCoords, AgentjsCoords, RandomPlaceVector);
                if (VectAbs(DiffCoords) <= RadiusOfCopter) {

                    isArrangementCorrect = false;

                }

            }

            StepCount++;

            if (StepCount > MaxStep) {
                fprintf(stderr, "Please, increase the initial area sizes!\n");
                exit(-1);
            }

        }

        InsertAgentsVelocity(Phase, ActualAgentsVelocity, i);
        InsertAgentsCoordinates(Phase, RandomPlaceVector, i);
        isArrangementCorrect = false;

    }

}

/* Setting up random initial conditions on a specific plane given by its normal vector */
void RandomizePhaseOnPlane(phase_t * Phase,
        const double XSize,
        const double YSize,
        const double XCenter,
        const double YCenter,
        const double ZCenter,
        double *PlaneNormalVect,
        double *XAxis,
        const int fromAgent, const int toAgent, const double RadiusOfCopter) {

    static int StepCount;
    StepCount = 0;
    static int MaxStep;
    MaxStep = 100 * Phase->NumberOfAgents;

    static bool IsArrangementCorrect;
    IsArrangementCorrect = false;

    /* Equation of this plane is n * (x - x_center) = 0 */
    static double AgentsCoordinates[3];
    NullVect(AgentsCoordinates, 3);
    static double TempCoord[3];

    /* Setting zero initial velocities */
    static double YAxis[3];
    VectorialProduct(YAxis, PlaneNormalVect, XAxis);

    int i, j;

    for (i = fromAgent; i < toAgent; i++) {

        while (false == IsArrangementCorrect) {

            IsArrangementCorrect = true;

            /* position = x * XAxis + d * YAxis */
            static double c, d;

            c = randomizeDouble(-XSize / 2.0, XSize / 2.0);
            d = randomizeDouble(-YSize / 2.0, YSize / 2.0);

            FillVect(AgentsCoordinates, XCenter + c * XAxis[0] + d * YAxis[0],
                    YCenter + c * XAxis[1] + d * YAxis[1],
                    ZCenter + c * XAxis[2] + d * YAxis[2]);

            for (j = 0; j < Phase->NumberOfAgents; j++) {

                if (i == j) {
                    j = toAgent - 1;
                    continue;
                }

                GetAgentsCoordinates(TempCoord, Phase, j);
                VectDifference(TempCoord, TempCoord, AgentsCoordinates);

                if (VectAbs(TempCoord) <= RadiusOfCopter) {
                    IsArrangementCorrect = false;
                }

            }

            StepCount++;

            if (StepCount > MaxStep) {
                fprintf(stderr, "Please, increase the initial area sizes!\n");
                exit(-1);
            }

            InsertAgentsCoordinates(Phase, AgentsCoordinates, i);
            NullVect(AgentsCoordinates, 3);
            InsertAgentsVelocity(Phase, AgentsCoordinates, i);
            IsArrangementCorrect = false;
        }

    }

}

/* Placing agents on specific planes (using main XYZ coordinate system) */
void PlaceAgentsOnXYPlane(phase_t * Phase,
        const double XSize,
        const double YSize,
        const double XCenter,
        const double YCenter,
        const double ZCenter,
        const int fromAgent, const int toAgent, const double RadiusOfCopter) {

    static int StepCount;
    StepCount = 0;
    static int MaxStep;
    MaxStep = 100 * Phase->NumberOfAgents;

    static bool IsArrangementCorrect;
    IsArrangementCorrect = false;

    static double AgentsCoordinates[3];
    NullVect(AgentsCoordinates, 3);
    static double TempCoord[3];

    int i, j;

    for (i = fromAgent; i < toAgent; i++) {

        while (false == IsArrangementCorrect) {

            IsArrangementCorrect = true;

            FillVect(AgentsCoordinates,
                    XCenter + randomizeDouble(-XSize / 2.0, XSize / 2.0),
                    YCenter + randomizeDouble(-YSize / 2.0, YSize / 2.0),
                    ZCenter);

            for (j = 0; j < Phase->NumberOfAgents; j++) {

                if (i == j) {
                    j = toAgent - 1;
                    continue;
                }

                GetAgentsCoordinates(TempCoord, Phase, j);
                VectDifference(TempCoord, TempCoord, AgentsCoordinates);

                if (VectAbsXY(TempCoord) <= RadiusOfCopter) {
                    IsArrangementCorrect = false;
                }

            }

            StepCount++;

            if (StepCount > MaxStep) {
                fprintf(stderr, "Please, increase the initial area sizes!\n");
                exit(-1);
            }

        }

        InsertAgentsCoordinates(Phase, AgentsCoordinates, i);
        NullVect(AgentsCoordinates, 3);
        InsertAgentsVelocity(Phase, AgentsCoordinates, i);
        IsArrangementCorrect = false;

    }

}

void PlaceAgentsOnXZPlane(phase_t * Phase,
        const double XSize,
        const double ZSize,
        const double XCenter,
        const double YCenter,
        const double ZCenter,
        const int fromAgent, const int toAgent, const double RadiusOfCopter) {

    static int StepCount;
    StepCount = 0;
    static int MaxStep;
    MaxStep = 100 * Phase->NumberOfAgents;

    static bool IsArrangementCorrect;
    IsArrangementCorrect = false;

    static double AgentsCoordinates[3];
    NullVect(AgentsCoordinates, 3);
    static double TempCoord[3];

    int i, j;

    for (i = fromAgent; i < toAgent; i++) {

        while (false == IsArrangementCorrect) {

            IsArrangementCorrect = true;

            FillVect(AgentsCoordinates,
                    XCenter + randomizeDouble(-XSize / 2.0, XSize / 2.0),
                    YCenter, ZCenter + randomizeDouble(-ZSize / 2.0,
                            ZSize / 2.0));

            for (j = 0; j < Phase->NumberOfAgents; j++) {

                if (i == j) {
                    j = toAgent - 1;
                    continue;
                }

                GetAgentsCoordinates(TempCoord, Phase, j);
                VectDifference(TempCoord, TempCoord, AgentsCoordinates);

                if (VectAbs(TempCoord) <= RadiusOfCopter) {
                    IsArrangementCorrect = false;
                }

            }

            StepCount++;

            if (StepCount > MaxStep) {
                fprintf(stderr, "Please, increase the initial area sizes!\n");
                exit(-1);
            }

        }

        InsertAgentsCoordinates(Phase, AgentsCoordinates, i);
        NullVect(AgentsCoordinates, 3);
        InsertAgentsVelocity(Phase, AgentsCoordinates, i);
        IsArrangementCorrect = false;

    }

}

void PlaceAgentsOnYZPlane(phase_t * Phase,
        const double YSize,
        const double ZSize,
        const double XCenter,
        const double YCenter,
        const double ZCenter,
        const int fromAgent, const int toAgent, const double RadiusOfCopter) {

    static int StepCount;
    StepCount = 0;
    static int MaxStep;
    MaxStep = 100 * Phase->NumberOfAgents;

    static bool IsArrangementCorrect;
    IsArrangementCorrect = false;

    static double AgentsCoordinates[3];
    NullVect(AgentsCoordinates, 3);
    static double TempCoord[3];

    int i, j;

    for (i = fromAgent; i < toAgent; i++) {

        while (false == IsArrangementCorrect) {

            IsArrangementCorrect = true;

            FillVect(AgentsCoordinates, XCenter,
                    YCenter + randomizeDouble(-YSize / 2.0, YSize / 2.0),
                    ZCenter + randomizeDouble(-ZSize / 2.0, ZSize / 2.0));

            for (j = 0; j < Phase->NumberOfAgents; j++) {

                if (i == j) {
                    j = toAgent - 1;
                    continue;
                }

                GetAgentsCoordinates(TempCoord, Phase, j);
                VectDifference(TempCoord, TempCoord, AgentsCoordinates);

                if (VectAbs(TempCoord) <= RadiusOfCopter) {
                    IsArrangementCorrect = false;
                }

            }

            StepCount++;

            if (StepCount > MaxStep) {
                fprintf(stderr, "Please, increase the initial area sizes!\n");
                exit(-1);
            }

        }

        InsertAgentsCoordinates(Phase, AgentsCoordinates, i);
        NullVect(AgentsCoordinates, 3);
        InsertAgentsVelocity(Phase, AgentsCoordinates, i);
        IsArrangementCorrect = false;

    }

}

/* placing agents onto specific lines */
void PlaceAgentsOntoALine(phase_t * Phase, const int fromAgent,
        const int toAgent, double *Tangential, const double XCenter,
        const double YCenter, const double ZCenter, const double LineLength,
        const double RadiusOfCopter) {

    static int StepCount;
    StepCount = 0;
    static int MaxStep;
    MaxStep = 100 * Phase->NumberOfAgents;

    static bool IsArrangementCorrect;
    IsArrangementCorrect = false;

    static double AgentsCoordinates[3];
    NullVect(AgentsCoordinates, 3);
    static double TempCoord[3];

    int i, j;

    for (i = fromAgent; i < toAgent; i++) {

        while (false == IsArrangementCorrect) {

            IsArrangementCorrect = true;

            static double a;
            a = randomizeDouble(-LineLength / 2.0, LineLength / 2.0);

            FillVect(AgentsCoordinates, XCenter + Tangential[0] * a,
                    YCenter + Tangential[1] * a, ZCenter + Tangential[2] * a);

            for (j = 0; j < Phase->NumberOfAgents; j++) {

                if (i == j) {
                    j = toAgent - 1;
                    continue;
                }

                GetAgentsCoordinates(TempCoord, Phase, j);
                VectDifference(TempCoord, TempCoord, AgentsCoordinates);

                if (VectAbs(TempCoord) <= RadiusOfCopter) {
                    IsArrangementCorrect = false;
                }

            }

            StepCount++;

            if (StepCount > MaxStep) {
                fprintf(stderr, "Please, increase the initial area sizes!\n");
                exit(-1);
            }

        }

        InsertAgentsCoordinates(Phase, AgentsCoordinates, i);
        NullVect(AgentsCoordinates, 3);
        InsertAgentsVelocity(Phase, AgentsCoordinates, i);
        IsArrangementCorrect = false;

    }

}

/* Setting up random Initial conditions with zero Initial velocity */
void InitCond(phase_t ** PhaseData,
        const double InitialSizeX,
        const double InitialSizeY,
        const double InitialSizeZ, const double SizeOfCopter) {

    RandomizePhase(PhaseData[0], InitialSizeX, InitialSizeY, InitialSizeZ, 0.0,
            0.0, 0.0, 0, PhaseData[0]->NumberOfAgents, SizeOfCopter);

}

/* Counting collisions */
int HowManyCollisions(phase_t * ActualPhase,
        bool * AgentsInDanger,
        const bool CountCollisions, const double RadiusOfCopter) {

    int i, j;
    static int Collisions;
    Collisions = 0;

    static double ithAgentsCoordinates[3];
    static double jthAgentsCoordinates[3];
    static double RelativeCoordinates[3];

    static bool PreviousSituation;
    PreviousSituation = false;

    if (CountCollisions == true) {
        for (j = 0; j < ActualPhase->NumberOfAgents; j++) {
            PreviousSituation = AgentsInDanger[j];
            AgentsInDanger[j] = false;
            for (i = 0; i < j; i++) {
                if (i != j) {
                    GetAgentsCoordinates(ithAgentsCoordinates, ActualPhase, i);
                    GetAgentsCoordinates(jthAgentsCoordinates, ActualPhase, j);
                    VectDifference(RelativeCoordinates, ithAgentsCoordinates,
                            jthAgentsCoordinates);
                    if (VectAbs(RelativeCoordinates) <= RadiusOfCopter) {

                        AgentsInDanger[j] = true;
                        AgentsInDanger[i] = true;

                    }
                }
            }

            Collisions += (PreviousSituation == false
                    && AgentsInDanger[j] == true);
        }
    }

    return Collisions;

}

/* Swaps the states of two agents (ith and jth) */
void SwapAgents(phase_t * Phase, const int i, const int j) {

    double *temp_pointer;
    int id;

    /* Positions and velocities */
    temp_pointer = Phase->Coordinates[i];
    Phase->Coordinates[i] = Phase->Coordinates[j];
    Phase->Coordinates[j] = temp_pointer;
    temp_pointer = Phase->Velocities[i];
    Phase->Velocities[i] = Phase->Velocities[j];
    Phase->Velocities[j] = temp_pointer;

    /* Inner states */
    temp_pointer = Phase->InnerStates[i];
    Phase->InnerStates[i] = Phase->InnerStates[j];
    Phase->InnerStates[j] = temp_pointer;

    /* Real IDs */
    id = Phase->RealIDs[i];
    Phase->RealIDs[i] = Phase->RealIDs[j];
    Phase->RealIDs[j] = id;

}

/* Orders agents by distance from a given position */
/* Warning! Simple insertion short! */
void OrderAgentsByDistance(phase_t * Phase, double *ReferencePosition) {

    static double DistFromRef[3];
    NullVect(DistFromRef, 3);

    static double Dist1;
    static double Dist2;
    Dist1 = 2e222;
    Dist2 = 2e222;

    int i, j;

    for (i = 1; i < Phase->NumberOfAgents; i++) {

        GetAgentsCoordinates(DistFromRef, Phase, i);
        VectDifference(DistFromRef, DistFromRef, ReferencePosition);
        Dist1 = VectAbs(DistFromRef);

        j = i;

        GetAgentsCoordinates(DistFromRef, Phase, j - 1);
        VectDifference(DistFromRef, DistFromRef, ReferencePosition);
        Dist2 = VectAbs(DistFromRef);

        while (j > 0 && Dist2 > Dist1) {

            /* Swapping velocities, positions, inner states and real IDs */
            SwapAgents(Phase, j - 1, j);

            j--;

            GetAgentsCoordinates(DistFromRef, Phase, j);
            VectDifference(DistFromRef, DistFromRef, ReferencePosition);
            Dist1 = VectAbs(DistFromRef);

            GetAgentsCoordinates(DistFromRef, Phase, j - 1);
            VectDifference(DistFromRef, DistFromRef, ReferencePosition);
            Dist2 = VectAbs(DistFromRef);

        }
    }
}

/* Packing of nearby agents to the first blocks of the phase space */
int SelectNearbyVisibleAgents(phase_t * Phase,
        double *ReferencePosition,
        double Range, double PacketLossQuadraticCoeff) {

    static double DistFromRef[3];
    NullVect(DistFromRef, 3);

    static double Dist = 0.0;

    int i;
    bool packet_lost;

    int NumberOfNearbyAgents = 1;

    for (i = Phase->NumberOfAgents - 1; i >= NumberOfNearbyAgents; i--) {

        GetAgentsCoordinates(DistFromRef, Phase, i);
        VectDifference(DistFromRef, DistFromRef, ReferencePosition);

        Dist = VectAbs(DistFromRef);
        packet_lost = (randomizeDouble(0, 1) <
                Dist * Dist * PacketLossQuadraticCoeff);

        if (Dist != 0 && Dist <= Range && !packet_lost) {
            SwapAgents(Phase, i, NumberOfNearbyAgents);
            NumberOfNearbyAgents++;
            i++;
        } else if (Dist == 0) {
            SwapAgents(Phase, i, 0);
            i++;
        }

    }

    return NumberOfNearbyAgents;

}
