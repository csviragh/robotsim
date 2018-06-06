/* vim:set ts=4 sw=4 sts=4 et: */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "file_utils.h"
#include "arenas.h"

static int ArenaCallback(void *user, const char *section, const char *name,
        const char *value) {

    arenas_t *arenas = user;
    float x, y, z;
    const char *Sep;

    /* if this is a new section */
    if (!arenas->a_count
            || strcmp(section, arenas->a[arenas->a_count - 1].name)) {
        if (arenas->a_count == MAX_ARENA_COUNT) {
            printf("ERROR: No space for more arenas.\n");
            return 0;
        }
        /* store new index, name and reset vertex count */
        arenas->a[arenas->a_count].index = arenas->a_count;
        strncpy(arenas->a[arenas->a_count].name, section,
                MAX_ARENA_NAME_LENGTH);
        arenas->a[arenas->a_count].p_count = 0;
        /* increase counter */
        (arenas->a_count)++;
    }

    if (strcmp(name, "point")) {
        printf("ERROR: unknown name while parsing arena (%s)\n", name);
        return 0;
    }

    if (sscanf(value, "%f %f %f", &x, &y, &z) != 3) {
        z = 0;
        if (sscanf(value, "%f %f", &x, &y) != 2) {
            printf("ERROR: could not parse arena point\n");
            return 0;
        }
    }

    /* store point */
    arenas->a[arenas->a_count -
            1].p[arenas->a[arenas->a_count - 1].p_count][0] = x;
    arenas->a[arenas->a_count -
            1].p[arenas->a[arenas->a_count - 1].p_count][1] = y;
    arenas->a[arenas->a_count -
            1].p[arenas->a[arenas->a_count - 1].p_count][2] = z;
    (arenas->a[arenas->a_count - 1].p_count)++;

    return 1;
}

int ParseArenaFile(const char *name, arenas_t * arenas,
        bool define_circle_and_square) {
    int result;
    memset(arenas, 0, sizeof(arenas_t));
    if (define_circle_and_square) {
        arenas->a_count = 2;
        // insert predefined CIRCLE at place 0
        strcpy(arenas->a[ARENA_CIRCLE].name, "circle");
        arenas->a[ARENA_CIRCLE].index = ARENA_CIRCLE;
        // insert predefined SQUARE at place 1
        strcpy(arenas->a[ARENA_SQUARE].name, "square");
        arenas->a[ARENA_SQUARE].index = ARENA_SQUARE;
        arenas->a[ARENA_SQUARE].p_count = 4;
        arenas->a[ARENA_SQUARE].p[0][0] = -1;
        arenas->a[ARENA_SQUARE].p[0][1] = -1;
        arenas->a[ARENA_SQUARE].p[1][0] = -1;
        arenas->a[ARENA_SQUARE].p[1][1] = 1;
        arenas->a[ARENA_SQUARE].p[2][0] = 1;
        arenas->a[ARENA_SQUARE].p[2][1] = 1;
        arenas->a[ARENA_SQUARE].p[3][0] = 1;
        arenas->a[ARENA_SQUARE].p[3][1] = -1;
    }
    // parse rest of arenas
    result = IniParse(name, ArenaCallback, arenas);
    if (result)
        printf("Error occured while reading row %d of arena file! \n", result);
    printf("Parsed %d arenas\n", arenas->a_count);
    return arenas->a_count;
}

// works only for CONVEX arenas, in 2D!
// returns 0 if WhichPoint is inside arena (CW)
// returns 1 if NearestPoint contains closest edge point (in shadow)
// returns 2 if NearestPoint contains closest vertex point (not in shadow)
int NearestArenaPoint(double *NearestPoint, double *WhichPoint,
        const arena_t * Arena, const double ArenaRadius,
        const double ArenaCenterX, const double ArenaCenterY) {
    int i;
    static double p[MAX_ARENA_VERTICES * 2 + 2];
    static double Temp1[3];
    static double Temp2[3];
    double dist, mindist = 1e222;

    // convert arena to polygon
    for (i = 0; i < Arena->p_count; i++) {
        p[i * 2] = Arena->p[i][0] * ArenaRadius + ArenaCenterX;
        p[i * 2 + 1] = Arena->p[i][1] * ArenaRadius + ArenaCenterY;
    }
    // close it with first vertex
    p[i * 2] = Arena->p[0][0] * ArenaRadius + ArenaCenterX;
    p[i * 2 + 1] = Arena->p[0][1] * ArenaRadius + ArenaCenterY;
    // check if we are inside at all. If so, nothing is returned in NearestPoint
    if (IsInsidePolygon(WhichPoint, p, Arena->p_count + 1)) {
        return 0;
    }
    // note that AtShadow works with 3D points but we do not care about
    // third dimension so this 2D-3D overlap bug does not count...
    for (i = 0; i < Arena->p_count; i++) {
        if (AtShadow(&p[i * 2], &p[(i + 1) * 2], WhichPoint) > 0) {
            // get closest point on edge
            VectDifference(Temp1, WhichPoint, &p[i * 2]);
            VectDifference(Temp2, &p[(i + 1) * 2], &p[i * 2]);
            Temp2[2] = 0.0;
            UnitVect(Temp2, Temp2);
            MultiplicateWithScalar(Temp2, Temp2,
                    ScalarProduct(Temp1, Temp2, 2), 2);
            VectSum(NearestPoint, &p[i * 2], Temp2);
            NearestPoint[2] = 0;
            // return 1 indicating that we have found an edge and closest point
            // is stored in NearestPoint
            return 1;
        } else {
            VectDifference(Temp1, WhichPoint, &p[i * 2]);
            dist = VectAbsXY(Temp1);
            if (dist < mindist) {
                mindist = dist;
                FillVect(NearestPoint, p[i * 2], p[i * 2 + 1], 0);
            }
        }
    }
    // return 2 indicating that we have not found an edge but a closest corner,
    // which is stored in NearestPoint
    return 2;
}

void
Shill_Wall(double *OutputVelocity, phase_t * Phase,
        const double ArenaCenterX, const double ArenaCenterY,
        const double ArenaRadius, const arena_t * Arena,
        const double C_Shill, const double V_Shill,
        const double Alpha_Shill, const double Gamma_Wall,
        const int WhichAgent, const int Dim_l) {

    const double C_VelDiff = 1.0 / 100.0;
    double *AgentsPosition;
    AgentsPosition = Phase->Coordinates[WhichAgent];
    double *AgentsVelocity;
    AgentsVelocity = Phase->Velocities[WhichAgent];
    static double GoalPositionVect[3];
    FillVect(GoalPositionVect, ArenaCenterX, ArenaCenterY, 0.0);

    static double FromGoal[3];
    static double ToArena[3];
    VectDifference(FromGoal, GoalPositionVect, AgentsPosition);
    if (2.0 == Dim_l) {
        FromGoal[2] = 0.0;
    }

    static double ToArenaLength;

    // SQUARE
    if (Arena->index == ARENA_SQUARE) {
        int i;
        for (i = 0; i < Dim_l; i++) {
            NullVect(ToArena, 3);
            ToArena[i] = FromGoal[i];
            UnitVect(ToArena, ToArena);
            MultiplicateWithScalar(ToArena, ToArena, V_Shill, 3);
            VectDifference(ToArena, ToArena, AgentsVelocity);
            if (fabs(Alpha_Shill) > 1e-12) {
                if (2.0 == Dim_l)
                    ToArenaLength = VectAbsXY(ToArena);
                else
                    ToArenaLength = VectAbs(ToArena);
                if (ToArenaLength > 0.0) {
                    MultiplicateWithScalar(ToArena, ToArena,
                            pow(C_VelDiff * ToArenaLength,
                                    Alpha_Shill), (int) Dim_l);
                }
            }
            //MultiplicateWithScalar (ToArena, ToArena, C_Shill * 
            //        (1.0 - Sigmoid(fabs(FromGoal[i]), Gamma_Wall, ArenaRadius)), (int) Dim_l);
            MultiplicateWithScalar(ToArena, ToArena, C_Shill *
                    (1.0 - SigmoidLin(fabs(FromGoal[i]),
                                    1.0 / Gamma_Wall, 1.0,
                                    ArenaRadius)), (int) Dim_l);
            VectSum(OutputVelocity, ToArena, OutputVelocity);
        }
        // CIRCLE
    } else if (Arena->index == ARENA_CIRCLE) {
        UnitVect(ToArena, FromGoal);
        MultiplicateWithScalar(ToArena, ToArena, V_Shill, 3);
        VectDifference(ToArena, ToArena, AgentsVelocity);
        if (fabs(Alpha_Shill) > 1e-12) {
            if (2.0 == Dim_l)
                ToArenaLength = VectAbsXY(ToArena);
            else
                ToArenaLength = VectAbs(ToArena);
            if (ToArenaLength > 0.0) {
                MultiplicateWithScalar(ToArena, ToArena,
                        pow(C_VelDiff * ToArenaLength,
                                Alpha_Shill), (int) Dim_l);
            }
        }
        //MultiplicateWithScalar (ToArena, ToArena, C_Shill * 
        //        (1.0 - Sigmoid(VectAbs(FromGoal), Gamma_Wall, ArenaRadius)), (int) Dim_l);
        MultiplicateWithScalar(ToArena, ToArena, C_Shill *
                (1.0 - SigmoidLin(VectAbs(FromGoal),
                                1.0 / Gamma_Wall, 1.0,
                                ArenaRadius)), (int) Dim_l);
        VectSum(OutputVelocity, ToArena, OutputVelocity);
        // GENERAL ARENA POLYGON
    } else {
        // note that general polygon calculation is in 2D only
        if (NearestArenaPoint
                (ToArena, AgentsPosition, Arena, ArenaRadius, ArenaCenterX,
                        ArenaCenterY)) {
            VectDifference(ToArena, ToArena, AgentsPosition);
            ToArenaLength = VectAbsXY(ToArena);
            UnitVect(ToArena, ToArena);
            MultiplicateWithScalar(ToArena, ToArena, V_Shill, 2);
            VectDifference(ToArena, ToArena, AgentsVelocity);
            if (fabs(Alpha_Shill) > 1e-12) {
                //TODO: VectAbs can be 0!
                MultiplicateWithScalar(ToArena, ToArena,
                        pow(C_VelDiff * VectAbsXY(ToArena), Alpha_Shill), 2);
            }
            //MultiplicateWithScalar (ToArena, ToArena, C_Shill *
            //        (1.0 - Sigmoid(ToArenaLength, Gamma_Wall, 0.0)), 2);
            MultiplicateWithScalar(ToArena, ToArena, C_Shill *
                    (1.0 - SigmoidLin(ToArenaLength,
                                    1.0 / Gamma_Wall, 1.0, 0.0)), 2);
            VectSum(OutputVelocity, ToArena, OutputVelocity);
            OutputVelocity[2] = 0.0;
        } else {
            // we are inside, no need to have shill wall output velocity
            NullVect(OutputVelocity, 3);
        }
    }
    // zero out third dimension once again for sure
    if (2.0 == Dim_l) {
        OutputVelocity[2] = 0.0;
    }
}

void
Shill_Wall_LinSqrt(double *OutputVelocity, phase_t * Phase,
        const double ArenaCenterX, const double ArenaCenterY,
        const double ArenaRadius, const arena_t * Arena,
        const double V_Shill, const double R0_Offset_Shill,
        const double Acc_Shill, const double Slope_Shill,
        const int WhichAgent, const int Dim_l) {

    int i;
    double *AgentsPosition;
    AgentsPosition = Phase->Coordinates[WhichAgent];
    double *AgentsVelocity;
    AgentsVelocity = Phase->Velocities[WhichAgent];
    static double ArenaCenter[3];
    FillVect(ArenaCenter, ArenaCenterX, ArenaCenterY, 0.0);

    static double ToCenter[3];  // from pos towards center
    static double ToArena[3];

    VectDifference(ToCenter, ArenaCenter, AgentsPosition);
    if (2.0 == Dim_l) {
        ToCenter[2] = 0.0;
    }

    static double VelDiff;
    static double DistFromWall; // negative outside, positive inside
    static double MaxVelDiff;

    // SQUARE
    if (Arena->index == ARENA_SQUARE) {
        for (i = 0; i < Dim_l; i++) {
            // DistFromWall is positive if inside arena, negative if outside
            DistFromWall = ArenaRadius - fabs(ToCenter[i]);
            NullVect(ToArena, 3);
            ToArena[i] = ToCenter[i];
            UnitVect(ToArena, ToArena);
            MultiplicateWithScalar(ToArena, ToArena, V_Shill, 3);
            VectDifference(ToArena, ToArena, AgentsVelocity);
            if (2.0 == Dim_l) {
                ToArena[2] = 0.0;
            }
            VelDiff = VectAbs(ToArena);
            UnitVect(ToArena, ToArena);
            // calculate max allowed velocity difference at a given distance based
            // on an optimal linsqrt breaking curve
            MaxVelDiff = VelDecayLinSqrt(DistFromWall, Slope_Shill, Acc_Shill,
                    VelDiff, R0_Offset_Shill);
            // if velocity difference is larger than allowed, we compensate it
            if (VelDiff > MaxVelDiff) {
                MultiplicateWithScalar(ToArena, ToArena,
                        VelDiff - MaxVelDiff, Dim_l);
                VectSum(OutputVelocity, OutputVelocity, ToArena);
            }
        }
        // CIRCLE
    } else if (Arena->index == ARENA_CIRCLE) {
        DistFromWall = ArenaRadius - VectAbs(ToCenter);
        UnitVect(ToArena, ToCenter);
        MultiplicateWithScalar(ToArena, ToArena, V_Shill, 3);
        VectDifference(ToArena, ToArena, AgentsVelocity);
        if (2.0 == Dim_l) {
            ToArena[2] = 0.0;
        }
        VelDiff = VectAbs(ToArena);
        UnitVect(ToArena, ToArena);
        // calculate max allowed velocity difference at a given distance based
        // on an optimal linsqrt breaking curve
        MaxVelDiff = VelDecayLinSqrt(DistFromWall, Slope_Shill, Acc_Shill,
                VelDiff, R0_Offset_Shill);
        // if velocity difference is larger than allowed, we compensate it
        if (VelDiff > MaxVelDiff) {
            MultiplicateWithScalar(ToArena, ToArena,
                    VelDiff - MaxVelDiff, Dim_l);
            VectSum(OutputVelocity, OutputVelocity, ToArena);
        }
        // GENERAL ARENA POLYGON
    } else {
        // TODO: this is not implemented yet, as new method acts also inside
        // the arena and reaches its maximal value right at arena edge
        // we have to find it out how to resolve general polygon in this case.
        // Also with concave polygons it is not trivial how to define poly
        // center towards which we should go.
        NullVect(OutputVelocity, 3);
    }
    // zero out third dimension once again for sure
    if (2.0 == Dim_l) {
        OutputVelocity[2] = 0.0;
    }
}
