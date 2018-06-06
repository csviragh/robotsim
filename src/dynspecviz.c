/* vim:set ts=4 sw=4 sts=4 et: */

/* Drawing copters, etc. (dynamics-specific objects) */

#include "dynspecviz.h"

static double EyeFromCenter[3];
static double DistanceFromCenter;
static double NewEyeZ;

/* Draws velocity arrow above the copter */
void DrawVelocityArrow_2D(const double x, const double y,
        const double velocity_x, const double velocity_y,
        const double MapSizexy, const float *arrowcolor) {

    /* Drawing arrow... */
    DrawArrow(RealToGlCoord_2D(x, MapSizexy),
            RealToGlCoord_2D(y, MapSizexy),
            RealToGlCoord_2D(20.0 * pow(-50.0 + sqrt(velocity_x * velocity_x +
                                    velocity_y * velocity_y), 1.0 / 3.0),
                    MapSizexy), atan2(-velocity_y, velocity_x), arrowcolor);

}

/* Draws...
 * - a circle (with radius=40cm) as a representation of a quadcopter
 * - "Dangerous area" - a circle (with radius=1.5m)
 */
void DrawCopter_2D(const double x, const double y, const double MapSizexy,
        const double Radius, const float *agentcolor) {

    /* Real size of the copter */
    DrawFullCircle(RealToGlCoord_2D(x, MapSizexy), RealToGlCoord_2D(y,
                    MapSizexy), RealToGlCoord_2D(40, MapSizexy), agentcolor);
    /* Dangerous zone */
    DrawCircle(RealToGlCoord_2D(x, MapSizexy), RealToGlCoord_2D(y, MapSizexy),
            RealToGlCoord_2D(Radius / 2.0, MapSizexy), agentcolor);

}

/* Labels above agents */
void DrawAgentLabel_2D(phase_t * Phase, const int WhichAgent, char *Label,
        bool Display, vizmode_params_t * VizParams, const float *Color) {

    double *Position;
    Position = Phase->Coordinates[WhichAgent];

    if (Display == true) {

        DrawString(RealToGlCoord_2D(Position[0] - VizParams->CenterX + 120.0,
                        VizParams->MapSizeXY),
                RealToGlCoord_2D(Position[1] - VizParams->CenterY + 120.0,
                        VizParams->MapSizeXY), GLUT_BITMAP_TIMES_ROMAN_10,
                Label, Color);

    }

}

/* Draws a network arrow between two agents */
void DrawNetworkArrow_2D(phase_t * Phase, const int FromWhichAgent,
        const int ToWhichAgent, vizmode_params_t * VizParams,
        const float *color) {

    double *FromCoords;
    double *ToCoords;
    FromCoords = Phase->Coordinates[FromWhichAgent];
    ToCoords = Phase->Coordinates[ToWhichAgent];

    static double DifferenceVector[3];
    VectDifference(DifferenceVector, ToCoords, FromCoords);

    static double ArrowCenterX;
    ArrowCenterX = (FromCoords[0] + ToCoords[0]) * 0.5 - VizParams->CenterX;
    static double ArrowCenterY;
    ArrowCenterY = (FromCoords[1] + ToCoords[1]) * 0.5 - VizParams->CenterY;

    static double angle;
    DifferenceVector[2] = 0;
    angle = acos(DifferenceVector[0] / VectAbs(DifferenceVector));
    if (DifferenceVector[1] > 0) {
        angle *= -1;
    }

    DrawThinArrow(RealToGlCoord_2D(ArrowCenterX, VizParams->MapSizeXY),
            RealToGlCoord_2D(ArrowCenterY, VizParams->MapSizeXY),
            RealToGlCoord_2D(VectAbs(DifferenceVector) - 60,
                    VizParams->MapSizeXY), RealToGlCoord_2D(30,
                    VizParams->MapSizeXY), angle, color);

}

/* Draws a network arrow between two positions */
void DrawNetworkArrowBetweenPositions_2D(double *FromCoords, double *ToCoords,
        vizmode_params_t * VizParams, const float *color) {

    static double DifferenceVector[3];
    VectDifference(DifferenceVector, ToCoords, FromCoords);

    static double ArrowCenterX;
    ArrowCenterX = (FromCoords[0] + ToCoords[0]) * 0.5 - VizParams->CenterX;
    static double ArrowCenterY;
    ArrowCenterY = (FromCoords[1] + ToCoords[1]) * 0.5 - VizParams->CenterY;

    static double angle;
    DifferenceVector[2] = 0;
    angle = acos(DifferenceVector[0] / VectAbs(DifferenceVector));
    if (DifferenceVector[1] > 0) {
        angle *= -1;
    }

    DrawThinArrow(RealToGlCoord_2D(ArrowCenterX, VizParams->MapSizeXY),
            RealToGlCoord_2D(ArrowCenterY, VizParams->MapSizeXY),
            RealToGlCoord_2D(VectAbs(DifferenceVector) - 60,
                    VizParams->MapSizeXY), RealToGlCoord_2D(30,
                    VizParams->MapSizeXY), angle, color);

}

/* Drawing sensor range network */
void DrawSensorRangeNetwork_2D(phase_t * PhaseData,
        const double SensorRangeToDisplay,
        const int WhichAgent,
        const double Delay,
        const int Now,
        const double h, vizmode_params_t * VizParams, const float *color) {

    int i;

    double *ActualAgentsCoordinates;
    ActualAgentsCoordinates = PhaseData[Now].Coordinates[WhichAgent];
    GetAgentsCoordinatesFromTimeLine(ActualAgentsCoordinates, PhaseData,
            WhichAgent, Now);

    static double NeighboursCoordinates[3];
    static double DifferenceVector[3];

    static double ArrowCenterX;
    static double ArrowCenterY;
    static double angle;

    for (i = 0; i < PhaseData[0].NumberOfAgents; i++) {
        if (i != WhichAgent) {

            GetAgentsCoordinatesFromTimeLine(NeighboursCoordinates, PhaseData,
                    i, Now);
            VectDifference(DifferenceVector, ActualAgentsCoordinates,
                    NeighboursCoordinates);
            if (VectAbs(DifferenceVector) < SensorRangeToDisplay) {
                GetAgentsCoordinatesFromTimeLine(NeighboursCoordinates,
                        PhaseData, i, Now);
                VectDifference(DifferenceVector, ActualAgentsCoordinates,
                        NeighboursCoordinates);

                ArrowCenterX =
                        (ActualAgentsCoordinates[0] +
                        NeighboursCoordinates[0]) * 0.5 - VizParams->CenterX;
                ArrowCenterY =
                        (ActualAgentsCoordinates[1] +
                        NeighboursCoordinates[1]) * 0.5 - VizParams->CenterY;

                DifferenceVector[2] = 0;
                angle = -atan2(DifferenceVector[1], DifferenceVector[0]);

                DrawThinArrow(RealToGlCoord_2D(ArrowCenterX,
                                VizParams->MapSizeXY),
                        RealToGlCoord_2D(ArrowCenterY, VizParams->MapSizeXY),
                        RealToGlCoord_2D(VectAbs(DifferenceVector) - 60,
                                VizParams->MapSizeXY), RealToGlCoord_2D(30,
                                VizParams->MapSizeXY), angle, color);

            }

        }
    }

}

/* 3D objects */

/* Drawing sensor range network */
void DrawSensorRangeNetwork_3D(phase_t * PhaseData,
        const double SensorRangeToDisplay,
        const int WhichAgent,
        const double Delay,
        const int Now,
        const double h, const double MapSizexy, const float *color) {

    int i;

    double *ActualAgentsCoordinates;
    ActualAgentsCoordinates = PhaseData[Now].Coordinates[WhichAgent];
    GetAgentsCoordinatesFromTimeLine(ActualAgentsCoordinates, PhaseData,
            WhichAgent, Now);

    static double NeighboursCoordinates[3];
    NullVect(NeighboursCoordinates, 3);
    static double DifferenceVector[3];
    NullVect(DifferenceVector, 3);

    for (i = 0; i < PhaseData[0].NumberOfAgents; i++) {
        if (i != WhichAgent) {

            GetAgentsCoordinatesFromTimeLine(NeighboursCoordinates, PhaseData,
                    i, Now);
            VectDifference(DifferenceVector, ActualAgentsCoordinates,
                    NeighboursCoordinates);
            if (VectAbs(DifferenceVector) < SensorRangeToDisplay) {

                glColor3f(color[0], color[1], color[2]);

                glBegin(GL_LINES);

                glVertex3f(RealToGlCoord_3D(ActualAgentsCoordinates[0],
                                MapSizexy),
                        RealToGlCoord_3D(ActualAgentsCoordinates[1], MapSizexy),
                        RealToGlCoord_3D(ActualAgentsCoordinates[2],
                                MapSizexy));
                glVertex3f(RealToGlCoord_3D(NeighboursCoordinates[0],
                                MapSizexy),
                        RealToGlCoord_3D(NeighboursCoordinates[1], MapSizexy),
                        RealToGlCoord_3D(NeighboursCoordinates[2], MapSizexy));

                glEnd();

            }

        }
    }

}

/* 3D camera movement */

void TranslateCameraOnXYPlane(vizmode_params_t * VizParams, double *Direction,
        const double StepSize) {

    /* Projection onto XY plane */
    Direction[2] = 0.0;
    UnitVect(Direction, Direction);

    /* Translation */
    VizParams->CenterX += Direction[0] * StepSize;
    VizParams->CenterY += Direction[1] * StepSize;
    VizParams->EyeX += Direction[0] * StepSize;
    VizParams->EyeY += Direction[1] * StepSize;

}

void RotateCameraAroundCenter(vizmode_params_t * VizParams, double *Axis,
        const double angle) {

    /* Creating eye from center vector */
    FillVect(EyeFromCenter,
            VizParams->EyeX - VizParams->CenterX,
            VizParams->EyeY - VizParams->CenterY,
            VizParams->EyeZ - VizParams->CenterZ);

    /* Rotation */
    RotateVectAroundSpecificAxis(EyeFromCenter, EyeFromCenter, Axis, angle);
    NewEyeZ = VizParams->CenterZ + EyeFromCenter[2];

    DistanceFromCenter = VectAbsXY(EyeFromCenter);

    if (VizParams->DistanceFromCenterLimit_XY > DistanceFromCenter ||
            VizParams->EyeZLimit > NewEyeZ) {
        return;
    } else {
        VizParams->EyeX = VizParams->CenterX + EyeFromCenter[0];
        VizParams->EyeY = VizParams->CenterY + EyeFromCenter[1];
        VizParams->EyeZ = NewEyeZ;
    }

}

void ZoomOnCenter(vizmode_params_t * VizParams, const double StepSize) {

    /* Negative StepSize means zoom in... */

    FillVect(EyeFromCenter, VizParams->EyeX - VizParams->CenterX,
            VizParams->EyeY - VizParams->CenterY,
            VizParams->EyeZ - VizParams->CenterZ);
    DistanceFromCenter = VectAbs(EyeFromCenter);
    UnitVect(EyeFromCenter, EyeFromCenter);
    NewEyeZ = EyeFromCenter[2] * StepSize + VizParams->EyeZ;

    if (DistanceFromCenter + StepSize < VizParams->DistanceFromCenterLimit
            || NewEyeZ < VizParams->EyeZLimit) {
        return;
    } else {
        VizParams->EyeX += EyeFromCenter[0] * StepSize;
        VizParams->EyeY += EyeFromCenter[1] * StepSize;
        VizParams->EyeZ = NewEyeZ;
    }

}
