//MIT License
//Copyright (c) 2018 Eotvos Lorand University, Budapest

/* vim:set ts=4 sw=4 sts=4 et: */

/*
 *
 * Main code of simulation and visualiser tool "robotsim".
 *
 * Programmed by Csaba Viragh
 * Eotvos Lorand University, Budapest
 * e-mail: viraghcs@hal.elte.hu
 *
 */

#define MAX(a,b) (a>b?a:b)
#define MIN(a,b) (a<b?a:b)

/* Standard C includes */
#include <sys/time.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <string.h>

/* Math tools and datastructs */
#include "utilities/datastructs.h"
#include "utilities/math_utils.h"
#include "utilities/file_utils.h"
#include "utilities/dynamics_utils.h"
#include "utilities/output_utils.h"
#include "robotmodel.h"

/* Tools for OpenGL visualization and GUI */
#include "objects_menu.h"
#include "colors.h"
#include "vizualizer/objects_3d.h"
#include "algo_gui.h"
#include "algo_stat.h"
#include "dynspecviz.h"
#include "stat.h"

/* Tools for image output */
#ifdef PNG_OUT
#include "utilities/pngout_utils.h"
#endif

//Dozens of GLOBAL VARIABLES...

/* Variables containing model parameters */

unit_model_params_t ActualUnitParams;   /* Model of a single unit */
unit_model_params_t *ActualUnitParamSets;
sit_parameters_t ActualSitParams;       /* Situation parameters e.g. number of units */
flocking_model_params_t ActualFlockingParams;   /* Flocking model parameters */
flocking_model_params_t *ActualFlockingParamSets;

/* Variables containing phasespace data */

#define STORED_TIME (20.0+ActualSitParams.LengthToStore)        // Stored time (in seconds). (Cannot be reduced under 20 s ...)

/* Real data (delayed and actual) */
phase_t ActualPhase;

/* Real data (full timeline) */
phase_t *PhaseData;
phase_t *InnerStatesTimeLine;

/* Transformation terms of inner noises (delayed and actual) */
phase_t GPSPhase;
phase_t GPSDelayedPhase;

/* For taking Wind into account */
static double WindVelocityVector[2];

// Temporary...
static bool HighRes = true;

/* Variables for handling map display */

/* Visualisation mode */
#define EYE_Z_LIMIT 2000.0      // Minimum value of Z component of camera position
#define DISTANCE_FROM_CENTER_LIMIT 12000        // Minimum value of distance from center
#define DISTANCE_FROM_CENTER_XY_LIMIT 4000      // Minimum distance from center on XY plane

int MenuWindowID, VizWindowID;  /* IDs of windows */
color_config_t ActualColorConfig;       /* contains color setups */
vizmode_params_t ActualVizParams;       /* contains mode setup (e.g. CoM-following toggle, etc.) */
stat_utils_t ActualStatUtils;   /* for statistical calculations... */
output_modes_t ActualSaveModes;
statistics_t ActualStatistics;  /* contains averages and standard deviations of basic "order"parameters */

/* Other variables ... */
int Now = 0;                    // Actual visualized timestep (resets after "Length" seconds)
int TimeStep = 0;               // Actual visualized timestep (resets after F12 is pressed)
double TimeBeforeFlock = 0.0;   // Waiting time (before a simulated measurement)
bool ConditionsReset[2];
bool TrajViz = false;
bool *AgentsInDanger;
int Collisions;

int NumberOfModelSpecificColors;
model_specific_color_t ModelSpecificColors[MAX_NUMBER_OF_COLORS];

/* Output files */
FILE *f_OutPhase, *f_OutInnerStates;
bool PNGOutVid = false;

/* Functions for Initializing, displaying and refreshing windows */

/* Setting up Initial conditions */
void Initialize() {

    /* Initializing vizualization mode
     * CoM-following off, 2D visualization on,
     * Agent-following off.
     */
    ActualVizParams.CoMFollowing = false;
    ActualVizParams.TwoDimViz = true;
    ActualVizParams.DisplayGPSGhosts = false;
    ActualVizParams.WhichAgentIsSelected = ActualSitParams.NumberOfAgents;
    ActualVizParams.UnitParamsDisplayed = true;
    ActualVizParams.CamTraj = false;

    RefreshFlockingParams(&ActualFlockingParams);

    /* Randomized Initial positions */
    InitCond(&PhaseData, ActualSitParams.InitialX, ActualSitParams.InitialY,
            ActualSitParams.InitialZ, ActualSitParams.Radius);

    int i, j;

    for (i = 0; i < ActualSitParams.NumberOfAgents; i++) {
        for (j = 0; j < 3; j++) {
            ActualPhase.Velocities[i][j] = PhaseData[0].Velocities[i][j];
            ActualPhase.Coordinates[i][j] = PhaseData[0].Coordinates[i][j];
        }
    }

    InitializePhase(&ActualPhase, &ActualFlockingParams, &ActualSitParams);

    /* Waiting... */
    Wait(PhaseData, 5.0 + ActualUnitParams.t_del.Value, ActualSitParams.DeltaT);
    Now += (int) round((5.0 +
                    ActualUnitParams.t_del.Value) / ActualSitParams.DeltaT);
    TimeBeforeFlock = 10.0 + ActualUnitParams.t_del.Value;

    /* Initializing "condition reset" variables.
     * These are necessary for setting up random positions during a simulated experiment. */
    ConditionsReset[0] = true;
    ConditionsReset[1] = true;

    /* Initializing visualization speed-up */
    ActualVizParams.Paused = true;
    ActualVizParams.VizSpeedUp = 10;

    /* Setting up map and camera properties */
    ActualVizParams.CenterX = 0.0;
    ActualVizParams.CenterY = 0.0;
    ActualVizParams.CenterZ = 0.0;
    ActualVizParams.EyeX = ActualSitParams.InitialX + 7000.0;
    ActualVizParams.EyeY = ActualSitParams.InitialY + 7000.0;
    ActualVizParams.EyeZ = ActualSitParams.InitialZ + 7000.0;
    ActualVizParams.UpZ = 1.0;
    ActualVizParams.UpY = 0.0;
    ActualVizParams.UpX = 0.0;

    /* Communication network display is OFF */
    ActualVizParams.DisplayCommNetwork = false;

    /* Tail display is OFF. */
    ActualVizParams.DisplayTail = false;
    ActualVizParams.LengthOfTail = 500;

    /* Synchronizing visualization speed */
    ActualVizParams.VizSpeedUp = ActualSitParams.VizSpeedUp;

    /* Setting up limit values */
    ActualVizParams.DistanceFromCenterLimit = DISTANCE_FROM_CENTER_LIMIT;
    ActualVizParams.DistanceFromCenterLimit_XY = DISTANCE_FROM_CENTER_XY_LIMIT;
    ActualVizParams.EyeZLimit = EYE_Z_LIMIT;
    ActualVizParams.MapSizeXY =
            MAX(ActualSitParams.InitialX, ActualSitParams.InitialY);

    /* Setting up resolution */
    //TODO: Read it from file!
    ActualVizParams.Resolution = (true == HighRes ? 750 : 600);

    /* Calling model-specific vizmode Initialization */
    InitializeVizParams(&ActualVizParams);

    /* Match the steady state timestamps in the 2 corresponding structures. */
    ActualStatUtils.StartOfSteadyState = ActualSitParams.StartOfSteadyState;

}

/* Displaying "menu" window */
char VizSpeedUpName[19] = "Visualization Speed";
void DisplayMenu() {

    static char VizSpeedUpValue[10];

    /* Clearing before drawing the new scene */
    glClearColor(ActualColorConfig.EraseColor[0],
            ActualColorConfig.EraseColor[1], ActualColorConfig.EraseColor[2],
            1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (ActualVizParams.UnitParamsDisplayed == true) {
        DrawMenuItemsOfUnitModel(&ActualUnitParams,
                ActualColorConfig.MenuItemColor);
    } else if (ActualVizParams.FlockingParamsDisplayed == true) {
        DrawMenuItemsOfFlockingModel(&ActualFlockingParams,
                ActualColorConfig.MenuItemColor);
    }

    /* Drawing menu item about visualisation speed to the bottom of the window */
    sprintf(VizSpeedUpValue, "%d", ActualVizParams.VizSpeedUp);
    DrawMenuItem('s', VizSpeedUpName, "", VizSpeedUpValue, -0.95,
            ActualColorConfig.MenuItemColor);

    /* Drawing menu selection (two triangles) */
    //HEIGHT_OF_MENU_ITEM is defined in objects_menu.h
    if (ActualVizParams.WhichParamIsSelected == 0) {
        DrawMenuSelection(-0.95, -0.97 + HEIGHT_OF_MENU_ITEM * 0.5, 0.05,
                ActualColorConfig.MenuSelectionColor);
    } else {
        DrawMenuSelection(-0.95,
                HEIGHT_OF_MENU_ITEM * 0.5 + 0.9 -
                ActualVizParams.WhichParamIsSelected * HEIGHT_OF_MENU_ITEM,
                0.05, ActualColorConfig.MenuSelectionColor);
    }

    /* End of drawing */
    glutSwapBuffers();

}

/* Displaying quadcopters */
void DrawCopters(phase_t * Phase, phase_t * GPSPhase, const int TimeStep) {

    static double AgentsCoordinates[3];
    static double AgentsCoordinates_Temp[3];
    static double AgentsGPSCoordinates[3];
    static double AgentsVelocity[3];
    int i, h, z;

    static float GhostColor[3];
    static float TempColor[3];
    static char VelocityLabel[10];

    static double LengthOfAxis = 0.0;
    static double TicDensity = 0.0;
    static int HowManyTics = 0;

    /* 2D viz mode */
    if (ActualVizParams.TwoDimViz == true) {

        glDisable(GL_DEPTH_TEST);

        DrawModelSpecificObjects_2D(Phase, &ActualFlockingParams,
                &ActualUnitParams, &ActualVizParams, &ActualColorConfig,
                &ActualSitParams);

        /* Draw Tails */
        if (true == ActualVizParams.DisplayTail) {
            for (i = 0; i < Phase->NumberOfAgents; i++) {
                if (ActualVizParams.LengthOfTail < Now) {
                    for (h = 1; h < ActualVizParams.LengthOfTail; h += 10) {

                        GetAgentsCoordinatesFromTimeLine(AgentsCoordinates_Temp,
                                PhaseData, i, Now - h);

                        /* Create fading color */
                        for (z = 0; z < 3; z++) {
                            TempColor[z] = ActualColorConfig.AgentsColor[i][z];
                            TempColor[z] -=
                                    h *
                                    fabs(ActualColorConfig.AgentsColor[i][z] -
                                    ActualColorConfig.EraseColor[z]) /
                                    ActualVizParams.LengthOfTail;
                        }

                        DrawCopter_2D(AgentsCoordinates_Temp[0] -
                                ActualVizParams.CenterX,
                                AgentsCoordinates_Temp[1] -
                                ActualVizParams.CenterY,
                                ActualVizParams.MapSizeXY,
                                ActualSitParams.Radius, TempColor);
                    }
                }
            }
        }

        /* GPS Ghosts are the positions transformed by the amount of GPS inaccuracy */
        /* Color of the ghosts are "halfway" between the background color and the color of the agents */
        if (ActualVizParams.DisplayGPSGhosts == true) {

            for (i = 0; i < Phase->NumberOfAgents; i++) {

                GetAgentsCoordinates(AgentsGPSCoordinates, GPSPhase, i);
                GetAgentsCoordinates(AgentsCoordinates, Phase, i);

                /* Color of the ghosts is halfway between the color of the background and the color of the actual agent */
                GhostColor[0] =
                        fabs(ActualColorConfig.AgentsColor[i][0] -
                        ActualColorConfig.EraseColor[0]) * 0.5 +
                        ActualColorConfig.EraseColor[0];
                GhostColor[1] =
                        fabs(ActualColorConfig.AgentsColor[i][1] -
                        ActualColorConfig.EraseColor[1]) * 0.5 +
                        ActualColorConfig.EraseColor[1];
                GhostColor[2] =
                        fabs(ActualColorConfig.AgentsColor[i][2] -
                        ActualColorConfig.EraseColor[2]) * 0.5 +
                        ActualColorConfig.EraseColor[2];

                /* Draw GPS Ghosts */
                DrawCopter_2D(AgentsCoordinates[0] + AgentsGPSCoordinates[0] -
                        ActualVizParams.CenterX,
                        AgentsCoordinates[1] + AgentsGPSCoordinates[1] -
                        ActualVizParams.CenterY, ActualVizParams.MapSizeXY,
                        ActualSitParams.Radius, GhostColor);

                /* Draw Agents */
                if (AgentsInDanger[i] == true) {
                    DrawCopter_2D(AgentsCoordinates[0] -
                            ActualVizParams.CenterX,
                            AgentsCoordinates[1] - ActualVizParams.CenterY,
                            ActualVizParams.MapSizeXY, ActualSitParams.Radius,
                            ActualColorConfig.AgentsInDangerColor);
                } else {
                    DrawCopter_2D(AgentsCoordinates[0] -
                            ActualVizParams.CenterX,
                            AgentsCoordinates[1] - ActualVizParams.CenterY,
                            ActualVizParams.MapSizeXY, ActualSitParams.Radius,
                            ActualColorConfig.AgentsColor[i]);
                }

            }

        } else {

            for (i = 0; i < Phase->NumberOfAgents; i++) {

                GetAgentsCoordinates(AgentsCoordinates, Phase, i);

                /* Draw Agents */
                if (AgentsInDanger[i] == true) {
                    DrawCopter_2D(AgentsCoordinates[0] -
                            ActualVizParams.CenterX,
                            AgentsCoordinates[1] - ActualVizParams.CenterY,
                            ActualVizParams.MapSizeXY, ActualSitParams.Radius,
                            ActualColorConfig.AgentsInDangerColor);
                } else {
                    DrawCopter_2D(AgentsCoordinates[0] -
                            ActualVizParams.CenterX,
                            AgentsCoordinates[1] - ActualVizParams.CenterY,
                            ActualVizParams.MapSizeXY, ActualSitParams.Radius,
                            ActualColorConfig.AgentsColor[i]);
                }

            }

        }

        /* Draw Velocity Arrows */
        for (i = 0; i < Phase->NumberOfAgents; i++) {

            GetAgentsVelocity(AgentsVelocity, Phase, i);
            GetAgentsCoordinates(AgentsCoordinates, Phase, i);

            DrawVelocityArrow_2D(AgentsCoordinates[0] - ActualVizParams.CenterX,
                    AgentsCoordinates[1] - ActualVizParams.CenterY,
                    AgentsVelocity[0], AgentsVelocity[1],
                    ActualVizParams.MapSizeXY,
                    ActualColorConfig.VelocityArrowColor);

            /* Draw Velocity Labels */
            if (ActualVizParams.MapSizeXY < 3000.0) {

                /* If we are zoomed colse to the agents... */
                sprintf(VelocityLabel, "%1.1f m/s",
                        VectAbs(AgentsVelocity) / 100.0);

                DrawString(RealToGlCoord_2D(AgentsCoordinates[0] -
                                ActualVizParams.CenterX,
                                ActualVizParams.MapSizeXY),
                        RealToGlCoord_2D(AgentsCoordinates[1] -
                                ActualVizParams.CenterY + 60.0,
                                ActualVizParams.MapSizeXY),
                        GLUT_BITMAP_TIMES_ROMAN_10, VelocityLabel,
                        ActualColorConfig.VelocityLabelColor);

            }

        }

        /* Drawing wind speed */
        if (ActualUnitParams.Wind_Magn_Avg.Value > 0.0) {
            DrawArrow(0.85,
                    -0.9,
                    0.01 * pow(-50.0 +
                            sqrt(ActualUnitParams.Wind_Magn_Avg.Value *
                                    WindVelocityVector[0] *
                                    ActualUnitParams.Wind_Magn_Avg.Value *
                                    WindVelocityVector[0] +
                                    ActualUnitParams.Wind_Magn_Avg.Value *
                                    WindVelocityVector[1] *
                                    ActualUnitParams.Wind_Magn_Avg.Value *
                                    WindVelocityVector[1]), 1.0 / 3.0),
                    atan2(-ActualUnitParams.Wind_Magn_Avg.Value *
                            WindVelocityVector[1],
                            ActualUnitParams.Wind_Magn_Avg.Value *
                            WindVelocityVector[0]),
                    ActualColorConfig.VelocityArrowColor);
            static char WindLabel[10];
            sprintf(WindLabel, "Wind Speed: %1.1f m/s",
                    0.01 * hypot(ActualUnitParams.Wind_Magn_Avg.Value *
                            WindVelocityVector[0],
                            ActualUnitParams.Wind_Magn_Avg.Value *
                            WindVelocityVector[1]));
            DrawString(0.55, -0.9, GLUT_BITMAP_TIMES_ROMAN_10, WindLabel,
                    ActualColorConfig.VelocityArrowColor);

        }

        /* Drawing communication network, if it's toggled on */
        if (ActualVizParams.DisplayCommNetwork == true) {
            for (i = 0; i < Phase->NumberOfAgents; i++) {
                DrawSensorRangeNetwork_2D(PhaseData, ActualUnitParams.R_C.Value,
                        i,
                        ActualUnitParams.t_del.Value,
                        Now,
                        ActualSitParams.DeltaT,
                        &ActualVizParams, ActualColorConfig.CommNetWorkColor);
            }
        }

        /* 3D viz mode */
    } else {

        glEnable(GL_DEPTH_TEST);

        /* Setting up camera position and clamping */
        glFrustum(-0.5, 0.5, -0.5, 0.5, 1.0, 25.0);
        gluLookAt(RealToGlCoord_3D(ActualVizParams.EyeX,
                        ActualVizParams.MapSizeXY),
                RealToGlCoord_3D(ActualVizParams.EyeY,
                        ActualVizParams.MapSizeXY),
                RealToGlCoord_3D(ActualVizParams.EyeZ,
                        ActualVizParams.MapSizeXY),
                RealToGlCoord_3D(ActualVizParams.CenterX,
                        ActualVizParams.MapSizeXY),
                RealToGlCoord_3D(ActualVizParams.CenterY,
                        ActualVizParams.MapSizeXY),
                RealToGlCoord_3D(ActualVizParams.CenterZ,
                        ActualVizParams.MapSizeXY), ActualVizParams.UpX,
                ActualVizParams.UpY, ActualVizParams.UpZ);

        /* Setting up size of the coordinate system */
        LengthOfAxis = RealToGlCoord_3D(100000.0, ActualVizParams.MapSizeXY);
        TicDensity = RealToGlCoord_3D(2000.0, ActualVizParams.MapSizeXY);
        HowManyTics = (int) LengthOfAxis / TicDensity;
        HowManyTics = (int) HowManyTics *1.5;

        /* Drawing ground */

        glColor3f(ActualColorConfig.AxisColor[0],
                ActualColorConfig.AxisColor[1], ActualColorConfig.AxisColor[2]);
        glBegin(GL_LINES);
        for (i = -HowManyTics; i < HowManyTics; i++) {

            glVertex3d(i * TicDensity, -HowManyTics * TicDensity,
                    RealToGlCoord_3D(-1000.0, ActualVizParams.MapSizeXY));
            glVertex3d(i * TicDensity, LengthOfAxis, RealToGlCoord_3D(-1000.0,
                            ActualVizParams.MapSizeXY));

        }
        for (i = -HowManyTics; i < HowManyTics; i++) {

            glVertex3d(-HowManyTics * TicDensity, i * TicDensity,
                    RealToGlCoord_3D(-1000.0, ActualVizParams.MapSizeXY));
            glVertex3d(LengthOfAxis, i * TicDensity, RealToGlCoord_3D(-1000.0,
                            ActualVizParams.MapSizeXY));

        }
        glEnd();

        /* Drawing agents */
        for (i = 0; i < Phase->NumberOfAgents; i++) {
            GetAgentsCoordinates(AgentsCoordinates, Phase, i);

            DrawCopter_3D(AgentsCoordinates[0],
                    AgentsCoordinates[1],
                    AgentsCoordinates[2],
                    ActualVizParams.MapSizeXY,
                    ActualSitParams.Radius, ActualColorConfig.AgentsColor[i]);

            /* Trajectories */
            if (true == ActualVizParams.DisplayTail) {

                glEnable(GL_BLEND);
                glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

                glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
                glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);

                if (ActualVizParams.LengthOfTail < Now) {
                    for (h = 1; h < ActualVizParams.LengthOfTail; h += 10) {

                        GetAgentsCoordinatesFromTimeLine(AgentsCoordinates,
                                PhaseData, i, Now - h);

                        /* Create fading color */
                        for (z = 0; z < 3; z++) {
                            TempColor[z] = ActualColorConfig.AgentsColor[i][z];
                            TempColor[z] -=
                                    h *
                                    fabs(ActualColorConfig.AgentsColor[i][z] -
                                    ActualColorConfig.EraseColor[z]) /
                                    ActualVizParams.LengthOfTail;
                        }

                        DrawCopter_3D(AgentsCoordinates[0],
                                AgentsCoordinates[1],
                                AgentsCoordinates[2],
                                ActualVizParams.MapSizeXY,
                                ActualSitParams.Radius, TempColor);
                    }
                }

                glDisable(GL_BLEND);

            }

        }

        /* Drawing communication network, if it's toggled on */
        if (ActualVizParams.DisplayCommNetwork == true) {
            for (i = 0; i < Phase->NumberOfAgents; i++) {
                DrawSensorRangeNetwork_3D(PhaseData, ActualUnitParams.R_C.Value,
                        i,
                        ActualUnitParams.t_del.Value,
                        Now,
                        ActualSitParams.DeltaT,
                        ActualVizParams.MapSizeXY,
                        ActualColorConfig.CommNetWorkColor);
            }
        }

        /* Model-specific objects */
        DrawModelSpecificObjects_3D(Phase, &ActualFlockingParams,
                &ActualUnitParams, &ActualVizParams, &ActualColorConfig,
                &ActualSitParams);

    }

}

/* Refreshing menu system */
void UpdateMenu() {

    /* Refresh both windows */
    glutSetWindow(MenuWindowID);
    glutPostRedisplay();
    glutSetWindow(VizWindowID);
    glutPostRedisplay();

}

/* Displaying "trajectory" window */
void DisplayTrajs() {

    static char CollisionsToWriteOut[18];
    static char TimeToWriteOut[40];
    static const char PausedCaption[6] = "Paused";
    static int LastTimeStep = 0;
    static struct timeval LastTime = { 0, 0 };
    struct timeval NowTime;
    static float RealSpeed;

    /* Clearing */
    glClearColor(ActualColorConfig.EraseColor[0],
            ActualColorConfig.EraseColor[1], ActualColorConfig.EraseColor[2],
            1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glLoadIdentity();

    /* Drawing Copters */
    if (Now > (5.0 + ActualUnitParams.t_del.Value) / ActualSitParams.DeltaT) {
        DrawCopters(&ActualPhase, &GPSPhase, Now);
    }

    /* Writing out number of collisions */
    if (Collisions != 0 && ActualVizParams.TwoDimViz == true) {

        /* Grammar... */
        if (Collisions != 1) {
            sprintf(CollisionsToWriteOut, "%d collisions!", Collisions);
        } else {
            sprintf(CollisionsToWriteOut, "%d collision!", Collisions);
        }

        DrawString(-0.95, -0.9, GLUT_BITMAP_9_BY_15, CollisionsToWriteOut,
                ActualColorConfig.MenuSelectionColor);

    }
    /* Calculate real visualization speed */
    gettimeofday(&NowTime, 0);
    RealSpeed =
            RealSpeed * 0.9 + 0.1 * (TimeStep -
            LastTimeStep) * ActualSitParams.DeltaT / (NowTime.tv_sec -
            LastTime.tv_sec + (NowTime.tv_usec - LastTime.tv_usec) / 1e6);
    LastTime = NowTime;
    LastTimeStep = TimeStep;

    /* Writing out Elapsed Time */
    if (true == PNGOutVid) {
        sprintf(TimeToWriteOut, "Elapsed time: %1.1f s",
                TimeStep * ActualSitParams.DeltaT);
    } else {
        sprintf(TimeToWriteOut, "Elapsed time: %1.1f s   Speed: %1.2f x",
                TimeStep * ActualSitParams.DeltaT, RealSpeed);
    }
    if (ActualVizParams.TwoDimViz == true) {
        DrawString(-0.95, -0.95, GLUT_BITMAP_9_BY_15, TimeToWriteOut,
                ActualColorConfig.MenuSelectionColor);
    }

    /* Write out model specific string */
    if (ActualVizParams.TwoDimViz == true) {
        DrawString(-0.95, 0.81, GLUT_BITMAP_9_BY_15,
                GetModelSpecificStringToDisplay(&ActualPhase,
                        &ActualFlockingParams, &ActualUnitParams,
                        &ActualSitParams, TimeStep),
                ActualColorConfig.MenuSelectionColor);
    }

    /* Information: is CoM-following mode on? */
    if (ActualVizParams.CoMFollowing == true
            && ActualVizParams.TwoDimViz == true) {
        DrawString(0.4, 0.95, GLUT_BITMAP_9_BY_15, "CoM-Following On",
                ActualColorConfig.MenuSelectionColor);
    }
    /* Is Agent-following mode on? */
    else if (ActualVizParams.WhichAgentIsSelected !=
            ActualSitParams.NumberOfAgents
            && ActualVizParams.TwoDimViz == true) {
        static char FollowingLabel[20];
        sprintf(FollowingLabel, "Following agent %d",
                ActualVizParams.WhichAgentIsSelected);
        DrawString(0.4, 0.95, GLUT_BITMAP_9_BY_15, FollowingLabel,
                ActualColorConfig.MenuSelectionColor);
    }

    /* Drawing Length scale */
    if (ActualVizParams.TwoDimViz == true) {

        /* Communication range */
        static double TempColor[3];
        FillVect(TempColor,
                0.5 * ActualColorConfig.MenuSelectionColor[0],
                0.5 * ActualColorConfig.MenuSelectionColor[1],
                0.5 * ActualColorConfig.MenuSelectionColor[2]);
        glColor3f(TempColor[0], TempColor[1], TempColor[2]);
        glBegin(GL_LINES);
        glVertex3f(-0.95, 0.925, 1.0);
        glVertex3f(-0.95 + RealToGlCoord_2D(ActualUnitParams.R_C.Value,
                        ActualVizParams.MapSizeXY), 0.925, 1.0);
        glEnd();
        glBegin(GL_LINES);
        glVertex3f(-0.95 + RealToGlCoord_2D(ActualUnitParams.R_C.Value,
                        ActualVizParams.MapSizeXY), 0.9, 1.0);
        glVertex3f(-0.95 + RealToGlCoord_2D(ActualUnitParams.R_C.Value,
                        ActualVizParams.MapSizeXY), 0.95, 1.0);
        glEnd();

        static char R_C_Label[20];
        sprintf(R_C_Label, "%1.0f m", ActualUnitParams.R_C.Value / 100.0);
        DrawString(-0.945 + RealToGlCoord_2D(ActualUnitParams.R_C.Value,
                        ActualVizParams.MapSizeXY), 0.875,
                GLUT_BITMAP_TIMES_ROMAN_10, R_C_Label,
                ActualColorConfig.MenuSelectionColor);

        /* Length scale */
        DrawScale(ActualVizParams.MapSizeXY / 2.5, -0.95, 0.95,
                ActualVizParams.MapSizeXY,
                ActualColorConfig.MenuSelectionColor);

    }

    if (ActualVizParams.Paused == true && ActualVizParams.TwoDimViz == true) {
        DrawString(-0.2, 0, GLUT_BITMAP_TIMES_ROMAN_24, PausedCaption,
                ActualColorConfig.PausedCaptionColor);
    }

    /* Saving screenshot */
#ifdef PNG_OUT
    static char PNGOutFileName[512];
    static int k_1 = 0;
    static int k_2 = 0;
    if (true == PNGOutPic) {

        sprintf(PNGOutFileName, "%s/screenshots/screenshot_%d.png",
                ActualStatUtils.OutputDirectory, k_1++);
        TakeScreenshot(PNGOutFileName);
        PNGOutPic = false;

        /* Saving png video frames */
    } else if (true == PNGOutVid) {

        sprintf(PNGOutFileName, "%s/frames/vid%d/frame_%d.png",
                ActualStatUtils.OutputDirectory, 1000 + VidNum, 1000000 + k_2);
        TakeScreenshot(PNGOutFileName);
        k_2++;

    } else if (false == PNGOutVid) {

        k_2 = 0;

    }
#endif

    /* End of drawing */
    glutSwapBuffers();

}

/* Refreshing "trajectory" window */
void UpdatePositionsToDisplay() {

    int i;

    static int TimeStepsToStore = 0;
    /* For agent-following and CoM-following mode */
    static double CoMCoords[3];
    NullVect(CoMCoords, 3);
    static double AgentsCoords[3];
    TimeStepsToStore = (int) (((STORED_TIME) / ActualSitParams.DeltaT) - 1.0);

    /* Opening output files, if necessary */
    static bool FilesOpened = false;
    static int n = 0;           /* Which File? */
    static char FileName[512];
    if (true == PNGOutVid && false == FilesOpened) {
        static char tempchar[128];
        if (true == ActualSaveModes.SaveTrajectories) {
            strcpy(FileName, ActualStatUtils.OutputDirectory);
            sprintf(tempchar, "/posandvel_%d.dat", n);
            strcat(FileName, tempchar);
            f_OutPhase = fopen(FileName, "w");
            fprintf(f_OutPhase,
                    "\n# Output format:\n\n# (column)     1        2   3   4   5    6    7       8   9  10   11   12   13    ... etc ... \n# (data)    time_(s), (x_1 y_1 z_2 vx_1 vy_1 vz_1), (x_2 y_2 z_2 vx_2 vy_2 vz_2), ... etc ...\n\n# Position values are in cm, velocity values are in cm/s\n\n");
        }
        if (true == ActualSaveModes.SaveInnerStates) {
            strcpy(FileName, ActualStatUtils.OutputDirectory);
            sprintf(tempchar, "/innerstates_%d.dat", ++n);
            strcat(FileName, tempchar);
            f_OutInnerStates = fopen(FileName, "w");
        }
        FilesOpened = true;
    }

    glutSetWindow(VizWindowID);

    double *Accelerations;
    Accelerations = malloc(ActualSitParams.NumberOfAgents * sizeof(double));

    if (ActualVizParams.Paused == false) {

        /* The "ActualVizParams.VizSpeedUp" variable defines
         * the number of calculated steps between two "frames"
         */
        for (i = 0; i < ActualVizParams.VizSpeedUp; i++) {

            if (Now < TimeStepsToStore - 1) {

                /* Calculating 1 step with the robot model */
                Step(&ActualPhase, &GPSPhase, &GPSDelayedPhase,
                        PhaseData, &ActualUnitParams, &ActualFlockingParams,
                        &ActualSitParams, &ActualVizParams, Now, TimeStep,
                        true, ConditionsReset, &Collisions, AgentsInDanger,
                        WindVelocityVector, Accelerations);

                HandleOuterVariables(&ActualPhase, &ActualVizParams,
                        &ActualSitParams, &ActualUnitParams,
                        TimeStep * ActualSitParams.DeltaT,
                        ActualStatUtils.OutputDirectory);

                /* Inserting output phase of the "step" function into the
                 * globally-allocated PhaseData and InnerStatesTimeLine
                 */
                InsertPhaseToDataLine(PhaseData, &ActualPhase, Now + 1);
                InsertInnerStatesToDataLine(PhaseData, &ActualPhase, Now + 1);

            } else {

                /* Shifting Data line, if PhaseData is overloaded */
                ShiftDataLine(PhaseData,
                        (int) ((STORED_TIME / ActualSitParams.DeltaT) - 1.0),
                        (int) (20.0 / ActualSitParams.DeltaT));
                ShiftInnerStateDataLine(PhaseData,
                        (int) ((STORED_TIME / ActualSitParams.DeltaT) - 1.0),
                        (int) (20.0 / ActualSitParams.DeltaT));
                Step(&ActualPhase, &GPSPhase, &GPSDelayedPhase, PhaseData,
                        &ActualUnitParams, &ActualFlockingParams,
                        &ActualSitParams, &ActualVizParams, Now, TimeStep, true,
                        ConditionsReset, &Collisions, AgentsInDanger,
                        WindVelocityVector, Accelerations);

                HandleOuterVariables(&ActualPhase, &ActualVizParams,
                        &ActualSitParams, &ActualUnitParams,
                        TimeStep * ActualSitParams.DeltaT,
                        ActualStatUtils.OutputDirectory);

                /* Inserting output phase of the "step" function into the
                 * globally-allocated PhaseData and InnerStatesTimeLine
                 */
                Now = (int) ((20.0 / ActualSitParams.DeltaT) - 1.0);

                InsertPhaseToDataLine(PhaseData, &ActualPhase, Now + 1);
                InsertInnerStatesToDataLine(PhaseData, &ActualPhase, Now + 1);

            }

            if (true == PNGOutVid) {

                /* Saving trajectories */
                WriteOutTrajectories(&ActualPhase,
                        ActualSaveModes.SaveTrajectories,
                        ActualSaveModes.SaveInnerStates,
                        TimeStep * ActualSitParams.DeltaT, f_OutPhase,
                        f_OutInnerStates);

            }

            Now++;
            TimeStep++;

        }

    }

    /* Closing output trajectory and inner state files */
    if (false == PNGOutVid && true == FilesOpened) {
        if (true == ActualSaveModes.SaveTrajectories) {
            fclose(f_OutPhase);
        }
        if (true == ActualSaveModes.SaveInnerStates) {
            fclose(f_OutInnerStates);
        }
        FilesOpened = false;
    }

    /* CoM following mode, if it is toggled on */
    if (ActualVizParams.CoMFollowing == true) {
        GetCoM(CoMCoords, &ActualPhase);
        ActualVizParams.CenterX = CoMCoords[0];
        ActualVizParams.CenterY = CoMCoords[1];
        ActualVizParams.CenterZ = CoMCoords[2];
        /* Agent-following mode, if it is toggled on */
    } else if (ActualVizParams.WhichAgentIsSelected !=
            ActualSitParams.NumberOfAgents) {
        GetAgentsCoordinates(AgentsCoords, &ActualPhase,
                ActualVizParams.WhichAgentIsSelected);
        ActualVizParams.CenterX = AgentsCoords[0];
        ActualVizParams.CenterY = AgentsCoords[1];
        ActualVizParams.CenterZ = AgentsCoords[2];
    }

    /* Refresh both windows */
    glutSetWindow(MenuWindowID);
    glutPostRedisplay();
    glutSetWindow(VizWindowID);
    glutPostRedisplay();

}

/* Functions for handling keyboard and mouse events */

/* Keyboard function, non-special keys */
void HandleKeyBoard(unsigned char key, int x, int y) {

    static int index = 0;
    static int Modder = 0;
    index = ActualVizParams.WhichParamIsSelected;

    Modder = glutGetModifiers();

    /* Backspace brings simulation back a bit */
    if (key == 8) {
        int back = MIN(MIN(Now, TimeStep), ActualVizParams.LengthOfTail);
        Now -= back;
        TimeStep -= back;
        UpdateMenu();
        /* Space pauses/unpauses the simulation */
    } else if (key == ' ') {
        ActualVizParams.Paused = !ActualVizParams.Paused;
    }
    /* 'v' toggles tail visualizaton in 2D and 3D. */
    else if (key == 'v') {
        //if (ActualSitParams.Length == ActualSitParams.LengthToStore) {
        ActualVizParams.DisplayTail = !ActualVizParams.DisplayTail;
        //} else {
        //      fprintf (stderr, "Please, increase the stored simulation time to draw tails!\n");
        //}
    }
    /* 'p' pauses vizualization */
    else if (key == 'p') {

        if (true == ActualVizParams.UnitParamsDisplayed) {
            SelectedUnitParamSet =
                    (SelectedUnitParamSet + 1) % (NumberOfUnitParamSets);
            ActualUnitParams = ActualUnitParamSets[SelectedUnitParamSet];
        } else if (true == ActualVizParams.FlockingParamsDisplayed) {
            SelectedFlockingParamSet =
                    (SelectedFlockingParamSet +
                    1) % (NumberOfFlockingParamSets);
            ActualFlockingParams =
                    ActualFlockingParamSets[SelectedFlockingParamSet];
        }

    }
    /* Displayed parameterset can be changed by pressing 'm' */
    else if (key == 'm') {
        ActualVizParams.UnitParamsDisplayed =
                !ActualVizParams.UnitParamsDisplayed;
        ActualVizParams.FlockingParamsDisplayed =
                !ActualVizParams.UnitParamsDisplayed;

        /* Avoiding too large values for parameter selection index. */
        if (ActualVizParams.UnitParamsDisplayed == true
                && ActualVizParams.WhichParamIsSelected > 12) {

            ActualVizParams.WhichParamIsSelected = 12;

        } else if (ActualVizParams.FlockingParamsDisplayed == true
                && ActualVizParams.WhichParamIsSelected >
                ActualFlockingParams.NumberOfParameters) {

            ActualVizParams.WhichParamIsSelected =
                    ActualFlockingParams.NumberOfParameters;

        }

        /* Key 's' means that the selected parameter is the visualization speed (=0) */
    } else if (key == 's') {
        ActualVizParams.WhichParamIsSelected = 0;
    } else if (index == 0 && key == '+') {
        /* Ctrl+ is zoom in */
        if (Modder == GLUT_ACTIVE_CTRL) {
            /* 2D visualization mode functions */
            if (ActualVizParams.TwoDimViz == true) {
                /* Mapsize cannot be negative! */
                if (ActualVizParams.MapSizeXY - 1000 > 0)
                    ActualVizParams.MapSizeXY -= 1000;
            } else {
                ZoomOnCenter(&ActualVizParams, -1000.0);
            }
        } else {
            if (ActualVizParams.VizSpeedUp < 99) {
                ActualVizParams.VizSpeedUp += 1;
            } else {
                ActualVizParams.VizSpeedUp = 100;
            }
        }
    } else if (index == 0 && key == '-') {
        /* Ctrl+ is zoom in */
        if (Modder == GLUT_ACTIVE_CTRL) {
            /* 2D visualization mode functions */
            if (ActualVizParams.TwoDimViz == true) {
                /* Mapsize cannot be negative! */
                ActualVizParams.MapSizeXY += 1000;
            } else {
                ZoomOnCenter(&ActualVizParams, 1000.0);
            }
        } else {
            if (ActualVizParams.VizSpeedUp > 1) {
                ActualVizParams.VizSpeedUp -= 1;
            } else {
                ActualVizParams.VizSpeedUp = 1;
            }
        }
        /* Changing robot model parameters */
    } else if (key == '+' && ActualVizParams.UnitParamsDisplayed == true) {

        ChangeUnitModelParameter(&ActualUnitParams, index, 1.0,
                ActualSitParams.DeltaT);

    } else if (key == '-' && ActualVizParams.UnitParamsDisplayed == true) {

        ChangeUnitModelParameter(&ActualUnitParams, index, -1.0,
                ActualSitParams.DeltaT);

        /* Changing flocking algorithm parameters */
    } else if (key == '+' && ActualVizParams.FlockingParamsDisplayed == true) {

        ChangeFlockingModelParameter(&ActualFlockingParams, index - 1,
                ActualFlockingParams.Params[-1 + index].SizeOfStep);

    } else if (key == '-' && ActualVizParams.FlockingParamsDisplayed == true) {

        ChangeFlockingModelParameter(&ActualFlockingParams, index - 1,
                -ActualFlockingParams.Params[-1 + index].SizeOfStep);

    } else {
        /* Generating integer from hexadecimal string */
        char key_temp[1];
        key_temp[0] = key;
        int keyint = strtol(key_temp, NULL, 16);

        if (ActualVizParams.UnitParamsDisplayed == true) {

            if (keyint <= 13 && keyint > 0) {
                ActualVizParams.WhichParamIsSelected = keyint;
            }

        } else if (ActualVizParams.FlockingParamsDisplayed == true) {

            if (keyint < ActualFlockingParams.NumberOfParameters + 1
                    && keyint > 0
                    && ActualFlockingParams.Params[keyint - 1].InMenu == true) {
                ActualVizParams.WhichParamIsSelected = keyint;
            }

        }

    }

    RefreshFlockingParams(&ActualFlockingParams);

    /* Handling model-specific keyboard events */
    HandleSpecialKeyBoardEvent(key, x, y, &ActualFlockingParams,
            &ActualVizParams, &ActualSitParams, Modder);

    /* Refreshing both windows */
    glutSetWindow(MenuWindowID);
    glutPostRedisplay();
    glutSetWindow(VizWindowID);
    glutPostRedisplay();

}

/* Keyboard function, special keys */
void HandleKeyBoardSpecial(int key, int x, int y) {

    int i;

    static int index;

    /* Tools for saving video frames */
    static char VidFileName[16];
    static struct stat st;
    //st = {0};

    /* Tools for... whatever... */
    static double Previous_MapSize = 2000.0;
    index = 0;
    static int Modder;
    static double EyeToCenter[3];
    static double Agent[3];
    NullVect(Agent, 3);

    Modder = glutGetModifiers();

    /* F2 toggles the CoM-Following mode */
    if (key == GLUT_KEY_F2) {
        ActualVizParams.CoMFollowing = !ActualVizParams.CoMFollowing;
        /* F3 toggles the 2D/3D visualization mode */
    } else if (key == GLUT_KEY_F3) {
        ActualVizParams.TwoDimViz = !ActualVizParams.TwoDimViz;

        /* Setting up Initial eye position */
        if (ActualVizParams.TwoDimViz == false) {
            GetAgentsCoordinates(Agent, &ActualPhase,
                    rand() % ActualSitParams.NumberOfAgents);
            Previous_MapSize = ActualVizParams.MapSizeXY;
            ActualVizParams.MapSizeXY = 5000.0;
            ActualVizParams.EyeX = Agent[0] + 10000.0;
            ActualVizParams.EyeY = Agent[1] + 10000.0;
            ActualVizParams.EyeZ = Agent[2] + 10000.0;
        } else {
            ActualVizParams.MapSizeXY = Previous_MapSize;
        }

        /* INSERT toggles displaying of GPS ghosts */
    } else if (key == GLUT_KEY_INSERT) {
        ActualVizParams.DisplayGPSGhosts = !ActualVizParams.DisplayGPSGhosts;
        /* F12 randomizes the PhaseSpace */
    } else if (key == GLUT_KEY_F12) {

        /* ALT + F12 - randomize positions in an area with the size initsizeX * initsizeY * initsizeZ */
        if (Modder == GLUT_ACTIVE_ALT) {
            ConditionsReset[0] = true;
        } else {
            ConditionsReset[1] = true;
        }

        for (i = 0; i < ActualFlockingParams.NumberOfParameters; i++) {
            if (ActualFlockingParams.Params[i].Constant == true) {
                ActualFlockingParams.Params[i].Value =
                        ActualFlockingParams.Params[i].StoredValue;
            }
        }

        /* Reset number of collisions and elapsed time */
        Collisions = 0;
        TimeStep = 0;

    } else if (key == GLUT_KEY_F11) {

        if (Modder == GLUT_ACTIVE_ALT) {
            PNGOutVid = !PNGOutVid;

#ifdef PNG_OUT
            if (true == PNGOutVid) {
                VidNum++;
                char *OutputFramesDir = (char *) calloc(270, sizeof(char));
                sprintf(VidFileName, "/frames/vid%d", VidNum + 1000);
                strcpy(OutputFramesDir, ActualStatUtils.OutputDirectory);
                strcat(OutputFramesDir, VidFileName);
                if (stat(OutputFramesDir, &st) == -1) {
                    mkdir(OutputFramesDir, 0700);
                }
                free(OutputFramesDir);
            }
#endif

        } else {

#ifdef PNG_OUT
            PNGOutPic = true;
#endif
            PNGOutVid = false;

        }

    } else if (key == GLUT_KEY_LEFT && ActualVizParams.CamTraj == false) {
        /* ALT + Left/Right changes selected agent */
        if (Modder == GLUT_ACTIVE_ALT
                && ActualVizParams.WhichAgentIsSelected != 0
                && ActualVizParams.CoMFollowing == false) {
            ActualVizParams.WhichAgentIsSelected -= 1;
        } else if (Modder == GLUT_ACTIVE_ALT
                && ActualVizParams.WhichAgentIsSelected == 0
                && ActualVizParams.CoMFollowing == false) {
            ActualVizParams.WhichAgentIsSelected =
                    ActualSitParams.NumberOfAgents;
        } else {
            if (ActualVizParams.TwoDimViz == true) {
                ActualVizParams.CenterX -= 1000.0;
            } else {
                FillVect(EyeToCenter,
                        ActualVizParams.CenterY - ActualVizParams.EyeY,
                        -ActualVizParams.CenterX + ActualVizParams.EyeX, 0.0);
                TranslateCameraOnXYPlane(&ActualVizParams, EyeToCenter,
                        -1000.0);
            }
        }
    } else if (key == GLUT_KEY_RIGHT && ActualVizParams.CamTraj == false) {
        if (Modder == GLUT_ACTIVE_ALT
                && ActualVizParams.WhichAgentIsSelected !=
                ActualSitParams.NumberOfAgents
                && ActualVizParams.CoMFollowing == false) {
            ActualVizParams.WhichAgentIsSelected += 1;
        } else if (Modder == GLUT_ACTIVE_ALT
                && ActualVizParams.WhichAgentIsSelected ==
                ActualSitParams.NumberOfAgents
                && ActualVizParams.CoMFollowing == false) {
            ActualVizParams.WhichAgentIsSelected = 0;
        } else {
            if (ActualVizParams.TwoDimViz == true) {
                ActualVizParams.CenterX += 1000.0;
            } else {
                FillVect(EyeToCenter,
                        ActualVizParams.CenterY - ActualVizParams.EyeY,
                        -ActualVizParams.CenterX + ActualVizParams.EyeX, 0.0);
                TranslateCameraOnXYPlane(&ActualVizParams, EyeToCenter, 1000.0);
            }
        }

        /* F7 toggles communication network display */
    } else if (key == GLUT_KEY_F7) {
        ActualVizParams.DisplayCommNetwork =
                !ActualVizParams.DisplayCommNetwork;
        /* F5 Saves the parameters */
    } else if (key == GLUT_KEY_F5) {

        /* Unit model */
        FILE *UnitParamsFile = fopen(ActualUnitParams.FileName, "w");
        SaveUnitModelParamsToFile(UnitParamsFile, &ActualUnitParams);

        /* Flocking model */
        FILE *FlockingParamsFile = fopen(ActualFlockingParams.FileName, "w");
        SaveFlockingModelParamsToFile(FlockingParamsFile,
                &ActualFlockingParams);

        fclose(FlockingParamsFile);
        fclose(UnitParamsFile);

    } else if (key == GLUT_KEY_UP && ActualVizParams.CamTraj == false) {
        if (ActualVizParams.TwoDimViz == true) {
            ActualVizParams.CenterY += 1000.0;
        } else {
            FillVect(EyeToCenter,
                    ActualVizParams.CenterX - ActualVizParams.EyeX,
                    ActualVizParams.CenterY - ActualVizParams.EyeY, 0.0);
            TranslateCameraOnXYPlane(&ActualVizParams, EyeToCenter, 1000.0);
        }
    } else if (key == GLUT_KEY_DOWN && ActualVizParams.CamTraj == false) {
        if (ActualVizParams.TwoDimViz == true) {
            ActualVizParams.CenterY -= 1000.0;
        } else {
            FillVect(EyeToCenter,
                    -ActualVizParams.CenterX + ActualVizParams.EyeX,
                    -ActualVizParams.CenterY + ActualVizParams.EyeY, 0.0);
            TranslateCameraOnXYPlane(&ActualVizParams, EyeToCenter, 1000.0);
        }
    } else if (ActualVizParams.WhichParamIsSelected == 0) {

        /* Changing viualization speed with different StepSizes */
        switch (key) {
        case GLUT_KEY_PAGE_UP:{
            if (ActualVizParams.VizSpeedUp < 90) {
                ActualVizParams.VizSpeedUp += 10;
            } else {
                ActualVizParams.VizSpeedUp = 100;
            }
            break;
        }
        case GLUT_KEY_PAGE_DOWN:{
            if (ActualVizParams.VizSpeedUp > 10) {
                ActualVizParams.VizSpeedUp -= 10;
            } else {
                ActualVizParams.VizSpeedUp = 1;
            }
            break;
        }
        case GLUT_KEY_HOME:
            ActualVizParams.VizSpeedUp = 100;
            break;
        case GLUT_KEY_END:
            ActualVizParams.VizSpeedUp = 1;
            break;
        }

    } else if (ActualVizParams.UnitParamsDisplayed == true) {

        /* Reset GPS trajectories, when some of the inner sensory parameters are changed */
        if (ActualVizParams.WhichParamIsSelected == 7
                || ActualVizParams.WhichParamIsSelected == 8) {
            ResetGPSNoises(&GPSPhase, &GPSDelayedPhase);
        }

        switch (key) {
        case GLUT_KEY_PAGE_UP:{
            ChangeUnitModelParameter(&ActualUnitParams,
                    ActualVizParams.WhichParamIsSelected, 10.0,
                    ActualSitParams.DeltaT);
        }
            break;
        case GLUT_KEY_PAGE_DOWN:{
            ChangeUnitModelParameter(&ActualUnitParams,
                    ActualVizParams.WhichParamIsSelected, -10.0,
                    ActualSitParams.DeltaT);
        }
            break;
        case GLUT_KEY_HOME:{
            ChangeUnitModelParameter(&ActualUnitParams,
                    ActualVizParams.WhichParamIsSelected, 100.0,
                    ActualSitParams.DeltaT);
        }
            break;
        case GLUT_KEY_END:{
            ChangeUnitModelParameter(&ActualUnitParams,
                    ActualVizParams.WhichParamIsSelected, -100.0,
                    ActualSitParams.DeltaT);
        }
            break;
        }

    } else if (ActualVizParams.FlockingParamsDisplayed == true) {

        index = -1 + ActualVizParams.WhichParamIsSelected;

        switch (key) {
        case GLUT_KEY_PAGE_UP:{
            ChangeFlockingModelParameter(&ActualFlockingParams, index,
                    10.0 * ActualFlockingParams.Params[index].SizeOfStep);
        }
            break;
        case GLUT_KEY_PAGE_DOWN:{
            ChangeFlockingModelParameter(&ActualFlockingParams, index,
                    -10.0 * ActualFlockingParams.Params[index].SizeOfStep);
        }
            break;
        case GLUT_KEY_HOME:{
            ChangeFlockingModelParameter(&ActualFlockingParams, index,
                    100.0 * ActualFlockingParams.Params[index].SizeOfStep);
        }
            break;
        case GLUT_KEY_END:{
            ChangeFlockingModelParameter(&ActualFlockingParams, index,
                    -100.0 * ActualFlockingParams.Params[index].SizeOfStep);
        }
            break;
        }

    }

    RefreshFlockingParams(&ActualFlockingParams);

    /* Handling model-specific keyboard events */
    HandleSpecialSpecKeyEvent(key, x, y, &ActualFlockingParams,
            &ActualVizParams, &ActualSitParams, Modder);

    /* Refreshing both windows */
    glutSetWindow(MenuWindowID);
    glutPostRedisplay();
    glutSetWindow(VizWindowID);
    glutPostRedisplay();

}

/* Mouse functions */
void HandleMouse(int button, int state, int x, int y) {

    static int Modder = 0;
    static double RotationAxis[3];
    Modder = glutGetModifiers();

    /* 2D visualization mode functions */
    if (ActualVizParams.TwoDimViz == true) {

        /* Scroll down - zoom in */
        if (button == 3) {
            if (state == GLUT_DOWN) {
                /* Mapsize cannot be negative! */
                if (ActualVizParams.MapSizeXY - 1000 > 0) {
                    ActualVizParams.MapSizeXY -= 1000;
                }
            }
        }

        /* Scroll up - zoom out */
        else if (button == 4) {
            if (state == GLUT_DOWN) {
                ActualVizParams.MapSizeXY += 1000;
            }
        }

        /* If CoM-Following mode is off, then right click moves the center of the map */
        else if (button == 2 && state == GLUT_DOWN
                && ActualVizParams.CoMFollowing == false) {
            ActualVizParams.WhichAgentIsSelected =
                    ActualSitParams.NumberOfAgents;
            ActualVizParams.CenterX +=
                    MouseCoordToReal_2D(x, ActualVizParams.MapSizeXY,
                    ActualVizParams.Resolution);
            ActualVizParams.CenterY +=
                    -MouseCoordToReal_2D(y, ActualVizParams.MapSizeXY,
                    ActualVizParams.Resolution);
        }

    } else {

        /* Scroll down - zoom in */
        if (button == 3 && state == GLUT_DOWN) {
            if (Modder == GLUT_ACTIVE_ALT) {
                FillVect(RotationAxis,
                        -ActualVizParams.EyeY + ActualVizParams.CenterY,
                        ActualVizParams.EyeX - ActualVizParams.CenterX, 0.0);
                RotateCameraAroundCenter(&ActualVizParams, RotationAxis, -0.2);
            } else if (Modder == GLUT_ACTIVE_SHIFT) {
                FillVect(RotationAxis, 0.0, 0.0, 1.0);
                RotateCameraAroundCenter(&ActualVizParams, RotationAxis, -0.2);
            } else {
                ZoomOnCenter(&ActualVizParams, -1000.0);
            }
        }

        /* Scroll up - zoom out */
        else if (button == 4 && state == GLUT_DOWN) {
            if (Modder == GLUT_ACTIVE_ALT) {
                FillVect(RotationAxis,
                        -ActualVizParams.EyeY + ActualVizParams.CenterY,
                        ActualVizParams.EyeX - ActualVizParams.CenterX, 0.0);
                RotateCameraAroundCenter(&ActualVizParams, RotationAxis, 0.2);
            } else if (Modder == GLUT_ACTIVE_SHIFT) {
                FillVect(RotationAxis, 0.0, 0.0, 0.1);
                RotateCameraAroundCenter(&ActualVizParams, RotationAxis, 0.2);
            } else {
                ZoomOnCenter(&ActualVizParams, 1000.0);
            }
        }

    }

    /* Call of modell-specific mouse handling function */
    HandleSpecialMouseEvent(button, state, x, y, &ActualFlockingParams,
            &ActualVizParams, Modder);

    /* Refreshing both windows */
    glutSetWindow(MenuWindowID);
    glutPostRedisplay();
    glutSetWindow(VizWindowID);
    glutPostRedisplay();

}

/* Handles passive mouse motions */
void HandleMouseMotion(int x, int y) {

}

/* print help */
void print_help(void) {
    printf("This is robotsim created at ELTE Department of Biological Physics.\n"
           "\n"
           "Command line options:\n"
           "\n"
           "-c FILE     define color configuration file\n"
           "-f FILE     define flockingparams file\n"
           "-h, --help  print help and exit\n"
           "-i FILE     define initparams file\n"
           "-novis      do not open GUI\n"
           "-o PATH     define output directory\n"
           "-u FILE     define unitparams file\n"
           "\n"
    );
}

/* Main program */
int main(int argc, char *argv[]) {

    InstallSegfaultHandler();

    int i, j, k;

    /* Initializing random seed with "time.h" tools
     * (the seed changes in every usecs)
     */
    static struct timeval tv;
    gettimeofday(&tv, NULL);
    srand((tv.tv_sec * 1000.0) + (tv.tv_usec / 1000.0));

    /* print help if needed */
    for (i = 0; i < argc; i++) {
        if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
            print_help();
            return 0;
        }
    }

    /* Initializing output directories */
    char OutputFileName[512];
    ActualStatUtils.OutputDirectory = "output_default";
    for (i = 0; i < argc - 1; i++) {
        if (strcmp(argv[i], "-o") == 0) {
            ActualStatUtils.OutputDirectory = argv[i + 1];
        }
    }


    /* Checking existence of output directory */
    //struct stat st = {0};
    struct stat st;
    if (stat(ActualStatUtils.OutputDirectory, &st) == -1) {
        mkdir(ActualStatUtils.OutputDirectory, 0700);
    }

    /* If one of the input parameters is "-novis", then "VizEnabled" should be "false" */
    ActualVizParams.VizEnabled = true;
    for (i = 0; i < argc && ActualVizParams.VizEnabled == true; i++) {
        if (strcmp(argv[i], "-novis") == 0) {
            ActualVizParams.VizEnabled = false;
        }
    }

    /* Get the current working directory */
    static char CurrentDirectory[512];
    getcwd(CurrentDirectory, sizeof(CurrentDirectory));

    /* Setting up structures containing the parameters of the "situation".
     * These parameters are:
     * Number of agents, Initial are sizes (X, Y, Z), Length of measurement, accuracy of the Euler-Maruyama method
     */
    char ParamsFileName[512];
    strcpy(ParamsFileName, CurrentDirectory);
    strcat(ParamsFileName, "/parameters/initparams.dat");
    /* option flag "-i" defines an input file for the "situation" parameters */
    FILE *SitParamsFile =
            CheckInputFile(ParamsFileName, argc, argv, CurrentDirectory,
            ParamsFileName, "-i");
    printf("Using initparams: %s\n", ParamsFileName);
    if (SitParamsFile == NULL) {
        fprintf(stderr, "  Could not open file for reading!\n");
        exit(-1);
    }

    ActualSitParams = GetSituationParamsFromFile(SitParamsFile);
    fclose(SitParamsFile);

    /* Min. of "Length" is 25 sec + delay, approx ~ 50.0 sec */
    if (ActualSitParams.Length < 50.0) {
        ActualSitParams.Length = 50.0;
    }
    if (ActualSitParams.LengthToStore < 10.0) {
        ActualSitParams.LengthToStore = 10.0;
    }

    /* Initializing colors */
    if (ActualVizParams.VizEnabled == true) {

        strcpy(ParamsFileName, CurrentDirectory);
        strcat(ParamsFileName, "/config/defaultcolors.ini\0");
        /* If one of the arguments is '-c', then the next argument will be the color-config file */
        FILE *ColorsFile =
                CheckInputFile(ParamsFileName, argc, argv, CurrentDirectory,
                ParamsFileName, "-c");
        printf("Using colors: %s\n", ParamsFileName);
        if (ColorsFile == NULL) {
            fprintf(stderr, "  Could not open file for reading!\n");
            exit(-1);
        }

        NumberOfModelSpecificColors = 0;
        InitializeModelSpecificColors(ModelSpecificColors,
                &NumberOfModelSpecificColors);
        LoadColorConfig(&ActualColorConfig, ColorsFile,
                ActualSitParams.NumberOfAgents, ModelSpecificColors,
                &NumberOfModelSpecificColors);

        fclose(ColorsFile);

    }

    /* Setting up structures containing the parameters of the "unit" model.
     * By "unit model", we mean the set
     * {TauPID_XY, TauPID_Z, t_GPS, simga_GPS_XY, Sigma_GPS_Z, Sigma_Outer_XY, Sigma_Outer_Z, t_del, R_C, ...}
     */

    /* Counting number of input files and allocating memory for parameter sets */
    CountNumberOfInputs(&NumberOfFlockingParamSets, &NumberOfUnitParamSets,
            argc, argv);

    /* Only one input file is allowed during non-vis mode! */
    if (NumberOfFlockingParamSets > 1 && ActualVizParams.VizEnabled == false) {
        fprintf(stderr,
                "Only one parameter-set is allowed for the flocking algorithm!\n");
        exit(-1);
    } else if (NumberOfUnitParamSets > 1 && false == ActualVizParams.VizEnabled) {
        fprintf(stderr,
                "Only one parameter-set is allowed for the robot model!\n");
        exit(-1);
    }

    /* Otherwise, memory allocation is necessary... */
    ActualFlockingParamSets =
            (flocking_model_params_t *) calloc(NumberOfFlockingParamSets + 1,
            sizeof(flocking_model_params_t));
    ActualUnitParamSets =
            (unit_model_params_t *) calloc(NumberOfUnitParamSets + 1,
            sizeof(unit_model_params_t));

    for (i = 0; i < NumberOfFlockingParamSets + 1; i++) {

        ActualFlockingParamSets[i].NumberOfParameters = 0;
        InitializeFlockingParams(&ActualFlockingParamSets[i]);
        RefreshFlockingParams(&ActualFlockingParamSets[i]);

    }

    if (!FillParameterSetsFromFile(ActualFlockingParamSets, ActualUnitParamSets,
                    argc, &NumberOfFlockingParamSets, &NumberOfUnitParamSets,
                    argv, CurrentDirectory, ParamsFileName))
        return 0;

    /* Setting up structures conatining the parameters of the flocking model
     * This set is model specific, check out the models own ".c" files for further details!
     */
    for (i = 0; i < NumberOfFlockingParamSets; i++) {

        ActualFlockingParamSets[i].NumberOfInputs = argc;
        for (j = 0; j < argc; j++) {
            ActualFlockingParamSets[i].Inputs[j] = argv[j];
        }

        /* Cut-offs at minimum and maximum values */
        for (j = 0; j < ActualFlockingParamSets[i].NumberOfParameters; j++) {

            if (ActualFlockingParamSets[i].Params[j].Value >
                    ActualFlockingParamSets[i].Params[j].Max) {
                ActualFlockingParamSets[i].Params[j].Value =
                        ActualFlockingParamSets[i].Params[j].Max;
            } else if (ActualFlockingParamSets[i].Params[j].Value <
                    ActualFlockingParamSets[i].Params[j].Min) {
                ActualFlockingParamSets[i].Params[j].Value =
                        ActualFlockingParamSets[i].Params[j].Min;
            }
        }
    }

    ActualUnitParams = ActualUnitParamSets[0];
    ActualFlockingParams = ActualFlockingParamSets[0];

    /* Allocating phasespace for actual and delayed timestep */
    /* Creating inner states */
    AllocatePhase(&ActualPhase, ActualSitParams.NumberOfAgents,
            ActualFlockingParams.NumberOfInnerStates);

    /* Allocating phasespace for GPS signals */
    AllocatePhase(&GPSPhase, ActualSitParams.NumberOfAgents, 0);
    AllocatePhase(&GPSDelayedPhase, ActualSitParams.NumberOfAgents, 0);

    /* Allocating a matrix containing all necessary phase data (real) */
    int TimeStepsToStore =
            (int) (((STORED_TIME) / ActualSitParams.DeltaT) - 1.0);
    PhaseData = (phase_t *) calloc(1 + TimeStepsToStore, sizeof(phase_t));
    for (i = 0; i < 1 + TimeStepsToStore; i++) {
        AllocatePhase(&(PhaseData[i]), ActualSitParams.NumberOfAgents,
                ActualPhase.NumberOfInnerStates);
    }
    AgentsInDanger = BooleanData(ActualSitParams.NumberOfAgents);
    InitializePreferredVelocities(&ActualPhase, &ActualFlockingParams,
            &ActualSitParams, &ActualUnitParams, WindVelocityVector);
    for (i = 0; i < ActualSitParams.NumberOfAgents; i++) {
        AgentsInDanger[i] = false;
    }

    /* Initializing positions, "conditions reset" variables and map properties */
    Initialize();

    /* Informations presented to the user. */
    printf("Simulation started with %d", ActualSitParams.NumberOfAgents);
    if (ActualSitParams.NumberOfAgents > 1) {
        printf(" agents\n");
    } else {
        printf(" agent\n");
    }
    printf("Sizes of the starting area: %1.0f cm x %1.0f cm x %1.0f cm\n",
            ActualSitParams.InitialX, ActualSitParams.InitialY,
            ActualSitParams.InitialZ);

    /* Starting main loop of GL environment */

        /************************************************************/

    if (ActualVizParams.VizEnabled == true) {

#ifdef PNG_OUT
        /* Checking existence of output image directories */
        char *OutputImageDir = (char *) calloc(270, sizeof(char));
        strcpy(OutputImageDir, ActualStatUtils.OutputDirectory);
        strcat(OutputImageDir, "/screenshots");
        if (stat(OutputImageDir, &st) == -1) {
            mkdir(OutputImageDir, 0700);
        }
        strcpy(OutputImageDir, ActualStatUtils.OutputDirectory);
        strcat(OutputImageDir, "/frames");
        if (stat(OutputImageDir, &st) == -1) {
            mkdir(OutputImageDir, 0700);
        }
        free(OutputImageDir);

        /* Initializing Devil Package */
        ilInit();
        iluInit();
        ilutRenderer(ILUT_OPENGL);
#endif

        /* Starting of GL loop */
        glutInit(&argc, argv);

        DisplayWindow(400, ActualVizParams.Resolution,
                ActualVizParams.Resolution + 65, 0);
        MenuWindowID = glutCreateWindow("Parameters of actual model");
        glutDisplayFunc(DisplayMenu);
        glutIdleFunc(UpdateMenu);
        glutKeyboardFunc(HandleKeyBoard);
        glutSpecialFunc(HandleKeyBoardSpecial);

        DisplayWindow(ActualVizParams.Resolution, ActualVizParams.Resolution, 0,
                0);
        VizWindowID = glutCreateWindow("Simulation of flying robots");
        gluPerspective(65, 1, 0.01, 100);

        glutIdleFunc(UpdatePositionsToDisplay);
        glutDisplayFunc(DisplayTrajs);
        glutKeyboardFunc(HandleKeyBoard);
        glutSpecialFunc(HandleKeyBoardSpecial);
        glutMouseFunc(HandleMouse);
        glutPassiveMotionFunc(HandleMouseMotion);

        glutMainLoop();

        /*************************************************************/

    } else {

        /* If visualization is OFF, then statistics will be created */

        int h, j;
        ActualStatUtils.ElapsedTime =
                (Now * ActualSitParams.DeltaT) - 5.0 -
                ActualUnitParams.t_del.Value;
        double *StatData;
        ConditionsReset[0] = true;
        ConditionsReset[1] = true;

        /* Allocating and initializing average and sum stats containers */
        ResetStatistics(&ActualStatistics);

        /* Setting up output mode */
        static char OutputModeFileName[512];
        strcpy(OutputModeFileName, CurrentDirectory);
        strcat(OutputModeFileName, "/config/output_config.ini\0");

        /* option flag "-outputconf" defines an input file for output configuration file */
        FILE *OutputModeFile =
                CheckInputFile(OutputModeFileName, argc, argv, CurrentDirectory,
                OutputModeFileName, "-outputconf");
        printf("Using output config: %s\n", OutputModeFileName);
        if (OutputModeFile) {
            ReadOutputModes(&(ActualSaveModes), OutputModeFile);
        } else {
            printf("Could not open config file for reading!\nDefault values will be set.\n");
            SetDefaultOutputModes(&(ActualSaveModes));
        }
        ActualStatUtils.SaveMode = ActualSaveModes.SaveModelSpecifics;

        /* Opening output files */
        FILE *f_Correlation, *f_CoM, *f_Velocity,
                *f_DistanceBetweenNeighbours, *f_DistanceBetweenUnits,
                *f_CollisionRatio, *f_Acceleration, *f_Collisions;
        FILE *f_Correlation_StDev, *f_CoM_StDev, *f_Velocity_StDev,
                *f_CollisionRatio_StDev, *f_DistanceBetweenUnits_StDev,
                *f_Acceleration_StDev, *f_DistanceBetweenNeighbours_StDev;

        /* Positions and velocities */

        if (true == ActualSaveModes.SaveTrajectories) {
            strcpy(OutputFileName, ActualStatUtils.OutputDirectory);
            strcat(OutputFileName, "/posandvel.dat\0");
            f_OutPhase = fopen(OutputFileName, "w");
            fprintf(f_OutPhase,
                    "\n# Output format:\n\n# (column)     1        2   3   4   5    6    7       8   9  10   11   12   13    ... etc ... \n# (data)    time_(s), (x_1 y_1 z_2 vx_1 vy_1 vz_1), (x_2 y_2 z_2 vx_2 vy_2 vz_2), ... etc ...\n\n# Position values are in cm, velocity values are in cm/s\n\n");
        }

        /* Inner states */

        if (true == ActualSaveModes.SaveInnerStates) {
            strcpy(OutputFileName, ActualStatUtils.OutputDirectory);
            strcat(OutputFileName, "/innerstates.dat\0");
            f_OutInnerStates = fopen(OutputFileName, "w");
            fprintf(f_OutInnerStates, "\n# Output format:\n\n#\t\t");
            for (i = 0; i < ActualPhase.NumberOfInnerStates; i++) {
                fprintf(f_OutInnerStates, "%d\t", i + 1);
            }
            fprintf(f_OutInnerStates, "\ntime_(s)\t(\t");
            fprintf(f_OutInnerStates, ")... etc.\n\n");
        }

        /* Opening output files for saving statistics */

        if (FALSE != ActualSaveModes.SaveCollisions) {
            strcpy(OutputFileName, ActualStatUtils.OutputDirectory);
            strcat(OutputFileName, "/collisions.dat\0");
            f_Collisions = fopen(OutputFileName, "w");
            fprintf(f_Collisions, "time_(s) \t number_of_collisions\n\n");
        }

        if (FALSE != ActualSaveModes.SaveDistanceBetweenUnits) {
            strcpy(OutputFileName, ActualStatUtils.OutputDirectory);
            strcat(OutputFileName, "/dist_between_units.dat\0");
            f_DistanceBetweenUnits = fopen(OutputFileName, "w");
            fprintf(f_DistanceBetweenUnits,
                    "\n# 1. time_(s)\n# 2. avg_of_distance_between_units_(cm)\n# 3. stdev_of_distance_between_units_(cm)\n# 4. min_of_distance_between_units_(cm)\n# 5. max_of_distance_between_units_(cm)\n\n");
            fprintf(f_DistanceBetweenUnits,
                    "time_(s) \t avg_of_distance_between_units_(cm) \t stdev_of_distance_between_units_(cm) \t min_of_distance_between_units_(cm) \t max_of_distance_between_units_(cm)\n");
            if (STAT == ActualSaveModes.SaveDistanceBetweenUnits
                    || STEADYSTAT == ActualSaveModes.SaveDistanceBetweenUnits) {
                strcpy(OutputFileName, ActualStatUtils.OutputDirectory);
                strcat(OutputFileName, "/dist_between_units_stdev.dat\0");
                f_DistanceBetweenUnits_StDev = fopen(OutputFileName, "w");
                fprintf(f_DistanceBetweenUnits_StDev,
                        "This file contains standard deviations. Check out \"dist_between_units.dat\" for more details!\n");
                fprintf(f_DistanceBetweenUnits_StDev,
                        "time_(s) \t avg_of_distance_between_units_(cm) \t stdev_of_distance_between_units_(cm) \t min_of_distance_between_units_(cm) \t max_of_distance_between_units_(cm)\n");
            }
        }

        if (FALSE != ActualSaveModes.SaveDistanceBetweenNeighbours) {
            strcpy(OutputFileName, ActualStatUtils.OutputDirectory);
            strcat(OutputFileName, "/dist_between_neighbours.dat\0");
            f_DistanceBetweenNeighbours = fopen(OutputFileName, "w");
            fprintf(f_DistanceBetweenNeighbours,
                    "\n# 1. time_(s)\n# 2. avg_of_distance_between_neighbours_(cm)\n# 3. stdev_of_distance_between_neighbours_(cm)\n# 4. max_of_distance_between_neighbours_(cm)\n\n");
            fprintf(f_DistanceBetweenNeighbours,
                    "time_(s) \t avg_of_distance_between_neighbours_(cm) \t stdev_of_distance_between_neighbours_(cm) \t max_of_distance_between_neighbours_(cm)\n");
            if (STAT == ActualSaveModes.SaveDistanceBetweenNeighbours
                    || STEADYSTAT ==
                    ActualSaveModes.SaveDistanceBetweenNeighbours) {
                strcpy(OutputFileName, ActualStatUtils.OutputDirectory);
                strcat(OutputFileName, "/dist_between_neighbours_stdev.dat\0");
                f_DistanceBetweenNeighbours_StDev = fopen(OutputFileName, "w");
                fprintf(f_DistanceBetweenNeighbours_StDev,
                        "This file contains standard deviations. Check out \"dist_between_neighbours.dat\" for more details!\n");
                fprintf(f_DistanceBetweenNeighbours_StDev,
                        "time_(s) \t avg_of_distance_between_neighbours_(cm) \t stdev_of_distance_between_neighbours_(cm) \t min_of_distance_between_neighbours_(cm) \t max_of_distance_between_neighbours_(cm)\n");
            }
        }

        if (FALSE != ActualSaveModes.SaveVelocity) {
            strcpy(OutputFileName, ActualStatUtils.OutputDirectory);
            strcat(OutputFileName, "/velocity.dat\0");
            f_Velocity = fopen(OutputFileName, "w");
            fprintf(f_Velocity,
                    "\n# 1. time_(s)\n# 2. avg_of_velocity_Magnitude_(cm/s)\n# 3. stdev_of_velocity_Magnitude_(cm/s)\n# 4. min_of_velocity_Magnitude_(cm/s)\n# 5. max_of_velocity_Magnitude_(cm/s)\n# 6. Length_of_avg_velocity_(cm/s)\n# 7. avg_velocity_x_(cm/s)\n# 8. avg_velocity_y_(cm/s)\n# 9. avg_velocity_z_(cm/s)\n\n");
            fprintf(f_Velocity,
                    "time_(s) \t avg_of_velocity_Magnitude_(cm/s) \t stdev_of_velocity_Magnitude_(cm/s) \t min_of_velocity_Magnitude_(cm/s) \t max_of_velocity_Magnitude_(cm/s) \t Length_of_avg_velocity_(cm/s) \t avg_velocity_x_(cm/s) \t avg_velocity_y_(cm/s) \t avg_velocity_z_(cm/s)\n\n");
            if (STAT == ActualSaveModes.SaveVelocity
                    || STEADYSTAT == ActualSaveModes.SaveVelocity) {
                strcpy(OutputFileName, ActualStatUtils.OutputDirectory);
                strcat(OutputFileName, "/velocity_stdev.dat\0");
                f_Velocity_StDev = fopen(OutputFileName, "w");
                fprintf(f_Velocity_StDev,
                        "This file contains standard deviations. Check out \"velocity.dat\" for more details!\n");
                fprintf(f_Velocity_StDev,
                        "time_(s) \t avg_of_velocity_Magnitude_(cm/s) \t stdev_of_velocity_Magnitude_(cm/s) \t min_of_velocity_Magnitude_(cm/s) \t max_of_velocity_Magnitude_(cm/s) \t Length_of_avg_velocity_(cm/s) \t avg_velocity_x_(cm/s) \t avg_velocity_y_(cm/s) \t avg_velocity_z_(cm/s)\n\n");
            }
        }

        if (FALSE != ActualSaveModes.SaveCoM) {
            strcpy(OutputFileName, ActualStatUtils.OutputDirectory);
            strcat(OutputFileName, "/CoM.dat\0");
            f_CoM = fopen(OutputFileName, "w");
            fprintf(f_CoM,
                    "time_(s) \t CoM_x_(cm) \t CoM_y_(cm) \t CoM_z_(cm)\n\n");
            if (STAT == ActualSaveModes.SaveCoM
                    || STEADYSTAT == ActualSaveModes.SaveCoM) {
                strcpy(OutputFileName, ActualStatUtils.OutputDirectory);
                strcat(OutputFileName, "/CoM_stdev.dat\0");
                f_CoM_StDev = fopen(OutputFileName, "w");
                fprintf(f_CoM_StDev,
                        "This file contains standard deviations. Check out \"CoM.dat\" for more details!\n");
                fprintf(f_CoM_StDev,
                        "time_(s) \t CoM_x_(cm) \t CoM_y_(cm) \t CoM_z_(cm)\n\n");
            }
        }

        if (FALSE != ActualSaveModes.SaveCorrelation) {
            strcpy(OutputFileName, ActualStatUtils.OutputDirectory);
            strcat(OutputFileName, "/correlation.dat\0");
            f_Correlation = fopen(OutputFileName, "w");
            fprintf(f_Correlation,
                    "time_(s) \t avg_of_velocity_correlation \t stdev_of_normalized_velocity_scalar_product \t min_of_normalized_velocity_scalar_product \t max_of_normalized_velocity_scalar_product\n\n");
            if (STAT == ActualSaveModes.SaveCorrelation
                    || STEADYSTAT == ActualSaveModes.SaveCorrelation) {
                strcpy(OutputFileName, ActualStatUtils.OutputDirectory);
                strcat(OutputFileName, "/correlation_stdev.dat\0");
                f_Correlation_StDev = fopen(OutputFileName, "w");
                fprintf(f_Correlation_StDev,
                        "This file contains standard deviations. Check out \"correlation.dat\" for more details!\n");
                fprintf(f_Correlation_StDev,
                        "time_(s) \t avg_of_velocity_correlation \t stdev_of_normalized_velocity_scalar_product \t min_of_normalized_velocity_scalar_product \t max_of_normalized_velocity_scalar_product\n\n");
            }
        }

        if (FALSE != ActualSaveModes.SaveCollisionRatio) {
            strcpy(OutputFileName, ActualStatUtils.OutputDirectory);
            strcat(OutputFileName, "/collision_ratio.dat\0");
            f_CollisionRatio = fopen(OutputFileName, "w");
            fprintf(f_CollisionRatio, "time_(s) \t ratio_of_collisions\n\n");
            if (STAT == ActualSaveModes.SaveCollisionRatio
                    || STEADYSTAT == ActualSaveModes.SaveCollisionRatio) {
                strcpy(OutputFileName, ActualStatUtils.OutputDirectory);
                strcat(OutputFileName, "/collision_ratio_stdev.dat\0");
                f_CollisionRatio_StDev = fopen(OutputFileName, "w");
                fprintf(f_CollisionRatio_StDev,
                        "This file contains standard deviations. Check out \"collision_ratio.dat\" for more details!\n");
                fprintf(f_CollisionRatio_StDev,
                        "time_(s) \t ratio_of_collisions\n\n");
            }
        }

        if (FALSE != ActualSaveModes.SaveAcceleration) {
            strcpy(OutputFileName, ActualStatUtils.OutputDirectory);
            strcat(OutputFileName, "/acceleration.dat\0");
            f_Acceleration = fopen(OutputFileName, "w");
            fprintf(f_Acceleration,
                    "\n# 1. time_(s)\n# 2. avg_of_acceleration_Magnitude_(cm/s^2)\n# 3. stdev_of_acceleration_Magnitude_(cm/s^2)\n# 4. min_of_acceleration_Magnitude_(cm/s^2)\n# 5. max_of_acceleration_Magnitude_(cm/s^2)\n# 6. Length_of_avg_acceleration_(cm/s^2)\n# 7. avg_acceleration_x_(cm/s^2)\n# 8. avg_acceleration_y_(cm/s^2)\n# 9. avg_acceleration_z_(cm/s^2)\n\n");
            fprintf(f_Acceleration,
                    "time_(s) \t avg_of_acceleration_Magnitude_(cm/s^2) \t stdev_of_acceleration_Magnitude_(cm/s^2) \t min_of_acceleration_Magnitude_(cm/s^2) \t max_of_acceleration_Magnitude_(cm/s^2) \t Length_of_avg_acceleration_(cm/s^2) \t avg_acceleration_x_(cm/s^2) \t avg_acceleration_y_(cm/s^2) \t avg_acceleration_z_(cm/s^2)\n\n");
            if (STAT == ActualSaveModes.SaveAcceleration
                    || STEADYSTAT == ActualSaveModes.SaveAcceleration) {
                strcpy(OutputFileName, ActualStatUtils.OutputDirectory);
                strcat(OutputFileName, "/acceleration_stdev.dat\0");
                f_Acceleration_StDev = fopen(OutputFileName, "w");
                fprintf(f_Acceleration_StDev,
                        "This file contains standard deviations. Check out \"acceleration.dat\" for more details!\n");
                fprintf(f_Acceleration_StDev,
                        "time_(s) \t avg_of_acceleration_Magnitude_(cm/s^2) \t stdev_of_acceleration_Magnitude_(cm/s^2) \t min_of_acceleration_Magnitude_(cm/s^2) \t max_of_acceleration_Magnitude_(cm/s^2) \t Length_of_avg_acceleration_(cm/s^2) \t avg_acceleration_x_(cm/s^2) \t avg_acceleration_y_(cm/s^2) \t avg_acceleration_z_(cm/s^2)\n\n");
            }
        }

        double *Accelerations;
        Accelerations = malloc(ActualSitParams.NumberOfAgents * sizeof(double));

        if (FALSE != ActualSaveModes.SaveModelSpecifics) {
            InitializeModelSpecificStats(&ActualStatUtils);
        }
        RefreshFlockingParams(&ActualFlockingParams);

        while (ActualStatUtils.ElapsedTime < ActualSitParams.Length
                && ActualVizParams.ExperimentOver == false) {

            if (Now < TimeStepsToStore) {

                Step(&ActualPhase, &GPSPhase, &GPSDelayedPhase,
                        PhaseData, &ActualUnitParams, &ActualFlockingParams,
                        &ActualSitParams, &ActualVizParams, Now,
                        (int) (ActualStatUtils.ElapsedTime /
                                ActualSitParams.DeltaT),
                        (FALSE != ActualSaveModes.SaveCollisions),
                        ConditionsReset, &Collisions, AgentsInDanger,
                        WindVelocityVector, Accelerations);

                HandleOuterVariables(&ActualPhase, &ActualVizParams,
                        &ActualSitParams, &ActualUnitParams,
                        ActualStatUtils.ElapsedTime,
                        ActualStatUtils.OutputDirectory);

                InsertPhaseToDataLine(PhaseData, &ActualPhase, Now + 1);
                InsertInnerStatesToDataLine(PhaseData, &ActualPhase, Now + 1);

            } else {

                ShiftDataLine(PhaseData,
                        TimeStepsToStore,
                        (int) (20.0 / ActualSitParams.DeltaT));
                ShiftInnerStateDataLine(PhaseData, TimeStepsToStore,
                        (int) (20.0 / ActualSitParams.DeltaT));

                Step(&ActualPhase, &GPSPhase, &GPSDelayedPhase,
                        PhaseData, &ActualUnitParams, &ActualFlockingParams,
                        &ActualSitParams, &ActualVizParams, Now,
                        (int) (ActualStatUtils.ElapsedTime /
                                ActualSitParams.DeltaT),
                        (FALSE != ActualSaveModes.SaveCollisions),
                        ConditionsReset, &Collisions, AgentsInDanger,
                        WindVelocityVector, Accelerations);

                HandleOuterVariables(&ActualPhase, &ActualVizParams,
                        &ActualSitParams, &ActualUnitParams,
                        ActualStatUtils.ElapsedTime,
                        ActualStatUtils.OutputDirectory);

                Now = (int) ((20.0 / ActualSitParams.DeltaT) - 1.0);

                InsertPhaseToDataLine(PhaseData, &ActualPhase, Now + 1);
                InsertInnerStatesToDataLine(PhaseData, &ActualPhase, Now + 1);

            }

            /* Reset number of collisions if we haven't passed the steady state timstamp...
               Yes, it is a hack. */
            if (STEADYSTAT == ActualSaveModes.SaveCollisions &&
                    ActualStatUtils.ElapsedTime <
                    ActualSitParams.StartOfSteadyState) {
                Collisions = 0;
            }

            /* Saving trajectories */
            WriteOutTrajectories(&ActualPhase, ActualSaveModes.SaveTrajectories,
                    ActualSaveModes.SaveInnerStates,
                    ActualStatUtils.ElapsedTime, f_OutPhase, f_OutInnerStates);

            /* Saving statistics */
            // TODO: Some nice method instead of cutting trees... ... ...
            if (TIMELINE == ActualSaveModes.SaveCollisions) {
                fprintf(f_Collisions, "%lf\t%d\n", ActualStatUtils.ElapsedTime,
                        Collisions);
            }

            if (FALSE != ActualSaveModes.SaveDistanceBetweenUnits) {
                StatData = StatOfDistanceBetweenUnits(&ActualPhase);
            }
            switch (ActualSaveModes.SaveDistanceBetweenUnits) {
            case TIMELINE:{
                fprintf(f_DistanceBetweenUnits, "%lf\t%lf\t%lf\t%lf\t%lf\n",
                        ActualStatUtils.ElapsedTime, StatData[0], StatData[1],
                        StatData[2], StatData[3]);
                break;
            }
            case STAT:{
                UPDATE_STATISTICS(DistanceBetweenUnits, 4);
                break;
            }
            case STEADYSTAT:{
                UPDATE_STATISTICS(DistanceBetweenUnits, 4);
                break;
            }
            }

            if (FALSE != ActualSaveModes.SaveDistanceBetweenNeighbours) {
                StatData = StatOfDistanceBetweenNearestNeighbours(&ActualPhase);
            }
            switch (ActualSaveModes.SaveDistanceBetweenNeighbours) {
            case TIMELINE:{
                fprintf(f_DistanceBetweenNeighbours, "%lf\t%lf\t%lf\t%lf\n",
                        ActualStatUtils.ElapsedTime, StatData[0], StatData[1],
                        StatData[2]);
                break;
            }
            case STAT:{
                UPDATE_STATISTICS(DistanceBetweenNeighbours, 3);
                break;
            }
            case STEADYSTAT:{
                UPDATE_STATISTICS(DistanceBetweenNeighbours, 3);
                break;
            }
            }

            if (FALSE != ActualSaveModes.SaveCorrelation) {
                StatData = StatOfCorrelation(&ActualPhase);
            }
            switch (ActualSaveModes.SaveCorrelation) {
            case TIMELINE:{
                fprintf(f_Correlation, "%lf\t%lf\t%lf\t%lf\t%lf\n",
                        ActualStatUtils.ElapsedTime, StatData[0], StatData[1],
                        StatData[2], StatData[3]);
                break;
            }
            case STAT:{
                UPDATE_STATISTICS(Correlation, 4);
                break;
            }
            case STEADYSTAT:{
                UPDATE_STATISTICS(Correlation, 4);
                break;
            }
            }

            if (FALSE != ActualSaveModes.SaveVelocity) {
                StatData = StatOfVelocity(&ActualPhase);
            }
            switch (ActualSaveModes.SaveVelocity) {
            case TIMELINE:{
                fprintf(f_Velocity,
                        "%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\t%lf\n",
                        ActualStatUtils.ElapsedTime, StatData[0], StatData[1],
                        StatData[2], StatData[3], StatData[4], StatData[5],
                        StatData[6], StatData[7]);
                break;
            }
            case STAT:{
                UPDATE_STATISTICS(Velocity, 8);
                break;
            }
            case STEADYSTAT:{
                UPDATE_STATISTICS(Velocity, 8);
                break;
            }
            }

            if (FALSE != ActualSaveModes.SaveCoM) {
                GetCoM(StatData, &ActualPhase);
            }
            switch (ActualSaveModes.SaveCoM) {
            case TIMELINE:{
                fprintf(f_CoM, "%lf\t%lf\t%lf\t%lf\n",
                        ActualStatUtils.ElapsedTime, StatData[0], StatData[1],
                        StatData[2]);
                break;
            }
            case STAT:{
                UPDATE_STATISTICS(CoM, 3);
                break;
            }
            case STEADYSTAT:{
                UPDATE_STATISTICS(CoM, 3);
                break;
            }
            }

            switch (ActualSaveModes.SaveCollisionRatio) {
            case TIMELINE:{
                fprintf(f_CollisionRatio, "%lf\t%g\n",
                        ActualStatUtils.ElapsedTime,
                        RatioOfDangerousSituations(&ActualPhase,
                                ActualSitParams.Radius));
                break;
            }
            case STAT:{
                double Ratio_Temp = RatioOfDangerousSituations(&ActualPhase,
                        ActualSitParams.Radius);
                ActualStatistics.Data_CollisionRatio_Sum +=
                        Ratio_Temp * ActualSitParams.DeltaT;
                ActualStatistics.Data_CollisionRatio_StDev +=
                        Ratio_Temp * Ratio_Temp * ActualSitParams.DeltaT;
                break;
            }
            case STEADYSTAT:{
                if ((ActualStatUtils.ElapsedTime -
                                ActualSitParams.StartOfSteadyState) > 0.0) {
                    double Ratio_Temp = RatioOfDangerousSituations(&ActualPhase,
                            ActualSitParams.Radius);
                    ActualStatistics.Data_CollisionRatio_Sum +=
                            Ratio_Temp * ActualSitParams.DeltaT;
                    ActualStatistics.Data_CollisionRatio_StDev +=
                            Ratio_Temp * Ratio_Temp * ActualSitParams.DeltaT;
                }
                break;
            }
            }

            if (FALSE != ActualSaveModes.SaveAcceleration) {
                // Using the savedout-before-noise acceleration data instead of derivating a noisy function
                StatData =
                        StatOfAcceleration(Accelerations,
                        ActualSitParams.NumberOfAgents);
            }
            switch (ActualSaveModes.SaveAcceleration) {
            case TIMELINE:{
                fprintf(f_Acceleration, "%lf\t%lf\t%lf\t%lf\t%lf\n",
                        ActualStatUtils.ElapsedTime, StatData[0], StatData[1],
                        StatData[2], StatData[3]);
                break;
            }
            case STAT:{
                UPDATE_STATISTICS(Acceleration, 4);
                break;
            }
            case STEADYSTAT:{
                UPDATE_STATISTICS(Acceleration, 4);
                break;
            }

            }

            /* Saving model-specific statistics */
            if (FALSE != ActualSaveModes.SaveModelSpecifics) {
                SaveModelSpecificStats(&ActualPhase, &ActualStatUtils,
                        &ActualUnitParams, &ActualFlockingParams,
                        &ActualSitParams);
            }

            ActualStatUtils.ElapsedTime += ActualSitParams.DeltaT;
            Now++;

        }

        /* Closing files */
        if (true == ActualSaveModes.SaveTrajectories) {
            fclose(f_OutPhase);
        }
        if (true == ActualSaveModes.SaveInnerStates) {
            fclose(f_OutInnerStates);
        }

        if (FALSE != ActualSaveModes.SaveDistanceBetweenUnits) {
            SAVE_STATISTICS(DistanceBetweenUnits, 4);
            fclose(f_DistanceBetweenUnits);
        }
        if (FALSE != ActualSaveModes.SaveDistanceBetweenNeighbours) {
            SAVE_STATISTICS(DistanceBetweenNeighbours, 3);
            fclose(f_DistanceBetweenNeighbours);
        }
        if (FALSE != ActualSaveModes.SaveVelocity) {
            SAVE_STATISTICS(Velocity, 8);
            fclose(f_Velocity);
        }
        if (FALSE != ActualSaveModes.SaveCoM) {
            SAVE_STATISTICS(CoM, 3);
            fclose(f_CoM);
        }
        if (FALSE != ActualSaveModes.SaveCorrelation) {
            SAVE_STATISTICS(Correlation, 4);
            fclose(f_Correlation);
        }
        if (FALSE != ActualSaveModes.SaveCollisionRatio) {

            static double temp_value;
            static double temp_time;
            temp_time = (STEADYSTAT == ActualSaveModes.SaveCollisionRatio ?
                    ActualStatUtils.ElapsedTime -
                    ActualSitParams.
                    StartOfSteadyState : ActualStatUtils.ElapsedTime);

            if (STAT == ActualSaveModes.SaveCollisionRatio
                    || STEADYSTAT == ActualSaveModes.SaveCollisionRatio) {
                temp_value =
                        ActualStatistics.Data_CollisionRatio_Sum / temp_time;
                fprintf(f_CollisionRatio, "%lf\t%g\n", temp_time, temp_value);
                fprintf(f_CollisionRatio_StDev, "%lf\t%g\n",
                        temp_time,
                        sqrt(ActualStatistics.Data_CollisionRatio_StDev /
                                temp_time - temp_value * temp_value));
            }
            fclose(f_CollisionRatio);
            fclose(f_CollisionRatio_StDev);
        }
        if (FALSE != ActualSaveModes.SaveCollisions) {
            if (STAT == ActualSaveModes.SaveCollisions) {
                fprintf(f_Collisions, "%lf\t%d\n", ActualStatUtils.ElapsedTime,
                        Collisions);
            } else if (STEADYSTAT == ActualSaveModes.SaveCollisions) {
                fprintf(f_Collisions, "%lf\t%d\n",
                        ActualStatUtils.ElapsedTime -
                        ActualSitParams.StartOfSteadyState, Collisions);
            }
            fclose(f_Collisions);
        }
        if (FALSE != ActualSaveModes.SaveAcceleration) {
            SAVE_STATISTICS(Acceleration, 4);
            fclose(f_Acceleration);
        }

        if (FALSE != ActualSaveModes.SaveModelSpecifics) {
            CloseModelSpecificStats(&ActualStatUtils);
        }

    }

    /* Destroy model specific stuff */
    DestroyPhase(&ActualPhase, &ActualFlockingParams, &ActualSitParams);
    /* Free memory */
    free(ActualFlockingParamSets);
    free(ActualUnitParamSets);
    freePreferredVelocities(&ActualPhase, &ActualFlockingParams,
            &ActualSitParams);
    if (ActualVizParams.VizEnabled == true) {
        for (i = 0; i < ActualSitParams.NumberOfAgents; i++) {
            free(ActualColorConfig.AgentsColor[i]);
        }
    }

    free(ActualColorConfig.AgentsColor);
    free(AgentsInDanger);
    freePhase(&ActualPhase);
    freePhase(&GPSPhase);
    freePhase(&GPSDelayedPhase);

    for (i = 0; i < 1 + TimeStepsToStore; i++) {
        freePhase(&(PhaseData[i]));
    }
    free(PhaseData);

    return 1;

}
