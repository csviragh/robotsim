//MIT License
//Copyright (c) 2018 Eotvos Lorand University, Budapest

/* vim:set ts=4 sw=4 sts=4 et: */

/* Drawing objects related to specific dynamical systems */

#ifndef DYNSPECVIZ_H
#define DYNSPECVIZ_H

#include <math.h>
#include "vizualizer/objects_2d.h"
#include "vizualizer/objects_3d.h"
#include "utilities/datastructs.h"
#include "utilities/dynamics_utils.h"
#include "utilities/math_utils.h"
#include "vizmode.h"

/* 2D */

/* Drawing copters, etc. (model-specific objects) */

/* Arrow represents the actual velocity 
 * "x" and "y" are the positions of the copter in real coordinates 
 * "velocity_x" and "velocity_y" are the components of the velocity vector 
 * "MapSizexy" are the Length of the displayed area in real coordinates (cm)
 * "arrowcolor" is the color of the arrow (r, g, b)
 */
void DrawVelocityArrow_2D(const double x,
        const double y,
        const double velocity_x,
        const double velocity_y,
        const double MapSizexy, const float *arrowcolor);

/* Filled circle with the size of a quadcopter (40 cm) 
 * "x" and "y" are the positions of the copter in real coordinates (cm)
 * "MapSizexy" are the Length of the displayed area in real coordinates (cm)
 * "agentscolor" is the color of the copter (RGB)
 */
void DrawCopter_2D(const double x, const double y, const double MapSizexy,
        const double Radius, const float *agentcolor);

/* Labels above agents
 */
void DrawAgentLabel_2D(phase_t * Phase,
        const int WhichAgent,
        char *Label,
        bool Display, vizmode_params_t * VizParams, const float *Color);

/* Network arrow between two agents
 */
void DrawNetworkArrow_2D(phase_t * Phase,
        const int FromWhichAgent,
        const int ToWhichAgent,
        vizmode_params_t * VizParams, const float *color);

/* Network arrow between two positions 
 */
void DrawNetworkArrowBetweenPositions_2D(double *FromCoords,
        double *ToCoords, vizmode_params_t * VizParams, const float *color);

/* Sensor Range Newtork for agent "WhichAgent"
 */
void DrawSensorRangeNetwork_2D(phase_t * PhaseData,
        const double SensorRangeToDisplay,
        const int WhichAgent,
        const double Delay,
        const int Now,
        const double h, vizmode_params_t * VizParams, const float *color);

/* 3D */

/* Sensor range network for agent "WhichAgent"
 */
void DrawSensorRangeNetwork_3D(phase_t * PhaseData,
        const double SensorRangeToDisplay,
        const int WhichAgent,
        const double Delay,
        const int Now,
        const double h, const double MapSizexy, const float *color);

/* 3D camera movement */

void TranslateCameraOnXYPlane(vizmode_params_t * VizParams, double *Direction,
        const double StepSize);
void RotateCameraAroundCenter(vizmode_params_t * VizParams, double *Axis,
        const double angle);
void ZoomOnCenter(vizmode_params_t * VizParams, const double StepSize);

#endif
