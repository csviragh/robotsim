//MIT License
//Copyright (c) 2018 Eotvos Lorand University, Budapest

/* vim:set ts=4 sw=4 sts=4 et: */

/* Model-Specific GUI tools
 */

#ifndef ALGO_GUI_H
#define ALGO_GUI_H

#include <math.h>

#include "vizmode.h"
#include "colors.h"
#include "dynspecviz.h"
#include "utilities/param_utils.h"
#include "utilities/datastructs.h"
#include "vizualizer/objects_2d.h"
#include "vizualizer/objects_3d.h"
#include "utilities/dynamics_utils.h"
#include "utilities/file_utils.h"
#include "utilities/math_utils.h"

/* For changing variables from algo_..._gui.c */
static int jj;

/* For reseting Phases
 */
void ModelSpecificReset(phase_t * Phase,
        const double Size_X,
        const double Size_Y,
        const double Size_Z,
        vizmode_params_t * VizParams,
        flocking_model_params_t * FlockingParams, const double Radius);

/* For Initializing vizualisation mode parameters
 */
void InitializeVizParams(vizmode_params_t * VizParams);

/* For Initializing and saving model-specific colors
 */
void InitializeModelSpecificColors(model_specific_color_t * ModelSpecificColors,
        int *NumberOfModelSpecificColors);

/* For drawing model-specific objects in 2D and 3D
 * (e.g. Lines, Obstacles, Waypoints)
 */
void DrawModelSpecificObjects_2D(phase_t * Phase,
        flocking_model_params_t * FlockingParams,
        unit_model_params_t * UnitModelParams,
        vizmode_params_t * VizParams,
        color_config_t * Colors, sit_parameters_t * SitParams);
void DrawModelSpecificObjects_3D(phase_t * Phase,
        flocking_model_params_t * FlockingParams,
        unit_model_params_t * UnitModelParams,
        vizmode_params_t * VizParams,
        color_config_t * Colors, sit_parameters_t * SitParams);

/* For displaying a model specific string in the gui
 */
char *GetModelSpecificStringToDisplay(phase_t * Phase,
        flocking_model_params_t * FlockingParams,
        unit_model_params_t * UnitModelParams,
        sit_parameters_t * SitParams, double TimeStep);

/* For handling model-specific keyboard events
 */
void HandleSpecialKeyBoardEvent(unsigned char key,
        int x,
        int y,
        flocking_model_params_t * FlockingParams,
        vizmode_params_t * VizParams,
        sit_parameters_t * SitParams, const int Modifier);

/* For handling model-specific "special key" events
 */
void HandleSpecialSpecKeyEvent(unsigned char key,
        int x,
        int y,
        flocking_model_params_t * FlockingParams,
        vizmode_params_t * VizParams,
        sit_parameters_t * SitParams, const int Modifier);

/* For handling model-specific mouse events
 */
void HandleSpecialMouseEvent(int button,
        int state,
        int x,
        int y,
        flocking_model_params_t * FlockingParams,
        vizmode_params_t * VizParams, const int Modifier);

#endif
