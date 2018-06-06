/* vim:set ts=4 sw=4 sts=4 et: */

/* Comments, comments, comments
 *
 */

#ifndef ALGO_H
#define ALGO_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "utilities/param_utils.h"
#include "utilities/dynamics_utils.h"
#include "utilities/datastructs.h"
#include "utilities/math_utils.h"
#include "utilities/debug_utils.h"
#include "vizmode.h"
#include "colors.h"

/* Helper variables...
 */
int tt;

/* Sets up default values of parameters of the flocking model
 * Number of inner states also should be defined here.
 */
void InitializeFlockingParams(flocking_model_params_t * FlockingParams);

/* Setting up Initial values of inner states, allocating memory for
 * model-specific objects.
 */
void InitializePhase(phase_t * Phase, flocking_model_params_t * FlockingParams,
        sit_parameters_t * SitParams);

/* Refreshing values of outer variables (e. g. "number of caught agents" in the chasing algorithm)
 * in every timestep
 */
void HandleOuterVariables(phase_t * Phase,
        vizmode_params_t * VizParams,
        sit_parameters_t * SitParams,
        unit_model_params_t * UnitParams,
        const double ActualTime, char *OutputDirectory);

/* Target velocity calculation.
 */
void CalculatePreferredVelocity(double *OutputVelocity,
        double *OutputInnerState,
        phase_t * Phase,
        const int WhichAgent,
        flocking_model_params_t * FlockingParams,
        vizmode_params_t * VizParams,
        const double Delay,
        const double ActualTime, agent_debug_info_t * DebugInfo);

/* Destruction of all model-specific objects initilalized in InitializePhase */
void DestroyPhase(phase_t * Phase, flocking_model_params_t * FlockingParams,
        sit_parameters_t * SitParams);

#endif
