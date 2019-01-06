//MIT License
//Copyright (c) 2018 Eotvos Lorand University, Budapest

/* vim:set ts=4 sw=4 sts=4 et: */

/* Tools for handling the robot model
 */

#ifndef ROBOTMODEL_H
#define ROBOTMODEL_H

#include "utilities/param_utils.h"
#include "utilities/math_utils.h"
#include "utilities/datastructs.h"
#include "vizmode.h"
#include "sensors.h"
#include "algo.h"
#include "algo_gui.h"
#include "vizmode.h"
#include "utilities/debug_utils.h"
#include <math.h>
#include <string.h>

/* Step positions and velocities
 */
void Step(phase_t * OutputPhase, phase_t * GPSPhase, phase_t * GPSDelayedPhase,
        phase_t * PhaseData, unit_model_params_t * UnitParams,
        flocking_model_params_t * FlockingParams, sit_parameters_t * SitParams,
        vizmode_params_t * VizParams, int TimeStepLooped, int TimeStepReal,
        bool CountCollisions, bool * ConditionsReset, int *Collisions,
        bool * AgentsInDanger, double *WindVelocityVector,
        double *Accelerations);

/*  Initialization and killing
 */
void InitializePreferredVelocities(phase_t * Phase,
        flocking_model_params_t * FlockingParams, sit_parameters_t * SitParams,
        unit_model_params_t * UnitParams, double *WindVelocityVector);
void freePreferredVelocities(phase_t * Phase,
        flocking_model_params_t * FlockingParams, sit_parameters_t * SitParams);

#endif
