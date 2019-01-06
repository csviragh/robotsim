//MIT License
//Copyright (c) 2018 Eotvos Lorand University, Budapest

/* vim:set ts=4 sw=4 sts=4 et: */

/* Models of inaccuracy of sensors
 * "inner noise"
 */

#ifndef SENSORS_H
#define SENSORS_H

#include "utilities/math_utils.h"
#include "utilities/dynamics_utils.h"
#include "utilities/param_utils.h"

/* Setting up initial values of GPS-measured positions and velocities 
 */
void ResetGPSNoises(phase_t * GPSPhase, phase_t * GPSDelayedPhase);

/* Steps Stored (actual and delayed) GPS fluctuations 
 */
void StepGPSNoises(phase_t * WhichPhase, unit_model_params_t * UnitParams);

#endif
