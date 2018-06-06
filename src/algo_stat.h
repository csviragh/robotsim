/* vim:set ts=4 sw=4 sts=4 et: */

/* Header file for algo_..._stat.c */

#ifndef ALGO_STAT
#define ALGO_STAT

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "utilities/math_utils.h"
#include "utilities/output_utils.h"
#include "utilities/dynamics_utils.h"
#include "utilities/param_utils.h"

/* Macro for initializing output file 
 */
static char ModelSpecificOutputFileName[512];
#define INITIALIZE_OUTPUT_FILE(filename, file) \
    strcpy (ModelSpecificOutputFileName, StatUtils->OutputDirectory); \
    strcat (ModelSpecificOutputFileName, "/"); \
    strcat (ModelSpecificOutputFileName, filename); \
    strcat (ModelSpecificOutputFileName, "\0"); \
    file = fopen (ModelSpecificOutputFileName, "w");

/* This struct conatins all (?) necessary parameters
 * for creating statistical calculations 
 */
typedef struct {

    double ElapsedTime;
    double StartOfSteadyState;
    char *OutputDirectory;
    save_mode_t SaveMode;

} stat_utils_t;

/* Function for opening stat files, creating header lines, etc. 
 */
void InitializeModelSpecificStats(stat_utils_t * StatUtils);

void SaveModelSpecificStats(phase_t * Phase, stat_utils_t * StatUtils,
        unit_model_params_t * UnitParams,
        flocking_model_params_t * FlockingParams, sit_parameters_t * SitParams);

/* For closing stat files 
 */
void CloseModelSpecificStats(stat_utils_t * StatUtils);

#endif
