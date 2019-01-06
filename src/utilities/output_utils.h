//MIT License
//Copyright (c) 2018 Eotvos Lorand University, Budapest

/* vim:set ts=4 sw=4 sts=4 et: */

/* This file contains structs and function declarations
 * for setting up special output modes
 */

#ifndef OUTPUT_UTILS_H
#define OUTPUT_UTILS_H

#include <stdbool.h>
#include <stdio.h>
#include "dynamics_utils.h"

typedef enum {
    FALSE = 0,
    TIMELINE = 1,
    STAT = 2,
    STEADYSTAT = 3
} save_mode_t;

/* With this struct, one can set which outputs have to be saved 
 */
typedef struct {

    /* Position, velocity and inner states */
    bool SaveTrajectories;
    bool SaveInnerStates;

    /* Order parameters */
    save_mode_t SaveDistanceBetweenUnits;
    save_mode_t SaveDistanceBetweenNeighbours;
    save_mode_t SaveVelocity;
    save_mode_t SaveCorrelation;
    save_mode_t SaveCoM;
    save_mode_t SaveCollisions;
    save_mode_t SaveCollisionRatio;
    save_mode_t SaveAcceleration;

    /* Model-specific parameters */
    save_mode_t SaveModelSpecifics;

} output_modes_t;

/* Sets default values for all output mode 
 */
void SetDefaultOutputModes(output_modes_t * OutputModes);

/* Reads output modes from config file 
 */
void ReadOutputModes(output_modes_t * OutputModes, FILE * InputFile);

/* Prints actual state into file
 */
void WriteOutTrajectories(phase_t * Phase, const bool SaveTrajs,
        const bool SaveInnerStates,
        const double ActualTime, FILE * f_OutPhase, FILE * f_OutInnerStates);

#endif
