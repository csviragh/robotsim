//MIT License
//Copyright (c) 2018 Eotvos Lorand University, Budapest

/* vim:set ts=4 sw=4 sts=4 et: */

#ifndef DEBUG_INFO_H
#define DEBUG_INFO_H

#include <stdio.h>
#include "dynamics_utils.h"

#ifdef DEBUG
#include <signal.h>
#include <execinfo.h>
#define MAX_STACKTRACE_DEPTH 32
#endif

/* This struct is an input of the CalculatePreferredVelocity function,
 * can be used only for displaying debug information.
 */
typedef struct {

    /* Sequential number, position and velocity of the actual agent */
    int AgentsSeqNumber;
    double AgentsRealPosition[3];
    double AgentsRealVelocity[3];

    /* Real PhaseSpace and Inner States */
    phase_t *RealPhase;

} agent_debug_info_t;

//////////////////////////////////////////////////////////////////////
/* Debug functions */

/* Printing out the components of a vector to the console
 * "Dim" is the number of components of the vector "Vector"
 */
void PrintVector(double *Vector, const int Dim);

/* Debug mode utils for detecting segmentation faults */
void InstallSegfaultHandler();

#endif
