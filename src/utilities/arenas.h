//MIT License
//Copyright (c) 2018 Eotvos Lorand University, Budapest

/* vim:set ts=4 sw=4 sts=4 et: */

#ifndef ARENAS_H
#define ARENAS_H

#include "dynamics_utils.h"

#define MAX_ARENA_VERTICES 128
#define MAX_ARENA_COUNT 32
#define MAX_ARENA_NAME_LENGTH 32

/* predefined arena shapes with predefined indices */
#define ARENA_CIRCLE 0
#define ARENA_SQUARE 1

/**
 * Struct containing info about an arena.
 */
typedef struct {
    int index;
    char name[MAX_ARENA_NAME_LENGTH];
    double p[MAX_ARENA_VERTICES][3];
    int p_count;
} arena_t;

/**
 * Struct needed to store actual number of arenas, too
 */
typedef struct {
    arena_t a[MAX_ARENA_COUNT];
    int a_count;
} arenas_t;

/**
 * Parses an arena file.
 *
 * \param  name        the name of the configuration file. A leading tilde (~) will
 *                     be resolved to the home directory of the user.
 * \param  arenas      the arenas structure that hold the parsed information
 * \param  define_circle_and_square  should we define circle and square automatically?
 *
 * \return             number of arenas parsed successfully
 */
int ParseArenaFile(const char *name, arenas_t * arenas,
        bool define_circle_and_square);

/* More-or-less general interaction with specific types of arena */
void Shill_Wall(double *OutputVelocity, phase_t * Phase,
        const double ArenaCenterX, const double ArenaCenterY,
        const double ArenaSize, const arena_t * Arena,
        const double C_Shill, const double V_Shill,
        const double Alpha_Shill, const double Gamma_Wall,
        const int WhichAgent, const int Dim_l);

void Shill_Wall_LinSqrt(double *OutputVelocity, phase_t * Phase,
        const double ArenaCenterX, const double ArenaCenterY,
        const double ArenaSize, const arena_t * Arena,
        const double V_Shill, const double R0_Offset_Shill,
        const double Acc_Shill, const double Slope_Shill,
        const int WhichAgent, const int Dim_l);

#endif
