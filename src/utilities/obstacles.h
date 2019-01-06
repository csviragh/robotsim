//MIT License
//Copyright (c) 2018 Eotvos Lorand University, Budapest

/* vim:set ts=4 sw=4 sts=4 et: */

#ifndef OBSTACLES_H
#define OBSTACLES_H

#define MAX_OBSTACLES 32
#define MAX_OBSTACLE_POINTS 10
#define MAX_OBSTACLE_NAME_LENGTH 32

/**
 * Struct containing info about a single obstacle used in flights with obstacles.
 */
typedef struct {
    char name[MAX_OBSTACLE_NAME_LENGTH];        /* name of the obstacle */
    double p[MAX_OBSTACLE_POINTS][3];   /* list of polygon points */
    double center[2];           /* center of the obstacle polygon */
    int p_count;                /* number of points in the obstacle polygon */
} obstacle_t;

/**
 * Struct containing all info about obstacles
 */
typedef struct {
    obstacle_t o[MAX_OBSTACLES];        /* list of obstacles */
    int o_count;                /* number of obstacles in the list */
} obstacles_t;

/**
 * Parses an obstacle file.
 *
 * \param  name        the name of the configuration file. A leading tilde (~) will
 *                     be resolved to the home directory of the user.
 * \param  obstacles   the obstacles structure that hold the parsed information
 *
 * \return             number of obstacles parsed successfully
 */
int ParseObstacleFile(const char *name, obstacles_t * obstacles);

#endif
