//MIT License
//Copyright (c) 2018 Eotvos Lorand University, Budapest

/* vim:set ts=4 sw=4 sts=4 et: */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "file_utils.h"
#include "obstacles.h"

typedef struct {
    obstacles_t *obstacles;
    double angle;
    char last_obstacle_name[MAX_OBSTACLE_NAME_LENGTH];
} obstacle_callback_data_t;

static int obstacle_callback(void *user, const char *section, const char *name,
        const char *value) {
    obstacle_callback_data_t *data = user;
    float x, y;
    const char *sep;
    char obstacle_name[MAX_OBSTACLE_NAME_LENGTH];

    if (!strcmp(section, "init")) {
        /* Initialization section */
        if (!strcmp(name, "angle")) {
            /* parse angle between obstacle coordinate system and NE(-D) */
            data->angle = atof(value) * M_PI / 180.0;
            return 1;
        } else {
            printf("ERROR: unknown name in section init (%s)\n", name);
            return 0;
        }
    } else if (!strcmp(section, "obstacles")) {
        /* check for propoer format */
        sep = strchr(name, '.');
        if (sep == 0 || sep < name) {
            printf("Handler for %s/%s not implemented yet\n", section, name);
            return 0;
        }
        /* get obstacle name */
        strncpy(obstacle_name, name, sep - name);
        obstacle_name[sep - name] = 0;
        /* it is a new obstacle name */
        if (strcmp(obstacle_name, data->last_obstacle_name)) {
            /* increase obstacle counter */
            if (data->obstacles->o_count == MAX_OBSTACLES) {
                printf("Max number of obstacles (%d) reached; ignoring "
                        "remaining obstacles.\n", MAX_OBSTACLES);
                return 0;
            }
            data->obstacles->o_count++;
            data->obstacles->o[data->obstacles->o_count - 1].p_count = 0;
            /* save name (twice) */
            strncpy(data->last_obstacle_name, obstacle_name, sep - name);
            data->last_obstacle_name[sep - name] = 0;
            strncpy(data->obstacles->o[data->obstacles->o_count - 1].name,
                    obstacle_name, sep - name);
            data->obstacles->o[data->obstacles->o_count - 1].name[sep - name] =
                    0;
        }
        sep++;
        /* Next obstacle point */
        if (!strcmp(sep, "point")) {
            if (data->obstacles->o[data->obstacles->o_count - 1].p_count ==
                    MAX_OBSTACLE_POINTS) {
                printf("Max number of obstacle points (%d) reached in obstacle '%s'; " "ignoring remaining obstacles.\n", MAX_OBSTACLE_POINTS, obstacle_name);
                return 0;
            } else if (sscanf(value, "%f %f", &x, &y) != 2) {
                printf("Error parsing obstacle %s point #%d: %s\n",
                        obstacle_name,
                        data->obstacles->o[data->obstacles->o_count -
                                1].p_count, value);
                return 0;
            } else {
                /* convert from local frame to NE(-D) */
                data->obstacles->o[data->obstacles->o_count -
                        1].p[data->obstacles->o[data->obstacles->o_count -
                                1].p_count][0] =
                        x * cos(data->angle) - y * sin(data->angle);
                data->obstacles->o[data->obstacles->o_count -
                        1].p[data->obstacles->o[data->obstacles->o_count -
                                1].p_count][1] =
                        x * sin(data->angle) + y * cos(data->angle);
                /* increase obstacle point counter */
                data->obstacles->o[data->obstacles->o_count - 1].p_count++;
                return 1;
            }
        }
    }

    printf("Unknown section/key in obstacle file: %s/%s\n", section, name);
    return 0;
}

int ParseObstacleFile(const char *name, obstacles_t * obstacles) {
    obstacle_callback_data_t data = {
        .obstacles = obstacles,
        .angle = 0,
        .last_obstacle_name[0] = 0
    };
    int result;
    int i, j;
    float x, y;

    obstacles->o_count = 0;
    result = IniParse(name, obstacle_callback, &data);

    if (result) {
        printf("%s: parse error on line %d\n", name, result);
        return 0;
    }

    printf("Parsed %d obstacles.\n", obstacles->o_count);

    /* calculate center of obstacles */
    for (i = 0; i < obstacles->o_count; i++) {
        x = y = 0;
        for (j = 0; j < obstacles->o[i].p_count; j++) {
            x += obstacles->o[i].p[j][0];
            y += obstacles->o[i].p[j][1];
        }
        if (j == 0) {
            obstacles->o[i].center[0] = 0;
            obstacles->o[i].center[1] = 0;
        } else {
            obstacles->o[i].center[0] = x / j;
            obstacles->o[i].center[1] = y / j;
        }
    }

    /* return number of obstacles parsed */
    return obstacles->o_count;
}
