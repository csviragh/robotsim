/* vim:set ts=4 sw=4 sts=4 et: */

/* Tools for calculating basic statistical quantities */

#ifndef STAT_H
#define STAT_H

#include <math.h>
#include "utilities/output_utils.h"
#include "utilities/math_utils.h"
#include "utilities/datastructs.h"
#include "utilities/dynamics_utils.h"

/* This struct contains data arrays for storing general statistical properties
 */
typedef struct {

    /* Sums for averages */
    double Data_Acceleration_Sum[4];
    double Data_Correlation_Sum[4];
    double Data_DistanceBetweenUnits_Sum[4];
    double Data_DistanceBetweenNeighbours_Sum[3];
    double Data_CollisionRatio_Sum;
    double Data_Velocity_Sum[8];
    double Data_CoM_Sum[3];

    /* stDevs */
    double Data_Acceleration_StDev[4];
    double Data_Correlation_StDev[4];
    double Data_DistanceBetweenUnits_StDev[4];
    double Data_DistanceBetweenNeighbours_StDev[3];
    double Data_CollisionRatio_StDev;
    double Data_CoM_StDev[3];
    double Data_Velocity_StDev[8];

} statistics_t;

/* Macro for updating statistics
 * "stat" denotes an element from the statistics_t struct (e.g. CoM means Data_CoM_Sum and Data_CoM_StDev) , while
 * "n" denotes the number of parameters to save
 */
#define UPDATE_STATISTICS(stat, n) \
    if (ActualSaveModes.Save##stat != STEADYSTAT || (ActualStatUtils.ElapsedTime - ActualSitParams.StartOfSteadyState) > 0.0) { \
        static int d; \
        for (d = 0; d < n; d++) { \
            ActualStatistics.Data_##stat##_Sum[d] += StatData[d] * ActualSitParams.DeltaT; \
            ActualStatistics.Data_##stat##_StDev[d] += StatData[d] * StatData [d] * ActualSitParams.DeltaT; \
        } \
    }

/* Macro for saving statistics
 * "stat" denotes an element from the statistics_t struct (e.g. CoM means Data_CoM_Sum and Data_CoM_StDev) , while
 * "n" denotes the number of parameters to save
 */
#define SAVE_STATISTICS(stat, n) \
    static int z; \
    static double temp_value; \
    static double temp_time; \
    temp_time = ActualStatUtils.ElapsedTime; \
    if (STEADYSTAT == ActualSaveModes.Save##stat) { \
        temp_time -=  ActualSitParams.StartOfSteadyState; \
    }; \
    if (STEADYSTAT == ActualSaveModes.Save##stat || STAT == ActualSaveModes.Save##stat) { \
        fprintf (f_##stat, "%lf\t", temp_time); \
        fprintf (f_##stat##_StDev, "%lf\t", temp_time); \
        for (z = 0; z < n; z++) { \
            temp_value = ActualStatistics.Data_##stat##_Sum[z] / temp_time; \
            fprintf (f_##stat, "%lf", temp_value); \
            fprintf (f_##stat##_StDev, "%lf", \
                    sqrt (ActualStatistics.Data_##stat##_StDev[z] / temp_time - pow (temp_value, 2))); \
            if (z < n - 1) { \
                fprintf (f_##stat, "\t"); \
                fprintf (f_##stat##_StDev, "\t"); \
            } else {\
                fprintf (f_##stat, "\n"); \
                fprintf (f_##stat##_StDev, "\n"); \
            } \
        } \
        fclose (f_##stat##_StDev); \
    }

/* Setting up initial values for all statistical properties
 */
void ResetStatistics(statistics_t * Statistics);

/////////////////////////////////////////////////////////////////////////////////
/* General stat functions */

/* Returns an array that contains the average, deviation, minimum and maximum of
 * distance between units
 */
double *StatOfDistanceBetweenUnits(phase_t * Phase);

/* Returns an array that contains the average, deviation, minimum and maximum of
 * distance between nearest neighbours
 */
double *StatOfDistanceBetweenNearestNeighbours(phase_t * Phase);

/* Returns an array that contains the average, deviation, minimum and maximum of
 * velocity Length
 * and the Length and XYZ components of the average velocity
 */
double *StatOfVelocity(phase_t * Phase);

/* Returns an array that contains the average, deviation, minimum and maximum of
 * velocity scalar products
 */
double *StatOfCorrelation(phase_t * Phase);

/* Returns an array that contains the average, deviation, minimum and maximum of
 * accelerations
 */
double *StatOfAcceleration(double *Accelerations, const double NumberOfAgents);

/* Calculating a parameter that is proportional with
 * the number of collisions and dangerous situations.
 */
double RatioOfDangerousSituations(phase_t * Phase, const double RadiusOfCopter);

#endif
