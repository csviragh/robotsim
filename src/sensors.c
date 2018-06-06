/* vim:set ts=4 sw=4 sts=4 et: */

/* Models of inaccuracy of the sensors
 * "inner noise"
 */

#include "sensors.h"
#include "utilities/datastructs.h"
#include <math.h>

/* Setting up initial values of GPS-measured positions and velocities */
void ResetGPSNoises(phase_t * GPSPhase, phase_t * GPSDelayedPhase) {

    int i, j;
    for (j = 0; j < GPSPhase->NumberOfAgents; j++) {
        for (i = 0; i < 3; i++) {
            GPSPhase->Coordinates[j][i] = 0;
            GPSPhase->Velocities[j][i] = 0;
            GPSDelayedPhase->Coordinates[j][i] = 0;
            GPSDelayedPhase->Velocities[j][i] = 0;
        }
    }

}

/* Fluctuations of GPS signal */
void StepGPSNoises(phase_t * WhichPhase, unit_model_params_t * UnitParams) {

    /* GPS fluctuations are modelled as a random Gaussian noise with damping in a central potantial around the real position of the agent.
     */
    static double GPSPosition[3];
    NullVect(GPSPosition, 3);
    static double GPSVelocity[3];
    NullVect(GPSVelocity, 3);

    static double GPSNoiseToAdd[3];
    static double Force[3];     // Central potential 
    static double Damping[3];   // Damping which is proportional with GPS-measured velocity. 

    int i;

    for (i = 0; i < WhichPhase->NumberOfAgents; i++) {

        if (Noises[i] == false)
            continue;

        GetAgentsCoordinates(GPSPosition, WhichPhase, i);
        GetAgentsVelocity(GPSVelocity, WhichPhase, i);

        /* Random acceleration vector */
        NullVect(GPSNoiseToAdd, 3);
        GPSNoiseToAdd[0] = randomizeGaussDouble(0, 1);
        GPSNoiseToAdd[1] = randomizeGaussDouble(0, 1);
        GPSNoiseToAdd[2] = randomizeGaussDouble(0, 1);

        /* Quadratic potential with friction */
        static double lambda_GPS_XY = 0.1;
        static double lambda_GPS_Z = 0.1;
        double D_GPS_XY =
                sqrt(2 * lambda_GPS_XY * UnitParams->Sigma_GPS_XY.Value) / 300;
        double D_GPS_Z =
                sqrt(2 * lambda_GPS_Z * UnitParams->Sigma_GPS_Z.Value) / 300;

        /* Spring-like force */
        NullVect(Force, 3);
        VectSum(Force, Force, GPSPosition);
        UnitVect(Force, Force);
        MultiplicateWithScalar(Force, Force,
                -D_GPS_XY * VectAbsXY(GPSPosition) * UnitParams->t_GPS.Value,
                2);
        Force[2] *= -D_GPS_Z * fabs(GPSPosition[2]) * UnitParams->t_GPS.Value;

        /* Damping */
        NullVect(Damping, 3);
        VectSum(Damping, Damping, GPSVelocity);
        MultiplicateWithScalar(Damping, Damping,
                -lambda_GPS_XY * UnitParams->t_GPS.Value, 2);
        Damping[2] *= -lambda_GPS_Z * UnitParams->t_GPS.Value;

        /* Sigma_GPS refers to an anisotropic diffusion constant */
        MultiplicateWithScalar(GPSNoiseToAdd, GPSNoiseToAdd,
                sqrt(2 * lambda_GPS_XY * UnitParams->t_GPS.Value *
                        UnitParams->Sigma_GPS_XY.Value), 2);
        GPSNoiseToAdd[2] *=
                sqrt(2 * lambda_GPS_Z * UnitParams->t_GPS.Value *
                UnitParams->Sigma_GPS_Z.Value);
        VectSum(GPSNoiseToAdd, GPSVelocity, GPSNoiseToAdd);

        /* Add GPS-measured terms to the real terms to get full measured velocities and positions */
        VectSum(GPSNoiseToAdd, GPSNoiseToAdd, Force);
        VectSum(GPSNoiseToAdd, GPSNoiseToAdd, Damping);

        InsertAgentsVelocity(WhichPhase, GPSNoiseToAdd, i);

        MultiplicateWithScalar(GPSNoiseToAdd, GPSNoiseToAdd,
                UnitParams->t_GPS.Value, 3);

        VectSum(GPSPosition, GPSPosition, GPSNoiseToAdd);

        InsertAgentsCoordinates(WhichPhase, GPSPosition, i);

    }

}
