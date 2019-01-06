//MIT License
//Copyright (c) 2018 Eotvos Lorand University, Budapest

/* vim:set ts=4 sw=4 sts=4 et: */

#ifndef DYNAMICS_UTILS_H
#define DYNAMICS_UTILS_H

#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "datastructs.h"
#include "math_utils.h"
#include "param_utils.h"

/* Array that containts noise boolean variable
 * If Noise[i] is "false", then no output noise will be added to its acceleration
 * and the measured position will not fluctuate
 *
 * Default value is "true", but it can be changed individually for all agents
 * in the "InitializeInnerStates function
 *
 */
bool *Noises;

/* Functions for manipluating phase spaces and dynamics.
 */

/* Struct that contains every agents' coordinates, velocities
 * and inner states
 *
 * Note that innner states are always double variables and
 * the number of them is determined by the "NumberOfInnerStates" variable
 *
 */
typedef struct {

    double **Coordinates;
    double **Velocities;
    double **InnerStates;
    int *RealIDs;
    int NumberOfInnerStates;
    int NumberOfAgents;

} phase_t;

/* Simple tools for allocating phase space and cleaning memory
 */
void AllocatePhase(phase_t * Phase, const int NumberOfAgents,
        const int NumberOfInnerStates);
void freePhase(phase_t * Phase);

/* Insert specific agent's coordinates and velocity (denoted by "Agent") into
 * an existing phase space denoted by "Phase".
 * "WhichAgent" denotes the ID number of the agent.
 */
void InsertAgentPhase(phase_t * Phase, double *Agent, const int WhichAgent);

/* Get specific agent's coordinates and velocity (denoted by "Agent") from
 * an existing phase space denoted by "Phase".
 * "WhichAgent" denotes the ID number of the agent.
 */
void GetAgentPhase(double *Agent, phase_t * Phase, const int WhichAgent);

/* Get specific agent's velocity vector (denoted by "Velocity") from
 * an existing phase space denoted by "Phase".
 * "WhichAgent" denotes the ID number of the agent.
 */
void GetAgentsVelocity(double *Velocity, phase_t * Phase, const int WhichAgent);

/* Insert specific agent's velocity vector (denoted by "Velocity") into
 * an existing phase space denoted by "Phase".
 * "WhichAgent" denotes the ID number of the agent.
 */
void InsertAgentsVelocity(phase_t * Phase, double *Velocity,
        const int WhichAgent);

/* Get specific agent's coordinate vector (denoted by "Coords") from
 * an existing phase space denoted by "Phase".
 * "WhichAgent" denotes the ID number of the agent.
 */
void GetAgentsCoordinates(double *Coords, phase_t * Phase,
        const int WhichAgent);

/* Get specific agent's extrapolated coordinate vector (denoted by "Coords") from
 * an existing phase space denoted by "Phase", the Delay time and velocity
 * "WhichAgent" denotes the ID number of the agent.
 */
void GetAgentsCoordinatesEp(double *Coords, phase_t * Phase,
        const int WhichAgent, const double Delay);

 /* Insert specific agent's coordinate vector (denoted by "Coords") into
  * an existing phase space denoted by "Phase".
  * "WhichAgent" denotes the ID number of the agent.
  */
void InsertAgentsCoordinates(phase_t * Phase, double *Coords,
        const int WhichAgent);

/* Calculate number of agents that are closer than "RadiusOfLocalArea" to a specific agent
 * denoted by its ID ("WhichAgent")
 * With "Phase", the actual phase space is denoted.
 */
int NumberOfNearbyUnits(phase_t * Phase, const int WhichAgent,
        const double RadiusOfLocalArea);

/* Calculates euclidean distance between two agents denoted by their ID number ("A1" and "A2")
 * "Dim" determines the number of spatial dimensions (can be 2 or 3)
 * With "Phase", the actual phase space is denoted.
 */
double DistanceBetweenAgents(phase_t * Phase, const int A1, const int A2,
        const int Dim);

/* Gets global centre of mass (CoM) from a phase space "Phase"
 * CoM coordinates will be placed into "CoMCoord" vector.
 */
void GetCoM(double *CoMCoord, phase_t * Phase);

/* Gets local centre of mass (CoM) from a phase space ("Phase")
 * from a viewpoint of a specific agent denoted by its ID number ("WhichAgent")
 * Vision range of an agent is defined as "RadiusOfLocalArea".
 * CoM coordinates will be placed into "CoMCoord" vector.
 */
void GetLocalCoM(double *CoMCoord, phase_t * Phase, const int WhichAgent,
        const double RadiusOfLocalArea);

/* Calculating CoM for specific agents
 *
 * Phase space is denoted as "Phase"
 * CoM coordinates will be placed into "CoMCoord" vector.
 * "ExceptWhichAgents" is a set of ID numbers (this agents will not be taken into account)
 *
 */
void GetAgentSpecificCoM(double *CoMCoord, phase_t * Phase,
        int *ExceptWhichAgents);

/* Calculating local CoM for specific agents
 * from a viewpoint of a specific agent denoted by its ID number ("AroundWhichAgent")
 *
 * Phase space is denoted as "Phase"
 * CoM coordinates will be placed into "CoMCoord" vector.
 * "ExceptWhichAgents" is a set of ID numbers (this agents will not be taken into account)
 * "NeighbourhoodRange" defines the vision range of "AroundWhichAgent"th agent.
 *
 */
void GetAgentSpecificLocalCoM(double *CoMCoord, phase_t * Phase,
        const int AroundWhichAgent,
        int *ExceptWhichAgents, const double NeighbourhoodRange);

/* Calculates average velocity vector and place it into the "CoMVel" vector.
 * "Phase" is the actual phase space.
 */
void GetAvgOfVelocity(double *CoMVel, phase_t * Phase);

/* Calculates local average XY velocity vector
 * from the viewpoint of the "WhichAgent"th unit and place it into the "CoMVel" vector.
 * "Phase" is the actual phase space, "AreaRadius" is the vision range of the "WhichAgent"th agent.
 */
void GetLocalAverageOfXYVelocity(double *CoMVel, phase_t * Phase,
        const int WhichAgent, const double AreaRadius);

/* Get average of tangential velocities around a centre point denoted by "RefPoint".
 * Final output velocity will be placed in "OutputLAPVel".
 */
void GetAverageOfXYTangentialVelocity(double *OutputLAPVel, phase_t * Phase,
        double *RefPoint, const int WhichAgent);

/* Get local average of tangential velocities from the viewpoint of the "WhichAgent"th agent,
 * around a centre point denoted by "RefPoint"
 *
 * "AreaSize" denotes the vision range of the agent.
 * Final output velocity will be placed in "OutputLAPVel".
 */
void GetLocalAverageOfXYTangentialVelocity(double *OutputLAPVel,
        phase_t * Phase, double *RefPoint,
        const int WhichAgent, const double AreaSize);

/* Get local average of tangential velocities from the viewpoint of the "WhichAgent"th agent,
 * around a centre point denoted by "RefPoint"
 * with the exception of the agents listed in "ExceptWhichAgents" interger array.
 *
 * "AreaSize" denotes the vision range of the agent.
 * Final output velocity will be placed in "OutputLAPVel".
 */
void GetAgentSpecificLocalAverageOfXYTangentialVelocity(double *OutputLAPVel,
        phase_t * Phase, double *RefPoint, int
        *ExceptWhichAgents, const int WhichAgent, const double AreaSize);

/* Get local average of tangential velocity on a specific plane.
 * The meaning of the other variables are the same as presented above.
 *
 * This plane is defined by its normal vector, "Axis"
 */
void GetLocalAverageOfTangentialVelocity(double *OutputLAPVel,
        phase_t * Phase,
        double *RefPoint,
        double *Axis, const int WhichAgent, const double AreaSize);

/* Get coordinates from a timeline (timeline means a set of phase spaces)
 * "WhichStep" defines the timestep, "WhichAgents" is the ID number of the specific agent,
 * "PhaseData" is the timeline and "Coords" is the output position vector.
 */
void GetAgentsCoordinatesFromTimeLine(double *Coords, phase_t * PhaseData,
        const int WhichAgent, const int WhichStep);

/* Get velocity vector from a timeline (timeline means a set of phase spaces)
 * "WhichStep" defines the timestep, "WhichAgents" is the ID number of the specific agent,
 * "PhaseData" is the timeline and "Coords" is the output position vector.
 */
void GetAgentsVelocityFromTimeLine(double *Velocity, phase_t * PhaseData,
        const int WhichAgent, const int WhichStep);

/* Inserting positions and velocities into timeline
 */
void InsertPhaseToDataLine(phase_t * PhaseData, phase_t * Phase,
        const int WhichStep);

/* Inserting inner states into inner state timeline
 */
void InsertInnerStatesToDataLine(phase_t * PhaseData, phase_t * Phase,
        const int WhichStep);

/* Shifting position and velocity data line
 */
void ShiftDataLine(phase_t * PhaseData, const int HowManyRows,
        const int HowManyRowsToSave);

/* Shifting inner state data line
 */
void ShiftInnerStateDataLine(phase_t * PhaseData, const int HowManyRows,
        const int HowManyRowsToSave);

/* Setting up random Initial conditions (with zero velocities and randomly distributed positions.)
 */
void InitCond(phase_t ** PhaseData,
        const double InitialSizeX,
        const double InitialSizeY,
        const double InitialSizeZ, const double SizeOfCopter);

/* Waiting - filling up timelines with initial conditions */
void Wait(phase_t * PhaseData, const double TimeToWait, const double h);

/* Setting up random conditions (with zero velocities)
 */
void RandomizePhase(phase_t * Phase, const double XSize, const double YSize,
        const double ZSize, const double XCenter,
        const double YCenter, const double ZCenter,
        const int fromAgent, const int toAgent, const double RadiusOfCopter);

/* Setting up random conditions inside a ring (with zero velocities)
 */
void PlaceAgentsInsideARing(phase_t * Phase, const double SizeOfRing,
        const int fromAgent, const int toAgent,
        const double XCenter, const double YCenter,
        const double ZCenter, const double ZSize, const double RadiusOfCopter);

/* Setting up random positions inside a sphere
 */
void PlaceAgentsInsideASphere(phase_t * Phase, const double SizeOfSphere,
        const int fromAgent, const int toAgent,
        const double XCenter, const double YCenter,
        const double ZCenter, const double RadiusOfCopter);

/* Setting up random initial conditions on a specific plane
 * A local XY coordinate system is given on the plane by "XAxis" and the normalvector of the plane ("PlaneNormalVect")
 */
void RandomizePhaseOnPlane(phase_t * Phase,
        const double XSize,
        const double YSize,
        const double XCenter,
        const double YCenter,
        const double ZCenter,
        double *PlaneNormalVect,
        double *XAxis,
        const int fromAgent, const int toAgent, const double RadiusOfCopter);
void PlaceAgentsOnXYPlane(phase_t * Phase,
        const double XSize,
        const double YSize,
        const double XCenter,
        const double YCenter,
        const double ZCenter,
        const int fromAgent, const int toAgent, const double RadiusOfCopter);
void PlaceAgentsOnXZPlane(phase_t * Phase,
        const double XSize,
        const double ZSize,
        const double XCenter,
        const double YCenter,
        const double ZCenter,
        const int fromAgent, const int toAgent, const double RadiusOfCopter);
void PlaceAgentsOnYZPlane(phase_t * Phase,
        const double YSize,
        const double ZSize,
        const double XCenter,
        const double YCenter,
        const double ZCenter,
        const int fromAgent, const int toAgent, const double RadiusOfCopter);

/* Placing agents onto specific lines */
void PlaceAgentsOntoALine(phase_t * Phase, const int fromAgent,
        const int toAgent, double *Tangential,
        const double XCenter, const double YCenter,
        const double ZCenter, const double LineLength,
        const double RadiusOfCopter);

/* Counting collisions (this is for internal use only :) )
 */
int HowManyCollisions(phase_t * ActualPhase, bool * AgentsInDanger,
        const bool CountCollisions, const double RadiusOfCopter);

/* Swaps the states of two agents (ith and jth)
 */
void SwapAgents(phase_t * Phase, const int i, const int j);

/* Orders agents by distance from a given position
 */
void OrderAgentsByDistance(phase_t * Phase, double *ReferencePosition);

/* Packing of nearby agents to the first blocks of the phase space
 * Returns the number of agents which are closer than R_C and whose packets
 * are not lost
 */
int SelectNearbyVisibleAgents(phase_t * Phase, double *ReferencePosition,
        const double Range, const double PacketLossQuadraticCoeff);

#endif
