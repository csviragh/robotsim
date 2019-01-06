//MIT License
//Copyright (c) 2018 Eotvos Lorand University, Budapest

/* vim:set ts=4 sw=4 sts=4 et: */

/* Basic types and structs for modes
 */

#ifndef VIZMODE_H
#define VIZMODE_H

#include <stdbool.h>

/* This struct contains the details of the actual vizualization mode. 
 */
typedef struct {

    /* Vizualization ON / OFF */
    bool VizEnabled;
    bool ExperimentOver;

    /* Visualization Speed */
    int VizSpeedUp;
    bool Paused;
    /* Two or three dimensions? */
    bool TwoDimViz;
    /* CoM at the center of the screen? */
    bool CoMFollowing;
    /* Which agent is selected? */
    /* "NumberOfAgents+1" means that agent-following is off */
    int WhichAgentIsSelected;
    /* Display parameters of unit model */
    bool UnitParamsDisplayed;
    /* Display parameters of flocking algorithm */
    bool FlockingParamsDisplayed;
    /* Display GPS ghosts */
    bool DisplayGPSGhosts;
    /* Display labels above agents */
    bool DisplayAgentLabels;
    /* Display Communication network */
    bool DisplayCommNetwork;
    /* Length of displayed tail */
    bool DisplayTail;
    int LengthOfTail;

    /* Which parameter is selected? */
    /* 0 means "Visualization speed" */
    int WhichParamIsSelected;

    /* Map sizes and center */
    double MapSizeXY;
    double MapSizeZ;
    double CenterX;
    double CenterY;
    double CenterZ;

    /* Eye position */
    double EyeX;
    double EyeY;
    double EyeZ;

    float UpX;
    float UpY;
    float UpZ;

    /* Position limits */
    double DistanceFromCenterLimit;
    double DistanceFromCenterLimit_XY;
    double EyeZLimit;

    /* Camera trajectory mode */
    bool CamTraj;

    /* Resolution of the visualization window. It is always square-shaped... */
    double Resolution;

} vizmode_params_t;

#endif
