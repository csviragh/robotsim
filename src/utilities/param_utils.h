//MIT License
//Copyright (c) 2018 Eotvos Lorand University, Budapest

/* vim:set ts=4 sw=4 sts=4 et: */

/* This file contains useful structs for creating a model
 * and drawing a menu system automatically
 * Tools for handling strings and input files are also declared here.
 */

#ifndef PARAM_UTILS_H
#define PARAM_UTILS_H

#include "file_utils.h"
#include <stdio.h>
#include <stdbool.h>

/* Number of parameter sets */
static int NumberOfFlockingParamSets = 0;
static int NumberOfUnitParamSets = 0;
static int SelectedFlockingParamSet = 0;
static int SelectedUnitParamSet = 0;

/* Max number of flocking model parameters */
#define MAX_FLOCKING_PARAMS 64

/* Macro to create variables */
#define CREATE_FLOCKING_PARAM(n, ...) \
    CreateFlockingParam(FlockingParams, (fl_param_double_t) { \
        .NameInFiles = #n, \
        .StaticValuePointer = &n, \
        .InMenu = true, \
        .Constant = false, \
        .NumberOfLabels = 0, \
        __VA_ARGS__});

/* Hidden variable (not visible in menu system) */
#define CREATE_HIDDEN_FLOCKING_PARAM(n, ...) \
    CreateFlockingParam(FlockingParams, (fl_param_double_t) { \
        .NameInFiles = #n, \
        .StaticValuePointer = &n, \
        .InMenu = false, \
        .Constant = false, \
        .NumberOfLabels = 0, \
        __VA_ARGS__});

/* Constant variable (Changes only when F12 is pressed) */
#define CREATE_CONSTANT_FLOCKING_PARAM(n, ...) \
    CreateFlockingParam(FlockingParams, (fl_param_double_t) { \
        .NameInFiles = #n, \
        .StaticValuePointer = &n, \
        .InMenu = true, \
        .Constant = true, \
        .NumberOfLabels = 0, \
        __VA_ARGS__});

/* Constant hidden variable (Changes only when F12 is pressed and not visible in menu system) */
#define CREATE_HIDDEN_CONSTANT_FLOCKING_PARAM(n, ...) \
    CreateFlockingParam(FlockingParams, (fl_param_double_t) { \
        .NameInFiles = #n, \
        .StaticValuePointer = &n, \
        .InMenu = false, \
        .Constant = true, \
        .NumberOfLabels = 0, \
        __VA_ARGS__});

/* Searching for correct parameter and change the value of it */
#define CHANGE_PARAMETER(param_Name, param_value) \
    jj = FlockingParams->NumberOfParameters; \
    while (jj >= -1 && strcmp(FlockingParams->Params[jj].NameInFiles, #param_Name) != 0) { jj --; } \
    if (jj > -1) {FlockingParams->Params[jj].Value = param_value; } \
    RefreshFlockingParams (FlockingParams);

/* Searching for correct parameter and change the possible maximum of it */
#define CHANGE_PARAMETER_MAX(param_Name, param_max_value) \
    jj = FlockingParams->NumberOfParameters; \
    while (jj >= -1 && strcmp(FlockingParams->Params[jj].NameInFiles, #param_Name) != 0) { jj --; } \
    if (jj > -1) {FlockingParams->Params[jj].Max = param_max_value; } \
    RefreshFlockingParams (FlockingParams);

/* Searching for correct parameter and change the possible minimum of it */
#define CHANGE_PARAMETER_MIN(param_Name, param_min_value) \
    jj = FlockingParams->NumberOfParameters; \
    while (jj >= -1 && strcmp(FlockingParams->Params[jj].NameInFiles, #param_Name) != 0) { jj --; } \
    if (jj > -1) {FlockingParams->Params[jj].Min = param_min_value; } \
    RefreshFlockingParams (FlockingParams);

/* Setting up a label-set for a given parameter. 
 */
#define SET_LABEL_FOR_PARAMETER_VALUE(param_Name, param_value, param_label) \
    if (sizeof(param_label) > 7) {fprintf (stderr, "Parameter value label should be shorter than 7 characters!\n"); exit(-1);} \
    tt = FlockingParams->NumberOfParameters; \
    while (tt >= -1 && strcmp(FlockingParams->Params[tt].NameInFiles, #param_Name) != 0) { tt --; } \
    if (tt > -1) { \
        strcpy (FlockingParams->Params[tt].Labels.Captions[FlockingParams->Params[tt].NumberOfLabels], param_label); \
        FlockingParams->Params[tt].Labels.Values[FlockingParams->Params[tt].NumberOfLabels] = param_value; \
        (FlockingParams->Params[tt].NumberOfLabels)++; \
    } \
    RefreshFlockingParams (FlockingParams);

/* Model of a robot */

/* Parameter of the model of a unit (double)
 */
typedef struct {

    /* Name  (for menu system and input files) */
    char Name[64];
    /* unit of measurement (for menu system) */
    char UnitOfMeas[10];
    /* value in double precision */
    double Value;

} unit_param_double_t;

/* Parameters of the model of a single unit
 */
typedef struct {

    /* Relaxation times of PID controller */
    unit_param_double_t Tau_PID_XY;
    unit_param_double_t Tau_PID_Z;
    /* Maximum acceleration */
    unit_param_double_t a_max;
    /* Refresh rate of GPS device */
    unit_param_double_t t_GPS;
    /* Time delay of communication */
    unit_param_double_t t_del;
    /* Range of communication */
    unit_param_double_t R_C;

    /* Packet loss is a random process modelled with quadradically increasing
       probability with distance, reaching a given packet_loss_ratio at
       packet_loss_distance, i.e., y = ax^2 where a = ratio/distance^2 */
    unit_param_double_t packet_loss_ratio;      /* 0-1 */
    unit_param_double_t packet_loss_distance;   /* 0- */

    /* Noise parameters */

    /* Noises are modelled with Wiener processes
     * with given Constant Sigma.
     * Inner noises are the inaccuracy of the sensors (e.g. GPS),
     * outer noises are the effects of the wind, inaccuracy of PID controller, etc
     */

    /* stdev of inner noise (XY) */
    unit_param_double_t Sigma_GPS_XY;
    /* stdev of outer noise (XY) */
    unit_param_double_t Sigma_Outer_XY;
    /* stdev of inner noise (Z) */
    unit_param_double_t Sigma_GPS_Z;
    /* stdev of outer noise (Z) */
    unit_param_double_t Sigma_Outer_Z;

    /* Effects of wind (stdev, magnitude avg, angle) */
    unit_param_double_t Wind_Magn_Avg;
    unit_param_double_t Wind_StDev;
    unit_param_double_t Wind_Angle;
    unit_param_double_t ViscosityCoeff;

    char FileName[512];

} unit_model_params_t;

/* Flocking model basic structure */

/* parameter of the flocking model (double)
 */
typedef struct {

    /* Name  (for menu system and input files) */
    char Name[64];
    /* unit of measurement (for menu system) */
    char UnitOfMeas[10];
    /* value in double precision */
    double Value;

    /* Parameters for display parameter in the menu system... */

    /* number of Digits (menu display) */
    int Digits;
    /* size of step (when changing) */
    double SizeOfStep;
    /* Multiplication (for example 1.0/100.0 between cm and m) */
    double Mult;

    /* min and max values */
    double Min;
    double Max;

    /* Name in the input file */
    char NameInFiles[64];

    /* Is it appears in the menu? */
    bool InMenu;

    /* Can be changed during measurement? 
     * Or can only be changed by pressing F12? */
    bool Constant;
    double StoredValue;

    /* This variable is for inner use to be able to define static
       parameters to use the param values simply. */
    double *StaticValuePointer;

    /* With this struct, up to 32 labels can be added 
     * for better representation of parameter values in menus during visualization.
     * Each label can be 7 character long
     */
    int NumberOfLabels;
    struct {
        double Values[32];
        char Captions[32][7];
    } Labels;

} fl_param_double_t;

/* Struct to store parameters of the flocking model
 */
typedef struct {

    char FileName[256];

    int NumberOfParameters;
    fl_param_double_t Params[MAX_FLOCKING_PARAMS];      /* increase if more parameters are needed */

    /* With the variables below, model-specific input files can be defined with specific option flags
     */
    int NumberOfInputs;
    char *Inputs[64];

    /* Number of inner states */
    int NumberOfInnerStates;

} flocking_model_params_t;

/* Other */

/* Parameters of the actual situation
 * e. g. number of agents, Initial area size, etc.
 */
typedef struct {

    /* Number of agents (cannot be changed during a measurement) */
    int NumberOfAgents;
    /* Initial X, Y, Z positions */
    double InitialX;
    double InitialY;
    double InitialZ;
    /* Length of the measurement (sec) */
    double Length;
    /* Accuracy of the Euler method for solving the ODE */
    double DeltaT;

    /* Radius of dangerous area around copters */
    double Radius;

    /* Time length to store */
    double LengthToStore;

    /* Default visualization speed */
    int VizSpeedUp;

    /* Estimated tarting time of steady state - 
     * Averaging order parameters is only neccessary from here.
     */
    double StartOfSteadyState;

} sit_parameters_t;

/* Functions for setting parameters */

/* Changes one of the 'robot model' parameters
 * "WhichParam" is the index of the parameter
 * "StepSize" will be added to the value of the parameter
 * "DeltaT" is the accuracy of the Euler method
 */
void ChangeUnitModelParameter(unit_model_params_t * Params, int WhichParam,
        double StepSize, double DeltaT);

void SetNamesOfUnitModelParams(unit_model_params_t * UnitParams);

/* Gets the parameter set 
 * {TauPID_XY, Tau_PID_Z, t_GPS, simga_GPS_XY, Sigma_GPS_Z, Sigma_Outer_XY, Sigma_Outer_Z, t_del, R_C, ...}
 * from "InputFile" 
 */
void GetUnitModelParamsFromFile(unit_model_params_t * UnitParams,
        FILE * InputFile);

/* Saves the parameter set 
 * {TauPID, t_GPS, simga_GPS_XY, Sigma_GPS_Z, Sigma_Outer_XY, Sigma_Outer_Z, t_del, R_C, ...}
 * into "OutputFile" 
 */
void SaveUnitModelParamsToFile(FILE * OutputFile,
        unit_model_params_t * UnitParams);

/* Gets the parameter set 
 * {Number of agents, Initial sizes {x, y, z}, Length of measurement, DeltaT}
 * from "InputFile" 
 */
sit_parameters_t GetSituationParamsFromFile(FILE * InputFile);

/* Changes one of the 'flocking algorithm' parameters
 */
void ChangeFlockingModelParameter(flocking_model_params_t * FlockingParams,
        const int WhichParam, const double StepSize);

/* Saves the parameter set of the specific algorithm
 */
void SaveFlockingModelParamsToFile(FILE * OutputFile,
        flocking_model_params_t * FlockingParams);

/* Get the index of a flocking model parameter in the model params list
 * based on its Name.
 *
 * Returns -1 if not found.
 */
int GetFlParamIndexByName(flocking_model_params_t * FlockingParams,
        const char *Name);

/* Allocate memory for a param and store it in the params structure.
 *
 * Note that this function must be called
 * in the InitializeFlockingParams() function of your model file!!!
 */
void CreateFlockingParam(flocking_model_params_t * FlockingParams,
        fl_param_double_t Param);

/* Refresh static double variables defined for flocking model parameters
 * in the model file from the main parameter structure, to simplify usage.
 *
 * Note that this function must be called only from your model file!!!
 */
void RefreshFlockingParams(flocking_model_params_t * FlockingParams);

/*
 */
bool FillParameterSetsFromFile(flocking_model_params_t * FlockingParamSets,
        unit_model_params_t * UnitParamSets,
        int argc,
        int *NumberOfFlockingModelParamSets,
        int *NumberOfUnitModelParamSets,
        char *argv[], char *CurrentDirectory, char *ParamsFileName);

#endif
