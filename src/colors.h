/* vim:set ts=4 sw=4 sts=4 et: */

/* Reading and applying color configs
 */

#ifndef COLORS_H
#define COLORS_H

/* Max number of model-specific colors */
#define MAX_NUMBER_OF_COLORS 32

/* Macro for creating model-specific color */
#define CREATE_COLOR(n, r, g, b) \
    ModelSpecificColors[*NumberOfModelSpecificColors].NameInFiles = #n; \
    ModelSpecificColors[*NumberOfModelSpecificColors].RGB_Values[0] = r; \
    ModelSpecificColors[*NumberOfModelSpecificColors].RGB_Values[1] = g; \
    ModelSpecificColors[*NumberOfModelSpecificColors].RGB_Values[2] = b; \
    ModelSpecificColors[*NumberOfModelSpecificColors].StaticValuePointer[0] = &n[0]; \
    ModelSpecificColors[*NumberOfModelSpecificColors].StaticValuePointer[1] = &n[1]; \
    ModelSpecificColors[*NumberOfModelSpecificColors].StaticValuePointer[2] = &n[2]; \
    (*NumberOfModelSpecificColors) ++ ;

#include <stdio.h>
#include <stdlib.h>
#include "utilities/file_utils.h"
#include "utilities/param_utils.h"

/* Struct for storing basic color schemes
 * Every "color" is an RGB value (a float number between 0 and 1)
 * "EraseColor" is the color of the background
 */
typedef struct {

    /* Background */
    float EraseColor[3];

    /* Colors of menusystem */
    float MenuItemColor[3];
    float MenuSelectionColor[3];

    /* Colors of "trajectory window" */
    float AgentsDefaultColor[3];
    float **AgentsColor;        //We can define different colors for each agents

    /* Color of agents in dangerous situation */
    float AgentsInDangerColor[3];
    /* Velocity label and arrow */
    float VelocityLabelColor[3];
    float VelocityArrowColor[3];
    /* User-defined labels */
    float LabelColor[3];

    /* Communication network */
    float CommNetWorkColor[3];

    /* */
    float PausedCaptionColor[3];
    float CollisionCaptionColor[3];
    float ElapsedTimeCaptionColor[3];

    /* 3d Coordinate System color */
    float AxisColor[3];

} color_config_t;

/* Structs for storing model-specific color schemes 
 */
typedef struct {

    char *NameInFiles;
    float RGB_Values[3];

    float *StaticValuePointer[3];

} model_specific_color_t;

/**
 * Creates a color structure from the individual HSV
 * components (where hue is cyclic in 0-255).
 *
 * \param  rgb         the constructed color (3 values in the range 0-1)
 * \param  hue         the hue component
 * \param  saturation  the saturation component
 * \param  value       the value component
 */
void MakeRGBFromHSV(float *rgb, unsigned char hue, unsigned char saturation,
        unsigned char value);

/* Loading color schemes from an ".ini" file
 * "InputFile" should contain all necessary color data
 * example: "background_r=..."
 * The variables below should be in the "InputFile" (with postfix _r, _g or _b):
 * background, menuitem, menuselection, etc...
 */
void LoadColorConfig(color_config_t * ColorConfigToSet, FILE * InputFile,
        int NumberOfAgents, model_specific_color_t * ModelSpecificColors,
        int *NumberOfModelSpecificColors);

/* Setting the color of a specific agent to "color" 
 * "WhichAgent" is the index of the agent
 */
void SetAgentsColor(color_config_t * ColorConfig, const int WhichAgent,
        const int NumberOfAgents, const float *color);

/* Reset the color of every agents to "color"
 */
void ResetAgentsColor(color_config_t * ColorConfig, const int NumberOfAgents,
        const float *color);

/* Load modelspecific colors
 * format: readed_Name=readed_value        
 */
void LoadModelSpecificColors(char *readed_Name, char *readed_value,
        model_specific_color_t * ModelSpecificColors,
        int *NumberOfModelSpecificColors);

#endif
