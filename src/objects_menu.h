/* vim:set ts=4 sw=4 sts=4 et: */

/* Tools for drawing menu objects
 */

#ifndef OBJECTS_MENU_H
#define OBJECTS_MENU_H

#define HEIGHT_OF_MENU_ITEM 0.113

#include "vizualizer/objects_2d.h"
#include "algo.h"
#include "utilities/param_utils.h"

/* Draws a menu item (a parameter of the model) with given label, unit of measurement and value.
 * "key" is the button referred to the parameter
 * "Y" defines the position of the item
 * "color" is an array that contains RGB values
 */
void DrawMenuItem(const char key, const char *MenuLabel,
        const char *UnitOfMeasurement, const char *Value, const float Y,
        const float *color);

/* Displays two triangles next to the selected menu item
 * like this:
 * |> menu label <|
 * "x" and "y" are the position of the selection in GL coordinates
 * "Length" is the distance between the two triangles
 * "color" is an array that contains RGB values
 */
void DrawMenuSelection(const double x, const double y, const double Length,
        const float *color);

/* Displays a parameter of the unit model
 * "key" is the button referred to the parameter
 * "Multiplier" is a Multiplier of the output. Useful for changing between cm and m, for example
 * "Digits" is the number of Digits displayed after decimal point. (possible values are 0, 1, 2 or 3)
 * "color" is an array that contains RGB values
 */
void DrawMenuItemOfUnitModel(const char key, const unit_param_double_t * Param,
        const int Digits, const double Multiplier, const double y,
        const float *color);

/* Displays a menusystem from a struct of model parameters (unit model)
 * "color" is an array that contains RGB values
 */
void DrawMenuItemsOfUnitModel(const unit_model_params_t * Params,
        const float *color);

/* Displays a menusystem from a struct of model parameters (flocking model)
 * "color" is an array that contains RGB values
 */
void DrawMenuItemsOfFlockingModel(const flocking_model_params_t * Params,
        const float *color);

#endif
