/* vim:set ts=4 sw=4 sts=4 et: */

/* Tools for drawing menu objects
 */

#include "objects_menu.h"
#include <math.h>

/* Draws a menu item (a parameter of the model) with given label, unit of measurement and value */
void DrawMenuItem(const char key,
        const char *MenuLabel,
        const char *UnitOfMeasurement,
        const char *Value, const float Y, const float *color) {

    glColor3f(color[0], color[1], color[2]);

    DrawTransparentShape(-0.9, Y - 0.025, 1.8, HEIGHT_OF_MENU_ITEM, color);

    static char keylabel[3];
    sprintf(keylabel, "'%c'", key);

    static double TopOfMenuItem;
    TopOfMenuItem = HEIGHT_OF_MENU_ITEM * 0.5 + Y;

    DrawString(-0.8, TopOfMenuItem - 0.015, GLUT_BITMAP_9_BY_15, keylabel,
            color);
    DrawString(-0.6, TopOfMenuItem - 0.015, GLUT_BITMAP_8_BY_13, MenuLabel,
            color);
    DrawString(-0.6, TopOfMenuItem - 0.07, GLUT_BITMAP_8_BY_13,
            UnitOfMeasurement, color);
    DrawString(0.6, TopOfMenuItem - 0.03, GLUT_BITMAP_9_BY_15, Value, color);

}

/* Displays two triangles next to the selected menu item */
void DrawMenuSelection(const double x, const double y, const double Length,
        const float *color) {

    DrawTriangle(x, y, Length, 0, color);
    DrawTriangle(x + 1.9, y, Length, M_PI, color);

}

/* Displays a menu item containing features of a parameter of a unit */
void DrawMenuItemOfUnitModel(const char key,
        const unit_param_double_t * UnitParams, const int Digits,
        const double Multiplier, const double y, const float *color) {

    static char value[10];

    /* Setting up Digits after decimal point */
    switch (Digits) {
    case 0:
        sprintf(value, "%1.0f", Multiplier * UnitParams->Value);
        break;
    case 1:
        sprintf(value, "%1.1f", Multiplier * UnitParams->Value);
        break;
    case 2:
        sprintf(value, "%1.2f", Multiplier * UnitParams->Value);
        break;
    case 3:
        sprintf(value, "%1.3f", Multiplier * UnitParams->Value);
        break;
    default:
        sprintf(value, "%1.2f", Multiplier * UnitParams->Value);
        break;
    }

    DrawMenuItem(key, UnitParams->Name, UnitParams->UnitOfMeas, value, y,
            color);

}

/* Displays a menusystem from a struct of model parameters (unit model) */
void DrawMenuItemsOfUnitModel(const unit_model_params_t * UnitParams,
        const float *color) {

    static double Offset = 0.0;

    /* Draw title */
    DrawString(-0.8, 0.92, GLUT_BITMAP_9_BY_15, "Parameters of the robot model",
            color);

    /* Control algorith parameters */
    DrawMenuItemOfUnitModel('1', &UnitParams->Tau_PID_XY, 2, 1.0, 0.8 - Offset,
            color);
    DrawMenuItemOfUnitModel('2', &UnitParams->Tau_PID_Z, 2, 1.0,
            0.8 - HEIGHT_OF_MENU_ITEM - Offset, color);
    DrawMenuItemOfUnitModel('3', &UnitParams->a_max, 1, 0.01,
            0.8 - HEIGHT_OF_MENU_ITEM * 2 - Offset, color);
    /* Parameters of communication */
    DrawMenuItemOfUnitModel('4', &UnitParams->R_C, 1, 0.01,
            0.8 - HEIGHT_OF_MENU_ITEM * 3 - Offset, color);
    DrawMenuItemOfUnitModel('5', &UnitParams->t_del, 2, 1.0,
            0.8 - HEIGHT_OF_MENU_ITEM * 4 - Offset, color);
    /* Parameters of sensors */
    DrawMenuItemOfUnitModel('6', &UnitParams->t_GPS, 2, 1.0,
            0.8 - HEIGHT_OF_MENU_ITEM * 5 - Offset, color);
    DrawMenuItemOfUnitModel('7', &UnitParams->Sigma_GPS_XY, 3, 0.0001,
            0.8 - HEIGHT_OF_MENU_ITEM * 6 - Offset, color);
    DrawMenuItemOfUnitModel('8', &UnitParams->Sigma_GPS_Z, 3, 0.0001,
            0.8 - HEIGHT_OF_MENU_ITEM * 7 - Offset, color);
    /* Parameters of environmental noises */
    DrawMenuItemOfUnitModel('9', &UnitParams->Sigma_Outer_XY, 2, 0.0001,
            0.8 - HEIGHT_OF_MENU_ITEM * 8 - Offset, color);
    DrawMenuItemOfUnitModel('a', &UnitParams->Sigma_Outer_Z, 2, 0.0001,
            0.8 - HEIGHT_OF_MENU_ITEM * 9 - Offset, color);
    DrawMenuItemOfUnitModel('b', &UnitParams->Wind_Magn_Avg, 2, 0.01,
            0.8 - HEIGHT_OF_MENU_ITEM * 10 - Offset, color);
    /* Parameters of packet loss */
    DrawMenuItemOfUnitModel('c', &UnitParams->packet_loss_ratio, 2, 1.0,
            0.8 - HEIGHT_OF_MENU_ITEM * 11 - Offset, color);
    DrawMenuItemOfUnitModel('d', &UnitParams->packet_loss_distance, 1, 0.01,
            0.8 - HEIGHT_OF_MENU_ITEM * 12 - Offset, color);

}

/* Displays a menu item containing features of a parameter of the flocking model */
void DrawMenuItemOfFlockingModel(const char key,
        const fl_param_double_t * Param, double Y, const float *color) {

    int i;
    static char value[7];
    static bool Labeled;
    Labeled = false;
    sprintf(value, " ");

    static double Multipliedvalue;

    /* Checking existence of labels for specific values */
    for (i = 0; i < (Param->NumberOfLabels); i++) {
        if (Param->Labels.Values[i] == Param->Value) {
            sprintf(value, "%s", Param->Labels.Captions[i]);
            Labeled = true;
            break;
        }
    }

    if (false == Labeled) {

        Multipliedvalue = (Param->Mult) * Param->Value;
        /* Setting up Digits after decimal point */
        switch (Param->Digits) {
        case 0:
            sprintf(value, "%1.0f", Multipliedvalue);
            break;
        case 1:
            sprintf(value, "%1.1f", Multipliedvalue);
            break;
        case 2:
            sprintf(value, "%1.2f", Multipliedvalue);
            break;
        case 3:
            sprintf(value, "%1.3f", Multipliedvalue);
            break;
        default:
            sprintf(value, "%1.2f", Multipliedvalue);
            break;
        }

    }

    DrawMenuItem(key, Param->Name, Param->UnitOfMeas, value, Y, color);

    /* Display stored value, if the parameter is Constant */
    if (Param->Constant == false) {
        return;
    }

    if (Param->Value != Param->StoredValue) {
        Labeled = false;
        /* Checking existence of labels for specific stored values */
        for (i = 0; i < (Param->NumberOfLabels); i++) {
            if (Param->Labels.Values[i] == Param->StoredValue) {
                sprintf(value, "Press F12 to change to '%s'!",
                        Param->Labels.Captions[i]);
                Labeled = true;
                break;
            }
        }
        if (false == Labeled) {
            /* Setting up Digits after decimal point */
            Multipliedvalue = (Param->Mult) * Param->StoredValue;
            switch (Param->Digits) {
            case 0:
                sprintf(value, "Press F12 to change to %1.0f!",
                        Multipliedvalue);
                break;
            case 1:
                sprintf(value, "Press F12 to change to %1.1f!",
                        Multipliedvalue);
                break;
            case 2:
                sprintf(value, "Press F12 to change to %1.2f!",
                        Multipliedvalue);
                break;
            case 3:
                sprintf(value, "Press F12 to change to %1.3f!",
                        Multipliedvalue);
                break;
            default:
                sprintf(value, "Press F12 to change to %1.2f!",
                        Multipliedvalue);
                break;
            }
        }
    } else {
        sprintf(value, " ");
    }

    DrawString(-0.5, Y - 0.015, GLUT_BITMAP_8_BY_13, value, color);

}

/* Displays a menusystem from a struct of model parameters (flocking model) */
void DrawMenuItemsOfFlockingModel(const flocking_model_params_t *
        FlockingParams, const float *color) {

    static double Offset = 0.0;

    /* Draw title */
    DrawString(-0.8, 0.92, GLUT_BITMAP_9_BY_15,
            "Parameters of the flocking algorithm", color);

    int i = 0;
    int n = 0;
    static char key[10];

    for (i = 0; i < FlockingParams->NumberOfParameters; i++) {
        if (FlockingParams->Params[i].InMenu == true) {
            sprintf(key, "%01x", i + 1);
            DrawMenuItemOfFlockingModel(key[0], &FlockingParams->Params[i],
                    0.8 - i * HEIGHT_OF_MENU_ITEM - Offset, color);
        }
    }

}
