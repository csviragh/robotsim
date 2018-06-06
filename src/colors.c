/* vim:set ts=4 sw=4 sts=4 et: */

/* Reading and applying color configs
 */

#include "colors.h"

void MakeRGBFromHSV(float *rgb, unsigned char hue, unsigned char saturation,
        unsigned char value) {
    /* code migrated from libnavictrl */
    int i;
    float f, p, q, t, h, s, v;

    /* If saturation is zero, color is gray and intensity is defined by value */
    if (!saturation) {
        // Note that /255 is needed because here we use 0-1 output.
        rgb[0] = rgb[1] = rgb[2] = value / 255.0;
        return;
    }
    /* map from u8 to [0-1] */
    h = hue / 255.0;
    s = saturation / 255.0;
    v = value / 255.0;
    /* prepare variables */
    i = (int) (h * 6.0);
    f = (h * 6.0) - i;
    p = v * (1.0 - s);
    q = v * (1.0 - s * f);
    t = v * (1.0 - s * (1.0 - f));
    i = (i % 6);
    /* create rgb color */
    if (i == 0) {
        rgb[0] = (v * 255);
        rgb[1] = (t * 255);
        rgb[2] = (p * 255);
    } else if (i == 1) {
        rgb[0] = (q * 255);
        rgb[1] = (v * 255);
        rgb[2] = (p * 255);
    } else if (i == 2) {
        rgb[0] = (p * 255);
        rgb[1] = (v * 255);
        rgb[2] = (t * 255);
    } else if (i == 3) {
        rgb[0] = (p * 255);
        rgb[1] = (q * 255);
        rgb[2] = (v * 255);
    } else if (i == 4) {
        rgb[0] = (t * 255);
        rgb[1] = (p * 255);
        rgb[2] = (v * 255);
    } else if (i == 5) {
        rgb[0] = (v * 255);
        rgb[1] = (p * 255);
        rgb[2] = (q * 255);
    } else {
        /* we cannot possibly reach this point, but compiler might give warning,
           so we return a fake black */
        rgb[0] = rgb[1] = rgb[2] = 0;
    }
    // Note that here we use [0-1] range in output but I kept the migrated code as it is... 
    rgb[0] /= 255;
    rgb[1] /= 255;
    rgb[2] /= 255;
    return;
}

/* Refresh static double param value variables defined in the model files */
void RefreshModelSpecificColors(model_specific_color_t * ModelSpecificColors,
        int *NumberOfModelSpecificColors) {

    int i = 0;
    for (i = 0; i < *NumberOfModelSpecificColors; i++) {
        *ModelSpecificColors[i].StaticValuePointer[0] =
                ModelSpecificColors[i].RGB_Values[0];
        *ModelSpecificColors[i].StaticValuePointer[1] =
                ModelSpecificColors[i].RGB_Values[1];
        *ModelSpecificColors[i].StaticValuePointer[2] =
                ModelSpecificColors[i].RGB_Values[2];
    }
}

/* Loading color schemes from an ".ini" file */
void LoadColorConfig(color_config_t * ColorConfigToSet, FILE * InputFile,
        int NumberOfAgents, model_specific_color_t * ModelSpecificColors,
        int *NumberOfModelSpecificColors) {

    /* Allocating an array for containing colors for each agents */
    ColorConfigToSet->AgentsColor =
            (float **) calloc(NumberOfAgents, sizeof(float *));
    int i;
    for (i = 0; i < NumberOfAgents; i++) {
        ColorConfigToSet->AgentsColor[i] = (float *) calloc(3, sizeof(float));
    }

    /* Some default values... */

    // YES, this is a monster

    ColorConfigToSet->MenuItemColor[0] = 0.1;
    ColorConfigToSet->MenuItemColor[1] = 0.9;
    ColorConfigToSet->MenuItemColor[2] = 0.4;
    ColorConfigToSet->MenuSelectionColor[0] = 0.1;
    ColorConfigToSet->MenuSelectionColor[1] = 0.9;
    ColorConfigToSet->MenuSelectionColor[2] = 0.4;
    ColorConfigToSet->PausedCaptionColor[0] = 0.1;
    ColorConfigToSet->PausedCaptionColor[1] = 0.9;
    ColorConfigToSet->PausedCaptionColor[2] = 0.4;
    ColorConfigToSet->EraseColor[0] = 0.1;
    ColorConfigToSet->EraseColor[1] = 0.1;
    ColorConfigToSet->EraseColor[2] = 0.1;
    ColorConfigToSet->AgentsInDangerColor[0] = 0.8;
    ColorConfigToSet->AgentsInDangerColor[1] = 0.1;
    ColorConfigToSet->AgentsInDangerColor[2] = 0.1;
    ColorConfigToSet->VelocityLabelColor[0] = 0.1;
    ColorConfigToSet->VelocityLabelColor[1] = 0.1;
    ColorConfigToSet->VelocityLabelColor[2] = 0.1;
    ColorConfigToSet->VelocityArrowColor[0] = 0.8;
    ColorConfigToSet->VelocityArrowColor[1] = 0.8;
    ColorConfigToSet->VelocityArrowColor[2] = 0.8;

    ColorConfigToSet->LabelColor[0] = 0.1;
    ColorConfigToSet->LabelColor[1] = 0.1;
    ColorConfigToSet->LabelColor[2] = 0.1;

    ColorConfigToSet->AxisColor[0] = 0.2;
    ColorConfigToSet->AxisColor[1] = 0.2;
    ColorConfigToSet->AxisColor[2] = 0.2;

    ColorConfigToSet->CommNetWorkColor[0] = 0.8;
    ColorConfigToSet->CommNetWorkColor[1] = 0.8;
    ColorConfigToSet->CommNetWorkColor[2] = 0.6;

    ColorConfigToSet->ElapsedTimeCaptionColor[0] = 0.1;
    ColorConfigToSet->ElapsedTimeCaptionColor[1] = 0.9;
    ColorConfigToSet->ElapsedTimeCaptionColor[2] = 0.4;
    ColorConfigToSet->CollisionCaptionColor[0] = 0.1;
    ColorConfigToSet->CollisionCaptionColor[1] = 0.9;
    ColorConfigToSet->CollisionCaptionColor[2] = 0.4;

    ColorConfigToSet->AgentsDefaultColor[0] = 0.2;
    ColorConfigToSet->AgentsDefaultColor[1] = 0.2;
    ColorConfigToSet->AgentsDefaultColor[2] = 0.6;
    for (i = 0; i < NumberOfAgents; i++) {

        ColorConfigToSet->AgentsColor[i][0] = 0.2;
        ColorConfigToSet->AgentsColor[i][1] = 0.2;
        ColorConfigToSet->AgentsColor[i][2] = 0.6;

    }

    /* format of an input line in the InputFile: 
     * readed_Name=readed_value             
     */
    static char line[256];
    static char templabel[10];
    static bool found = false;

    char *start, *end;

    char *readed_Name;
    char *readed_value;

    while (fgets(line, sizeof(line), InputFile) != NULL) {

        start = LSkip(RStrip(line));

        if (*start == ';' || *start == '#') {
            //These lines are skipped...
        } else {

            end = FindCharOrComment(start, '=');
            if (*end == '=') {
                *end = '\0';
                readed_Name = RStrip(start);
                readed_value = LSkip(end + 1);
                end = FindCharOrComment(readed_value, '\0');
            }
            //Reading color codes from input lines

            if (strcmp(readed_Name, "background_r") == 0) {
                ColorConfigToSet->EraseColor[0] = atof(readed_value);
            } else if (strcmp(readed_Name, "background_g") == 0) {
                ColorConfigToSet->EraseColor[1] = atof(readed_value);
            } else if (strcmp(readed_Name, "background_b") == 0) {
                ColorConfigToSet->EraseColor[2] = atof(readed_value);
            } else if (strcmp(readed_Name, "menu_selection_r") == 0) {
                ColorConfigToSet->MenuSelectionColor[0] = atof(readed_value);
            } else if (strcmp(readed_Name, "menu_selection_g") == 0) {
                ColorConfigToSet->MenuSelectionColor[1] = atof(readed_value);
            } else if (strcmp(readed_Name, "menu_selection_b") == 0) {
                ColorConfigToSet->MenuSelectionColor[2] = atof(readed_value);
            } else if (strcmp(readed_Name, "paused_caption_r") == 0) {
                ColorConfigToSet->MenuSelectionColor[2] = atof(readed_value);
            } else if (strcmp(readed_Name, "paused_caption_g") == 0) {
                ColorConfigToSet->MenuSelectionColor[2] = atof(readed_value);
            } else if (strcmp(readed_Name, "paused_caption_b") == 0) {
                ColorConfigToSet->MenuSelectionColor[2] = atof(readed_value);
            } else if (strcmp(readed_Name, "menu_item_r") == 0) {
                ColorConfigToSet->MenuItemColor[0] = atof(readed_value);
            } else if (strcmp(readed_Name, "menu_item_g") == 0) {
                ColorConfigToSet->MenuItemColor[1] = atof(readed_value);
            } else if (strcmp(readed_Name, "menu_item_b") == 0) {
                ColorConfigToSet->MenuItemColor[2] = atof(readed_value);
            } else if (strcmp(readed_Name, "agents_indanger_r") == 0) {
                ColorConfigToSet->AgentsInDangerColor[0] = atof(readed_value);
            } else if (strcmp(readed_Name, "agents_indanger_g") == 0) {
                ColorConfigToSet->AgentsInDangerColor[1] = atof(readed_value);
            } else if (strcmp(readed_Name, "agents_indanger_b") == 0) {
                ColorConfigToSet->AgentsInDangerColor[2] = atof(readed_value);
            } else if (strcmp(readed_Name, "agents_velocitylabel_r") == 0) {
                ColorConfigToSet->VelocityLabelColor[0] = atof(readed_value);
            } else if (strcmp(readed_Name, "agents_velocitylabel_g") == 0) {
                ColorConfigToSet->VelocityLabelColor[1] = atof(readed_value);
            } else if (strcmp(readed_Name, "agents_velocitylabel_b") == 0) {
                ColorConfigToSet->VelocityLabelColor[2] = atof(readed_value);
            } else if (strcmp(readed_Name, "agents_label_r") == 0) {
                ColorConfigToSet->LabelColor[0] = atof(readed_value);
            } else if (strcmp(readed_Name, "agents_label_g") == 0) {
                ColorConfigToSet->LabelColor[1] = atof(readed_value);
            } else if (strcmp(readed_Name, "agents_label_b") == 0) {
                ColorConfigToSet->LabelColor[2] = atof(readed_value);
            } else if (strcmp(readed_Name, "agents_velocityarrow_r") == 0) {
                ColorConfigToSet->VelocityArrowColor[0] = atof(readed_value);
            } else if (strcmp(readed_Name, "agents_velocityarrow_g") == 0) {
                ColorConfigToSet->VelocityArrowColor[1] = atof(readed_value);
            } else if (strcmp(readed_Name, "agents_velocityarrow_b") == 0) {
                ColorConfigToSet->VelocityArrowColor[2] = atof(readed_value);
            } else if (strcmp(readed_Name, "axis_r") == 0) {
                ColorConfigToSet->AxisColor[0] = atof(readed_value);
            } else if (strcmp(readed_Name, "axis_g") == 0) {
                ColorConfigToSet->AxisColor[1] = atof(readed_value);
            } else if (strcmp(readed_Name, "axis_b") == 0) {
                ColorConfigToSet->AxisColor[2] = atof(readed_value);
            } else if (strcmp(readed_Name, "collisioncaption_r") == 0) {
                ColorConfigToSet->CollisionCaptionColor[0] = atof(readed_value);
            } else if (strcmp(readed_Name, "collisioncaption_g") == 0) {
                ColorConfigToSet->CollisionCaptionColor[1] = atof(readed_value);
            } else if (strcmp(readed_Name, "collisioncaption_b") == 0) {
                ColorConfigToSet->CollisionCaptionColor[2] = atof(readed_value);
            } else if (strcmp(readed_Name, "elapsedtimecaption_r") == 0) {
                ColorConfigToSet->ElapsedTimeCaptionColor[0] =
                        atof(readed_value);
            } else if (strcmp(readed_Name, "elapsedtimecaption_g") == 0) {
                ColorConfigToSet->ElapsedTimeCaptionColor[1] =
                        atof(readed_value);
            } else if (strcmp(readed_Name, "elapsedtimecaption_b") == 0) {
                ColorConfigToSet->ElapsedTimeCaptionColor[2] =
                        atof(readed_value);
            } else if (strcmp(readed_Name, "comm_network_r") == 0) {
                ColorConfigToSet->CommNetWorkColor[0] = atof(readed_value);
            } else if (strcmp(readed_Name, "comm_network_g") == 0) {
                ColorConfigToSet->CommNetWorkColor[1] = atof(readed_value);
            } else if (strcmp(readed_Name, "comm_network_b") == 0) {
                ColorConfigToSet->CommNetWorkColor[2] = atof(readed_value);
            } else if (strcmp(readed_Name, "agents_r") == 0) {
                for (i = 0; i < NumberOfAgents; i++) {
                    ColorConfigToSet->AgentsColor[i][0] = atof(readed_value);
                    ColorConfigToSet->AgentsDefaultColor[0] =
                            atof(readed_value);
                }
            } else if (strcmp(readed_Name, "agents_g") == 0) {
                for (i = 0; i < NumberOfAgents; i++) {
                    ColorConfigToSet->AgentsColor[i][1] = atof(readed_value);
                    ColorConfigToSet->AgentsDefaultColor[1] =
                            atof(readed_value);
                }
            } else if (strcmp(readed_Name, "agents_b") == 0) {
                for (i = 0; i < NumberOfAgents; i++) {
                    ColorConfigToSet->AgentsColor[i][2] = atof(readed_value);
                    ColorConfigToSet->AgentsDefaultColor[2] =
                            atof(readed_value);
                }
            } else {

                while (i < NumberOfAgents && found == false) {

                    sprintf(templabel, "agent_%d_r", i);

                    if (strcmp(readed_Name, templabel) == 0) {
                        ColorConfigToSet->AgentsColor[i][0] =
                                atof(readed_value);
                        i = NumberOfAgents;
                        found = true;
                    } else {
                        sprintf(templabel, "agent_%d_g", i);
                        if (strcmp(readed_Name, templabel) == 0) {
                            ColorConfigToSet->AgentsColor[i][1] =
                                    atof(readed_value);
                            i = NumberOfAgents;
                            found = true;
                        } else {
                            sprintf(templabel, "agent_%d_b", i);
                            if (strcmp(readed_Name, templabel) == 0) {
                                ColorConfigToSet->AgentsColor[i][2] =
                                        atof(readed_value);
                                i = NumberOfAgents;
                                found = true;
                            }
                        }

                    }

                    i++;

                }

                if (found == false) {
                    LoadModelSpecificColors(readed_Name, readed_value,
                            ModelSpecificColors, NumberOfModelSpecificColors);
                }

                found = false;

            }

        }

    }

    /* Calling function to load model-specific colors */
    RefreshModelSpecificColors(ModelSpecificColors,
            NumberOfModelSpecificColors);

}

/* Setting the color of a specific agent */
void SetAgentsColor(color_config_t * ColorConfig, const int WhichAgent,
        const int NumberOfAgents, const float *color) {

    if (WhichAgent < NumberOfAgents) {
        ColorConfig->AgentsColor[WhichAgent][0] = color[0];
        ColorConfig->AgentsColor[WhichAgent][1] = color[1];
        ColorConfig->AgentsColor[WhichAgent][2] = color[2];
    }

}

/* Reset the color of every agents */
void ResetAgentsColor(color_config_t * ColorConfig, const int NumberOfAgents,
        const float *color) {

    int i;

    for (i = 0; i < NumberOfAgents; i++) {

        SetAgentsColor(ColorConfig, i, NumberOfAgents, color);

    }

}

/* For saving model-specific colors */
void LoadModelSpecificColors(char *readed_Name, char *readed_value,
        model_specific_color_t * ModelSpecificColors,
        int *NumberOfModelSpecificColors) {

    int i;

    /* format of an input line in the InputFile: 
     * readed_Name=readed_value             
     */

    static char value_to_compare[32];

    for (i = 0; i < *NumberOfModelSpecificColors; i++) {
        sprintf(value_to_compare, "%s_r", ModelSpecificColors[i].NameInFiles);
        if (strcmp(readed_Name, value_to_compare) == 0) {
            ModelSpecificColors[i].RGB_Values[0] = atof(readed_value);
            return;
        }

        sprintf(value_to_compare, "%s_g", ModelSpecificColors[i].NameInFiles);
        if (strcmp(readed_Name, value_to_compare) == 0) {
            ModelSpecificColors[i].RGB_Values[1] = atof(readed_value);
            return;
        }

        sprintf(value_to_compare, "%s_b", ModelSpecificColors[i].NameInFiles);
        if (strcmp(readed_Name, value_to_compare) == 0) {
            ModelSpecificColors[i].RGB_Values[2] = atof(readed_value);
            return;
        }
    }

}
