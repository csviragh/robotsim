//MIT License
//Copyright (c) 2018 Eotvos Lorand University, Budapest

/* vim:set ts=4 sw=4 sts=4 et: */

#include "output_utils.h"
#include "file_utils.h"

/* Sets default values for all output mode */
void SetDefaultOutputModes(output_modes_t * OutputModes) {

    /* Default saving mode means that we save nothing... */

    // Trajectories and inner states
    OutputModes->SaveTrajectories = false;
    OutputModes->SaveInnerStates = false;

    // Order parameters
    OutputModes->SaveDistanceBetweenUnits = FALSE;
    OutputModes->SaveDistanceBetweenNeighbours = FALSE;
    OutputModes->SaveVelocity = FALSE;
    OutputModes->SaveCorrelation = FALSE;
    OutputModes->SaveCoM = FALSE;
    OutputModes->SaveCollisions = FALSE;
    OutputModes->SaveAcceleration = FALSE;

    // Mode-specific order parameters (still not complete)
    OutputModes->SaveModelSpecifics = FALSE;

}

save_mode_t CheckSaveMode(char *ReadedValue, char *ReadedName) {

    if (strcmp(ReadedValue, "false") == 0) {
        return FALSE;
    } else if (strcmp(ReadedValue, "timeline") == 0) {
        return TIMELINE;
    } else if (strcmp(ReadedValue, "stat") == 0) {
        return STAT;
    } else if (strcmp(ReadedValue, "steadystat") == 0) {
        return STEADYSTAT;
    } else {
        fprintf(stderr,
                "For the \"%s\" variable, the valid options are \"timeline\", \"stat\", \"steadystat\" and \"false\"\n(Default is \"false\")\n",
                ReadedValue);
        return FALSE;
    }

}

/* Reads output modes from config file */
void ReadOutputModes(output_modes_t * OutputModes, FILE * InputFile) {

    if (NULL == InputFile) {

        SetDefaultOutputModes(OutputModes);

        printf("WARNING! I couldn't find config.ini file!\n");

        return;

    }

    SetDefaultOutputModes(OutputModes);

    /* format of an input line in the InputFile: 
     * ReadedName=ReadedValue
     */
    char line[256];

    char *start, *end;

    char *ReadedName;
    char *ReadedValue;

    int lineno = 0;

    while (fgets(line, sizeof(line), InputFile) != NULL) {

        start = LSkip(RStrip(line));

        if (*start == ';' || *start == '#') {
            //These lines are skipped...
        } else {

            end = FindCharOrComment(start, '=');
            if (*end == '=') {
                *end = '\0';
                ReadedName = RStrip(start);
                ReadedValue = LSkip(end + 1);
                end = FindCharOrComment(ReadedValue, '\0');
            }

            /* Reading logical values from input lines */
            if (strcmp(ReadedName, "SaveTrajectories") == 0) {
                OutputModes->SaveTrajectories =
                        (strcmp(ReadedValue, "true") == 0);
                if (strcmp(ReadedValue, "true") != 0
                        && strcmp(ReadedValue, "false") != 0) {
                    fprintf(stderr,
                            "For the \"SaveTrajectories\" variable, the valid options are \"true\" and \"false\"\n(Default is \"false\")\n");
                }
            } else if (strcmp(ReadedName, "SaveInnerStates") == 0) {
                OutputModes->SaveInnerStates =
                        (strcmp(ReadedValue, "true") == 0);
                if (strcmp(ReadedValue, "true") != 0
                        && strcmp(ReadedValue, "false") != 0) {
                    fprintf(stderr,
                            "For the \"SaveInnerStates\" variable, the valid options are \"true\" and \"false\"(Default is \"false\")\n");
                }
            } else if (strcmp(ReadedName, "SaveCorrelation") == 0) {
                OutputModes->SaveCorrelation = CheckSaveMode(ReadedValue, ReadedName);  //(strcmp (ReadedValue, "true") == 0); 
            } else if (strcmp(ReadedName, "SaveVelocity") == 0) {
                OutputModes->SaveVelocity =
                        CheckSaveMode(ReadedValue, ReadedName);
            } else if (strcmp(ReadedName, "SaveDistanceBetweenNeighbours") == 0) {
                OutputModes->SaveDistanceBetweenNeighbours =
                        CheckSaveMode(ReadedValue, ReadedName);
            } else if (strcmp(ReadedName, "SaveCoM") == 0) {
                OutputModes->SaveCoM = CheckSaveMode(ReadedValue, ReadedName);
            } else if (strcmp(ReadedName, "SaveDistanceBetweenUnits") == 0) {
                OutputModes->SaveDistanceBetweenUnits =
                        CheckSaveMode(ReadedValue, ReadedName);
            } else if (strcmp(ReadedName, "SaveCollisionRatio") == 0) {
                OutputModes->SaveCollisionRatio =
                        CheckSaveMode(ReadedValue, ReadedName);
            } else if (strcmp(ReadedName, "SaveCollisions") == 0) {
                OutputModes->SaveCollisions =
                        CheckSaveMode(ReadedValue, ReadedName);
            } else if (strcmp(ReadedName, "SaveAcceleration") == 0) {
                OutputModes->SaveAcceleration =
                        CheckSaveMode(ReadedValue, ReadedName);
            } else if (strcmp(ReadedName, "SaveModelSpecificStats") == 0) {
                OutputModes->SaveModelSpecifics =
                        CheckSaveMode(ReadedValue, ReadedName);
            }

        }

    }

}

/* Prints actual state into file
 */
void
WriteOutTrajectories(phase_t * Phase, const bool SaveTrajs,
        const bool SaveInnerStates, const double ActualTime,
        FILE * f_OutPhase, FILE * f_OutInnerStates) {

    int j, h;

    if (SaveTrajs) {
        fprintf(f_OutPhase, "%lf\t", ActualTime);
    }
    if (SaveInnerStates) {
        fprintf(f_OutInnerStates, "%lf\t", ActualTime);
    }

    for (j = 0; j < Phase->NumberOfAgents; j++) {

        if (true == SaveTrajs) {
            for (h = 0; h < 3; h++) {
                fprintf(f_OutPhase, "%lf\t", Phase->Coordinates[j][h]);
            }
            for (h = 0; h < 3; h++) {
                fprintf(f_OutPhase, "%lf\t", Phase->Velocities[j][h]);
            }
        }

        for (h = 0; h < Phase->NumberOfInnerStates; h++) {
            if (true == SaveInnerStates) {
                fprintf(f_OutInnerStates, "%lf\t", Phase->InnerStates[j][h]);
            }
        }

    }

    if (SaveTrajs) {
        fprintf(f_OutPhase, "\n");
    }
    if (SaveInnerStates) {
        fprintf(f_OutInnerStates, "\n");
    }

}
