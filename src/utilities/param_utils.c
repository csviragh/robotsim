/* vim:set ts=4 sw=4 sts=4 et: */

/* Tools for handling parameters.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include "param_utils.h"

/* Changes one of the 'robot model' parameters */
void ChangeUnitModelParameter(unit_model_params_t * params, int WhichParam,
        double StepSize, double DeltaT) {

    switch (WhichParam) {
    case 1:
    {
        params->Tau_PID_XY.Value += StepSize * 0.01;
        if (params->Tau_PID_XY.Value < DeltaT) {
            params->Tau_PID_XY.Value = DeltaT;
        }
    }
        break;
    case 2:
    {
        params->Tau_PID_Z.Value += StepSize * 0.01;
        if (params->Tau_PID_Z.Value < DeltaT) {
            params->Tau_PID_Z.Value = DeltaT;
        }
    }
        break;
    case 3:
    {
        params->a_max.Value += StepSize;
        if (params->a_max.Value < 0.0) {
            params->a_max.Value = 0.0;
        }
    }
        break;
    case 4:
    {
        params->R_C.Value += StepSize * 10.0;
        if (params->R_C.Value < 1.0) {
            params->R_C.Value = 1.0;
        }
    }
        break;
    case 5:
    {
        params->t_del.Value += StepSize * 0.01;
        if (params->t_del.Value < 0.0) {
            params->t_del.Value = 0.0;
        }
    }
        break;
    case 6:
    {
        params->t_GPS.Value += StepSize * 0.01;
        if (params->t_GPS.Value < DeltaT) {
            params->t_GPS.Value = DeltaT;
        }
    }
        break;
    case 7:
    {
        params->Sigma_GPS_XY.Value += StepSize * 10.0;
        if (params->Sigma_GPS_XY.Value < 0.0) {
            params->Sigma_GPS_XY.Value = 0.0;
        }
    }
        break;
    case 8:
    {
        params->Sigma_GPS_Z.Value += StepSize * 10.0;
        if (params->Sigma_GPS_Z.Value < 0.0) {
            params->Sigma_GPS_Z.Value = 0.0;
        }
    }
        break;
    case 9:
    {
        params->Sigma_Outer_XY.Value += StepSize * 10.0;
        if (params->Sigma_Outer_XY.Value < 0.0) {
            params->Sigma_Outer_XY.Value = 0.0;
        }
    }
        break;
    case 10:
    {
        params->Sigma_Outer_Z.Value += StepSize * 10.0;
        if (params->Sigma_Outer_Z.Value < 0.0) {
            params->Sigma_Outer_Z.Value = 0.0;
        }
    }
        break;
    case 11:
    {
        params->Wind_Magn_Avg.Value += StepSize * 10.0;
        if (params->Wind_Magn_Avg.Value < 0.0) {
            params->Wind_Magn_Avg.Value = 0.0;
        }
    }
        break;
    case 12:
    {
        params->packet_loss_ratio.Value += StepSize * 0.01;
        if (params->packet_loss_ratio.Value < 0.0) {
            params->packet_loss_ratio.Value = 0.0;
        } else if (params->packet_loss_ratio.Value > 1.0) {
            params->packet_loss_ratio.Value = 1.0;
        }
    }
        break;
    case 13:
    {
        params->packet_loss_distance.Value += StepSize * 10.0;
        if (params->packet_loss_distance.Value < 0.0) {
            params->packet_loss_distance.Value = 0.0;
        }
    }
        break;
    }

}

/* Sets the Name and unit of measurement of the unit model parameters */
void SetNamesOfUnitModelParams(unit_model_params_t * UnitParams) {

    /* Measurement units of each parameter */
    sprintf(UnitParams->Tau_PID_XY.UnitOfMeas, "s");
    sprintf(UnitParams->Tau_PID_Z.UnitOfMeas, "s");
    sprintf(UnitParams->a_max.UnitOfMeas, "m/s^2");
    sprintf(UnitParams->t_del.UnitOfMeas, "s");
    sprintf(UnitParams->t_GPS.UnitOfMeas, "s");
    sprintf(UnitParams->R_C.UnitOfMeas, "m");
    sprintf(UnitParams->Sigma_GPS_XY.UnitOfMeas, "m^2/s^2");
    sprintf(UnitParams->Sigma_GPS_Z.UnitOfMeas, "m^2/s^2");
    sprintf(UnitParams->Sigma_Outer_XY.UnitOfMeas, "m^2/s^3");
    sprintf(UnitParams->Sigma_Outer_Z.UnitOfMeas, "m^2/s^3");
    sprintf(UnitParams->Wind_Magn_Avg.UnitOfMeas, "m/s");
    sprintf(UnitParams->packet_loss_distance.UnitOfMeas, "m");
    sprintf(UnitParams->packet_loss_ratio.UnitOfMeas, "-");

    /* Parameter Names */
    sprintf(UnitParams->Tau_PID_XY.Name, "Tau_PID (XY)");
    sprintf(UnitParams->Tau_PID_Z.Name, "Tau_PID (Z)");
    sprintf(UnitParams->a_max.Name, "Maximum Acceleration");
    sprintf(UnitParams->t_del.Name, "Delay Time");
    sprintf(UnitParams->t_GPS.Name, "Refresh Rate of GPS");
    sprintf(UnitParams->R_C.Name, "Communication Range");
    sprintf(UnitParams->Sigma_GPS_XY.Name, "GPS XY Accuracy");
    sprintf(UnitParams->Sigma_GPS_Z.Name, "GPS Z Accuracy");
    sprintf(UnitParams->Sigma_Outer_XY.Name, "Strength of outer noise (XY)");
    sprintf(UnitParams->Sigma_Outer_Z.Name, "Strength of outer noise (Z)");
    sprintf(UnitParams->Wind_Magn_Avg.Name, "Average Wind Speed");
    sprintf(UnitParams->packet_loss_ratio.Name, "Packet Loss Ratio");
    sprintf(UnitParams->packet_loss_distance.Name, "Packet Loss Distance");
}

/* Gets the "unit model" parameter set from an input file
 */
void GetUnitModelParamsFromFile(unit_model_params_t * UnitParams,
        FILE * InputFile) {

    /* Setting parameter Names and measurement units */
    SetNamesOfUnitModelParams(UnitParams);

    if (NULL == InputFile) {

        /* Set us some default values, if input file is not found */
        UnitParams->Tau_PID_XY.Value = 1.0;
        UnitParams->Tau_PID_Z.Value = 1.0;
        UnitParams->a_max.Value = 700.0;
        UnitParams->t_GPS.Value = 0.2;
        UnitParams->t_del.Value = 0.0;
        UnitParams->R_C.Value = 80000.0;
        UnitParams->Tau_PID_XY.Value = 1.0;
        UnitParams->Sigma_GPS_XY.Value = 0.0;
        UnitParams->Sigma_GPS_Z.Value = 0.0;
        UnitParams->Sigma_Outer_XY.Value = 0.0;
        UnitParams->Sigma_Outer_Z.Value = 0.0;
        UnitParams->packet_loss_ratio.Value = 0.5;
        UnitParams->packet_loss_distance.Value = UnitParams->R_C.Value;

        printf("WARNING! Unit model params are not loaded!\n");

        return;

    }

    /* format of an input line in the InputFile: 
     * ReadedName=ReadedValue               
     */
    char line[256];

    char *start, *end;

    char *ReadedName;
    char *ReadedValue;
    int NumberOfReadedNames = 0;

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
            //Reading parameters from input lines
            if (strcmp(ReadedName, "tau_PID_XY") == 0) {
                UnitParams->Tau_PID_XY.Value = atof(ReadedValue);
                NumberOfReadedNames++;
            } else if (strcmp(ReadedName, "tau_PID_Z") == 0) {
                UnitParams->Tau_PID_Z.Value = atof(ReadedValue);
                NumberOfReadedNames++;
            } else if (strcmp(ReadedName, "a_max") == 0) {
                UnitParams->a_max.Value = atof(ReadedValue);
                NumberOfReadedNames++;
            } else if (strcmp(ReadedName, "t_GPS") == 0) {
                UnitParams->t_GPS.Value = atof(ReadedValue);
                NumberOfReadedNames++;
            } else if (strcmp(ReadedName, "t_del") == 0) {
                UnitParams->t_del.Value = atof(ReadedValue);
                NumberOfReadedNames++;
            } else if (strcmp(ReadedName, "R_C") == 0) {
                UnitParams->R_C.Value = atof(ReadedValue);
                NumberOfReadedNames++;
            } else if (strcmp(ReadedName, "Sigma_GPS_XY") == 0) {
                UnitParams->Sigma_GPS_XY.Value = atof(ReadedValue);
                NumberOfReadedNames++;
            } else if (strcmp(ReadedName, "Sigma_GPS_Z") == 0) {
                UnitParams->Sigma_GPS_Z.Value = atof(ReadedValue);
                NumberOfReadedNames++;
            } else if (strcmp(ReadedName, "Sigma_outer_XY") == 0) {
                UnitParams->Sigma_Outer_XY.Value = atof(ReadedValue);
                NumberOfReadedNames++;
            } else if (strcmp(ReadedName, "Sigma_outer_Z") == 0) {
                UnitParams->Sigma_Outer_Z.Value = atof(ReadedValue);
                NumberOfReadedNames++;
            } else if (strcmp(ReadedName, "Wind_Magn_Avg") == 0) {
                UnitParams->Wind_Magn_Avg.Value = atof(ReadedValue);
                NumberOfReadedNames++;
            } else if (strcmp(ReadedName, "Wind_StDev") == 0) {
                UnitParams->Wind_StDev.Value = atof(ReadedValue);
                NumberOfReadedNames++;
            } else if (strcmp(ReadedName, "Wind_Angle") == 0) {
                UnitParams->Wind_Angle.Value = atof(ReadedValue);
                NumberOfReadedNames++;
            } else if (strcmp(ReadedName, "Packet_Loss_Ratio") == 0) {
                UnitParams->packet_loss_ratio.Value = atof(ReadedValue);
                NumberOfReadedNames++;
            } else if (strcmp(ReadedName, "Packet_Loss_Distance") == 0) {
                UnitParams->packet_loss_distance.Value = atof(ReadedValue);
                NumberOfReadedNames++;
            }
        }
    }

    //Checking the existance of parameters...
    if (NumberOfReadedNames < 15) {

        printf("15 paramers are necessary in \n'%s' \n\nRequired format (example):\n\n", recover_fileName(InputFile));
        printf("tau_PID_XY=1\n");
        printf("tau_PID_Z=1\n");
        printf("a_max=600\n");
        printf("R_C=10000\n");
        printf("t_del=1\n");
        printf("t_GPS=0.2\n");
        printf("Sigma_GPS_XY=0.0\n");
        printf("Sigma_GPS_Z=0.0\n");
        printf("Sigma_outer_XY=1000\n");
        printf("Sigma_outer_Z=0.0\n");
        printf("Wind_Magn_Avg=0.0\n");
        printf("Wind_StDev=0.0\n");
        printf("Wind_Angle=0.0\n");
        printf("Packet_Loss_Ratio=0.5\n");
        printf("Packet_Loss_Distance=10000\n");

        exit(-1);

    }

    /* Setting up viscosity */
    UnitParams->ViscosityCoeff.Value = 1.0;

}

/* Saves the "unit model" parameter set into an output file
 */
void SaveUnitModelParamsToFile(FILE * OutputFile_Unit,
        unit_model_params_t * UnitParams) {

    fprintf(OutputFile_Unit, "# Relaxation times of PID Controller (s)\n");
    fprintf(OutputFile_Unit, "tau_PID_XY=%lf\n", UnitParams->Tau_PID_XY.Value);
    fprintf(OutputFile_Unit, "tau_PID_Z=%lf\n", UnitParams->Tau_PID_Z.Value);
    fprintf(OutputFile_Unit, "# Maximum Value of acceleration (cm / s^2)\n");
    fprintf(OutputFile_Unit, "a_max=%lf\n", UnitParams->a_max.Value);
    fprintf(OutputFile_Unit, "# Comm. range (cm)\n");
    fprintf(OutputFile_Unit, "R_C=%lf\n", UnitParams->R_C.Value);
    fprintf(OutputFile_Unit, "# Delay of comm. (s)\n");
    fprintf(OutputFile_Unit, "t_del=%lf\n", UnitParams->t_del.Value);
    fprintf(OutputFile_Unit, "# GPS refresh rate (s)\n");
    fprintf(OutputFile_Unit, "t_GPS=%lf\n", UnitParams->t_GPS.Value);
    fprintf(OutputFile_Unit, "# Parameters of inner noises (cm^2 / s^2)\n");
    fprintf(OutputFile_Unit, "Sigma_GPS_XY=%lf\n",
            UnitParams->Sigma_GPS_XY.Value);
    fprintf(OutputFile_Unit, "Sigma_GPS_Z=%lf\n",
            UnitParams->Sigma_GPS_Z.Value);
    fprintf(OutputFile_Unit, "# Parameters of outer noises (cm^2 / s^3)\n");
    fprintf(OutputFile_Unit, "Sigma_outer_XY=%lf\n",
            UnitParams->Sigma_Outer_XY.Value);
    fprintf(OutputFile_Unit, "Sigma_outer_Z=%lf\n",
            UnitParams->Sigma_Outer_Z.Value);

    fprintf(OutputFile_Unit, "Wind_Magn_Avg=%lf\n",
            UnitParams->Wind_Magn_Avg.Value);
    fprintf(OutputFile_Unit, "Wind_StDev=%lf\n", UnitParams->Wind_StDev.Value);
    fprintf(OutputFile_Unit, "Wind_Angle=%lf\n", UnitParams->Wind_Angle.Value);

    fprintf(OutputFile_Unit, "# Packet Loss Ratio (0-1)\n");
    fprintf(OutputFile_Unit, "Packet_Loss_Ratio=%lf\n",
            UnitParams->packet_loss_ratio.Value);
    fprintf(OutputFile_Unit, "# Packet Loss Distance (cm)\n");
    fprintf(OutputFile_Unit, "Packet_Loss_Distance=%lf\n",
            UnitParams->packet_loss_distance.Value);

}

/* Gets the "situation" parameters from an input file
 */
sit_parameters_t GetSituationParamsFromFile(FILE * InputFile) {

    sit_parameters_t temp_sit_parameters;

    /* format of an input line in the InputFile: 
     * ReadedName=ReadedValue               
     */
    char line[256];

    char *start, *end;

    char *ReadedName;
    char *ReadedValue;
    int NumberOfReadedNames = 0;

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
            //Reading parameters from input lines
            if (strcmp(ReadedName, "NumberOfAgents") == 0) {
                temp_sit_parameters.NumberOfAgents = atoi(ReadedValue);
                NumberOfReadedNames++;
            } else if (strcmp(ReadedName, "Length") == 0) {
                temp_sit_parameters.Length = atof(ReadedValue);
                NumberOfReadedNames++;
            } else if (strcmp(ReadedName, "DeltaT") == 0) {
                temp_sit_parameters.DeltaT = atof(ReadedValue);
                NumberOfReadedNames++;
            } else if (strcmp(ReadedName, "InitialX") == 0) {
                temp_sit_parameters.InitialX = atof(ReadedValue);
                NumberOfReadedNames++;
            } else if (strcmp(ReadedName, "InitialY") == 0) {
                temp_sit_parameters.InitialY = atof(ReadedValue);
                NumberOfReadedNames++;
            } else if (strcmp(ReadedName, "InitialZ") == 0) {
                temp_sit_parameters.InitialZ = atof(ReadedValue);
                NumberOfReadedNames++;
            } else if (strcmp(ReadedName, "Radius") == 0) {
                temp_sit_parameters.Radius = atof(ReadedValue);
                NumberOfReadedNames++;
            } else if (strcmp(ReadedName, "LengthToStore") == 0) {
                temp_sit_parameters.LengthToStore = atof(ReadedValue);
                NumberOfReadedNames++;
            } else if (strcmp(ReadedName, "VizSpeedUp") == 0) {
                temp_sit_parameters.VizSpeedUp = atoi(ReadedValue);
                NumberOfReadedNames++;
            } else if (strcmp(ReadedName, "StartOfSteadyState") == 0) {
                temp_sit_parameters.StartOfSteadyState = atof(ReadedValue);
                NumberOfReadedNames++;
            }

        }

    }

    /*Checking existence of parameters */
    if (NumberOfReadedNames < 10) {

        printf("Presence of 10 paramers are necessary in \n '%s'.\n\nRequired format (example):\n\n", recover_fileName(InputFile));
        printf("NumberOfAgents=10\n");
        printf("Length=1000\n");
        printf("InitialX=1000\n");
        printf("InitialY=1000\n");
        printf("InitialZ=1000\n");
        printf("DeltaT=0.01\n");
        printf("Radius=300.0\n");
        printf("LengthToStore=1.0\n");
        printf("VizSpeedUp=10\n");
        printf("StartOfSteadyState=10\n");

        exit(-1);

    }

    /* Checking validity of parameters */
    if (temp_sit_parameters.LengthToStore < temp_sit_parameters.DeltaT) {

        printf("Stored time length (\"LengthToStore\") must be greater than the accuracy of Euler method (\"DeltaT\")!\n");
        exit(-1);

    } else if (temp_sit_parameters.DeltaT <= 0.0) {

        printf("Accuracy of Euler method (\"DeltaT\") must be greater than 0!\n");
        exit(-1);

    } else if (temp_sit_parameters.InitialX < 0.0
            || temp_sit_parameters.InitialY < 0.0
            || temp_sit_parameters.InitialZ < 0.0) {

        printf("Initial sizes must be positive!\n");
        exit(-1);

    }

    /* Cut-off for stored timesteps */
    if (temp_sit_parameters.LengthToStore + 20.0 > temp_sit_parameters.Length) {

        temp_sit_parameters.LengthToStore = temp_sit_parameters.Length - 20.0;

    }

    return temp_sit_parameters;

}

/* Saves flocking model parameters into "OutputFile". 
 */
void SaveFlockingModelParamsToFile(FILE * OutputFile_FL,
        flocking_model_params_t * FlockingParams) {

    int i;

    for (i = 0; i < FlockingParams->NumberOfParameters; i++) {
        if (strcmp(FlockingParams->Params[i].UnitOfMeas, "")) {
            if (FlockingParams->Params[i].Mult != 1.0) {
                fprintf(OutputFile_FL,
                        "# %s (Multiply it with %lf to get its Value in %s)\n%s=%lf\n",
                        FlockingParams->Params[i].Name,
                        FlockingParams->Params[i].Mult,
                        FlockingParams->Params[i].UnitOfMeas,
                        FlockingParams->Params[i].NameInFiles,
                        FlockingParams->Params[i].Value);
            } else {
                fprintf(OutputFile_FL, "# %s (%s)\n%s=%lf\n",
                        FlockingParams->Params[i].Name,
                        FlockingParams->Params[i].UnitOfMeas,
                        FlockingParams->Params[i].NameInFiles,
                        FlockingParams->Params[i].Value);
            }
        } else {
            if (FlockingParams->Params[i].Mult != 1.0) {
                fprintf(OutputFile_FL,
                        "# %s (Multiply it with %lf to get its real Value)\n%s=%lf\n",
                        FlockingParams->Params[i].Name,
                        FlockingParams->Params[i].Mult,
                        FlockingParams->Params[i].NameInFiles,
                        FlockingParams->Params[i].Value);
            } else {
                fprintf(OutputFile_FL, "# %s\n%s=%lf\n",
                        FlockingParams->Params[i].Name,
                        FlockingParams->Params[i].NameInFiles,
                        FlockingParams->Params[i].Value);
            }
        }
    }

}

/* Gets the flocking model parameters from an input file
 */
void GetFlockingModelParamsFromFile(flocking_model_params_t * FlockingParams,
        FILE * InputFile) {

    if (NULL == InputFile) {
        printf("ERROR: Flocking model params file not found...\n");
        return;
    }

    int i;

    /* format of an input line in the InputFile: 
     * ReadedName=ReadedValue               
     */
    char line[256];

    char *start, *end;

    char *ReadedName;
    char *ReadedValue;
    int NumberOfReadedNames = 0;

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

            for (i = 0; i < FlockingParams->NumberOfParameters; i++) {
                if (strcmp(ReadedName, FlockingParams->Params[i].NameInFiles)
                        == 0) {
                    FlockingParams->Params[i].Value = atof(ReadedValue);
                }
            }
        }

    }

    /* Setting up Initial stored Values for Constant flocking model parameters */
    for (i = 0; i < FlockingParams->NumberOfParameters; i++) {
        if (FlockingParams->Params[i].Constant) {
            FlockingParams->Params[i].StoredValue =
                    FlockingParams->Params[i].Value;
        }
    }

}

/* Changes one of the 'flocking algorithm' parameters */
void ChangeFlockingModelParameter(flocking_model_params_t * FlockingParams,
        const int WhichParam, const double StepSize) {

    if (FlockingParams->Params[WhichParam].Constant == true) {
        FlockingParams->Params[WhichParam].StoredValue += StepSize;
        if (FlockingParams->Params[WhichParam].StoredValue >
                FlockingParams->Params[WhichParam].Max) {
            FlockingParams->Params[WhichParam].StoredValue =
                    FlockingParams->Params[WhichParam].Max;
        } else if (FlockingParams->Params[WhichParam].StoredValue <
                FlockingParams->Params[WhichParam].Min) {
            FlockingParams->Params[WhichParam].StoredValue =
                    FlockingParams->Params[WhichParam].Min;
        }
        return;
    }

    FlockingParams->Params[WhichParam].Value += StepSize;
    if (FlockingParams->Params[WhichParam].Value >
            FlockingParams->Params[WhichParam].Max) {
        FlockingParams->Params[WhichParam].Value =
                FlockingParams->Params[WhichParam].Max;
    } else if (FlockingParams->Params[WhichParam].Value <
            FlockingParams->Params[WhichParam].Min) {
        FlockingParams->Params[WhichParam].Value =
                FlockingParams->Params[WhichParam].Min;
    }

}

/* Get the index of a flocking model parameter */
int GetFlParamIndexByName(flocking_model_params_t * FlockingParams,
        const char *Name) {
    fl_param_double_t *result;
    int i;
    for (i = 0; i < FlockingParams->NumberOfParameters; i++) {
        result = &FlockingParams->Params[i];
        if (!strcmp(Name, result->Name) || !strcmp(Name, result->NameInFiles))
            return i;
    }
    return -1;
}

/* Allocate memory for a param and store it in the params structure */
void CreateFlockingParam(flocking_model_params_t * FlockingParams,
        fl_param_double_t param) {
    FlockingParams->Params[FlockingParams->NumberOfParameters++] = param;
}

/* Refresh static double param Value variables defined in the model files */
void RefreshFlockingParams(flocking_model_params_t * FlockingParams) {
    int i = 0;
    for (i = 0; i < FlockingParams->NumberOfParameters; i++)
        *FlockingParams->Params[i].StaticValuePointer =
                FlockingParams->Params[i].Value;
}

/* */
bool FillParameterSetsFromFile(flocking_model_params_t * FlockingParamSets,
        unit_model_params_t * UnitParamSets,
        int argc,
        int *NumberOfFlockingModelParamSets,
        int *NumberOfUnitModelParamSets,
        char *argv[], char *CurrentDirectory, char *ParamsFileName) {

    FILE *ParamsFile;
    int i, j;

    /* 
     * Unit model parameters
     */

    /* Checking existence of default file */
    if (0 == (*NumberOfUnitModelParamSets)) {
        strcpy(ParamsFileName, CurrentDirectory);
        strcat(ParamsFileName, "/parameters/unitparams.dat\0");
        printf("Using unitparams: %s\n", ParamsFileName);
        ParamsFile = fopen(ParamsFileName, "r");
        if (ParamsFile) {
            strcpy(UnitParamSets[*NumberOfUnitModelParamSets].FileName,
                    ParamsFileName);
            GetUnitModelParamsFromFile(&UnitParamSets
                    [*NumberOfUnitModelParamSets], ParamsFile);
            (*NumberOfUnitModelParamSets)++;
            fclose(ParamsFile);
        } else {
            printf("  Could not open file for reading!\n");
            return false;
        }
    } else {
        *NumberOfUnitModelParamSets = 0;
    }

    /* Checking existence of flagged input files */
    for (i = 0; i < argc - 1; i++) {
        if (strcmp(argv[i + 1], ParamsFileName) == 0)
            continue;
        if (strcmp(argv[i], "-u") == 0) {
            printf("Using unitparams: %s\n", argv[i + 1]);
            ParamsFile = fopen(argv[i + 1], "r");
            if (ParamsFile) {
                strcpy(UnitParamSets[*NumberOfUnitModelParamSets].FileName,
                        argv[i + 1]);
                strcat(UnitParamSets[*NumberOfUnitModelParamSets].FileName,
                        "\0");
                GetUnitModelParamsFromFile(&UnitParamSets
                        [*NumberOfUnitModelParamSets], ParamsFile);
                (*NumberOfUnitModelParamSets)++;
                fclose(ParamsFile);
            } else {
                printf("  Could not open file for reading!\n");
                return false;
            }
        }
    }

    /*
     * Flocking algorithm parameters 
     */

    /* Checking existence of default file */
    if (0 == (*NumberOfFlockingModelParamSets)) {
        strcpy(ParamsFileName, CurrentDirectory);
        strcat(ParamsFileName, "/parameters/flockingparams.dat\0");
        printf("Using flockingparams: %s\n", ParamsFileName);
        ParamsFile = fopen(ParamsFileName, "r");
        if (ParamsFile) {
            strcpy(FlockingParamSets[*NumberOfFlockingModelParamSets].FileName,
                    ParamsFileName);
            GetFlockingModelParamsFromFile(&FlockingParamSets
                    [*NumberOfFlockingModelParamSets], ParamsFile);
            (*NumberOfFlockingModelParamSets)++;
            fclose(ParamsFile);
        } else {
            printf("  Could not open file for reading!\n");
            return false;
        }
    } else {
        *NumberOfFlockingModelParamSets = 0;
    }
    /* Checking existence of flagged input files */
    for (i = 0; i < argc - 1; i++) {
        if (strcmp(argv[i + 1], ParamsFileName) == 0)
            continue;
        if (strcmp(argv[i], "-f") == 0) {
            printf("Using flockingparams: %s\n", argv[i + 1]);
            ParamsFile = fopen(argv[i + 1], "r");
            if (ParamsFile) {
                strcpy(FlockingParamSets
                        [*NumberOfFlockingModelParamSets].FileName,
                        argv[i + 1]);
                strcat(FlockingParamSets
                        [*NumberOfFlockingModelParamSets].FileName, "\0");
                GetFlockingModelParamsFromFile(&FlockingParamSets
                        [*NumberOfFlockingModelParamSets], ParamsFile);
                (*NumberOfFlockingModelParamSets)++;
                fclose(ParamsFile);
            } else {
                printf("  Could not open file for reading!\n");
                return false;
            }
        }
    }
    return true;
}
