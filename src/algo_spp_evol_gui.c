/* vim:set ts=4 sw=4 sts=4 et: */

/* Model-Specific GUI tools
 */

#include "algo_gui.h"
#include "algo_spp_evol.h"

/* Edges of the arena */
static double VerticesOfArena[12];
static double EdgesOfObstacle[MAX_OBSTACLES][MAX_OBSTACLE_POINTS * 2];

/* Colors of model-specific objects */
static float ArenaEdgeColor[3];
static float ObstacleColor[3];

/* For reseting Phases */
void ModelSpecificReset(phase_t * Phase,
        const double Size_X,
        const double Size_Y,
        const double Size_Z,
        vizmode_params_t * VizParams,
        flocking_model_params_t * FL_Params, const double Radius) {

}

void SetupVertices(vizmode_params_t * VizParams) {
    int i;

    /* Cutting trees, sorry! */
    for (i = 0; i < 12; i++) {
        // Z components
        if (2 == (i % 3)) {
            VerticesOfArena[i] =
                    RealToGlCoord_3D(-ArenaRadius, VizParams->MapSizeXY);
            // X components
        } else if (0 == (i % 3)) {
            if (i < 4) {
                VerticesOfArena[i] =
                        RealToGlCoord_3D(-ArenaRadius + ArenaCenterX,
                        VizParams->MapSizeXY);
            } else {
                VerticesOfArena[i] =
                        RealToGlCoord_3D(ArenaRadius + ArenaCenterX,
                        VizParams->MapSizeXY);
            }
            // Y components
        } else {
            if (1 == (i % 9)) {
                VerticesOfArena[i] =
                        RealToGlCoord_3D(-ArenaRadius + ArenaCenterY,
                        VizParams->MapSizeXY);
            } else {
                VerticesOfArena[i] =
                        RealToGlCoord_3D(ArenaRadius + ArenaCenterY,
                        VizParams->MapSizeXY);
            }
        }
    }
}

/* Creating vertex set for prisms and polygons to draw (obstacles and arena) */
void CreateObstacleVertexSet(obstacles_t * Obstacles,
        vizmode_params_t * VizParams) {
    int i, j;
    for (j = 0; j < Obstacles->o_count; j++) {
        for (i = 0; i < Obstacles->o[j].p_count; i++) {
            if (VizParams->TwoDimViz == false) {
                EdgesOfObstacle[j][i * 3] =
                        RealToGlCoord_3D(Obstacles->o[j].p[i][0],
                        VizParams->MapSizeXY);
                EdgesOfObstacle[j][i * 3 + 1] =
                        RealToGlCoord_3D(Obstacles->o[j].p[i][1],
                        VizParams->MapSizeXY);
                EdgesOfObstacle[j][i * 3 + 2] =
                        RealToGlCoord_3D(-990.0, VizParams->MapSizeXY);
            } else {
                EdgesOfObstacle[j][i * 3] =
                        RealToGlCoord_2D(Obstacles->o[j].p[i][0] -
                        VizParams->CenterX, VizParams->MapSizeXY);
                EdgesOfObstacle[j][i * 3 + 1] =
                        RealToGlCoord_2D(Obstacles->o[j].p[i][1] -
                        VizParams->CenterY, VizParams->MapSizeXY);
                EdgesOfObstacle[j][i * 3 + 2] =
                        RealToGlCoord_2D(0.0, VizParams->MapSizeXY);
            }
        }
    }
}

/* For Initializing vizualisation mode parameters */
void InitializeVizParams(vizmode_params_t * VizParams) {
    if (0.0 != ArenaShape)
        SetupVertices(VizParams);
    CreateObstacleVertexSet(&obstacles, VizParams);
}

/* For Initializing mode-specific colors */
void InitializeModelSpecificColors(model_specific_color_t * ModelSpecificColors,
        int *NumberOfModelSpecificColors) {
    CREATE_COLOR(ArenaEdgeColor, 0.0, 0.0, 0.0);
    CREATE_COLOR(ObstacleColor, 1.0, 0.0, 0.0);
}

/* For drawing model-specific objects */
void DrawModelSpecificObjects_2D(phase_t * Phase,
        flocking_model_params_t * FlockingParams,
        unit_model_params_t * UnitParams, vizmode_params_t * VizParams,
        color_config_t * Colors, sit_parameters_t * SitParams) {
    int i;
    double Gamma_Shill = V_Shill / Slope_Shill + R_0_Shill;

    // draw arena
    if (0.0 == ArenaShape) {
        DrawThickEdgedCircle(RealToGlCoord_2D(-VizParams->CenterX +
                        ArenaCenterX, VizParams->MapSizeXY),
                RealToGlCoord_2D(-VizParams->CenterY + ArenaCenterY,
                        VizParams->MapSizeXY),
                RealToGlCoord_2D(ArenaRadius - Gamma_Shill / 2,
                        VizParams->MapSizeXY), RealToGlCoord_2D(Gamma_Shill,
                        VizParams->MapSizeXY), ArenaEdgeColor);
    } else {
        SetupVertices(VizParams);
        DrawThickEdgedTransparentShape(RealToGlCoord_2D(ArenaCenterX -
                        VizParams->CenterX, VizParams->MapSizeXY),
                RealToGlCoord_2D(ArenaCenterY - VizParams->CenterY,
                        VizParams->MapSizeXY),
                RealToGlCoord_2D(2.0 * ArenaRadius - Gamma_Shill,
                        VizParams->MapSizeXY),
                RealToGlCoord_2D(2.0 * ArenaRadius - Gamma_Shill,
                        VizParams->MapSizeXY), RealToGlCoord_2D(Gamma_Shill,
                        VizParams->MapSizeXY), 0.0, ArenaEdgeColor);
    }

    // draw obstacles
    CreateObstacleVertexSet(&obstacles, VizParams);
    for (i = 0; i < obstacles.o_count; i++) {
        DrawThickEdgedPolygon(EdgesOfObstacle[i], obstacles.o[i].p_count,
                RealToGlCoord_2D(Gamma_Shill, VizParams->MapSizeXY),
                ObstacleColor);
    }
}

void DrawModelSpecificObjects_3D(phase_t * Phase,
        flocking_model_params_t * FlockingParams,
        unit_model_params_t * UnitParams,
        vizmode_params_t * VizParams,
        color_config_t * Colors, sit_parameters_t * SitParams) {
    int i;
    // draw arena
    if (1.0 == ArenaShape) {
        DrawPrism_3D(VerticesOfArena,
                4,
                RealToGlCoord_3D(ArenaRadius * 2.0, VizParams->MapSizeXY),
                10000.0 * RealToGlCoord_3D(300.0, VizParams->MapSizeXY),
                ArenaEdgeColor);
    } else if (0.0 == ArenaShape || 2.0 == ArenaShape) {
        DrawWireSphere_3D(RealToGlCoord_2D(ArenaCenterX, VizParams->MapSizeXY),
                RealToGlCoord_2D(ArenaCenterY, VizParams->MapSizeXY),
                RealToGlCoord_2D(0.0, VizParams->MapSizeXY),
                RealToGlCoord_2D(ArenaRadius, VizParams->MapSizeXY),
                ArenaEdgeColor);

    }
    // draw obstacles
    CreateObstacleVertexSet(&obstacles, VizParams);
    for (i = 0; i < obstacles.o_count; i++) {
        DrawPrism_3D(EdgesOfObstacle[i], obstacles.o[i].p_count,
                RealToGlCoord_3D(2000.0, VizParams->MapSizeXY),
                10000.0 * RealToGlCoord_3D(300.0, VizParams->MapSizeXY),
                ObstacleColor);
    }
}

/* For writing out a model-specific string */
char *GetModelSpecificStringToDisplay(phase_t * Phase,
        flocking_model_params_t * FlockingParams,
        unit_model_params_t * UnitModelParams,
        sit_parameters_t * SitParams, double TimeStep) {
    return "";
}

/* For handling model-specific keboard events */
void HandleSpecialKeyBoardEvent(unsigned char key,
        int x,
        int y,
        flocking_model_params_t * FlockingParams,
        vizmode_params_t * VizParams,
        sit_parameters_t * SitParams, const int Modifier) {

    SetupVertices(VizParams);
    CreateObstacleVertexSet(&obstacles, VizParams);
}

/* For handling model-specific "special key" events */
void HandleSpecialSpecKeyEvent(unsigned char key,
        int x,
        int y,
        flocking_model_params_t * FlockingParams,
        vizmode_params_t * VizParams,
        sit_parameters_t * SitParams, const int Modifier) {

    SetupVertices(VizParams);
    CreateObstacleVertexSet(&obstacles, VizParams);

    if (GLUT_KEY_F6 == key) {
        CHANGE_PARAMETER(ArenaShape,
                (double) ((int) (ArenaShape + 1) % (int) 2));
    }
}

/* For handling model-specific mouse events */
void HandleSpecialMouseEvent(int button,
        int state,
        int x,
        int y,
        flocking_model_params_t * FlockingParams,
        vizmode_params_t * VizParams, const int Modifier) {

    if (VizParams->TwoDimViz && button == GLUT_LEFT && state == GLUT_DOWN) {
        CHANGE_PARAMETER(ArenaCenterX, MouseCoordToReal_2D(x,
                        VizParams->MapSizeXY,
                        VizParams->Resolution) + VizParams->CenterX);
        CHANGE_PARAMETER(ArenaCenterY, -MouseCoordToReal_2D(y,
                        VizParams->MapSizeXY,
                        VizParams->Resolution) + VizParams->CenterY);
    }

    SetupVertices(VizParams);
    CreateObstacleVertexSet(&obstacles, VizParams);
}
