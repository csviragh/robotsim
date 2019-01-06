//MIT License
//Copyright (c) 2018 Eotvos Lorand University, Budapest

/* vim:set ts=4 sw=4 sts=4 et: */

/* Tools for drawing objects in 3D visualization
 */

#ifndef OBJECTS_3D_H
#define OBJECTS_3D_H

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <GL/freeglut.h>

#include <stdbool.h>
#include <math.h>
#include <string.h>

void DrawSolidSphere_3D(const float x,
        const float y, const float z, const float r, const float *color);

void DrawWireSphere_3D(const float x,
        const float y, const float z, const float r, const float *color);

void DrawSolidTetrahedron_3D(const float x,
        const float y, const float z, const float r, const float *color);

void DrawWiredTetrahedron_3D(const float x,
        const float y,
        const float z, const float r, const float width, const float *color);

void DrawLine_3D(const float x1,
        const float y1,
        const float z1,
        const float x2,
        const float y2, const float z2, const float width, const float *color);

void DrawCircle_3D(const float x, const float y, const float z,
        const float radius,
        const float width, double *NormalVect, const float *color);

void DrawHorizontalRectangle_3D(const float x,
        const float y,
        const float z,
        const float width, const float height, const float *color);

void DrawPrism_3D(double *Edges_On_XY_Plane,
        const int NumberOfVertices,
        const float height, const float WireWidth, const float *color);

void DrawCylinder_3D(const float x_center,
        const float y_center,
        const float z_center,
        const float radius,
        const float height, const float WireWidth, const float *color);

void DrawCopter_3D(double x,
        double y,
        double z,
        const double MapSizexy, const double Radius, const float *color);

void DrawAgentLabel_3D(double **Phase,
        const int WhichAgent,
        char *Label,
        bool Display,
        double CenterX, double CenterY, double MapSizeXY, const float *Color);

double RealToGlCoord_3D(const double coord, const double MapSizexy);

#endif
