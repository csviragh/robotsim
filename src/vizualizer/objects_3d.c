//MIT License
//Copyright (c) 2018 Eotvos Lorand University, Budapest

/* vim:set ts=4 sw=4 sts=4 et: */

/*
 * Tools for drawing 3D objects
 */

#include "objects_3d.h"

#include <stdio.h>

/* Sphere */
void DrawSolidSphere_3D(const float x, const float y, const float z,
        const float r, const float *color) {

    glTranslatef(x, y, z);
    glColor3f(color[0], color[1], color[2]);
    glutSolidSphere(r, 30, 30);
    glTranslatef(-x, -y, -z);

}
void DrawWireSphere_3D(const float x, const float y, const float z,
        const float r, const float *color) {

    glTranslatef(x, y, z);
    glColor3f(color[0], color[1], color[2]);
    glutWireSphere(r, 30, 30);
    glTranslatef(-x, -y, -z);

}

/* Tetrahedron */
void DrawSolidTetrahedron_3D(const float x,
        const float y, const float z, const float r, const float *color) {

    glTranslatef(x, y, z);
    glColor3f(0.0, 0.0, 0.0);
    glScalef(r, r, r);
    glutWireTetrahedron();
    glColor3f(color[0], color[1], color[2]);
    glutSolidTetrahedron();
    glScalef(1.0 / r, 1.0 / r, 1.0 / r);
    glTranslatef(-x, -y, -z);

}

/* Tetrahedron */
void DrawWiredTetrahedron_3D(const float x,
        const float y,
        const float z, const float r, const float width, const float *color) {

    glTranslatef(x, y, z);
    glColor3f(0.0, 0.0, 0.0);
    glScalef(r, r, r);
    glLineWidth(width);
    glutWireTetrahedron();
    glLineWidth(1.0);
    glColor3f(color[0], color[1], color[2]);
    //glutSolidTetrahedron ();
    glScalef(1.0 / r, 1.0 / r, 1.0 / r);
    glTranslatef(-x, -y, -z);

}

/* Line */
void DrawLine_3D(const float x1,
        const float y1,
        const float z1,
        const float x2,
        const float y2, const float z2, const float width, const float *color) {

    /* Setting up color */
    glColor3f(color[0], color[1], color[2]);

    /* Setting up width of the line (in GL coordinates) */
    glLineWidth(width);

    /* Draw line */
    glBegin(GL_LINES);
    glVertex3f(x1, y1, z1);
    glVertex3f(x2, y2, z2);
    glEnd();

    /* Reset previous line width */
    glLineWidth(1.0);

}

/* Circle */
void DrawCircle_3D(const float x, const float y, const float z,
        const float radius,
        const float width, double *NormalVect, const float *color) {

    // STOP READING. NOW! (This code is not optimal... )

    glColor3f(color[0], color[1], color[2]);
    double i;
    int j, k;

    /* Normalizing normal vect, if it is necessary */
    static double Magnitude;
    Magnitude =
            sqrt(NormalVect[0] * NormalVect[0] + NormalVect[1] * NormalVect[1] +
            NormalVect[2] * NormalVect[2]);
    if (Magnitude != 0.0 && Magnitude != 1.0) {
        for (j = 0; j < 3; j++) {
            NormalVect[j] /= Magnitude;
        }
    }

    /* Circle equals several lines connected to an N-gon, where N is large enough */
    static double ToSideVect[3];
    static double Temp[3];
    static const double a = 0.2, b = 0.6;
    static const double sinAngle = 0.009999833334166664;        // sin (0.01)
    static const double cosAngle = 0.999950000416665300;        // cos (0.01)

    /* Creating a normalized perpendicular vector */
    ToSideVect[0] = NormalVect[1] * a - NormalVect[2] * b;
    ToSideVect[1] = -NormalVect[0] * a;
    ToSideVect[2] = NormalVect[0] * b;

    Magnitude =
            sqrt(ToSideVect[0] * ToSideVect[0] + ToSideVect[1] * ToSideVect[1] +
            ToSideVect[2] * ToSideVect[2]);

    for (j = 0; j < 3; j++) {
        ToSideVect[j] /= Magnitude;
    }

    glLineWidth(width);

    glBegin(GL_LINES);
    for (i = 0; i < 2 * M_PI; i += 0.01) {

        glVertex3f(x + radius * ToSideVect[0], y + radius * ToSideVect[1],
                z + radius * ToSideVect[2]);

        memcpy(Temp, ToSideVect, 3 * sizeof(double));
        ToSideVect[0] =
                (cosAngle + NormalVect[0] * NormalVect[0] * (1 -
                        cosAngle)) * Temp[0]
                + (NormalVect[0] * NormalVect[1] * (1 - cosAngle) -
                NormalVect[2] * sinAngle) * Temp[1]
                + (NormalVect[0] * NormalVect[2] * (1 - cosAngle) +
                NormalVect[1] * sinAngle) * Temp[2];
        ToSideVect[1] =
                (NormalVect[0] * NormalVect[1] * (1 - cosAngle) +
                NormalVect[2] * sinAngle) * Temp[0]
                + (cosAngle + NormalVect[1] * NormalVect[1] * (1 -
                        cosAngle)) * Temp[1]
                + (NormalVect[1] * NormalVect[2] * (1 - cosAngle) -
                NormalVect[0] * sinAngle) * Temp[2];
        ToSideVect[2] =
                (NormalVect[2] * NormalVect[0] * (1 - cosAngle) -
                NormalVect[1] * sinAngle) * Temp[0]
                + (NormalVect[2] * NormalVect[1] * (1 - cosAngle) +
                NormalVect[0] * sinAngle) * Temp[1]
                + (cosAngle + NormalVect[2] * NormalVect[2] * (1 -
                        cosAngle)) * Temp[2];

    }
    glEnd();

    glLineWidth(1.0);

}

void DrawPrism_3D(double *Edges_On_XY_Plane,
        const int NumberOfVertices,
        const float height, const float WireWidth, const float *color) {

    int i;

    glColor3f(color[0], color[1], color[2]);
    glLineWidth(WireWidth);

    glBegin(GL_LINES);
    for (i = 1; i < NumberOfVertices + 1; i++) {
        glVertex3f(Edges_On_XY_Plane[((i - 1) % NumberOfVertices) * 3],
                Edges_On_XY_Plane[((i - 1) % NumberOfVertices) * 3 + 1],
                Edges_On_XY_Plane[((i - 1) % NumberOfVertices) * 3 + 2]);
        glVertex3f(Edges_On_XY_Plane[(i % NumberOfVertices) * 3],
                Edges_On_XY_Plane[(i % NumberOfVertices) * 3 + 1],
                Edges_On_XY_Plane[(i % NumberOfVertices) * 3 + 2]);
        glVertex3f(Edges_On_XY_Plane[((i - 1) % NumberOfVertices) * 3],
                Edges_On_XY_Plane[((i - 1) % NumberOfVertices) * 3 + 1],
                Edges_On_XY_Plane[((i - 1) % NumberOfVertices) * 3 + 2] +
                height);
        glVertex3f(Edges_On_XY_Plane[(i % NumberOfVertices) * 3],
                Edges_On_XY_Plane[(i % NumberOfVertices) * 3 + 1],
                Edges_On_XY_Plane[(i % NumberOfVertices) * 3 + 2] + height);
        glVertex3f(Edges_On_XY_Plane[((i - 1) % NumberOfVertices) * 3],
                Edges_On_XY_Plane[((i - 1) % NumberOfVertices) * 3 + 1],
                Edges_On_XY_Plane[((i - 1) % NumberOfVertices) * 3 + 2]);
        glVertex3f(Edges_On_XY_Plane[((i - 1) % NumberOfVertices) * 3],
                Edges_On_XY_Plane[((i - 1) % NumberOfVertices) * 3 + 1],
                Edges_On_XY_Plane[((i - 1) % NumberOfVertices) * 3 + 2] +
                height);
    }
    glEnd();

    glLineWidth(1.0);

}

void DrawCylinder_3D(const float x_center,
        const float y_center,
        const float z_center,
        const float radius,
        const float height, const float WireWidth, const float *color) {

    float i;

    glColor3f(color[0], color[1], color[2]);
    glLineWidth(WireWidth);

    /* Floor */
    glBegin(GL_LINE_STRIP);
    for (i = 0.0; i < 2 * M_PI; i += 0.01) {
        glVertex3f(x_center + radius * cos(i), -(-y_center + radius * sin(i)),
                z_center);
    }
    glEnd();

    /* Roof */
    glBegin(GL_LINE_STRIP);
    for (i = 0.0; i < 2 * M_PI; i += 0.01) {
        glVertex3f(x_center + radius * cos(i), -(-y_center + radius * sin(i)),
                z_center + height);
    }
    glEnd();

    /* 16 Pillars */
    glBegin(GL_LINES);
    for (i = 0; i < 2 * M_PI; i += 2 * M_PI / 16) {
        glVertex3f(x_center + radius * cos(i), -(-y_center + radius * sin(i)),
                z_center);
        glVertex3f(x_center + radius * cos(i), -(-y_center + radius * sin(i)),
                z_center + height);
    }
    glEnd();

    glLineWidth(1.0);

}

/* Rectangle */
void DrawHorizontalRectangle_3D(const float x,
        const float y,
        const float z,
        const float width, const float height, const float *color) {

}

/* A room with floor and transparent walls */
void DrawCubicRoom_3D(const float x,
        const float y, const float z, const float L, const float tiledensity) {

    /* Lines */
    glBegin(GL_LINES);

    glVertex3f(x - L / 2, y - L / 2, z - L / 2);
    glVertex3f(x - L / 2, y - L / 2, z + L / 2);
    glVertex3f(x - L / 2, y + L / 2, z - L / 2);
    glVertex3f(x - L / 2, y + L / 2, z + L / 2);
    glVertex3f(x + L / 2, y - L / 2, z - L / 2);
    glVertex3f(x + L / 2, y - L / 2, z + L / 2);
    glVertex3f(x + L / 2, y + L / 2, z - L / 2);
    glVertex3f(x + L / 2, y + L / 2, z + L / 2);
    glVertex3f(x - L / 2, y - L / 2, z - L / 2);
    glVertex3f(x - L / 2, y - L / 2, z + L / 2);
    glVertex3f(x - L / 2, y + L / 2, z - L / 2);
    glVertex3f(x - L / 2, y + L / 2, z + L / 2);

    glEnd();

}

void DrawCopter_3D(const double x, const double y, const double z,
        const double MapSizexy, const double Radius, const float *color) {

    DrawSolidSphere_3D(RealToGlCoord_3D(x, MapSizexy), RealToGlCoord_3D(y,
                    MapSizexy), RealToGlCoord_3D(z, MapSizexy),
            RealToGlCoord_3D(Radius / 2.0, MapSizexy), color);

}

void DrawAgentLabel_3D(double **Phase,
        const int WhichAgent,
        char *Label,
        bool Display,
        double CenterX, double CenterY, double MapSizeXY, const float *Color) {

}

/* Utilities */

double RealToGlCoord_3D(const double coord, const double MapSizexy) {

    return coord / MapSizexy;

}
