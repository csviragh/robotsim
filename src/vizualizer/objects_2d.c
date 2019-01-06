//MIT License
//Copyright (c) 2018 Eotvos Lorand University, Budapest

/* vim:set ts=4 sw=4 sts=4 et: */

/* Tools for drawing objects in 2D visualization
 */

#include "objects_2d.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

/* Useful global variables */
static float ActualColor[3];

/* Windows and other general tools */

/* Displays a window with given width and height at (posX, posY) coordinates */
void DisplayWindow(int width, int height, int posX, int posY) {

    glutInitWindowSize(width, height);
    glutInitWindowPosition(posX, posY);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);

    //glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH);                                  //Remove these slashes if you don't need double buffered mode
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

}

/* Simple euclidean objects in GL coords */

/* Draws a shape with given center coordinates (in GL coords), width, height and rotation angle */
void DrawShape(double CenterX, double CenterY, double width, double height,
        double angle, const float *color) {

    glColor3f(color[0], color[1], color[2]);
    glBegin(GL_QUADS);
    {
        glVertex2f(CenterX + cos(angle) * width / 2.0 +
                sin(angle) * height / 2.0,
                CenterY - sin(angle) * width / 2.0 + cos(angle) * height / 2.0);
        glVertex2f(CenterX - cos(angle) * width / 2.0 +
                sin(angle) * height / 2.0,
                CenterY + sin(angle) * width / 2.0 + cos(angle) * height / 2.0);
        glVertex2f(CenterX - cos(angle) * width / 2.0 -
                sin(angle) * height / 2.0,
                CenterY + sin(angle) * width / 2.0 - cos(angle) * height / 2.0);
        glVertex2f(CenterX + cos(angle) * width / 2.0 -
                sin(angle) * height / 2.0,
                CenterY - sin(angle) * width / 2.0 - cos(angle) * height / 2.0);
    }
    glEnd();

}

/* Draws a line with given endpoint coordinates (in GL coords) */
void DrawLine(double x1, double y1, double x2, double y2, double width,
        const float *color) {

    /* Line is a thin shape */
    DrawShape(0.5 * (x1 + x2), 0.5 * (y1 + y2), width,
            sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1)), atan2(x2 - x1,
                    y2 - y1), color);

}

/* Draws an empty tetragon with given vertices (?) */
void DrawTetragon(double x1, double y1,
        double x2, double y2,
        double x3, double y3,
        double x4, double y4, double width, const float *color) {

}

/* Draws a fulle tetragon with given vertices (?) */
void DrawFullTetragon(double x1, double y1,
        double x2, double y2,
        double x3, double y3, double x4, double y4, const float *color) {

}

/* Draws a trinagle (platonic, if the width and height of the window are equal) */
void DrawTriangle(double CenterX, double CenterY, double LengthOfEdge,
        double Angle, const float *color) {

    glColor3f(color[0], color[1], color[2]);
    double m = LengthOfEdge / (2 * sqrt(3));
    double b = 2.0 * m;

    glBegin(GL_TRIANGLES);
    {
        glVertex2f(CenterX - cos(Angle) * m + sin(Angle) * LengthOfEdge / 2,
                CenterY + sin(Angle) * m + cos(Angle) * LengthOfEdge / 2);
        glVertex2f(CenterX + cos(Angle) * b, CenterY - sin(Angle) * b);
        glVertex2f(CenterX - cos(Angle) * m - sin(Angle) * LengthOfEdge / 2,
                CenterY + sin(Angle) * b - cos(Angle) * LengthOfEdge / 2);
    }
    glEnd();

}

/* Draws an empty circle with given center coordinates (GL coords) and radius */
void DrawCircle(double Centerx, double Centery, double radius,
        const float *color) {

    glColor3f(color[0], color[1], color[2]);
    float i;

    /* Cicrle equals several lines connected to an N-gon, where N is large enough */
    glBegin(GL_LINE_STRIP);
    for (i = 0; i < 2 * M_PI; i += 0.01) {

        glVertex3f(Centerx + radius * cos(i), -(-Centery + radius * sin(i)), 1);

    }
    glEnd();

}

/* Draws an empty circle with thick edges with given center coordinates (GL coord) and radius */
void DrawThickEdgedCircle(double Centerx, double Centery, double radius,
        double EdgeThickness, const float *color) {

    glColor3f(color[0], color[1], color[2]);
    double i;

    /* Circle equals several shapes... */
    for (i = 0; i < 2 * M_PI; i += 0.1) {
        DrawShape(Centerx + radius * cos(i), -(-Centery + radius * sin(i)),
                EdgeThickness, 0.1 * radius, i, color);
    }

}

void DrawGradientColoredCircle(const double Centerx, const double Centery,
        const double radius, const double EdgeThickness, const float *color1,
        const float *color2, const int HowManySteps) {

    int i;
    ActualColor[0] = color1[0];
    ActualColor[1] = color1[1];
    ActualColor[2] = color1[2];

    for (i = 0; i < HowManySteps; i++) {

        ActualColor[0] += (color2[0] - color1[0]) / HowManySteps;
        ActualColor[1] += (color2[1] - color1[1]) / HowManySteps;
        ActualColor[2] += (color2[2] - color1[2]) / HowManySteps;

        DrawThickEdgedCircle(Centerx, Centery,
                radius - EdgeThickness / 2.0 + i * EdgeThickness / HowManySteps,
                EdgeThickness / HowManySteps, ActualColor);

    }

}

/* Draws a full circle with given center coordinates (GL coords) and radius */
void DrawFullCircle(double Centerx, double Centery, double radius,
        const float *color) {

    double i;

    /* A full circle can be drawed by several rotated shapes */
    for (i = 0; i < 2 * M_PI; i += 0.1) {

        DrawShape(Centerx, Centery, radius * M_SQRT2, radius * M_SQRT2, i,
                color);

    }

}

/* Draws an arrow with given center coordinates ("x" and "y") and Length ("Length") */
void DrawArrow(double x, double y, double Length, double angle,
        const float *color) {

    /* An arrow is a shape with a triangle on one of its end */
    DrawTriangle(x + cos(angle) * Length / 2.0, y - sin(angle) * Length / 2.0,
            Length, angle, color);
    DrawShape(x, y, Length, Length / 4.0, angle, color);

}

/* Draws an arrow with given center coordinates ("x" and "y") and Length ("Length") */
void DrawThinArrow(double x, double y, double Length, double TriangleEdgeLength,
        double angle, const float *color) {

    /* An arrow is a shape with a triangle on one of its end */
    DrawTriangle(x + cos(angle) * Length / 2, y - sin(angle) * Length / 2,
            TriangleEdgeLength, angle, color);
    DrawShape(x, y, Length, 0.003, angle, color);

}

/* Draws a transparent (empty) shape */
void DrawTransparentShape(double x, double y, double width, double height,
        const float *color) {

    glColor3f(color[0], color[1], color[2]);

    glBegin(GL_LINES);

    glVertex3f(x, y, 1);
    glVertex3f(x + width, y, 1);
    glVertex3f(x + width, y, 1);
    glVertex3f(x + width, y + height, 1);
    glVertex3f(x, y + height, 1);
    glVertex3f(x + width, y + height, 1);
    glVertex3f(x, y + height, 1);
    glVertex3f(x, y, 1);

    glEnd();

}

/* Draws a polygon with "NumberOfVertices" vertices */
void DrawPolygon(float *Vertices, const int NumberOfVertices,
        const float *color) {

    glColor3f(color[0], color[1], color[2]);
    int i;

    glBegin(GL_LINE_STRIP);
    for (i = 1; i < NumberOfVertices + 1; i++) {
        glVertex3f(Vertices[(i % NumberOfVertices) * 3],
                Vertices[(i % NumberOfVertices) * 3 + 1], 1);
    }
    glEnd();

}

/* Draws a transparent (empty) shape with given edge thickness */
void DrawThickEdgedTransparentShape(const double x,
        const double y,
        const double width,
        const double height,
        const double EdgeThickness, double angle, const float *color) {

    angle *= -1;                //Oops...
    DrawShape(x + width / 2.0 * cos(angle),
            y - width / 2.0 * sin(angle),
            EdgeThickness, height + EdgeThickness, angle, color);
    DrawShape(x - height / 2.0 * sin(angle),
            y - height / 2.0 * cos(angle),
            width + EdgeThickness, EdgeThickness, angle, color);
    DrawShape(x - width / 2.0 * cos(angle),
            y + width / 2.0 * sin(angle),
            EdgeThickness, height + EdgeThickness, angle, color);
    DrawShape(x + height / 2.0 * sin(angle),
            y + height / 2.0 * cos(angle),
            width + EdgeThickness, EdgeThickness, angle, color);

}

/* Draws a gradient-colored transparent shape with given edge thickness */
void DrawGradientColoredShape(const double x,
        const double y,
        const double width,
        const double height,
        const double EdgeThickness,
        double angle,
        const float *color1, const float *color2, const int HowManySteps) {

    ActualColor[0] = color1[0];
    ActualColor[1] = color1[1];
    ActualColor[2] = color1[2];
    int i;

    for (i = 0; i < HowManySteps; i++) {

        ActualColor[0] += (color2[0] - color1[0]) / HowManySteps;
        ActualColor[1] += (color2[1] - color1[1]) / HowManySteps;
        ActualColor[2] += (color2[2] - color1[2]) / HowManySteps;

        DrawThickEdgedTransparentShape(x, y,
                width + i * EdgeThickness / HowManySteps,
                height + i * EdgeThickness / HowManySteps,
                EdgeThickness / HowManySteps, angle, ActualColor);
    }

}

/* Draws a polygon with "NumberOfVertices" vertices and given edge thickness */
void DrawThickEdgedPolygon(double *Vertices, const int NumberOfVertices,
        const double EdgeThickness, const float *color) {

    int i;

    for (i = 0; i < NumberOfVertices; i++) {
        DrawLine(Vertices[((i) % NumberOfVertices) * 3],
                Vertices[((i) % NumberOfVertices) * 3 + 1],
                Vertices[(i + 1) % NumberOfVertices * 3],
                Vertices[(i + 1) % NumberOfVertices * 3 + 1], EdgeThickness,
                color);
        DrawFullCircle(Vertices[((i) % NumberOfVertices) * 3],
                Vertices[((i) % NumberOfVertices) * 3 + 1], EdgeThickness / 2.0,
                color);
    }

}

/* Draws a polygon with "NumberOfVertices" vertices and given edge thickness */
void DrawGradientColoredPolygon(double *Vertices,
        const int NumberOfVertices,
        const double EdgeThickness,
        const float *color1, const float *color2, const int HowManySteps) {

    //HARDCORE TODO!!!

    int i, j;

    for (i = 1; i < NumberOfVertices + 1; i++) {

        ActualColor[0] = color1[0];
        ActualColor[1] = color1[1];
        ActualColor[2] = color1[2];

        for (j = 0; j < HowManySteps; j++) {

            ActualColor[0] += (color2[0] - color1[0]) / HowManySteps;
            ActualColor[1] += (color2[1] - color1[1]) / HowManySteps;
            ActualColor[2] += (color2[2] - color1[2]) / HowManySteps;

            DrawLine(Vertices[((i - 1) % NumberOfVertices) * 3] +
                    j * EdgeThickness / HowManySteps,
                    Vertices[((i - 1) % NumberOfVertices) * 3 + 1] -
                    j * EdgeThickness / HowManySteps,
                    Vertices[i % NumberOfVertices * 3] +
                    j * EdgeThickness / HowManySteps,
                    Vertices[i % NumberOfVertices * 3 + 1] -
                    j * EdgeThickness / HowManySteps,
                    EdgeThickness / HowManySteps, ActualColor);

            DrawLine(Vertices[((i - 1) % NumberOfVertices) * 3] -
                    j * EdgeThickness / HowManySteps,
                    Vertices[((i - 1) % NumberOfVertices) * 3 + 1] +
                    j * EdgeThickness / HowManySteps,
                    Vertices[i % NumberOfVertices * 3] -
                    j * EdgeThickness / HowManySteps,
                    Vertices[i % NumberOfVertices * 3 + 1] +
                    j * EdgeThickness / HowManySteps,
                    EdgeThickness / HowManySteps, ActualColor);

        }
    }

}

/* Draws a string with given label, font and position */
void DrawString(float x, float y, void *font, const char *string,
        const float *color) {

    glColor3f(color[0], color[1], color[2]);
    glRasterPos2f(x, y);

    glutBitmapString(font, (const unsigned char *) string);

}

/* Draws a Lengthscale wth given Length at a given position */
void DrawScale(double Length, double x, double y, double MapSizexy,
        const float *color) {

    glColor3f(color[0], color[1], color[2]);

    glBegin(GL_LINES);

    glVertex3f(x, y - 0.05, 1.0);
    glVertex3f(x, y, 1.0);
    glVertex3f(x, y - 0.025, 1.0);
    glVertex3f(x + RealToGlCoord_2D(Length, MapSizexy), y - 0.025, 1.0);
    glVertex3f(x + RealToGlCoord_2D(Length, MapSizexy), y, 1.0);
    glVertex3f(x + RealToGlCoord_2D(Length, MapSizexy), y - 0.05, 1.0);

    glEnd();

    /* Length of the scale in real coordinates (in metres) */
    static char ScaleLength[10];
    sprintf(ScaleLength, "%1.1f m", Length / 100.0);
    DrawString(x, y + 0.0125, GLUT_BITMAP_TIMES_ROMAN_10, ScaleLength, color);

}

/* Utilities */

/* Converts "real" coordinates into GL coordinates */
double RealToGlCoord_2D(double coord, double MapSizexy) {

    return coord / MapSizexy;

}

/* Converts "mouse" coordinates to real coords */
double MouseCoordToReal_2D(int mouseCoord, double MapSizexy,
        const double Resolution) {

    /* 300 and 600 are properties of the "vizualizer" window */
    return MapSizexy * (2.0 * (double) (mouseCoord -
                    0.5 * Resolution) / Resolution);

}
