//MIT License
//Copyright (c) 2018 Eotvos Lorand University, Budapest

/* vim:set ts=4 sw=4 sts=4 et: */

/* Tools for drawing objects in 2D visualization
 */

#ifndef OBJECTS_2D_H
#define OBJECTS_2D_H

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <GL/freeglut.h>

#include <stdbool.h>

/* Windows and other general tools */

/* Displays a window with given width and height at (posX, posY) coordinates
 * "width", "height", "posX" and "posY" are in pixels.
 */
void DisplayWindow(int width, int height, int posX, int posY);

/* Simple euclidean objects in GL coords */

/* Draws a shape with given center coordinates (in GL coords), width, height and rotation angle
 * "CenterX" and "CenterY" are the coordinates of the center of the shape
 * "color" is an array that contains RGB values
 */
void DrawShape(double CenterX, double CenterY, double width, double height,
        double angle, const float *color);

/* Draws a line with given endpoint coordinates (in GL coords). 
 * (x1, y1) and (x2, y2) are the endpoint coordinates
 * "width" is the width of the line
 * "color" is an array that contains RGB values 
 */
void DrawLine(double x1, double y1, double x2, double y2, double width,
        const float *color);

/* Draws an empty tetragon with given vertices (?) 
 * (x1, y1) , (x2, y2) , (x3, y3) , (x4, y4) are the vertices of the tetragon
 * "width" is the width of the edges
 * "color" is an array that contains RGB values
 */
void DrawTetragon(double x1, double y1, double x2,
        double y2, double x3, double y3,
        double x4, double y4, double width, const float *color);

/* Draws a fulle tetragon with given vertices (?) 
 * (x1, y1) , (x2, y2) , (x3, y3) , (x4, y4) are the vertices of the tetragon
 * "color" is an array that contains RGB values
 */
void DrawFullTetragon(double x1, double y1, double x2, double y2, double x3,
        double y3, double x4, double y4, const float *color);

/* Draws a trinagle (platonic, if the windth and height of the window are equal) 
 * "CenterX" and "CenterY" are the GL coordinates of the center of the tirangle
 * "LenghtOfEdge" defines the size of the triangle (relative to the window)
 * The final triangle will be rotated with "Angle"
 * "color" is an array that contains RGB values
 */
void DrawTriangle(double CenterX, double CenterY, double LengthOfEdge,
        double Angle, const float *color);

/* Draws an empty circle with given center coordinates (GL coords) and radius.
 * "color" is an array that contains RGB values
 */
void DrawCircle(double Centerx, double Centery, double radius,
        const float *color);

/* Draws an empty circle with thick edges with given center coordinates (GL coord) and radius 
 * "color" is an array that contains RGB values
 */
void DrawThickEdgedCircle(double Centerx, double Centery, double radius,
        double EdgeThickness, const float *color);

/* Draws an empty circle with gradient-colored edges 
 */
void DrawGradientColoredCircle(const double Centerx, const double Centery,
        const double radius, const double EdgeThickness, const float *color1,
        const float *color2, const int HowManySteps);

/* Draws a full circle with given center coordinates (GL coords) and radius. 
 * "color" is an array that contains RGB values
 */
void DrawFullCircle(double Centerx, double Centery, double radius,
        const float *color);

/* Draws an arrow with given center coordinates ("x" and "y") and Length ("Length") 
 * The final arrow will be rotated by "angle"
 * The width of the arrow will be Length/4
 * "color" is an array that contains RGB values
 */
void DrawArrow(double x, double y, double Length, double angle,
        const float *color);

/* Draws an arrow with given center coordinates ("x" and "y") and Length ("Length")
 * "TriangleEdgeLength" defines the size of the head of the arrow
 * The final arrow will be rotated with "angle".
 * The width of the arrow will be 0.003 in GL coords
 * "color" is an array that contains RGB values
 */
void DrawThinArrow(double x, double y, double Length, double TriangleEdgeLength,
        double angle, const float *color);

/* Draws a transparent (empty) shape.
 * x, y, width and height are in GL coordinates
 * "color" is an array that contains RGB values
 */
void DrawTransparentShape(double x, double y, double width, double height,
        const float *color);

/* Draws a polygon with "NumberOfVertices" vertices 
 */
void DrawPolygon(float *Vertices, const int NumberOfVertices,
        const float *color);

/* Draws a transparent (empty) shape with given edge thickness.
 * EdgeThickness, x, y, width and height are in GL coordinates
 * "color" is an array that contains RGB values
 */
void DrawThickEdgedTransparentShape(const double x, const double y,
        const double width, const double height, const double EdgeThickness,
        double angle, const float *color);

/* Draws a gradient-colored transparent shape with given edge thickness 
 */
void DrawGradientColoredShape(const double x, const double y,
        const double width, const double height, const double EdgeThickness,
        double angle, const float *color1, const float *color2,
        const int HowManySteps);

/* Draws a polygon with "NumberOfVertices" vertices and given edge thickness 
 */
void DrawThickEdgedPolygon(double *Vertices, const int NumberOfVertices,
        const double EdgeThickness, const float *color);

/* Draws a string with given label, font and position.
 * "color" is an array that contains RGB values
 * "font" should be chosen from GLUT font types
 */
void DrawString(float x, float y, void *font, const char *string,
        const float *color);

/* Draws a Lengthscale wth given Length at a given position
 * "MapSizexy" are in real coordinates
 * "Length", "x" and "y" are in GL coordinates
 */
void DrawScale(double Length, double x, double y, double MapSizexy,
        const float *color);

/* Utilities */

/* Converts "real" coordinates into GL coordinates
 * "MapSizexy" and "coord" are in real coordinates (cm)
 */
double RealToGlCoord_2D(double coord, double MapSizexy);

/* Converts "mouse" coordinates to real coords.
 * mouseCoord is in pixels (position of the mouse)
 * MapSizexy is the size of the map (in real coordinates, cm) 
 * Only works when the map is a rectangle
 */
double MouseCoordToReal_2D(int mouseCoord, const double MapSizexy,
        const double Resolution);

#endif
