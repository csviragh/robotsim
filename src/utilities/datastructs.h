/* vim:set ts=4 sw=4 sts=4 et: */

/*
 * Tools for allocating, freeing and saving/loading data structures 
 */

#ifndef DATASTRUCTS_H
#define DATASTRUCTS_H

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
typedef struct {

    double *states;
    char Name[64];

} inner_state_double_t;

/* Specific state structures (not position or velocity) */

/* Tools for allocating and freeing data structures */

/* Allocates a dynamic array of boolean variables 
 * "size" defines the size of the array
 */
bool *BooleanData(int size);

/* Allocates a dynamic array of integer variables 
 * "size" defines the size of the array
 */
int *intData(int size);

/* Allocates a dynamic array of double variables 
 * Arrays allocated with this can be used as vector structures
 * "size" defines the size of the array
 */
double *doubleVector(int size);

/* Allocates a 2 dimensional dynamic array of double variables 
 * Arrays allocated with this can be used as matrix structures
 * "rows" and "cols" defines the number of rows and columns of the array, respectively
 */
double **doubleMatrix(int rows, int cols);

/* Allocates a 3-indexed tensory data row
 * Arrays allocated with this can be used as a time-dependent matrix structure
 * "rows", "cols" and "timesteps" defines the size of the array. 
 */
double ***doubleTimeIndexedMatrix(int rows, int cols, int timesteps);

/* Allocating inner state vector 
 */
inner_state_double_t *InnerStateData(int size);

/* Frees inner state vectors 
 */
void freeInnerStates(inner_state_double_t * InnerStatesToFree, int size);

/* Frees 2D dynamic arrays (matrices) 
 * "rows" and "cols" defines the number of rows and columns of the array, respectively
 */
void freeMatrix(double **Matrix, int rows, int cols);

/* Frees 3D dynamic array
 */
void freeTimeIndexedMatrix(double ***TimeIndexedMatrix, int timesteps,
        int rows, int cols);

/* File I/O tools */

/* Saves data of a 1D double array ("vec") into a file ("outputfile")
 * "dimension" defines the size of the array
 */
void saveVector(FILE * outfile, double *vec, int dimension);

/* Reads data of a 2D double array ("mat") from a file ("input")
 * "rows" and "cols" defines the number of rows and columns of the array, respectively
 */
void loadMatrix(FILE * infile, double **mat, int rows, int cols);

/* Saves data of a 2D double array ("mat") into a file ("outputfile")
 * "rows" and "cols" defines the number of rows and columns of the array, respectively
 * The columns will be separated with '\t' in the output file
 */
void saveMatrix(FILE * outfile, double **mat, int rows, int cols);

#endif
