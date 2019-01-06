//MIT License
//Copyright (c) 2018 Eotvos Lorand University, Budapest

/* vim:set ts=4 sw=4 sts=4 et: */

/*
 * Tools for allocating, freeing and saving/loading data structures
 */

#include "datastructs.h"

/* Tools for allocating and freeing data structures */

/* Dynamic "boolean" array (1D) */
bool *BooleanData(int size) {

    bool *bools;
    bools = (bool *) calloc(size, sizeof(bool));

    if (bools == NULL) {
        fprintf(stderr, "Boolean vector allocation error!\n");
        exit(-1);
    }

    return bools;

}

/* Dynamic integer array (1D) */
int *intData(int size) {

    int *ints;
    ints = (int *) calloc(size, sizeof(int));

    if (ints == NULL) {
        fprintf(stderr, "Integer vector allocation error!\n");
        exit(-1);
    }

    return ints;

}

/* Dynamic double array (1D, vector) */
double *doubleVector(int size) {

    double *vect;
    vect = (double *) calloc(size, sizeof(double));

    if (vect == NULL) {
        fprintf(stderr, "Vector allocation error!\n");
        exit(-1);
    }

    return vect;

}

/* Dynamic double array (2D, matrix with 'rows' rows and 'cols' columns) */
double **doubleMatrix(int rows, int cols) {

    double **mat;

    mat = (double **) calloc(rows, sizeof(double *));

    if (mat == NULL) {
        fprintf(stderr, "Matrix allocation error!\n");
        exit(-1);
    }
    int i;
    for (i = 0; i < rows; i++) {
        mat[i] = (double *) calloc(cols, sizeof(double));

        if (mat[i] == NULL) {
            fprintf(stderr, "Matrix allocation error!\n");
            exit(-1);
        }

    }

    return mat;

}

/* Dynamic double time-indexed array */
double ***doubleTimeIndexedMatrix(int timesteps, int rows, int cols) {

    double ***mat;

    mat = (double ***) calloc(timesteps, sizeof(double **));

    if (mat == NULL) {
        fprintf(stderr, "Time-indexed Matrix allocation error!\n");
        exit(-1);
    }

    int i;
    for (i = 0; i < timesteps; i++) {
        mat[i] = (double **) calloc(rows, sizeof(double *));

        if (mat[i] == NULL) {
            fprintf(stderr, "Time-indexed Matrix allocation error!\n");
            exit(-1);
        }

    }

    int j;
    for (i = 0; i < timesteps; i++) {
        for (j = 0; j < rows; j++) {
            mat[i][j] = (double *) calloc(cols, sizeof(double));

            if (mat[i][j] == NULL) {
                fprintf(stderr, "Time-indexed Matrix allocation error!\n");
                exit(-1);
            }
        }

    }

    return mat;

}

/* Free 2D dynamic arrays (matrices) */
void freeMatrix(double **Matrix, int rows, int cols) {
    int i;

    for (i = 0; i < rows; i++) {
        free(Matrix[i]);
    }
    free(Matrix);
}

/* Frees 3D dynamic array
 */
void
freeTimeIndexedMatrix(double ***TimeIndexedMatrix, int timesteps, int rows,
        int cols) {

    int i, j;

    for (i = 0; i < timesteps; i++) {

        for (j = 0; j < rows; j++) {
            free(TimeIndexedMatrix[i][j]);
        }

        free(TimeIndexedMatrix[i]);

    }

    free(TimeIndexedMatrix);

}

/* File I/O tools */

/* Saving double vector to "outfile" */
void saveVector(FILE * outfile, double *vec, int dimension) {
    int i;

    for (i = 0; i < dimension; i++) {
        fprintf(outfile, "%10.10f\t", vec[i]);
    }

    fprintf(outfile, "\n");

}

/* Scanning double matrix from "infile" */
void loadMatrix(FILE * infile, double **mat, int rows, int cols) {

    int i, j;

    for (j = 0; j < rows; j++) {

        for (i = 0; i < cols; i++) {

            fscanf(infile, "%lf", &(mat[j][i]));

        }

    }

}

/* Saving doubel matrix to "outfile" */
void saveMatrix(FILE * outfile, double **mat, int rows, int cols) {

    int i, j;

    for (j = 0; j < rows; j++) {

        for (i = 0; i < cols; i++) {

            fprintf(outfile, "%lf\t", mat[j][i]);

        }

        fprintf(outfile, "\n");

    }

}
