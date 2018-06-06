/* vim:set ts=4 sw=4 sts=4 et: */

#ifndef FILE_UTILS_H
#define FILE_UTILS_H

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>

/* File size limits */
#define MAX_SECTION 32
#define MAX_LINE 256
#define MAX_Name 64

/* Functions for handling strings */

char *RStrip(char *s);
char *LSkip(const char *s);
char *FindCharOrComment(const char *s, char c);
char *StrnCpy0(char *Dest, const char *Src, int size);

/* Function for parsing files and setting up parameters*/

/* Recovers fileName from 'FILE *' 
 */
char *recover_fileName(FILE * f);

/* File parsers */
int IniParseFile(FILE * file,
        int (*handler) (void *, const char *, const char *,
                const char *), void *user);

int IniParse(const char *fileName,
        int (*handler) (void *, const char *, const char *,
                const char *), void *user);

/* Checks existence of input file
 * final_fileName will be set to the Name of a "preferred input file", if the list argv[] contains a valid input fileName after 
 * an option flag formatted as "-*"
 * otherwise, it will be set to "default_fileName"
 * 
 * returns the opened file or NULL if fails.
 */
FILE *CheckInputFile(char *final_fileName, int argc, char *argv[],
        char *actual_directory, char *default_fileName, char *optionflag);

/* Checks existence of output file
 * final_fileName will be set to the Name of a "preferred input file", if the list argv[] contains a valid input fileName after
 * an option flag formatted as "-*"
 */
void CheckOutputFile(char *final_fileName, int argc, char *argv[],
        FILE ** DefaultOutputFile, char *actual_directory,
        char *default_fileName, char *optionflag);

/* ???
 */
void CountNumberOfInputs(int *NumberOfFlParams, int *NumberOfUParams,
        int argc, char *argv[]);

#endif
