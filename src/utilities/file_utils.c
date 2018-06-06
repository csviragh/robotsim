/* vim:set ts=4 sw=4 sts=4 et: */

#include "file_utils.h"
#include <string.h>
#include <ctype.h>

/* Functions for handling strings and files */

char *RStrip(char *s) {
    char *p = s + strlen(s);
    while (p > s && isspace(*--p)) {
        *p = '\0';
    }
    return s;
}

char *LSkip(const char *s) {
    while (*s && isspace(*s)) {
        s++;
    }
    return (char *) s;
}

char *FindCharOrComment(const char *s, char c) {

    int WasWhitespace = 0;
    while (*s && *s != c && !(WasWhitespace && *s == ';')) {
        WasWhitespace = isspace(*s);
        s++;
    }

    return (char *) s;

}

char *StrnCpy0(char *Dest, const char *Src, int size) {

    strncpy(Dest, Src, size);
    Dest[size - 1] = '\0';
    return Dest;

}

/* Recovers fileName from FILE * struct */
char *recover_fileName(FILE * f) {

    int fd;
    char fd_path[255];
    char *fileName = malloc(255);

    fd = fileno(f);
    sprintf(fd_path, "/proc/self/fd/%d", fd);
    readlink(fd_path, fileName, 255);

    return fileName;

}

/* File parsers */
int IniParseFile(FILE * file,
        int (*handler) (void *, const char *, const char *,
                const char *), void *user) {

    char line[MAX_LINE];
    char section[MAX_SECTION] = "";
    char prev_Name[MAX_Name] = "";

    char *start, *end, *Name, *value;

    int lineno = 0;
    int error = 0;

    while (fgets(line, sizeof(line), file) != NULL) {

        lineno++;
        start = LSkip(RStrip(line));

        if (*prev_Name && *start && start > line) {
            if (!handler(user, section, prev_Name, start) && !error) {
                error = lineno;
            }

        } else {
            if (*start == ';' || *start == '#') {

            } else if (*start == '[') {
                end = FindCharOrComment(start + 1, ']');
                if (*end == ']') {
                    *end = '\0';
                    StrnCpy0(section, start + 1, sizeof(section));
                    *prev_Name = '\0';
                } else if (!error) {
                    error = lineno;
                }

            } else if (*start && *start != ';') {

                end = FindCharOrComment(start, '=');
                if (*end != '=') {
                    end = FindCharOrComment(start, ':');
                }
                if (*end == '=' || *end == ':') {
                    *end = '\0';
                    Name = RStrip(start);
                    value = LSkip(end + 1);
                    end = FindCharOrComment(value, '\0');
                    if (*end == ':') {
                        *end = '\0';
                    }

                    RStrip(value);
                    StrnCpy0(prev_Name, Name, sizeof(prev_Name));
                    if (!handler(user, section, Name, value) && !error) {
                        error = lineno;

                    }

                } else if (!error) {
                    error = lineno;
                }

            }
        }

    }

    return error;

}

int IniParse(const char *fileName,
        int (*handler) (void *, const char *, const char *, const char *),
        void *user) {

    FILE *file;
    int error;

    file = fopen(fileName, "r");
    if (!file) {
        return -1;
    }
    error = IniParseFile(file, handler, user);
    fclose(file);
    return error;

}

/* Checks existence of input file (and returns it) */
FILE *CheckInputFile(char *final_fileName, int argc, char *argv[],
        char *actual_directory, char *default_fileName, char *optionflag) {

    int i;

    if (final_fileName != default_fileName) {
        strcpy(final_fileName, default_fileName);
    }

    for (i = 0; i < argc - 1; i++) {
        if (strcmp(argv[i], optionflag) == 0) {
            strcpy(final_fileName, argv[i + 1]);
            strcat(final_fileName, "\0");
            break;
        }
    }

    return fopen(final_fileName, "r");

}

/* Checks existence of Output file (and returns it) */
void CheckOutputFile(char *final_fileName, int argc, char *argv[],
        FILE ** DefaultOutputFile, char *actual_directory,
        char *default_fileName, char *optionflag) {

    int i;

    if (final_fileName != default_fileName) {
        strcpy(final_fileName, default_fileName);
    }

    for (i = 0; i < argc - 1; i++) {
        if (strcmp(argv[i], optionflag) == 0) {
            if (access(argv[i + 1], W_OK) != -1) {
                if (*DefaultOutputFile != NULL) {
                    fclose(*DefaultOutputFile);
                }
                *DefaultOutputFile = fopen(argv[i + 1], "w");
                printf("Selected output file found! (%s)\n", argv[i + 1]);
                strcpy(final_fileName, argv[i + 1]);
                strcat(final_fileName, "\0");
            }
        }
    }

}

/* Count number of parameter sets (flockingparams and unitparams) */
void CountNumberOfInputs(int *NumberOfFlParams, int *NumberOfUParams, int argc,
        char *argv[]) {

    int i;
    for (i = 0; i < argc - 1; i++) {
        if (0 == strcmp(argv[i], "-u")) {       /* YodaCode forever!!!44!! */
            (*NumberOfUParams)++;
        } else if (0 == strcmp(argv[i], "-f")) {
            (*NumberOfFlParams)++;
        }
    }

}
