//MIT License
//Copyright (c) 2018 Eotvos Lorand University, Budapest

/* vim:set ts=4 sw=4 sts=4 et: */

/* Tools for debugging the code of an algorithm
 */

#include "debug_utils.h"

/* Printing out the components of a vector to stdout */
void PrintVector(double *Vector, const int Dim) {

    printf("( ");

    int i;
    for (i = 0; i < Dim; i++) {

        printf("%lf ", Vector[i]);

    }

    printf(")\n\n");

}

/* Segfault detection in debug mode */
/**
 * Signal handler for SIGSEGV, SIGABRT and SIGBUS to print a stack trace when
 * the app crashes.
 */
static void HandleFaults(int sig) {
#ifdef DEBUG
    void *array[MAX_STACKTRACE_DEPTH];
    size_t size;

    /* Print the stack trace */
    size = backtrace(array, MAX_STACKTRACE_DEPTH);
    printf("\nUnexpected termination (signal %d) at:\n", sig);
    backtrace_symbols_fd(array, size, STDOUT_FILENO);

    /* Restore the original signal handler and send the signal again to ensure
     * that we also get the core dump */
    signal(sig, SIG_DFL);
    raise(sig);
#endif
}

void InstallSegfaultHandler() {
#ifdef DEBUG
    signal(SIGABRT, HandleFaults);
    signal(SIGBUS, HandleFaults);
    signal(SIGSEGV, HandleFaults);
    signal(SIGFPE, HandleFaults);
#endif
    if (1 == 0)
        HandleFaults(0);
}
