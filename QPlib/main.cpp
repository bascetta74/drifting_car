#include <stdio.h>
#include <stdlib.h>

#include "CPLEXsolver_cwrapper.h"

int main(int argc, char **argv)
{
    int res;

    /* Init library */
    res = init_CPLEXsolver_cwrapper(3, 0, 0);
    printf("init_CPLEXsolver_cwrapper result: %d\n\n", res);

    /* Init problem */
    res = initProblem();
    printf("initProblem result: %d\n\n", res);

    /* Set problem */
    double H[3][3] = {
        { 3.0, 0.0, -1.0},
        { 0.0, 2.0,  0.0},
        {-1.0, 0.0,  1.0}
    };

    double** H_ptr = (double**) malloc(sizeof(double*)*3);
    for (int i=0; i<3; i++)
        H_ptr[i] = (double*) malloc(sizeof(double)*3);

    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++) {
            H_ptr[i][j] = H[i][j];
        }
    }

    double f[1][3] = {-2.0, 3.0, 1.0};

    double** f_ptr = (double**) malloc(sizeof(double*));
    f_ptr[0] = (double*) malloc(sizeof(double)*3);

    for (int j=0; j<3; j++) {
        f_ptr[0][j] = f[0][j];
    }

    res = setProblem(H_ptr, f_ptr);
    printf("setProblem result: %d\n\n", res);

    /* Solve problem */
    double* result_ptr = (double*) malloc(sizeof(double)*3);

    res = solveProblem(result_ptr);
    printf("solveProblem result: %d\n", res);
    printf("result: %.2f %.2f %.2f\n\n", result_ptr[0], result_ptr[1], result_ptr[2]);

    /* Deinit library */
    res = deinit_CPLEXsolver_cwrapper();
    printf("deinit_CPLEXsolver_cwrapper result: %d\n\n", res);

    return 0;
}