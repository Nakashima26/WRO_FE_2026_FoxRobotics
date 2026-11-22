/* Normal-mode MATLAB bindings for the same C99 plant the SIL uses. */
#include "mex.h"
#include <math.h>
#include <string.h>

void fox_twin_vehicle(double *state, const double *parameters);

static int real_vector(const mxArray *value, size_t length)
{
    return mxIsDouble(value) && !mxIsComplex(value) && !mxIsSparse(value)
           && mxGetNumberOfElements(value) == length;
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    if (nlhs != 1 || nrhs != 3 || !real_vector(prhs[0], 10)
        || !real_vector(prhs[1], 4)) {
        mexErrMsgIdAndTxt("FoxTwin:Inputs", "Expected one output and inputs of 10 states, 4 controls, and plant parameters.");
    }

    size_t length = mxGetNumberOfElements(prhs[2]);
    if (length < 35 || !real_vector(prhs[2], length)) {
        mexErrMsgIdAndTxt("FoxTwin:Parameters", "Invalid plant parameter vector.");
    }
    const double *parameters = mxGetPr(prhs[2]);
    double right = parameters[33], left = parameters[34];
    if (!isfinite(right) || !isfinite(left) || right < 0 || left < 0
        || floor(right) != right || floor(left) != left
        || 35.0 + 2.0 * (right + left) != (double)length) {
        mexErrMsgIdAndTxt("FoxTwin:SteeringTables", "Invalid steering table dimensions.");
    }
    const double *command = mxGetPr(prhs[1]);
    if (!(command[0] > 0 && command[0] <= 0.001)
        || !isfinite(command[1]) || !isfinite(command[2])
        || (command[3] != -1 && command[3] != 0 && command[3] != 1)) {
        mexErrMsgIdAndTxt("FoxTwin:Command", "Invalid physics substep or actuator command.");
    }

    double working[14];
    memcpy(working, mxGetPr(prhs[0]), 10 * sizeof(double));
    memcpy(working + 10, command, 4 * sizeof(double));
    fox_twin_vehicle(working, parameters);
    plhs[0] = mxCreateDoubleMatrix(10, 1, mxREAL);
    memcpy(mxGetPr(plhs[0]), working, 10 * sizeof(double));
}
