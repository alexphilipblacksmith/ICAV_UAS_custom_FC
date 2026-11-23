#include "accelFit.h"
#include <math.h>

#define numParams 6         // offset x, y, z, scale x, y, z
#define maxIterations 50


static float axisOf(vector3 v, int i){
    return (i == 0) ? v.x : (i == 1) ? v.y : v.z;
}


// Sum of squared errors, and optionally J^T J and J^T r for the step.
// Error per sample: r = |scale * (a - offset)| - 1
static float evaluate(const vector3 *samples, int count, const float p[numParams],
                      float jtj[numParams][numParams], float jtr[numParams]){

    if(jtj){
        for(int i = 0; i < numParams; i++){
            jtr[i] = 0.0f;
            for(int j = 0; j < numParams; j++) jtj[i][j] = 0.0f;
        }
    }

    float cost = 0.0f;
    for(int n = 0; n < count; n++){
        float d[3], u[3];
        for(int k = 0; k < 3; k++){
            d[k] = axisOf(samples[n], k) - p[k];
            u[k] = p[3 + k] * d[k];
        }
        float len = sqrtf(u[0]*u[0] + u[1]*u[1] + u[2]*u[2]);
        if(len < 1e-6f) continue;
        float r = len - 1.0f;
        cost += r * r;

        if(jtj){
            // dr/doffset_k = -scale_k * u_k / len,  dr/dscale_k = d_k * u_k / len
            float J[numParams];
            for(int k = 0; k < 3; k++){
                J[k] = -p[3 + k] * u[k] / len;
                J[3 + k] = d[k] * u[k] / len;
            }
            for(int i = 0; i < numParams; i++){
                jtr[i] += J[i] * r;
                for(int j = 0; j < numParams; j++) jtj[i][j] += J[i] * J[j];
            }
        }
    }
    return cost;
}


// Solves A x = b (Gaussian elimination, partial pivoting). A and b get overwritten
static bool solve(float A[numParams][numParams], float b[numParams], float x[numParams]){

    for(int col = 0; col < numParams; col++){
        int pivot = col;
        for(int row = col + 1; row < numParams; row++){
            if(fabsf(A[row][col]) > fabsf(A[pivot][col])) pivot = row;
        }
        if(fabsf(A[pivot][col]) < 1e-9f) return false;

        if(pivot != col){
            for(int k = 0; k < numParams; k++){
                float t = A[col][k]; A[col][k] = A[pivot][k]; A[pivot][k] = t;
            }
            float t = b[col]; b[col] = b[pivot]; b[pivot] = t;
        }

        for(int row = col + 1; row < numParams; row++){
            float f = A[row][col] / A[col][col];
            for(int k = col; k < numParams; k++) A[row][k] -= f * A[col][k];
            b[row] -= f * b[col];
        }
    }

    for(int row = numParams - 1; row >= 0; row--){
        float s = b[row];
        for(int k = row + 1; k < numParams; k++) s -= A[row][k] * x[k];
        x[row] = s / A[row][row];
    }
    return true;
}


bool accelFitEllipsoid(const vector3 *samples, int count, vector3 *offset, vector3 *scale, float *rmsError){

    if(count < numParams + 1) return false;

    // Starting guess from the extremes: centre = middle, scale = 2 / (max - min)
    float p[numParams];
    for(int k = 0; k < 3; k++){
        float lo = axisOf(samples[0], k), hi = lo;
        for(int n = 1; n < count; n++){
            float v = axisOf(samples[n], k);
            if(v < lo) lo = v;
            if(v > hi) hi = v;
        }
        if(hi - lo < 0.5f) return false;    // axis never turned over
        p[k] = 0.5f * (hi + lo);
        p[3 + k] = 2.0f / (hi - lo);
    }

    float lambda = 1e-3f;
    float jtj[numParams][numParams], jtr[numParams];
    float cost = evaluate(samples, count, p, jtj, jtr);

    for(int it = 0; it < maxIterations; it++){

        // (J^T J + lambda * diag) step = -J^T r
        float A[numParams][numParams], b[numParams], step[numParams];
        for(int i = 0; i < numParams; i++){
            for(int j = 0; j < numParams; j++) A[i][j] = jtj[i][j];
            A[i][i] += lambda * (jtj[i][i] + 1e-6f);
            b[i] = -jtr[i];
        }
        if(!solve(A, b, step)){
            lambda *= 10.0f;
            continue;
        }

        float trial[numParams];
        float stepSize = 0.0f;
        for(int i = 0; i < numParams; i++){
            trial[i] = p[i] + step[i];
            stepSize += step[i] * step[i];
        }

        float trialCost = evaluate(samples, count, trial, 0, 0);
        if(trialCost < cost){
            for(int i = 0; i < numParams; i++) p[i] = trial[i];
            cost = evaluate(samples, count, p, jtj, jtr);
            lambda *= 0.1f;
            if(lambda < 1e-7f) lambda = 1e-7f;
            if(stepSize < 1e-12f) break;
        }else{
            lambda *= 10.0f;
            if(lambda > 1e6f) break;    // can't improve any more
        }
    }

    *offset = (vector3){p[0], p[1], p[2]};
    *scale = (vector3){p[3], p[4], p[5]};
    *rmsError = sqrtf(cost / (float)count);
    return isfinite(*rmsError) != 0;
}
