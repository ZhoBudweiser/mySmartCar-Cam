#ifndef FETALHEART_BPNN_FIT_H
#define FETALHEART_BPNN_FIT_H

#include "bpnn_config.h"

#define D   IN_N
#define Q   HIDDEN_N
#define L   OUT_N

typedef struct bpnn {
    double (*v)[Q];
    double (*w)[L];
    double *r;
    double o;
    double *b;
} *T;
#define bpnn_t T

T bpnn_fit_new(void);
double bpnn_fit(T bpnn, double *in);

#endif //FETALHEART_BPNN_FIT_H
