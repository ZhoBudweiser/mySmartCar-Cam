//
// Created by wcjzj on 2017/5/22.
//

#include "bpnn.h"
#include <time.h>
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define RAND                    (rand() / (double)(RAND_MAX))

#define D   IN_N
#define Q   HIDDEN_N
#define L   OUT_N
#define U1  LEARN_RATE1
#define U2  LEARN_RATE2

// static double v[D][Q];
// static double w[Q][L];
// static double r[Q];
// static double o[L];

double v[D][Q] = {
{237.053980, 28.554327, -176.029779},
{574.840216, -101.001939, 302.360131},
{-194.339059, 13.353839, 181.236324},
{-507.038880, -158.803880, -184.380978},
{-77.217305, 2.022717, -5.188716},
{25.273584, -152.242608, 64.919540},
{680.738787, -200.757720, 98.781564},
{-1627.753625, -1.310819, -100.090781},
{181.051359, 689.390364, 13.595091},
{60.138077, -127.736807, -79.530280},
{-24.898867, 300.782998, 186.534714},



	
//	{-8.385606, -9.462421, -28.882592},
//	{-2.991122, -2.945963, 47.084541},
//	{-7.889531, -9.446880, -13.331357},
//	{-5.706423, -5.952253, -56.313531},
//	{-7.968573, -9.114580, -15.089383},
//	{-4.421672, -3.824722, 0.392108},
//	{-1.577557, -1.938006, 164.816084},
//	{-4.015966, -5.211047, -168.063331},
//	{-7.760762, -6.333200, -36.197791},
//	{-3.072559, -4.102246, -45.046439},
//	{-2.701435, -1.914770, 71.449903},
};
double w[Q][L] = {
{5.033192},
{7.695463},
{4.459584},


//	{0.900787},
//	{0.929040},
//	{5.592311},
};
double r[Q] = {
50.838241, 1.878661, 56.543582
//	0.957573, 1.210732, 12.880453
};
// double b[Q] = {
// 	0, 0, 0
// };

static double o[L] = {
13.65727
};

static double g[L];
static double e[Q];

static double x[D];
static double b[Q];
static double y[L];
static double yc[L];

static double Ek;

static double maxx[11] = {
    188, 120, 188, 120, 188, 120, 1000, 500, 255, 255, 255
};

static bool train_once(test_set_get_t f_get);

static bool save_parameter(void);

void bpnn_init(void) {
    srand((unsigned) time(NULL));
    for (size_t i = 0; i < D; i++)
        for (size_t h = 0; h < Q; h++)
            v[i][h] = RAND;
    for (size_t h = 0; h < Q; h++)
        for (size_t j = 0; j < L; j++)
            w[h][j] = RAND;
    for (size_t h = 0; h < Q; h++)
        r[h] = RAND;
    for (size_t j = 0; j < L; j++)
        o[j] = RAND;
}

void bpnn_train(test_set_get_t f_get, test_set_init_t f_init) {
    assert(f_get && f_init);

    printf("[BPNN] TRAIN STARTING...\n");

    f_init();

    int train_c = 0;
    double E = 0;
    while (train_c < LOOP_N) {
        size_t count = 0;
        while (train_once(f_get)) {
            count++;
            // printf("[BPNN] LOOP %d E %lf\n", train_c, E);
            E += Ek;
        }
        E /= count;
        train_c++;
        printf("[BPNN] LOOP %d E %lf\n", train_c, E);
        f_init();
        if (E < E_MIN)
            break;
    }

    printf("[BPNN] END TRAIN PROCESS LOOP %d E %lf .\n", train_c, E);

    if (save_parameter())
        printf("[BPNN] SAVE BPNN PARAM SUCCESS in %s\n", SAVE_PARAM_PATH);
    else
        printf("[BPNN] SAVE BPNN PARAM FAILED.\n");
}

double bpnn_sim(test_set_get_t f_get) {
    assert(f_get);

    double maxEk = 0, minEk = 1, E = 0;
    long count = 0;
    long long cc = 0;
    while (f_get(x, y)) {
        count++;

        /* Compute b[h] */
        for (size_t h = 0; h < Q; h++) {
            double alpha_h = 0;
            for (size_t i = 0; i < D; i++)
                alpha_h += v[i][h] * x[i];
            b[h] = ACTIVATION_FUNC(alpha_h - r[h]);
        }

        double Ek = 0;

        /* Compute yc[j] */
        for (size_t j = 0; j < L; j++) {
            double beta_j = 0;
            for (size_t h = 0; h < Q; h++)
                beta_j += w[h][j] * b[h];
            yc[j] = ACTIVATION_FUNC(beta_j - o[j]);
            Ek += (yc[j] - y[j]) * (yc[j] - y[j]);
            if ((y[j] > 0.5 && yc[j] > 0.5) || y[j] <= 0.5 && yc[j] <= 0.5)
                cc++; 
        }

        Ek = 0.5 * Ek;
        // printf("[BPNN] DATA INDEX %d EK %lf ==> ", count, Ek);
        // printf("INPUT: ");
        // for (size_t i = 0; i < D; i++)
        //     printf("%lf ", x[i]);
        // printf("OUTPUT:");
        // printf("\n");

        maxEk = (Ek > maxEk) ? Ek : maxEk;
        minEk = (Ek < minEk) ? Ek : minEk;
        E += Ek;
    }

    E = E / count;

    printf("maxEk %lf, minEk %lf, E %lf\n", maxEk, minEk, E);
    printf("correct: %lf\n", 1.0*cc/count);

    return 1.0*cc/count;
}

static bool train_once(test_set_get_t f_get) {
    if (!f_get(x, y))
        return false;

    /* Compute b[h] */
    for (size_t h = 0; h < Q; h++) {
        double alpha_h = 0;
        for (size_t i = 0; i < D; i++)
            alpha_h += v[i][h] * (x[i]/maxx[i]);
        b[h] = ACTIVATION_FUNC(alpha_h - r[h]);
    }

    /* Compute yc[j] */
    for (size_t j = 0; j < L; j++) {
        double beta_j = 0;
        for (size_t h = 0; h < Q; h++)
            beta_j += w[h][j] * b[h];
        yc[j] = ACTIVATION_FUNC(beta_j - o[j]);
    }

    Ek = 0;

    /* Compute g[j] and E */
    for (size_t j = 0; j < L; j++) {
        g[j] = yc[j] * (1 - yc[j]) * (y[j] - yc[j]);
        Ek += (yc[j] - y[j]) * (yc[j] - y[j]);
    }

    Ek = 0.5 * Ek;

    /* Compute e[h] */
    for (size_t h = 0; h < Q; h++) {
        double temp = 0;
        for (size_t j = 0; j < L; j++)
            temp += w[h][j] * g[j];
        e[h] = b[h] * (1 - b[h]) * temp;
    }

    /* Update v[i][h], w[h][j], r[h], o[j] */
    for (size_t i = 0; i < D; i++)
        for (size_t h = 0; h < Q; h++)
            v[i][h] += U2 * e[h] * x[i];
    for (size_t h = 0; h < Q; h++)
        for (size_t j = 0; j < L; j++)
            w[h][j] += U1 * g[j] * b[h];
    for (size_t h = 0; h < Q; h++)
        r[h] += ((-U2) * e[h]);
    for (size_t j = 0; j < L; j++)
        o[j] += ((-U1) * g[j]);

    return true;
}

static bool save_parameter(void) {
    FILE *out = NULL;
    out = fopen(SAVE_PARAM_PATH, "w+");
    if (out == NULL) {
        fprintf(stderr, "[BPNN] OPEN FILE %s FAILED.\n", SAVE_PARAM_PATH);
        return false;
    }

    fprintf(out, "#BPNN PARAM GENERATED AUTOMATICALLY IN %d SECONDS SINCE JABUARY 1, 1970\n", (int)time(NULL));
    fprintf(out, "#!!DO NOT MODIFY!!\n");
    fprintf(out, "D=%d\n", D);
    fprintf(out, "Q=%d\n", Q);
    fprintf(out, "L=%d\n", L);
    for (size_t i = 0; i < D; i++)
        for (size_t h = 0; h < Q; h++)
            fprintf(out, "%lf\n", v[i][h]);
    for (size_t h = 0; h < Q; h++)
        for (size_t j = 0; j < L; j++)
            fprintf(out, "%lf\n", w[h][j]);
    for (size_t h = 0; h < Q; h++)
        fprintf(out, "%lf\n", r[h]);
    for (size_t j = 0; j < L; j++)
        fprintf(out, "%lf\n", o[j]);

    fclose(out);

    return true;
}
