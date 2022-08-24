#ifndef FETALHEART_BPNN_CONFIG_H
#define FETALHEART_BPNN_CONFIG_H

#define IN_N                            11          /* INPUT NODE */
#define OUT_N                           1           /* OUTPUT_NODE */
#define HIDDEN_N                        3           /* HIDDEN_NODE */


#define ACTIVATION_FUNC(x)              (1/(1+exp(-(x))))

#endif //FETALHEART_BPNN_CONFIG_H
