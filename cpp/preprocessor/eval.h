#ifndef EVAL_H
#define EVAL_H

// Here, N is not the number of evaluations, but 2^(N+1) number of evaluations
//
// Note, each level doubles the previous level plus one, so
// - EVAL_N: 1 + EVAL_#
// - EVAL_1: 1 evals = 2^1 - 1
// - EVAL_2: 1 + 2*EVAL_1 = 3 evals = 2^2 - 1
// - EVAL_3: 1 + 2*EVAL_2 = 7 evals = 2^3 - 1
// - EVAL_4: 1 + 2*EVAL_3 = 15 evals = 2^4 - 1
// - EVAL_5: 1 + 2*EVAL_4 = 31 evals = 2^5 - 1
// - EVAL_6: 1 + 2*EVAL_5 = 63 evals = 2^6 - 1
// - EVAL_7: 1 + 2*EVAL_6 = 127 evals = 2^7 - 1
// - EVAL_8: 1 + 2*EVAL_7 = 255 evals = 2^8 - 1
//
// Then we define the default EVAL to 1+EVAL_8 = 256 evals = 2^8
#define EVAL_N(N, ...) EVAL_ ## N (__VA_ARGS__)
#define EVAL_1(...) __VA_ARGS__
#define EVAL_2(...) EVAL_1(EVAL_1(__VA_ARGS__))
#define EVAL_3(...) EVAL_2(EVAL_2(__VA_ARGS__))
#define EVAL_4(...) EVAL_3(EVAL_3(__VA_ARGS__))
#define EVAL_5(...) EVAL_4(EVAL_4(__VA_ARGS__))
#define EVAL_6(...) EVAL_5(EVAL_5(__VA_ARGS__))
#define EVAL_7(...) EVAL_6(EVAL_6(__VA_ARGS__))
#define EVAL_8(...) EVAL_7(EVAL_7(__VA_ARGS__))

#define EVAL(...) EVAL_8(__VA_ARGS__)

#endif // EVAL_H
