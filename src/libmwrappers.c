#include <math.h>

float __ieee754_logf(float);
float __ieee754_log10f(float);

float logf(float x) {
    return __ieee754_logf(x);
}

float log10f(float x) {
    return __ieee754_log10f(x);
}

float __wrap_log10f(float x) {
    return __ieee754_log10f(x);
}
