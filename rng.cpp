#include "rng.h"
#include <cmath>

// Float uniform_rng() {
//     thread_local static unsigned short int xsubi[3];
//     return erand48(xsubi);
// }

// Float sample_1d() { return uniform_rng(); }
// vec2f sample_2d() { return {uniform_rng(), uniform_rng()}; }
// vec3f sample_3d() { return {uniform_rng(), uniform_rng(), uniform_rng()}; }

template<int base>
double inverse_radical(unsigned long long n){
    constexpr double inverse = 1. / base;
    double base_n = 1;
    double reverse = 0;
    while(0 < n){
        auto next = n / base;
        auto digit = n - next*base;
        reverse = reverse*base + digit;
        base_n *= inverse;
        n = next;
    }
    return std::max<double>(10e-10, reverse * base_n);
}

Float sample_1d() {
    thread_local static unsigned long long n{3};
    return inverse_radical<2>(n++);
}
vec2f sample_2d() {
    thread_local static unsigned long long n{3};
    n++;
    return {(Float)inverse_radical<2>(n), (Float)inverse_radical<3>(n)};
}
vec3f sample_3d() {
    thread_local static unsigned long long n{3};
    n++;
    return {(Float)inverse_radical<2>(n), (Float)inverse_radical<3>(n),
            (Float)inverse_radical<5>(n)};
}

vec3f sample_uniform_direction() {
    // pbrt-v4 code
    auto u = sample_2d();
    Float z = 1 - 2 * u.x;
    Float r = std::sqrt(1 - z * z);
    Float phi = 2 * pi * u.y;
    return {r * std::cos(phi), r * std::sin(phi), z};
}
