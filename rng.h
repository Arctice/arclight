#pragma once
#include "base.h"

struct sample_sequence {
    virtual Float sample_1d() = 0;
    virtual vec2f sample_2d() = 0;
    virtual void skip_to(int dimension) = 0;
    virtual ~sample_sequence() {}
};

struct pixel_sampler {
    virtual std::unique_ptr<sample_sequence> nth_sample(int n) = 0;
    virtual ~pixel_sampler() {}
};

std::unique_ptr<pixel_sampler> independent_sampler(vec2<int>, int size);
std::unique_ptr<pixel_sampler> stratified_sampler(vec2<int>, int);
std::unique_ptr<pixel_sampler> multi_stratified_sampler(vec2<int>, int);
std::unique_ptr<pixel_sampler> rng_sampler(vec2<int>, int);

vec3f sample_uniform_direction(vec2f);
vec2f sample_uniform_disk(vec2f);
vec2f sample_concentric_disk(vec2f);
vec3f sample_cosine_hemisphere(vec2f);
vec3f sample_uniform_cone(vec2f, Float cos_a, const vec3f& x, const vec3f& y,
                          const vec3f& z);
Float pdf_uniform_cone(Float cos_a);
