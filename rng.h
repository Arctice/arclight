#pragma once
#include "base.h"

Float sample_1d();
vec2f sample_2d();
vec3f sample_3d();

vec3f sample_uniform_direction(vec2f);
vec2f sample_uniform_disk(vec2f);
vec2f sample_concentric_disk(vec2f);
vec3f sample_cosine_hemisphere(vec2f);
vec3f sample_uniform_cone(Float cos_a, const vec3f& x, const vec3f& y,
                          const vec3f& z);
Float pdf_uniform_cone(Float cos_a);
