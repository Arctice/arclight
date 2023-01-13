#include "rng.h"
#include <cmath>
#include <bitset>

struct sample_rng {
    sample_rng(vec2<i32> px, int d) {
        s = {0, (u32)d, (u32)px.x, (u32)px.y};
        u32 lcg_seed = (px.x << 16) ^ px.y + 1;
        d += 1;
        while (d-- > 0) {
            u32 next = ((u64)lcg_seed) * 48271 % 0x7fffffff;
            lcg_seed = next;
            s[d % 4] ^= next;
        }
        for (int n{}; n < 4; ++n) xoshiro128();
    }

    static inline u32 rotl(const u32 x, int k) {
        return (x << k) | (x >> (32 - k));
    }

    std::array<u32, 4> s{};
    u32 xoshiro128() {
        const u32 result = s[0] + s[3];
        const u32 t = s[1] << 9;
        s[2] ^= s[0];
        s[3] ^= s[1];
        s[1] ^= s[2];
        s[0] ^= s[3];
        s[2] ^= t;
        s[3] = rotl(s[3], 11);
        return result;
    }

    float operator()() {
        return std::clamp<float>((xoshiro128() >> 8) * 0x1.0p-24f, 0,
                                 1 - 0.00001f);
    }
};

struct multi_stratified_sequence : public pixel_sampler {
    static constexpr int subdivision = 32;
    using strata_row = u32;

    struct sample : public sample_sequence {
        multi_stratified_sequence* self;
        vec2<int> px;
        int d{0};

        vec2f sample_2d() {
            if (d >= 8)
                return {(Float)self->fallback(), (Float)self->fallback()};
            while ((int)self->dimensions.size() <= d)
                self->dimensions.push_back(sample_set{
                    .rng = sample_rng{self->px, (int)self->dimensions.size()}});
            return self->dimensions[d++].next_sample();
        }
        Float sample_1d() { return sample_2d().x; }
        void skip_to(int d) { this->d = d; }
    };

    struct sample_set {
        sample_rng rng;
        u16 count{};
        std::array<strata_row, subdivision> strata{};
        std::bitset<subdivision * subdivision> rows{};
        std::bitset<subdivision * subdivision> columns{};

        void mark_sample(const vec2f& s) {
            auto x = (int)std::floor(s.x * subdivision);
            auto y = (int)std::floor(s.y * subdivision);
            strata[y] |= 1 << x;
            columns.set((int)std::floor(s.x * subdivision * subdivision));
            rows.set((int)std::floor(s.y * subdivision * subdivision));
            count++;
        }

        vec2f first_sample() {
            auto s = vec2f{(Float)rng(), (Float)rng()};
            mark_sample(s);
            return s;
        }

        bool empty_quadrant(int width, int pos) {
            int size = subdivision / width;
            int row = (pos / width) * size;
            int column = (pos % width) * size;
            u32 mask = (((strata_row)-1) >> (subdivision - size)) << column;
            for (int n{row}; n < row + size; ++n) {
                if (strata[n] & mask) {
                    return false;
                }
            }
            return true;
        }

        vec2f strip_jitter(int size, int row, int column) {
            Float step = 1 / (Float)(size);
            auto offset = vec2f{column * step, row * step};
            auto jitter = vec2f{(Float)rng(), (Float)rng()};
            return offset + jitter * step;
        }

        bool column_empty(int width, int column) {
            int strip_width = (subdivision * subdivision) / (width * width);
            for (int n{column * strip_width}; n < (column + 1) * strip_width;
                 ++n) {
                if (columns.test(n))
                    return false;
            }
            return true;
        }

        bool row_empty(int width, int row) {
            int strip_width = (subdivision * subdivision) / (width * width);
            for (int n{row * strip_width}; n < (row + 1) * strip_width; ++n) {
                if (rows.test(n))
                    return false;
            }
            return true;
        }

        vec2f next_sample() {
            if (count == 0) {
                return first_sample();
            }

            int width{2};
            int maxsize{4};
            while (maxsize <= count) {
                width *= 2;
                maxsize *= 4;
            }

            int quad = (int)(rng() * maxsize);
            while (!empty_quadrant(width, quad)) quad = (quad + 1) % maxsize;
            int quad_x = (quad % width);
            int quad_y = (quad / width);

            int column_i = quad_x * width;
            int column_offset = (int)(rng() * width);
            while (!column_empty(width, column_i + column_offset))
                column_offset = (column_offset + 1) % width;
            column_i += column_offset;

            int row_i = quad_y * width;
            int row_offset = (int)(rng() * width);
            while (!row_empty(width, row_i + row_offset))
                row_offset = (row_offset + 1) % width;
            row_i += row_offset;

            auto sample = strip_jitter(width * width, row_i, column_i);
            mark_sample(sample);
            return sample;
        }
    };

    multi_stratified_sequence(vec2<int> px) : px(px), fallback(px, 0) {}

    std::unique_ptr<sample_sequence> nth_sample(int) {
        auto s = std::make_unique<sample>();
        s->self = this;
        return s;
    }

    vec2<i32> px;
    std::vector<sample_set> dimensions;
    sample_rng fallback;
};

struct stratified_sequence : public pixel_sampler {
    static constexpr int subdivision = 32;
    using strata_row = u32;

    struct sample : public sample_sequence {
        stratified_sequence* self;
        vec2<int> px;
        int d{0};

        vec2f sample_2d() {
            if (d >= 16)
                return {(Float)self->fallback(), (Float)self->fallback()};
            while ((int)self->dimensions.size() <= d)
                self->dimensions.push_back(sample_set{
                    .rng = sample_rng{self->px, (int)self->dimensions.size()}});
            return self->dimensions[d++].next_sample();
        }
        Float sample_1d() { return sample_2d().x; }
        void skip_to(int d) { this->d = d; }
    };

    struct sample_set {
        sample_rng rng;
        std::array<strata_row, subdivision> strata{};
        u16 count{};

        void mark_sample(const vec2f& s) {
            auto x = (int)std::floor(s.x * subdivision);
            auto y = (int)std::floor(s.y * subdivision);
            strata[y] |= 1 << x;
            count++;
        }

        vec2f first_sample() {
            auto s = vec2f{(Float)rng(), (Float)rng()};
            mark_sample(s);
            return s;
        }

        bool empty_quadrant(int width, int pos) {
            int size = subdivision / width;
            int row = (pos / width) * size;
            int column = (pos % width) * size;
            int mask = (((strata_row)-1) >> (subdivision - size)) << column;
            for (int n{row}; n < row + size; ++n) {
                if (strata[n] & mask) {
                    return false;
                }
            }
            return true;
        }

        vec2f quadrant_jitter(int width, int pos) {
            int row = (pos / width);
            int column = (pos % width);
            Float step = 1 / (Float)width;
            auto offset = vec2f{column * step, row * step};
            auto jitter = vec2f{(Float)rng(), (Float)rng()};
            return offset + jitter * step;
        }

        vec2f next_sample() {
            if (count == 0) {
                return first_sample();
            }

            int width{2};
            int maxsize{4};
            while (maxsize <= count) {
                width *= 2;
                maxsize *= 4;
            }

            int quad = (int)(rng() * maxsize);
            while (!empty_quadrant(width, quad)) quad = (quad + 1) % maxsize;
            auto sample = quadrant_jitter(width, quad);
            mark_sample(sample);
            return sample;
        }
    };

    stratified_sequence(vec2<int> px) : px(px), fallback(px, 0) {}

    std::unique_ptr<sample_sequence> nth_sample(int) {
        auto s = std::make_unique<sample>();
        s->self = this;
        return s;
    }

    vec2<i32> px;
    std::vector<sample_set> dimensions;
    sample_rng fallback;
};

struct rng_sequence : public pixel_sampler {
    struct sample : public sample_sequence {
        int d{};
        sample_rng rng;

        sample(vec2<i32> px, i32 d) : rng(px, d) {}
        Float sample_1d() {
            d++;
            return rng();
        }
        vec2f sample_2d() {
            d += 2;
            return {rng(), rng()};
        }
        void skip_to(int next) {
            if (next < d)
                throw std::runtime_error(
                    "rng_sequence::sample - backwards skip unimplemented");
            while (d < next) sample_1d();
        }
    };

    rng_sequence(vec2<i32> px, i32 d) : px(px), d(d) {}
    std::unique_ptr<sample_sequence> nth_sample(int n) {
        return std::unique_ptr<sample_sequence>(new sample{px, d + n});
    }

    vec2<i32> px;
    i32 d;
};

std::unique_ptr<pixel_sampler> multi_stratified_sampler(vec2<int> px, int) {
    return std::unique_ptr<pixel_sampler>(new multi_stratified_sequence{px});
}

std::unique_ptr<pixel_sampler> stratified_sampler(vec2<int> px, int) {
    return std::unique_ptr<pixel_sampler>(new stratified_sequence{px});
}

std::unique_ptr<pixel_sampler> rng_sampler(vec2<int> px, int d) {
    return std::unique_ptr<pixel_sampler>(new rng_sequence{px, d});
}

Float uniform_rng() {
    thread_local static int aslr;
    thread_local static unsigned short int xsubi[3]{
        u16(((u64)&aslr >> 32) & 0xFFFF), u16(((u64)&aslr > 16) & 0xFFFF),
        u16(((u64)&aslr) & 0xFFFF)};
    return erand48(xsubi);
}

struct independent_rng : public pixel_sampler {
    struct sample : public sample_sequence {
        Float sample_1d() { return uniform_rng(); }
        vec2f sample_2d() { return {uniform_rng(), uniform_rng()}; }
        void skip_to(int) {}
    };

    std::unique_ptr<sample_sequence> nth_sample(int) {
        return std::unique_ptr<sample_sequence>(new sample{});
    }
};

std::unique_ptr<pixel_sampler> independent_sampler(vec2<int>, int) {
    return std::unique_ptr<pixel_sampler>(new independent_rng{});
}

vec3f sample_uniform_direction(vec2f u) {
    // pbrt-v4 code
    Float z = 1 - 2 * u.x;
    Float r = std::sqrt(std::max<Float>(0, 1 - z * z));
    Float phi = 2 * π * u.y;
    return {r * std::cos(phi), r * std::sin(phi), z};
}

vec2f sample_uniform_disk(vec2f u) {
    // warning: warps sample distances
    auto r = std::sqrt(u.x);
    auto theta = 2 * π * u.y;
    return {r * std::sin(theta), r * std::cos(theta)};
}

vec2f sample_concentric_disk(vec2f u) {
    // uniform, preserves sample spacing
    auto u_off = u * 2 - vec2f{1, 1};

    if (u_off == vec2f{0, 0})
        return {};

    Float theta, r;
    if (std::abs(u_off.x) > std::abs(u_off.y)) {
        r = u_off.x;
        theta = (π / 4) * (u_off.y / u_off.x);
    }
    else {
        r = u_off.y;
        theta = (π / 2) - (π / 4) * (u_off.x / u_off.y);
    }

    return vec2f{std::cos(theta), std::sin(theta)} * r;
}

vec3f sample_cosine_hemisphere(vec2f u) {
    auto disk = sample_concentric_disk(u);
    auto z = std::sqrt(std::max<Float>(0, 1 - disk.length_sq()));
    return {disk.x, disk.y, z};
}

Float lerp(Float t, Float a, Float b) { return (1 - t) * a + t * b; }
vec3f sample_uniform_cone(vec2f u, Float cos_a, const vec3f& x, const vec3f& y,
                          const vec3f& z) {
    cos_a = lerp(u.x, cos_a, 1.f);
    Float sin_a = std::sqrt((Float)1. - cos_a * cos_a);
    Float phi = u.y * 2 * π;
    return x * std::cos(phi) * sin_a + y * std::sin(phi) * sin_a + z * cos_a;
}

Float pdf_uniform_cone(Float a) { return (2 * π * (1 - a)); }
