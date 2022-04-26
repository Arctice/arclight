#pragma once
#include "geometry.h"
#include <fmt/core.h>
#include <fmt/ostream.h>

using i16 = std::int16_t;
using i32 = std::int32_t;
using i64 = std::int64_t;
using u8 = std::uint8_t;
using u16 = std::uint16_t;
using u32 = std::uint32_t;
using u64 = std::uint64_t;

using Float = float;
using vec2f = vec2<Float>;
using vec3f = vec3<Float>;
constexpr Float ɛ = std::numeric_limits<Float>::epsilon() * 0.5;
constexpr Float π = 3.14159265358979323846;

struct bounding_box {
    vec3f min{std::numeric_limits<Float>::max()};
    vec3f max{std::numeric_limits<Float>::lowest()};

    bool intersect(const bounding_box& other) {
        return std::max(min.x, other.min.x) <= std::min(max.x, other.max.x) &&
               std::max(min.y, other.min.y) <= std::min(max.y, other.max.y) &&
               std::max(min.z, other.min.z) <= std::min(max.z, other.max.z);
    }

    bounding_box operator|(const bounding_box& other) const {
        return {
            vec3f{std::min(min.x, other.min.x), std::min(min.y, other.min.y),
                  std::min(min.z, other.min.z)},
            vec3f{std::max(max.x, other.max.x), std::max(max.y, other.max.y),
                  std::max(max.z, other.max.z)}};
    }

    bounding_box operator|(const vec3f& rhs) const {
        return {vec3f{std::min(min.x, rhs.x), std::min(min.y, rhs.y),
                      std::min(min.z, rhs.z)},
                vec3f{std::max(max.x, rhs.x), std::max(max.y, rhs.y),
                      std::max(max.z, rhs.z)}};
    }

    vec3f centroid() const { return (max + min) / 2; };
};

struct triangle {
    vec3f A, B, C;
    vec3f nA{}, nB{}, nC{};
    vec2f tA{0, 0}, tB{1, 0}, tC{0, 1};
};

struct indexed_mesh {
    std::vector<vec3f> vertices;
    std::vector<vec3f> normals;
    std::vector<vec2f> tex_coords;
    std::vector<vec3<int>> triangles;

    triangle reify(int n) const {
        auto [a, b, c] = triangles[n];

        triangle T = {
            vertices[a], vertices[b], vertices[c],
        };

        if (not normals.empty()) {
            T.nA = normals[a];
            T.nB = normals[b];
            T.nC = normals[c];
        }

        if (not tex_coords.empty()) {
            T.tA = tex_coords[a];
            T.tB = tex_coords[b];
            T.tC = tex_coords[c];
        }

        return T;
    }
};

indexed_mesh load_ply(std::string path, bool describe = false);
