#include "base.h"
#include "scene.h"
#include "rng.h"

#include <array>
#include <queue>

#include <SFML/Graphics.hpp>
#include <cblas.h>

matrix_sq4 matrix_sq4::matmul(const matrix_sq4& b) const {
    matrix_sq4 c{};
    auto A = reinterpret_cast<const Float(&)[16]>(m);
    auto B = reinterpret_cast<const Float(&)[16]>(b.m);
    auto C = reinterpret_cast<Float(&)[16]>(c.m);
    cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 4, 4, 4, 1., A, 4, B,
                4, 1., C, 4);

    return c;
}

bounding_box shape_bounds(const triangle& V) {
    vec3f min{std::min(std::min(V.A.x, V.B.x), V.C.x),
              std::min(std::min(V.A.y, V.B.y), V.C.y),
              std::min(std::min(V.A.z, V.B.z), V.C.z)};
    vec3f max{std::max(std::max(V.A.x, V.B.x), V.C.x),
              std::max(std::max(V.A.y, V.B.y), V.C.y),
              std::max(std::max(V.A.z, V.B.z), V.C.z)};
    return {min, max};
}

bool bbox_ray_intersection(const bounding_box& bb, const ray& ray,
                           Float d = std::numeric_limits<Float>::max()) {
    constexpr Float gamma3 = (3 * epsilon) / (1 - 3 * epsilon);
    auto inv = vec3f{1.f / ray.direction.x, 1.f / ray.direction.y,
                     1.f / ray.direction.z};
    Float t0 = 0., t1 = d;
    auto z = ray.origin * inv;
    for (auto d{0}; d < 3; ++d) {
        auto near = bb.min[d] * inv[d] - z[d];
        auto far = bb.max[d] * inv[d] - z[d];

        if (near > far)
            std::swap(near, far);
        far *= 1 + 2 * gamma3;
        t0 = std::max(t0, near);
        t1 = std::min(t1, far);
    }
    return t0 <= t1;
}

std::unique_ptr<node_instance> instance(const transform& T, const node& n) {
    auto i = std::make_unique<node_instance>(T, n);
    return i;
}

bounding_box node_bounds(const triangle& n) { return shape_bounds(n); }
bounding_box node_bounds(const node_instance& i) {
    auto [min, max] = node_bounds(i.n);
    return {i.T.point(min), i.T.point(max)};
}
bounding_box node_bounds(const bvh_mesh& n) { return n.bvh.bounds; }
bounding_box node_bounds(const node_bvh& n) { return n.bvh.bounds; }

bounding_box node_bounds(const node& n) {
    return std::visit([](const auto& n) { return node_bounds(*n); }, n);
}

bounding_box node_bounds(const node_view& n) {
    return std::visit([](const auto& n) { return node_bounds(*n); }, n);
}

std::optional<intersection> intersect(const triangle&, const ray&);
std::optional<intersection> intersect(const bvh_mesh&, const ray&);
std::optional<intersection> intersect(const node_instance& i, const ray& r) {
    auto Ti = i.T.inverse();
    auto ri = Ti(r);
    auto hit = intersect(i.n, ri);
    if (not hit)
        return {};
    return intersection{hit->distance, i.T.normal(hit->normal).normalized()};
}

std::optional<intersection> intersect(const node_bvh&, const ray&);

std::optional<intersection> intersect(const node& n, const ray& r) {
    return std::visit([r](const auto& n) { return intersect(*n, r); }, n);
}

std::optional<intersection> intersect(const node_view& n, const ray& r) {
    return std::visit([r](const auto& n) { return intersect(*n, r); }, n);
}

struct bvh_traversal {
    bvh_traversal(const BVH* bvh, const ray& r) : ray(r) { queue.push(bvh); }
    const std::vector<int>* next(Float bound) {
        if (queue.empty())
            return nullptr;
        const BVH* node = queue.front();
        queue.pop();
        if (!bbox_ray_intersection(node->bounds, ray))
            return next(bound);
        if (node->a)
            queue.push(node->a.get());
        if (node->b)
            queue.push(node->b.get());
        return &node->overlap;
    }
    operator bool() const { return not queue.empty(); }

private:
    std::queue<const BVH*> queue;
    const ray& ray;
};

std::optional<intersection> intersect(const triangle& V, const ray& ray) {
    auto BA = V.B - V.A;
    auto CA = V.C - V.A;
    auto n = BA.cross(CA);
    auto q = ray.direction.cross(CA);
    auto a = BA.dot(q);
    if (n.dot(ray.direction) >= 0 or std::abs(a) <= epsilon)
        return {};

    auto s = (ray.origin - V.A) / a;
    auto r = s.cross(BA);

    auto b = vec3f{s.dot(q), r.dot(ray.direction), 0};
    b.z = 1.0f - b.x - b.y;
    if (b.x < 0.f or b.y < 0.f or b.z < 0.f)
        return {};

    auto t = CA.dot(r);
    if (t < 0.f)
        return {};

    return intersection{t, n.normalized()};
}

std::optional<intersection> intersect(const bvh_mesh& m, const ray& ray) {
    auto search = bvh_traversal(&m.bvh, ray);
    std::optional<intersection> result{};
    auto nearest = std::numeric_limits<Float>::max();

    while (search) {
        auto objects = search.next(nearest);
        if (objects)
            for (const auto& index : *objects) {
                auto hit = intersect(m.data.reify(index), ray);
                if (not hit or nearest < hit->distance)
                    continue;

                nearest = hit->distance;
                result = hit;
            }
    }
    return result;
}

std::optional<intersection> intersect(const node_bvh& m, const ray& ray) {
    auto search = bvh_traversal(&m.bvh, ray);
    std::optional<intersection> result{};
    auto nearest = std::numeric_limits<Float>::max();

    while (search) {
        auto objects = search.next(nearest);
        if (objects)
            for (const auto& index : *objects) {
                auto hit = intersect(m.nodes[index], ray);
                if (not hit or nearest < hit->distance)
                    continue;

                nearest = hit->distance;
                result = hit;
            }
    }
    return result;
}

std::unique_ptr<bvh_mesh> load_model(std::string path) {
    auto ply_mesh = load_ply(path);
    auto mesh = bvh_mesh{ply_mesh, {}};
    mesh.bvh = build_bvh(mesh.data.triangles.size(), [&mesh](size_t n) {
        return shape_bounds(mesh.data.reify(n));
    });
    return std::make_unique<bvh_mesh>(std::move(mesh));
}

camera orthographic_camera(vec2f resolution, Float scale, vec3f position,
                           vec3f towards, vec3f up) {
    camera c;
    c.camera_to_world = transform::look_at(position, towards, up);

    c.raster_to_screen =
        transform::scale({1 / resolution.x, 1 / resolution.y, 1});
    c.raster_to_screen =
        transform::scale({2 * scale, 2 * scale, 1}).compose(c.raster_to_screen);
    c.raster_to_screen =
        transform::translate({-scale, -scale, 0}).compose(c.raster_to_screen);

    c.screen_to_camera = transform::identity();
    c.raster_to_camera = c.raster_to_screen;
    return c;
}

using light = vec3f;

bool same_hemisphere(vec3f norm, vec3f v) { return 0 < norm.dot(v); }

light naive_trace(scene& scene, ray r, int depth) {
    auto intersection = intersect(scene.root, r);
    if (not intersection)
        return {8};
    if (0 == depth)
        return {};

    auto inorm = intersection->normal;
    auto scatter_d = sample_uniform_direction().normalized();
    auto icos = inorm.dot(scatter_d);
    auto f = icos / pi;

    if (f <= 0 or not same_hemisphere(inorm, scatter_d))
        return {};

    auto reflection_point = r.distance(intersection->distance);
    auto scatter_ray = ray{reflection_point + inorm * epsilon, scatter_d};

    auto Li = naive_trace(scene, scatter_ray, depth - 1);
    return Li * f;
}

light debug_trace(scene& scene, ray r) {
    auto intersection = intersect(scene.root, r);
    if (not intersection)
        return {};
    auto ncos = intersection->normal.dot(r.direction);
    auto norm_l = 1 - (ncos + 1) / 2;
    auto d = 1 - (std::cos(intersection->distance * 4) + 1) / 2;
    return {(norm_l + d) / 2, (norm_l + d) / 2, (norm_l + d) / 2};
}

int main(int argc, char** argv) {
    auto scene_path = [argc, argv]() -> std::optional<std::string> {
        if (argc <= 1)
            return {};
        return {{argv[1]}};
    }();

    auto arg_present = [argc, argv](std::string name) -> bool {
        for (auto i{2}; i < argc; ++i) {
            if (std::string{argv[i]} == name)
                return true;
        }
        return false;
    };

    auto debug = arg_present("--debug");

    if (not scene_path) {
        fmt::print("missing scene path\n", *scene_path);
        exit(1);
    }

    auto scene = load_scene(*scene_path);
    auto resolution = scene.film.resolution;
    fmt::print("loaded {}\n", *scene_path);

    std::vector<unsigned char> out;
    out.resize(4 * resolution.y * resolution.x);

#pragma omp parallel for schedule(monotonic : dynamic)
    for (int y = 0; y < resolution.y; ++y) {
        for (int x = 0; x < resolution.x; ++x) {
            auto px = vec2{x, y};
            auto ray = scene.view.point(vec2f{px});
            ray.direction = ray.direction.normalized();

            light light{};

            if (debug)
                light = debug_trace(scene, ray);
            else {
                for (int s{}; s < scene.film.supersampling; ++s)
                    light += naive_trace(scene, ray, scene.film.depth);
                light /= scene.film.supersampling;
            }

            auto r = std::clamp<Float>(light.x, 0, 1);
            auto g = std::clamp<Float>(light.y, 0, 1);
            auto b = std::clamp<Float>(light.z, 0, 1);
            auto px_offset = 4 * (y * resolution.x + x);
            out[px_offset + 0] = r * 255;
            out[px_offset + 1] = g * 255;
            out[px_offset + 2] = b * 255;
            out[px_offset + 3] = 255;
        }
    }
    sf::Image img;
    img.create((unsigned int)(resolution.x), (unsigned int)(resolution.y),
               out.data());
    img.saveToFile("out.png");
}

// coordinates:
// camera view covered by x and y (-1, 1)
// right-handed world coordinates
// x and y horizontal, z vertical, positive upwards

// scene definition
// basic ray tracer
// ray-triangle intersection
