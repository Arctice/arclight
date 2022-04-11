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
    constexpr Float gamma3 = (3 * ɛ) / (1 - 3 * ɛ);
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
    return std::visit([](const auto& n) { return node_bounds(*n); }, n.shape);
}

bounding_box node_bounds(const node_view& n) {
    return std::visit([](const auto& n) { return node_bounds(*n); }, n.shape);
}

std::optional<intersection> intersect(const triangle&, const ray&);
std::optional<intersection> intersect(const bvh_mesh&, const ray&);
std::optional<intersection> intersect(const node_instance& i, const ray& r) {
    auto Ti = i.T.inverse();
    auto ri = Ti(r);
    auto hit = intersect(i.n, ri);
    if (hit) {
        hit->normal = i.T.normal(hit->normal).normalized();
    }
    return hit;
}

std::optional<intersection> intersect(const node_bvh&, const ray&);

std::optional<intersection> intersect(const node& n, const ray& r) {
    auto i =
        std::visit([r](const auto& n) { return intersect(*n, r); }, n.shape);
    if (i and n.material)
        i->mat = n.material;
    return i;
}

std::optional<intersection> intersect(const node_view& n, const ray& r) {
    auto i =
        std::visit([r](const auto& n) { return intersect(*n, r); }, n.shape);
    if (i and n.material)
        i->mat = n.material;
    return i;
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

// woop et al 2013
std::optional<intersection> intersect(const triangle& tri, const ray& ray) {
    auto rd = ray.direction;
    auto kz = std::abs(rd.x) > std::abs(rd.y)
                  ? (std::abs(rd.x) > std::abs(rd.z) ? 0 : 2)
                  : (std::abs(rd.z) > std::abs(rd.y) ? 2 : 1);
    auto kx = (kz + 1) % 3;
    auto ky = (kx + 1) % 3;
    if (rd[kz] < 0)
        std::swap(kx, ky);

    auto Sx = rd[kx] / rd[kz];
    auto Sy = rd[ky] / rd[kz];
    auto Sz = 1 / rd[kz];

    auto A = (tri.A - ray.origin);
    auto B = (tri.B - ray.origin);
    auto C = (tri.C - ray.origin);

    auto Ax = A[kx] - Sx * A[kz];
    auto Ay = A[ky] - Sy * A[kz];
    auto Bx = B[kx] - Sx * B[kz];
    auto By = B[ky] - Sy * B[kz];
    auto Cx = C[kx] - Sx * C[kz];
    auto Cy = C[ky] - Sy * C[kz];

    auto U = Cx * By - Cy * Bx;
    auto V = Ax * Cy - Ay * Cx;
    auto W = Bx * Ay - By * Ax;

    if ((U < 0 or V < 0 or W < 0) and (U > 0 or V > 0 or W > 0))
        return {};

    auto det = U + V + W;
    if (det == 0)
        return {};

    auto Az = Sz * A[kz];
    auto Bz = Sz * B[kz];
    auto Cz = Sz * C[kz];
    auto T = U * Az + V * Bz + W * Cz;

    if ((det < 0 and T >= 0) or (det > 0 and T <= 0))
        return {};

    auto t = T / det;

    vec2f uv[3] = {{0, 0}, {1, 0}, {1, 1}};
    vec2f duvAC = uv[0] - uv[2];
    vec2f duvBC = uv[1] - uv[2];
    vec3f dAC = tri.A - tri.B;
    vec3f dBC = tri.B - tri.C;
    Float determinant = duvAC.x * duvBC.y - duvAC.y * duvBC.x;
    if (determinant == 0)
        throw;
    else {
        auto id = 1 / determinant;
        auto dpdu = (dAC * duvBC.y - dBC * duvAC.y) * id;
        auto dpdv = (dAC * -duvBC.x + dBC * duvAC.x) * id;

        auto n = dpdu.cross(dpdv).normalized();
        if (det < 0)
            n = n * -1;

        return intersection{
            .distance = t,
            .normal = n,
            .shading = {dpdu, dpdv},
        };
    }
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

struct scatter_sample {
    light value;
    vec3f direction;
    Float probability;
};

vec3f shading_frame::to_local(vec3f v) const {
    auto surface_normal = dpdu.cross(dpdv).normalized();
    auto bz = surface_normal;
    auto bx = dpdu.normalized();
    auto by = bz.cross(bx);
    return {v.dot(bx), v.dot(by), v.dot(bz)};
}

vec3f shading_frame::to_world(vec3f v) const {
    auto surface_normal = dpdu.cross(dpdv).normalized();
    auto bz = surface_normal;
    auto bx = dpdu.normalized();
    auto by = bz.cross(bx);
    return bx * v.x + by * v.y + bz * v.z;
}

scatter_sample matte_scattering(const scene&, const intersection& isect,
                                const vec3f&) {
    auto away = sample_cosine_hemisphere(sample_2d());

    auto cos_theta = away.z;

    light reflectance{.5};
    if (isect.mat)
        reflectance = isect.mat->reflectance;

    return scatter_sample{
        .value = reflectance / π,
        .direction = isect.shading.to_world(away),
        .probability = std::abs(cos_theta) / π,
    };
}

scatter_sample sample_scattering(const scene& scene, const intersection& isect,
                                 const vec3f& towards) {
    return matte_scattering(scene, isect, towards);
}

bool same_hemisphere(vec3f norm, vec3f v) { return 0 < norm.dot(v); }

light debug_trace(scene& scene, ray r, int) {
    auto intersection = intersect(scene.root, r);
    if (not intersection)
        return {};
    auto ncos = intersection->normal.dot(r.direction);
    auto norm_l = 1 - (ncos + 1) / 2;
    auto d = 1 - (std::cos(intersection->distance * 4) + 1) / 2;
    return {(norm_l + d) / 2, (norm_l + d) / 2, (norm_l + d) / 2};
}

light naive_trace(scene& scene, ray r, int depth) {
    auto intersection = intersect(scene.root, r);
    if (not intersection)
        return {scene.film.global_radiance};

    vec3f Le{};
    if (intersection->mat and intersection->mat->light)
        Le = *intersection->mat->light;

    if (0 == depth)
        return Le;

    auto inorm = intersection->normal;
    auto scatter_d = sample_uniform_direction(sample_2d()).normalized();
    auto icos = std::abs(inorm.dot(scatter_d));
    auto pdf = 1 / (2 * π);
    auto f = (icos / π) / pdf;

    if (f <= 0 or not same_hemisphere(inorm, scatter_d))
        return Le;

    auto reflection_point = r.distance(intersection->distance);
    auto scatter_ray = ray{reflection_point + inorm * 0.00001, scatter_d};

    auto Li = naive_trace(scene, scatter_ray, depth - 1);
    return Li * f + Le;
}

light path_trace(scene& scene, ray r, int depth) {
    auto intersection = intersect(scene.root, r);
    if (not intersection)
        return {scene.film.global_radiance};

    light emission{};
    if (intersection->mat and intersection->mat->light) {
        emission = *intersection->mat->light;
    }

    if (0 == depth)
        return emission;

    auto scatter = sample_scattering(scene, *intersection, r.direction * -1);

    auto icos = scatter.direction.dot(intersection->normal);
    auto reflectance = (scatter.value * icos) / scatter.probability;

    if ((reflectance.x <= 0 and reflectance.y <= 0 and reflectance.z <= 0) or
        not same_hemisphere(intersection->normal, scatter.direction))
        return emission;

    auto reflection_point = r.distance(intersection->distance);
    auto scatter_ray =
        ray{reflection_point + intersection->normal *
                                   std::numeric_limits<Float>::epsilon() * 32,
            scatter.direction};

    auto L = path_trace(scene, scatter_ray, depth - 1);

    return emission + reflectance * L;
}

Float gamma_correct(Float value) {
    if (value <= 0.0031308f)
        return 12.92f * value;
    return 1.055f * std::pow(value, (Float)(1.f / 2.4f)) - 0.055f;
}

vec3f light_to_rgb(light light) {
    return {std::clamp<Float>(gamma_correct(light.x), 0, 1),
            std::clamp<Float>(gamma_correct(light.y), 0, 1),
            std::clamp<Float>(gamma_correct(light.z), 0, 1)};
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

    auto trace = [&scene, debug]() {
        if (debug)
            return debug_trace;
        switch (scene.film.method) {
        case integrator::path: return path_trace;
        case integrator::brute_force: return naive_trace;
        }
    }();

#pragma omp parallel for schedule(monotonic : dynamic)
    for (int y = 0; y < resolution.y; ++y) {
        for (int x = 0; x < resolution.x; ++x) {
            auto px = vec2{x, y};

            light light{};

            for (int s{}; s < scene.film.supersampling; ++s) {
                auto jitter = sample_2d();
                auto ray = scene.view.point(vec2f{px} + jitter);
                light += trace(scene, ray, scene.film.depth);
            }
            light /= scene.film.supersampling;

            auto [r, g, b] = light_to_rgb(light);

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

// todo:
// multithreaded deterministic rng
// glossy brdf
// rendering preview
// comparison tests
// adaptive sampling, convergence metering
// perspective camera
