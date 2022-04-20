#include "base.h"
#include "scene.h"
#include "rng.h"
#include "tev_ipc.h"

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
    return bounding_box{} | i.T.point(min) | i.T.point(max) |
           i.T.point(vec3f{max.x, min.y, min.z}) |
           i.T.point(vec3f{min.x, max.y, min.z}) |
           i.T.point(vec3f{min.x, min.y, max.z}) |
           i.T.point(vec3f{min.x, max.y, max.z}) |
           i.T.point(vec3f{max.x, min.y, max.z}) |
           i.T.point(vec3f{max.x, max.y, min.z}) |
           i.T.point(vec3f{max.x, max.y, max.z});
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
        hit->normal = Ti.normal(hit->normal).normalized();
        hit->shading.dpdu = i.T.vector(hit->shading.dpdu);
        hit->shading.dpdv = i.T.vector(hit->shading.dpdv);
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

scatter_sample scatter(const scene&, const lambertian& matte,
                       const intersection& isect, const vec3f&) {
    auto away = sample_cosine_hemisphere(sample_2d());
    auto angle_cos = away.z;
    return scatter_sample{
        .value = matte.reflectance / π,
        .direction = isect.shading.to_world(away),
        .probability = std::abs(angle_cos) / π,
    };
}

scatter_sample scatter(const scene&, const lambertian& matte,
                       const intersection& isect, const vec3f&,
                       const vec3f& away) {
    auto angle_cos = isect.shading.to_local(away).z;
    return scatter_sample{
        .value = matte.reflectance / π,
        .direction = away,
        .probability = std::abs(angle_cos) / π,
    };
}

scatter_sample scatter(const scene&, const emissive&, const intersection&,
                       const vec3f&) {
    return scatter_sample{
        .value = 0,
        .direction = {0, 0, 1},
        .probability = 1,
    };
}

scatter_sample scatter(const scene&, const emissive&, const intersection&,
                       const vec3f&, const vec3f&) {
    return scatter_sample{
        .value = 0,
        .direction = {0, 0, 1},
        .probability = 1,
    };
}

scatter_sample scatter(const scene&, const blinn_phong&, const intersection&,
                       const vec3f&, const vec3f&) {
    throw;
}

scatter_sample scatter(const scene&, const blinn_phong& material,
                       const intersection& isect, const vec3f& towards) {
    auto away = sample_cosine_hemisphere(sample_2d());
    auto angle_cos = away.z;
    auto probability = std::abs(angle_cos) / π;

    auto roughness = material.diffuse;
    auto glossiness = (Float)material.sharpness;
    light reflectance{material.diffuse_color};

    auto diffuse = reflectance * roughness * (1 / π);

    auto halfv = (away + isect.shading.to_local(towards)).normalized();
    auto specular_intensity = std::pow(std::max<Float>(halfv.z, 0), glossiness);

    auto energy_conservation = [](Float n) {
        return ((n + 2) * (n + 4)) / (8 * π * (std::pow<Float>(2, -n / 2) + 2));
    }(glossiness);

    auto specular = material.specular_color * energy_conservation *
                    specular_intensity * (1 - roughness);

    auto value = (specular + diffuse);

    return scatter_sample{
        .value = value,
        .direction = isect.shading.to_world(away),
        .probability = probability,
    };
}

Float beckmann_microfacet_area(Float roughness, Float a) {
    auto t = std::tan(a);
    if (std::isinf(t))
        return 0;
    auto r = roughness*roughness;
    return std::exp(-(t * t) / r) / (π * r * std::pow(std::cos(a), 4));
}

Float beckmann_microfacet_lambda(Float roughness, Float t) {
    auto a = 1 / (roughness * std::abs(std::tan(t)));
    if ((Float)1.6 <= a)
        return 0;
    return 0.5 * (std::erf(a) - 1 + std::exp(-a * a) / (a * std::sqrt(π)));
}

Float beckmann_microfacet_geometry(Float roughness, Float a, Float b) {
    auto A = 1 / (1 + beckmann_microfacet_lambda(roughness, a));
    auto B = 1 / (1 + beckmann_microfacet_lambda(roughness, b));
    return 1 / (1 + A + B);
}

auto fresnel_conductor(Float cos_t, light etai, light etat, light k) {
    light Eta = etat / etai;
    light Etak = k / etai;

    float CosTheta2 = cos_t * cos_t;
    float SinTheta2 = 1 - CosTheta2;
    light Eta2 = Eta * Eta;
    light Etak2 = Etak * Etak;

    light t0 = Eta2 - Etak2 - SinTheta2;
    light a2plusb2 = (t0 * t0 + Eta2 * Etak2 * 4);
    a2plusb2 = {std::sqrt(a2plusb2.x), std::sqrt(a2plusb2.y),
                std::sqrt(a2plusb2.z)};

    light t1 = {a2plusb2.x + CosTheta2, a2plusb2.y + CosTheta2,
                a2plusb2.z + CosTheta2};
    light a = (a2plusb2 + t0) * (Float)0.5;
    a = {std::sqrt(a.x), std::sqrt(a.y), std::sqrt(a.z)};
    light t2 = a * 2 * cos_t;
    light Rs = (t1 - t2) / (t1 + t2);

    light t3 = a2plusb2 * CosTheta2 + SinTheta2 * SinTheta2;
    light t4 = t2 * SinTheta2;
    light Rp = Rs * (t3 - t4) / (t3 + t4);

    return (Rp + Rs) * (Float)0.5;
}

scatter_sample scatter(const scene&, const specular& material,
                       const intersection& isect, const vec3f& towards) {
    auto to = isect.shading.to_local(towards);
    auto away = (to * -1 + vec3f{0, 0, 2 * std::abs(to.z)});

    auto reflectance = fresnel_conductor(
        std::abs(to.z), {1}, material.refraction, material.absorption);

    return scatter_sample{
        .value = reflectance,
        .direction = isect.shading.to_world(away),
        .probability = std::numeric_limits<Float>::infinity(),
    };
}

scatter_sample scatter(const scene&, const specular&, const intersection&,
                       const vec3f&, const vec3f& away) {
    return scatter_sample{
        .value = 0,
        .direction = away,
        .probability = 0,
    };
}

light glossy_reflection(const glossy& material, const vec3f& towards,
                        const vec3f& away) {
    auto halfv = (away + towards).normalized();
    auto a = std::acos(away.z);
    auto b = std::acos(towards.z);
    auto h = std::acos(halfv.z);

    auto F = fresnel_conductor(std::abs(away.dot(towards)), {1},
                               material.refraction, material.absorption);

    auto D = beckmann_microfacet_area(material.roughness, h);
    auto G = beckmann_microfacet_geometry(material.roughness, a, b);

    auto f = F * D * G / (4 * std::abs(away.z) * std::abs(towards.z));
    return f;
}

scatter_sample scatter(const scene&, const glossy& material,
                       const intersection& isect, const vec3f& towards) {
    auto away = sample_cosine_hemisphere(sample_2d());
    auto probability = std::abs(away.z) / π;

    auto f = glossy_reflection(material, isect.shading.to_local(towards), away);

    return scatter_sample{
        .value = f,
        .direction = isect.shading.to_world(away),
        .probability = probability,
    };
}

scatter_sample scatter(const scene&, const glossy& material,
                       const intersection& isect, const vec3f& towards,
                       const vec3f& out) {
    auto away = isect.shading.to_local(out);
    auto probability = std::abs(away.z) / π;
    auto f = glossy_reflection(material, isect.shading.to_local(towards), away);
    return scatter_sample{
        .value = f,
        .direction = out,
        .probability = probability,
    };
}

scatter_sample scatter(const scene& scene, const intersection& isect,
                       const vec3f& towards);
scatter_sample scatter(const scene& scene, const intersection& isect,
                       const vec3f& towards, const vec3f& away);

scatter_sample scatter(const scene&, const mixed_material&, const intersection&,
                       const vec3f&, const vec3f&) {
    throw;
}

scatter_sample scatter(const scene& scene, const mixed_material& mix,
                       const intersection& isect, const vec3f& towards) {
    auto scattering_source = std::clamp<int>(
        (int)(sample_1d() * mix.layers.size()), 0, mix.layers.size() - 1);

    auto scattering = std::visit(
        [&](const auto& m) { return scatter(scene, m, isect, towards); },
        *mix.layers[scattering_source].second);
    scattering.probability *= mix.layers[scattering_source].first;
    scattering.value *= mix.layers[scattering_source].first;

    auto away = scattering.direction;

    Float total_weight{};
    
    for (int i{}; i < (int)mix.layers.size(); ++i) {
        auto [weight, layer] = mix.layers[i];
        total_weight += weight;

        if (i == scattering_source)
            continue;

        auto s = std::visit(
            [&](const auto& m) {
                return scatter(scene, m, isect, towards, away);
            },
            *layer);

        scattering.probability += s.probability*weight;
        scattering.value += s.value*weight;
    }

    scattering.probability /= total_weight;
    scattering.value /= total_weight;

    return scattering;
}

scatter_sample scatter(const scene& scene, const intersection& isect,
                       const vec3f& towards, const vec3f& away) {
    if (isect.mat)
        return std::visit(
            [&](const auto& material) {
                return scatter(scene, material, isect, towards, away);
            },
            *isect.mat);
    else
        return scatter(scene, lambertian{}, isect, towards, away);
}

scatter_sample scatter(const scene& scene, const intersection& isect,
                       const vec3f& towards) {
    if (isect.mat)
        return std::visit(
            [&](const auto& material) {
                return scatter(scene, material, isect, towards);
            },
            *isect.mat);
    else
        return scatter(scene, lambertian{}, isect, towards);
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
    if (intersection->mat and std::get_if<emissive>(intersection->mat)) {
        Le = std::get_if<emissive>(intersection->mat)->value;
    }

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

    light emission{0};
    if (intersection->mat and std::get_if<emissive>(intersection->mat)) {
        emission = std::get_if<emissive>(intersection->mat)->value;
    }

    if (0 == depth)
        return emission;

    auto scattering = scatter(scene, *intersection, r.direction * -1);
    auto icos = scattering.direction.dot(intersection->normal);

    if (std::isinf(scattering.probability))
        scattering.probability = 1;

    auto reflectance = (scattering.value * icos) / scattering.probability;

    if ((reflectance.x <= 0 and reflectance.y <= 0 and reflectance.z <= 0) or
        not same_hemisphere(intersection->normal, scattering.direction))
        return emission;

    auto reflection_point = r.distance(intersection->distance);
    auto scatter_ray = ray{reflection_point + intersection->normal * 0.00004,
                           scattering.direction};

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

    auto trace = [&scene, debug]() {
        if (debug)
            return debug_trace;
        switch (scene.film.method) {
        case integrator::path: return path_trace;
        case integrator::brute_force: return naive_trace;
        }
    }();

    std::vector<light> light_total{};
    std::vector<float> image;
    light_total.resize(resolution.x * resolution.y);
    image.resize(3 * resolution.y * resolution.x);

    auto tev = std::optional<ipc::tev>{};
    if (arg_present("--tev")) {
        try {
            tev.emplace();
            tev->create_rgb_image(*scene_path, resolution.x, resolution.y);
        }
        catch (std::exception& err) {
            fmt::print("tev connection failed, image output only\n");
        }
    }

    for (int s{}; s < scene.film.supersampling; ++s) {
#pragma omp parallel for schedule(monotonic : dynamic)
        for (int y = 0; y < resolution.y; ++y) {
            for (int x = 0; x < resolution.x; ++x) {
                auto px = vec2{x, y};
                auto jitter = sample_2d();
                auto ray = scene.view.point(vec2f{px} + jitter);
                auto light = trace(scene, ray, scene.film.depth);

                auto px_offset = (y * resolution.x + x);
                light_total[px_offset] += light;

                auto [r, g, b] = light_total[px_offset] / (s + 1);
                px_offset *= 3;
                image[px_offset + 0] = r;
                image[px_offset + 1] = g;
                image[px_offset + 2] = b;
            }
        }

        if (tev)
            tev->update_rgb_image(*scene_path, 0, 0, resolution.x, resolution.y,
                                  image);
    }

    std::vector<unsigned char> out;
    out.resize(4 * resolution.y * resolution.x);
    for (int px = 0; px < resolution.y * resolution.x; ++px) {
        auto light = light_total[px] / scene.film.supersampling;
        auto [r, g, b] = light_to_rgb(light);
        out[px * 4 + 0] = r * 255;
        out[px * 4 + 1] = g * 255;
        out[px * 4 + 2] = b * 255;
        out[px * 4 + 3] = 255;
    }
    sf::Image img;
    img.create((u32)(resolution.x), (u32)(resolution.y), out.data());
    img.saveToFile("out.png");
}

// coordinates:
// camera view covered by x and y (-1, 1)
// right-handed world coordinates
// x and y horizontal, z vertical, positive upwards

// todo:
// multithreaded deterministic rng
// glossy brdf
// light sampling
// comparison tests
// adaptive sampling, convergence metering
// perspective camera
