#include "base.h"
#include "scene.h"
#include "rng.h"
#include "tev_ipc.h"

#include <array>
#include <queue>
#include <chrono>

#include <thread>
#include <mutex>
#include <atomic>

#include <SFML/Graphics.hpp>
#include <cblas.h>
#include <cassert>

Float power_heuristic(Float f, Float g) { return (f * f) / (f * f + g * g); }

Float gamma_correct(Float value) {
    if (value <= (Float)0.0031308)
        return Float(12.92) * value;
    return Float(1.055) * std::pow(value, 1 / (Float)2.4) - (Float)0.055;
}

Float inverse_gamma_correct(Float value) {
    if (value <= (Float)0.04045)
        return value * 1 / (Float)12.92;
    return std::pow((value + 0.055f) * 1 / (Float)1.055, (Float)2.4);
}

light sample_texture(const uv_texture&, const intersection& isect) {
    return {isect.uv.x, isect.uv.y, 0};
}

light sample_texture(const checkerboard_texture&, const intersection& isect) {
    Float scale{16};
    return light{0.01} +
           light{0.99} * std::round(std::fmod(
                             std::round(std::fmod((isect.uv.x) * scale, 1)) +
                                 std::round(std::fmod((isect.uv.y) * scale, 1)),
                             2));
}

light sample_texture(const constant_texture& tex, const intersection&) {
    return tex.v;
}

light sample_texture(const Float& constant, const intersection&) {
    return light{constant};
}

light sample_texture(const light& constant, const intersection&) {
    return constant;
}

light sample_texture(const image_texture& tex, const intersection& isect) {
    auto [x, y] = tex.img->getSize();

    auto sfc =
        tex.img->getPixel((i32)std::fmod(std::abs(isect.uv.x * x), x),
                          y - 1 - (i32)std::fmod(std::abs(isect.uv.y * y), y));

    return {inverse_gamma_correct(sfc.r / (Float)255),
            inverse_gamma_correct(sfc.g / (Float)255),
            inverse_gamma_correct(sfc.b / (Float)255)};
}

light sample_texture(const product_texture& tex, const intersection& isect);

light sample_texture_light(const texture&, const intersection&);
light sample_texture(const product_texture& tex, const intersection& isect) {
    return sample_texture_light(*tex.A, isect) *
           sample_texture_light(*tex.B, isect);
}

light sample_texture_light(const texture& tex, const intersection& isect) {
    return std::visit(
        [&isect](auto& tex) { return sample_texture(tex, isect); }, tex);
}

Float sample_texture_1d(const texture& tex, const intersection& isect) {
    return sample_texture_light(tex, isect).mean();
}

bool same_normal_hemisphere(vec3f norm, vec3f v) { return 0 < norm.dot(v); }
bool same_shading_hemisphere(vec3f a, vec3f b) { return 0 < a.z * b.z; }

bool is_emissive(material* m) { return std::holds_alternative<emissive>(*m); }

light emitted_light(const intersection& intersection) {
    return sample_texture_light(std::get_if<emissive>(intersection.mat)->value,
                                intersection);
}

bounding_sphere ritter_update(bounding_sphere S, vec3f P) {
    auto r2 = S.radius * S.radius;
    auto D = S.centre - P;
    auto d2 = D.length_sq();
    if (d2 < r2)
        return S;
    auto d = std::sqrt(d2);
    auto next_r = (S.radius + d) * 0.5;
    auto offset = d - next_r;
    S.centre = (S.centre * next_r + P * offset) / d;
    S.radius = next_r;
    return S;
}

bounding_sphere merge_spheres(bounding_sphere A, bounding_sphere B) {
    auto D = (A.centre - B.centre);
    if (D.length_sq() < ɛ)
        return A.radius < B.radius ? B : A;
    auto n = D.normalized();
    auto fA = A.centre + n * A.radius;
    auto fB = B.centre + n * (-B.radius);
    auto R = (fA - fB).length();
    if (R <= A.radius)
        return A;
    if (R <= B.radius)
        return B;
    auto C = fA + (fB - fA) * 0.5;
    return {C, R};
}

bounding_sphere find_bounding_sphere(const triangle*);
bounding_sphere find_bounding_sphere(const node_instance*);
bounding_sphere find_bounding_sphere(const node_view*);
bounding_sphere find_bounding_sphere(const node*);
bounding_sphere find_bounding_sphere(const node_bvh*);
bounding_sphere find_bounding_sphere(const bvh_mesh*);

bounding_sphere find_bounding_sphere(const triangle* t) {
    auto& [A, B, C, _] = *t;
    auto AC = C - A;
    auto AB = B - A;
    auto Z = AB.cross(AC);
    vec3f AO = (Z.cross(AB) * AC.length_sq() + AC.cross(Z) * AB.length_sq()) /
               (2 * Z.length_sq());
    return bounding_sphere{A + AO, AO.length()};
}

bounding_sphere find_bounding_sphere(const bvh_mesh* bvh) {
    // ritter's approximate bounding sphere
    auto& points = bvh->data.vertices;
    auto init_point = points[0];
    Float d2 = 0;
    vec3f x{init_point}, y{init_point};
    for (auto& p : points) {
        auto nd = (p - init_point).length_sq();
        if (nd > d2) {
            d2 = nd;
            x = p;
        }
    }
    for (auto& p : points) {
        auto nd = (y - x).length_sq();
        if (nd > d2) {
            d2 = nd;
            y = p;
        }
    }

    auto S =
        bounding_sphere{x + (y - x) * (Float)0.5, std::sqrt(d2) * (Float)0.5};
    for (auto& p : points) S = ritter_update(S, p);
    return S;
}

bounding_sphere find_bounding_sphere(const node_instance* instance) {
    auto T = instance->T;
    auto sphere = find_bounding_sphere(&instance->n);
    return {T.point(sphere.centre), T.vector({0, 0, sphere.radius}).length()};
}

bounding_sphere find_bounding_sphere(const node_bvh* bvh) {
    if (bvh->nodes.empty())
        std::runtime_error("empty node bvh");
    auto sphere = find_bounding_sphere(&bvh->nodes[0]);
    for (auto& node : bvh->nodes)
        sphere = merge_spheres(sphere, find_bounding_sphere(&node));

    return sphere;
}

bounding_sphere find_bounding_sphere(const node* node) {
    return std::visit([&](auto& v) { return find_bounding_sphere(v.get()); },
                      node->shape);
}

bounding_sphere find_bounding_sphere(const node_view* node) {
    return std::visit([&](auto& v) { return find_bounding_sphere(v); },
                      node->shape);
}

std::vector<light_source> collect_lights(const node_instance* node,
                                         transform T = transform::identity());
std::vector<light_source> collect_lights(const node* node,
                                         transform T = transform::identity());
std::vector<light_source> collect_lights(const node_bvh* node,
                                         transform T = transform::identity());

std::vector<light_source> collect_lights(const triangle*, const transform&) {
    return {};
}

std::vector<light_source> collect_lights(const bvh_mesh*, const transform&) {
    return {};
}

std::vector<light_source> collect_lights(const node_bvh* bvh, transform T) {
    std::vector<light_source> lights;
    for (auto& node : bvh->nodes)
        for (auto& light : collect_lights(&node, T)) lights.push_back(light);
    return lights;
}

std::vector<light_source> collect_lights(const node* node, transform T) {
    if (node->material and is_emissive(node->material)) {
        std::vector<light_source> lights;
        lights.push_back(
            {node_light{node_instance{T, *node}, find_bounding_sphere(node)}});
        return lights;
    }
    else
        return std::visit([&](auto& v) { return collect_lights(v.get(), T); },
                          node->shape);

    return {};
}

std::vector<light_source> collect_lights(const node_instance* node,
                                         transform T) {
    T = node->T.compose(T);

    if (node->n.material and is_emissive(node->n.material)) {
        std::vector<light_source> lights;
        lights.push_back({node_light{node_instance{T, node->n},
                                     find_bounding_sphere(node)}});
        return lights;
    }
    else
        return std::visit([&](auto& v) { return collect_lights(v, T); },
                          node->n.shape);

    return {};
}

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

bounding_box shape_bounds(const indexed_triangle& V) {
    return shape_bounds(triangle{V.mesh->vertices[V.vertices.x],
                                 V.mesh->vertices[V.vertices.y],
                                 V.mesh->vertices[V.vertices.z]});
}

bool bbox_ray_intersection(const bounding_box& bb, const ray& ray, Float d,
                           const vec3f& inv) {
    constexpr Float gamma3 = gamma(3);
    Float t0 = 0, t1 = d;
    for (auto d{0}; d < 3; ++d) {
        auto near = (bb.min[d] - ray.origin[d]) * inv[d];
        auto far = (bb.max[d] - ray.origin[d]) * inv[d];

        if (near > far)
            std::swap(near, far);
        far *= 1 + 2 * gamma3;
        t0 = std::max(t0, near);
        t1 = std::min(t1, far);
    }
    return t0 <= t1;
}

bool bbox_ray_intersection(const bounding_box& bb, const ray& ray, Float d) {
    auto inverse =
        vec3f{1 / ray.direction.x, 1 / ray.direction.y, 1 / ray.direction.z};
    return bbox_ray_intersection(bb, ray, d, inverse);
}

std::unique_ptr<node_instance> instance(const transform& T, const node& n) {
    return std::make_unique<node_instance>(T, n);
}

bounding_box node_bounds(const triangle& n) { return shape_bounds(n); }

bounding_box node_bounds(const node_instance& i) { return i.world_bounds; }

bounding_box node_bounds(const bvh_mesh& n) { return n.bvh.nodes[0].bounds; }
bounding_box node_bounds(const node_bvh& n) { return n.bvh.nodes[0].bounds; }

bounding_box node_bounds(const node& n) {
    return std::visit([](const auto& n) { return node_bounds(*n); }, n.shape);
}

bounding_box node_bounds(const node_view& n) {
    return std::visit([](const auto& n) { return node_bounds(*n); }, n.shape);
}

void node_instance::precompute_bounds() {
    this->local_bounds = node_bounds(n);
    auto& [min, max] = this->local_bounds;
    auto min_ = T.point(min);
    this->world_bounds = bounding_box{min_, min_} | T.point(max) |
                         T.point(vec3f{max.x, min.y, min.z}) |
                         T.point(vec3f{min.x, max.y, min.z}) |
                         T.point(vec3f{min.x, min.y, max.z}) |
                         T.point(vec3f{min.x, max.y, max.z}) |
                         T.point(vec3f{max.x, min.y, max.z}) |
                         T.point(vec3f{max.x, max.y, min.z});
};

std::optional<intersection> intersect(const node& n, const ray& r, Float max_t);
std::optional<intersection> intersect(const node_view& n, const ray& r,
                                      Float max_t);
std::optional<intersection> intersect(const triangle&, const ray&, Float max_t);
std::optional<intersection> intersect(const bvh_mesh&, const ray&, Float max_t);
std::optional<intersection> intersect(const node_bvh&, const ray&, Float max_t);

std::optional<intersection> intersect(const node_instance& i, const ray& r,
                                      Float max_t) {
    auto Ti = i.T.inverse();
    auto ri = Ti(r);
    if (not bbox_ray_intersection(i.local_bounds, ri, max_t))
        return {};

    auto hit = intersect(i.n, ri, max_t);
    if (hit) {
        hit->normal = Ti.normal(hit->normal).normalized();
        hit->shading.normal = Ti.normal(hit->shading.normal).normalized();

        hit->shading.dpdu = i.T.vector(hit->shading.dpdu);
        hit->shading.dpdv = i.T.vector(hit->shading.dpdv);
    }
    return hit;
}

std::optional<intersection> intersect(const node& n, const ray& r,
                                      Float max_t) {
    auto i = std::visit(
        [&r, max_t](const auto& n) { return intersect(*n, r, max_t); },
        n.shape);
    if (i and n.material and not i->mat)
        i->mat = n.material;
    return i;
}

std::optional<intersection> intersect(const node_view& n, const ray& r,
                                      Float max_t) {
    auto i = std::visit(
        [&r, max_t](const auto& n) { return intersect(*n, r, max_t); },
        n.shape);
    if (i and n.material and not i->mat)
        i->mat = n.material;
    return i;
}

struct bvh_traversal {
    bvh_traversal(const BVH* bvh, const ray& r) : bvh(bvh), ray(r) {
        inv = vec3f{1 / ray.direction.x, 1 / ray.direction.y,
                    1 / ray.direction.z};
        if (1 < bvh->nodes.size() or bvh->nodes[0].is_leaf())
            stack.push(0);
        else
            ; // degenerate BVH
    }

    std::pair<int, int> next(Float bound) {
        while (not stack.empty()) {
            int n = stack.top();
            auto& node = bvh->nodes[n];

            if (not bbox_ray_intersection(node.bounds, ray, bound, inv)) {
                stack.pop();
                continue;
            }

            if (node.is_leaf()) {
                stack.pop();
                return {node.offset, node.offset + node.count};
            }
            else {
                auto A = n + 1;
                auto B = node.offset;
                auto split_dir =
                    bvh->nodes[B].bounds.min - bvh->nodes[A].bounds.min;
                if (0 < split_dir.dot(ray.direction)) {
                    stack.top() = B;
                    stack.push(A);
                }
                else {
                    stack.top() = A;
                    stack.push(B);
                }
            }
        }

        return {0, 0};
    }

    operator bool() const { return not stack.empty(); }

private:
    const BVH* bvh;
    std::stack<int, std::vector<int>> stack;
    const ray& ray;
    vec3f inv;
};

// woop et al 2013
std::optional<intersection> intersect(const indexed_triangle& tri,
                                      const ray& ray, Float) {
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

    auto tA = tri.mesh->vertices[tri.vertices.x];
    auto tB = tri.mesh->vertices[tri.vertices.y];
    auto tC = tri.mesh->vertices[tri.vertices.z];

    auto A = (tA - ray.origin);
    auto B = (tB - ray.origin);
    auto C = (tC - ray.origin);

    auto Ax = A[kx] - Sx * A[kz];
    auto Ay = A[ky] - Sy * A[kz];
    auto Bx = B[kx] - Sx * B[kz];
    auto By = B[ky] - Sy * B[kz];
    auto Cx = C[kx] - Sx * C[kz];
    auto Cy = C[ky] - Sy * C[kz];

    auto U = Cx * By - Cy * Bx;
    auto V = Ax * Cy - Ay * Cx;
    auto W = Bx * Ay - By * Ax;

    // if (U == 0 || V == 0 || W == 0)
    // // should fall back to double precision
    //     fmt::print("!\n");

    if ((U < 0 or V < 0 or W < 0) and (U > 0 or V > 0 or W > 0))
        return {};

    auto det = U + V + W;
    if (det == 0)
        return {};

    auto Az = Sz * A[kz];
    auto Bz = Sz * B[kz];
    auto Cz = Sz * C[kz];
    auto T = U * Az + V * Bz + W * Cz;

    auto far = std::numeric_limits<float>::infinity();
    auto near = 0.0;
    if ((det < 0 and (T >= near * det or T < far * det)) or
        (det > 0 and (T <= near * det or T > far * det)))
        return {};

    vec2f uv[3] = {{0, 0}, {1, 0}, {1, 1}};
    vec2f duvAC = uv[0] - uv[2];
    vec2f duvBC = uv[1] - uv[2];
    vec3f dAC = tA - tC;
    vec3f dBC = tB - tC;
    Float determinant = duvAC.x * duvBC.y - duvAC.y * duvBC.x;
    if (determinant == 0)
        throw;
    else {
        auto id = 1 / determinant;
        auto dpdu = (dAC * duvBC.y - dBC * duvAC.y) * id;
        auto dpdv = (dAC * -duvBC.x + dBC * duvAC.x) * id;

        U = U / det;
        V = V / det;
        W = W / det;
        auto t = T / det;

        auto n = dpdu.cross(dpdv).normalized();
        auto ns = n;

        if (not tri.mesh->normals.empty()) {
            ns = (tri.mesh->normals[tri.vertices.x] * U +
                  tri.mesh->normals[tri.vertices.y] * V +
                  tri.mesh->normals[tri.vertices.z] * W);
            if (ns.length_sq() >= ɛ)
                ns = ns.normalized();
            else
                ns = n;

            auto Z = dpdu.cross(ns);
            if (Z.length_sq() <= ɛ)
                Z = dpdv.cross(ns);
            dpdu = Z.normalized();
            dpdv = dpdu.cross(ns);
        }

        auto isect_uv = (uv[0] * U + uv[1] * V + uv[2] * W);
        if (not tri.mesh->tex_coords.empty()) {
            isect_uv = (tri.mesh->tex_coords[tri.vertices.x] * U +
                        tri.mesh->tex_coords[tri.vertices.y] * V +
                        tri.mesh->tex_coords[tri.vertices.z] * W);
        }

        return intersection{
            .distance = t,
            .normal = ns,
            .uv = isect_uv,
            .shading = {dpdu, dpdv, ns},
        };
    }
}

std::optional<intersection> intersect(const triangle& T, const ray& ray,
                                      Float max_t) {
    indexed_mesh M{
        .vertices = {T.A, T.B, T.C},
        .tex_coords = {T.uv[0], T.uv[1], T.uv[2]},
        .triangles = {{0, 1, 2}},
    };
    return intersect(M.reify(0), ray, max_t);
}

std::optional<intersection> intersect(const bvh_mesh& m, const ray& ray,
                                      Float max_t) {
    auto search = bvh_traversal(&m.bvh, ray);
    std::optional<intersection> result{};
    auto nearest = max_t;

    while (search) {
        auto [first, last] = search.next(nearest);
        while (first != last) {
            auto index = first++;
            auto hit = intersect(m.data.reify(index), ray, nearest);
            if (not hit or nearest < hit->distance)
                continue;

            nearest = hit->distance;
            result = hit;
        }
    }
    return result;
}

std::optional<intersection> intersect(const node_bvh& m, const ray& ray,
                                      Float max_t) {
    auto search = bvh_traversal(&m.bvh, ray);
    std::optional<intersection> result{};
    auto nearest = max_t;

    while (search) {
        auto [first, last] = search.next(nearest);
        while (first != last) {
            auto index = first++;
            auto hit = intersect(m.nodes[index], ray, nearest);
            if (not hit or nearest < hit->distance)
                continue;

            nearest = hit->distance;
            result = hit;
        }
    }
    return result;
}

std::unique_ptr<bvh_mesh> load_model(std::string path) {
    indexed_mesh data = load_ply(path);
    auto mesh = bvh_mesh{{}, {}};
    mesh.bvh = build_bvh(
        data.triangles.size(),
        [&data](size_t n) { return shape_bounds(data.reify(n)); },
        [&data, &mesh](size_t index) {
            mesh.data.triangles.push_back(data.triangles[index]);
        });
    mesh.data.vertices = std::move(data.vertices);
    mesh.data.normals = std::move(data.normals);
    mesh.data.tex_coords = std::move(data.tex_coords);
    return std::make_unique<bvh_mesh>(std::move(mesh));
}

orthographic_camera::orthographic_camera(vec2f resolution, Float scale,
                                         vec3f position, vec3f towards,
                                         vec3f up) {
    camera_to_world = transform::look_at(position, towards, up);
    raster_to_screen =
        transform::scale({1 / resolution.x, 1 / resolution.y, 1});
    raster_to_screen =
        transform::scale({2 * scale, 2 * scale, 1}).compose(raster_to_screen);
    raster_to_screen =
        transform::translate({-scale, -scale, 0}).compose(raster_to_screen);
    screen_to_camera = transform::identity();
    raster_to_camera = raster_to_screen;
}

ray orthographic_camera::point(vec2f px) const {
    auto o = this->raster_to_camera.point(vec3f{px.x, px.y, 0});
    return camera_to_world(ray{o, {0, 0, 1}});
}

perspective_camera::perspective_camera(vec2f resolution, Float fov,
                                       vec3f position, vec3f towards,
                                       vec3f up) {
    fov *= π / 180;
    camera_to_world = transform::look_at(position, towards, up);
    if (resolution.x >= resolution.y) {
        auto aspect_ratio = resolution.x / resolution.y;
        raster_to_screen = transform::scale(
            {aspect_ratio / resolution.x, 1 / resolution.y, 1});
        raster_to_screen =
            transform::translate(
                vec3f{(Float)-0.5 * aspect_ratio, (Float)-0.5, 1})
                .compose(raster_to_screen);
    }
    else {
        auto aspect_ratio = resolution.y / resolution.x;
        raster_to_screen = transform::scale(
            {1 / resolution.x, aspect_ratio / resolution.y, 1});
        raster_to_screen =
            transform::translate(
                vec3f{(Float)-0.5, (Float)-0.5 * aspect_ratio, 1})
                .compose(raster_to_screen);
    }
    raster_to_screen =
        transform::scale({fov, fov, 1}).compose(raster_to_screen);
    screen_to_camera = transform::identity();
    raster_to_camera = raster_to_screen;
}

ray perspective_camera::point(vec2f px) const {
    auto d = raster_to_camera.point({px.x, px.y, 0});
    return camera_to_world(ray{vec3f{0, 0, 0}, d.normalized()});
}

struct scatter_sample {
    light value{0};
    vec3f direction;
    Float probability{};
    bool specular{false};
    light next_medium_refraction{1};

    operator bool() const { return 0 < probability; }
};

vec3f shading_frame::to_local(vec3f v) const {
    auto surface_normal = this->normal;
    auto bz = surface_normal;
    auto bx = dpdu.normalized();
    auto by = bz.cross(bx);
    return {v.dot(bx), v.dot(by), v.dot(bz)};
}

vec3f shading_frame::to_world(vec3f v) const {
    auto surface_normal = this->normal;
    auto bz = surface_normal;
    auto bx = dpdu.normalized();
    auto by = bz.cross(bx);
    return bx * v.x + by * v.y + bz * v.z;
}

scatter_sample scatter(const scene&, sample_sequence*, const material&,
                       const intersection&, const vec3f& towards);
scatter_sample scatter(const scene&, const material&, const intersection&,
                       const vec3f& towards, const vec3f& from);
scatter_sample scatter(const scene&, sample_sequence*, const intersection&,
                       const vec3f& towards);
scatter_sample scatter(const scene&, const intersection&, const vec3f& towards,
                       const vec3f& from);

scatter_sample scatter(const scene&, sample_sequence* rng,
                       const lambertian& matte, const intersection& isect,
                       const vec3f& towards) {
    auto in = isect.shading.to_local(towards);
    auto away = sample_cosine_hemisphere(rng->sample_2d());
    if (in.z < 0)
        away.z *= -1;

    return scatter_sample{
        .value = sample_texture_light(matte.reflectance, isect) / π,
        .direction = isect.shading.to_world(away),
        .probability = std::abs(away.z) / π,
    };
}

scatter_sample scatter(const scene&, const lambertian& matte,
                       const intersection& isect, const vec3f& towards,
                       const vec3f& away) {
    if (not same_shading_hemisphere(isect.shading.to_local(towards),
                                    isect.shading.to_local(away)))
        return {};

    auto angle_cos = isect.shading.to_local(away).z;
    return scatter_sample{
        .value = sample_texture_light(matte.reflectance, isect) / π,
        .direction = away,
        .probability = std::abs(angle_cos) / π,
    };
}

scatter_sample scatter(const scene&, sample_sequence* rng,
                       const translucent& mat, const intersection& isect,
                       const vec3f& towards) {
    auto in = isect.shading.to_local(towards);
    auto away = sample_cosine_hemisphere(rng->sample_2d());
    if (in.z < 0)
        away.z *= -1;

    auto T = sample_texture_1d(mat.transmission, isect);
    auto R = 1 - T;
    if (rng->sample_1d() < R) {
        return scatter_sample{
            .value = sample_texture_light(mat.reflectance, isect) * R / π,
            .direction = isect.shading.to_world(away),
            .probability = std::abs(away.z) * R / π,
        };
    }
    else {
        away.z *= -1;
        return scatter_sample{
            .value = sample_texture_light(mat.reflectance, isect) * T / π,
            .direction = isect.shading.to_world(away),
            .probability = std::abs(away.z) * T / π,
        };
    }
}

scatter_sample scatter(const scene&, const translucent& mat,
                       const intersection& isect, const vec3f& towards,
                       const vec3f& away) {
    auto angle_cos = isect.shading.to_local(away).z;
    auto V = sample_texture_1d(mat.transmission, isect);
    if (same_shading_hemisphere(isect.shading.to_local(towards),
                                isect.shading.to_local(away)))
        V = 1 - V;
    return scatter_sample{
        .value = sample_texture_light(mat.reflectance, isect) * V / π,
        .direction = away,
        .probability = std::abs(angle_cos) * V / π,
    };
}

scatter_sample scatter(const scene& scene, sample_sequence* rng,
                       const emissive&, const intersection& isect,
                       const vec3f& towards) {
    return scatter(scene, rng, lambertian{}, isect, towards);
}

scatter_sample scatter(const scene& scene, const emissive&,
                       const intersection& isect, const vec3f& towards,
                       const vec3f& from) {
    return scatter(scene, lambertian{}, isect, towards, from);
}

scatter_sample blinn_phong_forward(const blinn_phong& material,
                                   const intersection& isect,
                                   const vec3f& towards, const vec3f& away) {
    auto angle_cos = away.z;
    auto probability = std::abs(angle_cos) / π;

    auto roughness = material.diffuse;
    auto glossiness = (Float)material.sharpness;
    light reflectance{sample_texture_light(material.diffuse_color, isect)};

    auto diffuse = reflectance * roughness * (1 / π);

    auto halfv = (away + isect.shading.to_local(towards)).normalized();
    auto specular_intensity = std::pow(std::max<Float>(halfv.z, 0), glossiness);

    auto energy_conservation = [](Float n) {
        return ((n + 2) * (n + 4)) / (8 * π * (std::pow<Float>(2, -n / 2) + 2));
    }(glossiness);

    auto specular = sample_texture_light(material.specular_color, isect) *
                    energy_conservation * specular_intensity * (1 - roughness);

    auto value = (specular + diffuse);

    return scatter_sample{
        .value = value,
        .direction = isect.shading.to_world(away),
        .probability = probability,
    };
}

scatter_sample scatter(const scene&, const blinn_phong& material,
                       const intersection& isect, const vec3f& towards,
                       const vec3f& away) {
    return blinn_phong_forward(material, isect, towards,
                               isect.shading.to_local(away));
}

scatter_sample scatter(const scene&, sample_sequence* rng,
                       const blinn_phong& material, const intersection& isect,
                       const vec3f& towards) {
    auto away = sample_cosine_hemisphere(rng->sample_2d());
    return blinn_phong_forward(material, isect, towards, away);
}

auto fresnel_conductor(Float cos_t, light etai, light etat, light k) {
    cos_t = std::clamp<Float>(cos_t, -1, 1);
    auto leaving_medium = cos_t < 0;
    if (leaving_medium) {
        std::swap(etai, etat);
        cos_t = std::abs(cos_t);
    }

    light Eta = etat / etai;
    light Etak = k / etai;

    float CosTheta2 = cos_t * cos_t;
    float SinTheta2 = 1 - CosTheta2;
    light Eta2 = Eta * Eta;
    light Etak2 = Etak * Etak;

    light t0 = Eta2 - Etak2 - vec3f{SinTheta2};
    light a2plusb2 = (t0 * t0 + Eta2 * Etak2 * 4);
    a2plusb2 = {std::sqrt(a2plusb2.x), std::sqrt(a2plusb2.y),
                std::sqrt(a2plusb2.z)};



    light t1 = a2plusb2 + light{CosTheta2};
    light a = (a2plusb2 + t0) * (Float)0.5;
    a = {std::sqrt(a.x), std::sqrt(a.y), std::sqrt(a.z)};
    light t2 = a * 2 * cos_t;
    light Rs = (t1 - t2) / (t1 + t2);

    light t3 = a2plusb2 * CosTheta2 + vec3f{SinTheta2 * SinTheta2};
    light t4 = t2 * SinTheta2;
    light Rp = Rs * (t3 - t4) / (t3 + t4);

    return (Rp + Rs) * (Float)0.5;
}

std::optional<vec3f> transmission_direction(const vec3f& in, vec3f normal,
                                            Float refraction_ratio) {
    auto cos_in = in.dot(normal);
    if (cos_in < 0) {
        cos_in *= -1;
        normal *= -1;
    }

    auto sin2_in = std::max<Float>(0, (1 - cos_in * cos_in));
    auto eta2 = refraction_ratio * refraction_ratio;
    auto sin2_out = sin2_in * eta2;
    if (sin2_out >= 1)
        return {};
    auto cos_out = std::sqrt(1 - sin2_out);
    return in * -1 * refraction_ratio +
           normal * (refraction_ratio * cos_in - cos_out);
}

scatter_sample scatter(const scene&, sample_sequence* rng,
                       const specular& material, const intersection& isect,
                       const vec3f& towards) {
    auto in = isect.shading.to_local(towards);

    auto air_refraction = light{1};
    auto internal_refraction = sample_texture_light(material.refraction, isect);
    auto absorption = sample_texture_light(material.absorption, isect);
    auto transmission_rate = sample_texture_light(material.transmission, isect);

    auto Fr = fresnel_conductor(in.z, air_refraction, internal_refraction,
                                absorption);
    auto F = (Fr.x + Fr.y + Fr.z) / 3;

    auto non_transmissive = transmission_rate.length_sq() == 0;

    if (rng->sample_1d() < F or non_transmissive) {
        auto away = vec3f{-in.x, -in.y, in.z};
        return scatter_sample{.value = Fr / std::abs(away.z),
                              .direction = isect.shading.to_world(away),
                              .probability = non_transmissive ? 1 : F,
                              .specular = true};
    }

    auto entering = in.z > 0;
    auto refraction_from = entering ? air_refraction : internal_refraction;
    auto refraction_to = entering ? internal_refraction : air_refraction;
    auto refraction_ratio = refraction_from.length() / refraction_to.length();

    auto away = transmission_direction(
        in, vec3f{0, 0, (Float)(entering ? 1 : -1)}, refraction_ratio);
    if (not away)
        return {.probability = {}, .specular = true};

    auto transmission = (light{1} - Fr) * transmission_rate;
    transmission *= refraction_ratio * refraction_ratio;

    return scatter_sample{.value = transmission / std::abs(away->z),
                          .direction = isect.shading.to_world(*away),
                          .probability = 1 - F,
                          .specular = true,
                          .next_medium_refraction = refraction_to};
}

scatter_sample scatter(const scene&, const specular&, const intersection&,
                       const vec3f&, const vec3f& away) {
    return scatter_sample{
        .value = light{0},
        .direction = away,
        .probability = 0,
    };
}

auto norm_tan2(Float a) { return std::max<Float>(0, 1 - a * a) / (a * a); }

Float trowbridge_reitz_microfacet_area(Float roughness, const vec3f& a) {
    auto t2 = norm_tan2(a.z);
    if (std::isinf(t2))
        return 0;
    auto a2 = roughness * roughness;
    auto cos_a2 = a.z * a.z;
    auto cos_a4 = cos_a2 * cos_a2;
    auto a2t2 = a2 + t2;
    return a2 / (π * cos_a4 * a2t2 * a2t2);
}

Float trowbridge_reitz_microfacet_G1(Float roughness, const vec3f& a) {
    auto t2 = norm_tan2(a.z);
    if (std::isinf(t2))
        return 0;
    auto a2 = roughness * roughness;
    auto D = std::sqrt(std::max<Float>(0, (1 + a2 * t2)));
    return 2 / (1 + D);
}

Float trowbridge_reitz_microfacet_lambda(Float roughness, const vec3f& a) {
    return (1 / trowbridge_reitz_microfacet_G1(roughness, a)) - 1;
}

Float trowbridge_reitz_microfacet_geometry(Float roughness, const vec3f& a,
                                           const vec3f& b) {
    auto A = trowbridge_reitz_microfacet_lambda(roughness, a);
    auto B = trowbridge_reitz_microfacet_lambda(roughness, b);
    return 1 / (1 + A + B);
}

// heitz 2018
vec3f trowbridge_reitz_sample(sample_sequence* rng, Float alpha, vec3f v) {
    auto vh = vec3f{alpha * v.x, alpha * v.y, v.z}.normalized();
    if (vh.z < 0)
        vh *= -1;

    auto T1 = vh.z < (Float)0.99999 ? vec3f{0, 0, 1}.cross(vh).normalized()
                                    : vec3f{1, 0, 0};
    auto T2 = vh.cross(T1);

    auto [t1, t2] = sample_uniform_disk(rng->sample_2d());
    auto s = (1 + vh.z) / 2;
    auto h = std::sqrt(1 - t1 * t1);
    t2 = (1 - s) * h + s * t2;

    auto Nh = T1 * t1 + T2 * t2 +
              vh * std::sqrt(std::max<Float>(0, 1 - t1 * t1 - t2 * t2));
    auto Ne = vec3f{alpha * Nh.x, alpha * Nh.y, std::max<Float>(ɛ, Nh.z)};
    return Ne.normalized();
}

Float trowbridge_reitz_sample_pdf(Float alpha, vec3f n, vec3f v) {
    return trowbridge_reitz_microfacet_G1(alpha, v) *
           trowbridge_reitz_microfacet_area(alpha, n) * std::abs(v.dot(n)) /
           std::abs(v.z);
}

Float beckmann_microfacet_area(Float roughness, Float a) {
    auto t = std::tan(a);
    if (std::isinf(t))
        return 0;
    auto r = roughness * roughness;
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

Float glossy_reflection(const vec3f& towards, const vec3f& away,
                        const vec3f& normal, Float roughness) {
    auto D = trowbridge_reitz_microfacet_area(roughness, normal);
    auto G = trowbridge_reitz_microfacet_geometry(roughness, away, towards);
    return D * G / std::abs(4 * away.z * towards.z);
}

Float glossy_transmission(const vec3f& towards, const vec3f& away,
                          const vec3f& normal, Float roughness, Float eta) {
    auto D = trowbridge_reitz_microfacet_area(roughness, normal);
    auto G = trowbridge_reitz_microfacet_geometry(roughness, away, towards);
    auto denom = away.dot(normal) + towards.dot(normal) / eta;
    return (D * G / (denom * denom)) *
           std::abs((towards.dot(normal) * away.dot(normal)) /
                    (towards.z * away.z));
}

scatter_sample scatter(const scene& scene, sample_sequence* rng,
                       const glossy& material, const intersection& isect,
                       const vec3f& towards) {
    auto roughness = sample_texture_1d(material.roughness, isect);
    if (roughness <= 0.0001) {
        auto basically_specular =
            specular{.refraction = material.refraction,
                     .absorption = material.absorption,
                     .transmission = material.transmission};
        return scatter(scene, rng, basically_specular, isect, towards);
    }

    auto air_refraction = light{1};
    auto internal_refraction = sample_texture_light(material.refraction, isect);
    auto absorption = sample_texture_light(material.absorption, isect);
    auto transmission_rate = sample_texture_1d(material.transmission, isect);
    auto non_transmissive = not transmission_rate;

    auto in = isect.shading.to_local(towards).normalized();
    auto microfacet = trowbridge_reitz_sample(rng, roughness, in);
    auto microfacet_probability =
        trowbridge_reitz_sample_pdf(roughness, microfacet, in);

    auto cos_θ = in.dot(microfacet);
    auto reflectivity = fresnel_conductor(cos_θ, air_refraction,
                                          internal_refraction, absorption);
    auto transmittance = light{1} - reflectivity;

    if (rng->sample_1d() < reflectivity.mean() or non_transmissive) {
        auto reflection = (in * -1) + microfacet * 2 * cos_θ;
        auto f = glossy_reflection(in, reflection, microfacet, roughness);
        auto probability = microfacet_probability / (4 * std::abs(cos_θ));
        if (not non_transmissive)
            probability *= reflectivity.mean();
        return scatter_sample{.value = reflectivity * f,
                              .direction = isect.shading.to_world(reflection),
                              .probability = probability};
    }

    auto forward_eta = air_refraction.mean() / internal_refraction.mean();
    auto backwards_eta = 1 / forward_eta;
    if (cos_θ < 0)
        std::swap(forward_eta, backwards_eta);

    auto refraction = transmission_direction(in, microfacet, forward_eta);

    if (not refraction or same_shading_hemisphere(*refraction, in))
        return {.probability = {}};
    auto away = refraction->normalized();

    auto f =
        glossy_transmission(in, away, microfacet, roughness, backwards_eta);
    f /= backwards_eta * backwards_eta;
    auto transmission = transmittance * f;

    auto pdf_denom = away.dot(microfacet) + in.dot(microfacet) / backwards_eta;
    auto probability = transmittance.mean() * microfacet_probability *
                       std::abs(away.dot(microfacet)) / (pdf_denom * pdf_denom);

    return scatter_sample{.value = transmission,
                          .direction = isect.shading.to_world(away),
                          .probability = probability,
                          .next_medium_refraction = internal_refraction};
}

scatter_sample scatter(const scene&, const glossy& material,
                       const intersection& isect, const vec3f& towards,
                       const vec3f& from) {
    auto roughness = sample_texture_1d(material.roughness, isect);
    if (roughness <= 0.0001)
        return scatter_sample{
            .value = light{0},
            .direction = from,
            .probability = 0,
        };

    auto air_refraction = light{1};
    auto internal_refraction = sample_texture_light(material.refraction, isect);
    auto absorption = sample_texture_light(material.absorption, isect);
    auto transmission_rate = sample_texture_1d(material.transmission, isect);
    bool transmissive = transmission_rate;

    auto in = isect.shading.to_local(towards).normalized();
    auto away = isect.shading.to_local(from).normalized();
    auto is_reflection = same_shading_hemisphere(in, away);

    auto backwards_eta = air_refraction.mean();
    if (not is_reflection)
        backwards_eta =
            (in.z > 0 ? internal_refraction.mean() / air_refraction.mean()
                      : air_refraction.mean() / internal_refraction.mean());
    // auto forward_eta = 1 / backwards_eta;

    auto microfacet = (away * backwards_eta + in).normalized();
    if (microfacet.z < 0)
        microfacet *= -1;

    if (microfacet.dot(away) * away.z < 0 or microfacet.dot(in) * in.z < 0)
        return {};

    auto microfacet_probability =
        trowbridge_reitz_sample_pdf(roughness, microfacet, in);
    auto cos_θ = in.dot(microfacet);
    auto reflectivity = fresnel_conductor(cos_θ, air_refraction,
                                          internal_refraction, absorption);

    if (is_reflection) {
        auto f = glossy_reflection(in, away, microfacet, roughness);
        auto probability = microfacet_probability *
                           (transmissive ? reflectivity.mean() : 1) /
                           (4 * std::abs(cos_θ));
        return scatter_sample{.value = reflectivity * f,
                              .direction = from,
                              .probability = probability};
    }

    auto f =
        glossy_transmission(in, away, microfacet, roughness, backwards_eta);
    f /= backwards_eta * backwards_eta;
    auto transmittance = vec3f{1} - reflectivity;
    auto transmission = transmittance * f;

    auto pdf_denom = away.dot(microfacet) + in.dot(microfacet) / backwards_eta;

    auto probability = transmittance.mean() * microfacet_probability *
                       std::abs(away.dot(microfacet)) / (pdf_denom * pdf_denom);

    return scatter_sample{.value = vec3f{transmission},
                          .direction = from,
                          .probability = probability};
}

scatter_sample scatter(const scene& scene, sample_sequence* rng,
                       const coated& mat, const intersection& isect,
                       vec3f towards) {
    auto skip_coating =
        rng->sample_1d() < (1 - sample_texture_1d(mat.weight, isect));
    if (skip_coating)
        return scatter(scene, rng, *mat.base, isect, towards);

    auto seed = rng->sample_2d();
    auto local_rng = rng_sampler(
        vec2<i32>{i32(seed.x * (1 << 16)), i32(seed.y * (1 << 16))}, {});
    auto ss = local_rng->nth_sample({});
    auto coat_rng = ss.get();

    auto outgoing = isect.shading.to_local(towards);
    auto inside = outgoing.z < 0;

    if (inside) {
        outgoing.z *= -1;
        towards = isect.shading.to_world(outgoing);
    }

    auto direction = towards;
    scatter_sample sample{
        .value = light{1},
        .direction = towards,
        .probability = 1,
        .specular = true,
    };
    while (sample.probability) {
        auto first = scatter(scene, coat_rng, *mat.coat, isect, direction);
        auto coat_refraction = isect.shading.to_local(first.direction);
        if (coat_refraction.z >= 0) {
            sample.direction = first.direction;
            sample.value *= first.value;
            sample.probability *= first.probability;
            sample.specular = sample.specular and first.specular;
            break;
        }

        auto second =
            scatter(scene, coat_rng, *mat.base, isect, first.direction * -1);
        auto base_refraction = isect.shading.to_local(second.direction);
        if (base_refraction.z < 0) {
            // unhandled transmission
            sample = {.direction = second.direction};
            break;
        }

        direction = second.direction * -1;
        sample.value *= first.value * second.value *
                        std::abs(coat_refraction.z) *
                        std::abs(base_refraction.z);
        sample.probability *= first.probability * second.probability;
        sample.specular = false;
    }

    if (inside) {
        direction -= isect.normal * 2 * isect.normal.dot(direction);
    }

    return sample;
}

scatter_sample scatter(const scene& scene, const coated& mat,
                       const intersection& isect, const vec3f& towards,
                       const vec3f& from) {
    auto local_rng = rng_sampler(
        vec2<i32>{i32(towards.x * (1 << 16)),
                  i32(towards.y * (1 << 16) + towards.z * (1 << 16))},
        {});
    auto ss = local_rng->nth_sample({});
    auto rng = ss.get();

    auto skip_coating =
        rng->sample_1d() < (1 - sample_texture_1d(mat.weight, isect));
    if (skip_coating)
        return scatter(scene, *mat.base, isect, towards, from);

    auto in = isect.shading.to_local(towards).normalized();
    auto away = isect.shading.to_local(from).normalized();
    if (not same_shading_hemisphere(in, away))
        return {};

    // Position-Free Monte Carlo Simulation for Arbitrary Layered BSDFs
    // https://shuangz.com/projects/layered-sa18/

    auto forward = scatter(scene, rng, *mat.coat, isect, towards);
    auto v_forward = isect.shading.to_local(forward.direction);
    if (not forward or v_forward.z > 0)
        return {};

    auto reverse = scatter(scene, rng, *mat.coat, isect, from);
    auto v_reverse = isect.shading.to_local(reverse.direction);
    if (forward.specular and (not reverse or v_reverse.z > 0))
        return {};
    // veach(1996) symmetry adjustment
    reverse.value *=
        reverse.next_medium_refraction * reverse.next_medium_refraction;

    auto coat_reflection = scatter(scene, *mat.coat, isect, towards, from);
    light L = coat_reflection.value;

    Float approx_pdf =
        (reverse and v_reverse.z < 0)
            ? scatter(scene, *mat.base, isect, forward.direction * -1,
                      reverse.direction * -1)
                  .probability
            : 0;
    if (not forward.specular) {
        approx_pdf *= power_heuristic(reverse.probability, approx_pdf);
        auto reflect =
            scatter(scene, rng, *mat.base, isect, forward.direction * -1);
        if (reflect) {
            auto exit =
                scatter(scene, *mat.coat, isect, reflect.direction * -1, from);
            approx_pdf +=
                exit.probability *
                power_heuristic(reflect.probability, exit.probability);
        }

        approx_pdf += coat_reflection.probability;
    }
    approx_pdf = (Float(0.1) * (1 / (4 * π))) + (Float(0.9) * approx_pdf);

    enum class side { coat, base } layer{side::base};
    auto direction{forward.direction};
    auto beta{forward.value * std::abs(v_forward.z) / forward.probability};

    for (int depth{0}; depth < 5; ++depth) {
        if (layer == side::base) {
            if (reverse and v_reverse.z < 0) {
                auto reflect = scatter(scene, *mat.base, isect, direction * -1,
                                       reverse.direction * -1);
                Float weight = reverse.specular
                                   ? 1
                                   : power_heuristic(reverse.probability,
                                                     reflect.probability);
                L += beta * reflect.value * std::abs(v_reverse.z) *
                     reverse.value * weight / reverse.probability;
            }

            auto next = scatter(scene, rng, *mat.base, isect, direction * -1);
            auto v_next = isect.shading.to_local(next.direction);
            if (not next or v_next.z < 0)
                break;

            direction = next.direction;
            beta *= next.value * std::abs(v_next.z) / next.probability;
            layer = side::coat;

            if (not forward.specular) {
                auto exit =
                    scatter(scene, *mat.coat, isect, direction * -1, from);
                if (exit) {
                    L += beta * exit.value *
                         power_heuristic(next.probability, exit.probability);
                }
            }
        }

        else {
            auto next = scatter(scene, rng, *mat.coat, isect, direction * -1);
            auto v_next = isect.shading.to_local(next.direction);
            if (not next or v_next.z >= 0)
                break;
            direction = next.direction;
            beta *= next.value * std::abs(v_next.z) / next.probability;
            layer = side::base;
        }
    }

    return scatter_sample{
        .value = L, .direction = from, .probability = approx_pdf};
}

scatter_sample scatter(const scene& scene, sample_sequence* rng,
                       const alpha& mat, const intersection& isect,
                       const vec3f& towards) {
    auto skip = (0 == sample_texture_1d(mat.map, isect));
    if (skip)
        return scatter_sample{.value = light{1},
                              .direction = towards * -1,
                              .probability = 1,
                              .specular = true};

    return scatter(scene, rng, *mat.base, isect, towards);
}

scatter_sample scatter(const scene& scene, const alpha& mat,
                       const intersection& isect, const vec3f& towards,
                       const vec3f& away) {
    auto skip = (0 == sample_texture_1d(mat.map, isect));
    if (skip)
        return {};
    return scatter(scene, *mat.base, isect, towards, away);
}

scatter_sample scatter(const scene& scene, sample_sequence* rng,
                       const material& material, const intersection& isect,
                       const vec3f& towards) {
    return std::visit(
        [&](const auto& material) {
            return scatter(scene, rng, material, isect, towards);
        },
        material);
}

scatter_sample scatter(const scene& scene, const material& material,
                       const intersection& isect, const vec3f& towards,
                       const vec3f& from) {
    return std::visit(
        [&](const auto& material) {
            return scatter(scene, material, isect, towards, from);
        },
        material);
}

scatter_sample scatter(const scene& scene, const intersection& isect,
                       const vec3f& towards, const vec3f& from) {
    if (isect.mat)
        return scatter(scene, *isect.mat, isect, towards, from);
    else
        return scatter(scene, lambertian{}, isect, towards, from);
}

scatter_sample scatter(const scene& scene, sample_sequence* rng,
                       const intersection& isect, const vec3f& towards) {
    if (isect.mat)
        return scatter(scene, rng, *isect.mat, isect, towards);
    else
        return scatter(scene, rng, lambertian{}, isect, towards);
}

light debug_trace(const scene& scene, sample_sequence*, ray r, int) {
    auto intersection =
        intersect(scene.root, r, std::numeric_limits<Float>::max());
    if (not intersection)
        return {};
    auto ncos = intersection->normal.dot(r.direction);
    auto norm_l = 1 - (ncos + 1) / 2;
    auto d = 1 - (std::cos(intersection->distance * 4) + 1) / 2;
    return {(norm_l + d) / 2, (norm_l + d) / 2, (norm_l + d) / 2};
}

ray offset_origin(const intersection& isect, ray r) {
    bool transmission = not same_normal_hemisphere(isect.normal, r.direction);
    r.origin += isect.normal * 0.00004 * (transmission ? -1 : 1);
    return r;
}

light naive_trace(const scene& scene, sample_sequence* rng, ray r, int depth) {
    auto intersection =
        intersect(scene.root, r, std::numeric_limits<Float>::max());
    if (not intersection)
        return {scene.film.global_radiance};

    if (intersection->mat and is_emissive(intersection->mat))
        return emitted_light(*intersection);

    if (0 == depth)
        return {};

    auto inorm = intersection->normal;
    auto scatter_d = sample_uniform_direction(rng->sample_2d()).normalized();
    auto icos = std::abs(inorm.dot(scatter_d));
    auto pdf = 1 / (4 * π);
    auto f = scatter(scene, *intersection, r.direction * -1, scatter_d).value *
             icos / pdf;

    if (f.length_sq() <= 0)
        return {};

    auto reflection_point = r.distance(intersection->distance);
    auto scatter_ray =
        offset_origin(*intersection, ray{reflection_point, scatter_d});

    auto Li = naive_trace(scene, rng, scatter_ray, depth - 1);
    return Li * f;
}

light scatter_trace(const scene& scene, sample_sequence* rng, ray r,
                    int max_depth) {
    light beta{1};
    light L{};
    int depth{0};

    while (true) {
        auto intersection =
            intersect(scene.root, r, std::numeric_limits<Float>::max());
        if (not intersection) {
            L += light{scene.film.global_radiance} * beta;
            break;
        }

        if (intersection->mat and is_emissive(intersection->mat)) {
            L += beta * emitted_light(*intersection);
        }

        if (max_depth <= depth)
            return L;
        rng->skip_to(2 + depth * 2);
        depth++;

        auto scattering = scatter(scene, rng, *intersection, r.direction * -1);
        if (scattering.value.length_sq() <= 0)
            return L;

        auto icos = scattering.direction.dot(intersection->normal);
        auto reflectance =
            (scattering.value * std::abs(icos)) / scattering.probability;

        auto reflection_point = r.distance(intersection->distance);
        auto scatter_ray = offset_origin(
            *intersection, ray{reflection_point, scattering.direction});

        r = scatter_ray;
        beta *= reflectance;
    }

    return L;
}

std::pair<vec3f, Float>
sample_sphere(vec2f sample, const bounding_sphere& sphere, vec3f point) {
    auto disk_normal = (point - sphere.centre).normalized();
    auto disk_radius = sphere.radius;
    auto disk_centre = sphere.centre + disk_normal * disk_radius;

    auto t0 = vec3f{0, disk_normal.z, -disk_normal.y}.normalized();
    // auto t0 = vec3f{-disk_normal.z, 0, disk_normal.x}.normalized();
    // auto t0 = vec3f{-disk_normal.y, disk_normal.x, 0}.normalized();
    auto t1 = disk_normal.cross(t0).normalized();

    auto disk_edge = (disk_centre + t0 * disk_radius - point);
    auto cone_cos = (disk_normal * -1).dot(disk_edge) / disk_edge.length();
    auto direction =
        sample_uniform_cone(sample, cone_cos, t0, t1, disk_normal * -1)
            .normalized();
    auto probability = pdf_uniform_cone(cone_cos);
    return {direction, probability};
}

scatter_sample sample_light(const scene& scene,
                            const intersection& intersection,
                            const distant_light& source, const ray& r, vec2f) {
    auto intersection_point = r.distance(intersection.distance);
    auto towards = source.towards.normalized();
    auto visibility_check = offset_origin(
        intersection, ray{.origin = intersection_point, .direction = towards});
    auto occluded = intersect(scene.root, visibility_check,
                              std::numeric_limits<Float>::max());
    if (occluded)
        return {.probability = 0};

    return scatter_sample{
        .value = source.value,
        .direction = towards,
        .probability = 1.,
        .specular = true,
    };
}

Float light_contribution(const scene&, const distant_light& source, vec3f) {
    return source.value.length_sq();
}

scatter_sample sample_light(const scene& scene,
                            const intersection& intersection,
                            const node_light& source, const ray& r,
                            vec2f sample) {
    auto& [light, bounds] = source;
    auto intersection_point = r.distance(intersection.distance);
    auto [direction, sample_pdf] =
        sample_sphere(sample, bounds, intersection_point);

    auto probability = 1 / sample_pdf;

    auto light_ray =
        offset_origin(intersection, ray{.origin = intersection_point,
                                        .direction = direction});

    auto visible =
        intersect(source.node, light_ray, std::numeric_limits<Float>::max());
    if (not visible)
        return {.probability = probability};

    auto occluded = intersect(scene.root, light_ray, visible->distance);
    if (occluded and occluded->distance < visible->distance)
        return {.probability = probability};

    auto L = emitted_light(*visible);
    return scatter_sample{
        .value = L,
        .direction = light_ray.direction,
        .probability = probability,
    };
}

Float light_contribution(const scene&, const node_light& source, vec3f at) {
    auto r = source.bounds.radius;
    auto d = (source.bounds.centre - at).length();
    auto L = emitted_light({{}, {}, {}, {}, source.node.n.material});
    return L.length_sq() * r / d;
}

scatter_sample sample_light(const scene& scene, const intersection& isect,
                            const light_source& l, const ray& r, vec2f sample) {
    return std::visit(
        [&](const auto& l) { return sample_light(scene, isect, l, r, sample); },
        l);
}

Float light_contribution(const scene& scene, const light_source& l, vec3f at) {
    return std::visit(
        [&](const auto& l) { return light_contribution(scene, l, at); }, l);
}

std::pair<scatter_sample, std::vector<Float>>
sample_direct_lighting_weighted(const scene& scene, sample_sequence* rng,
                                const intersection& intersection,
                                const ray& ray) {
    if (scene.lights.empty())
        return {};

    std::vector<Float> weights;
    Float total_weight{};
    auto at = ray.distance(intersection.distance);
    for (auto& source : scene.lights) {
        auto w = light_contribution(scene, source, at);
        total_weight += w;
        weights.push_back(w);
    }
    for (auto& w : weights) w /= total_weight;

    auto roll = rng->sample_1d();
    size_t n{};
    for (; n < weights.size() - 1; ++n) {
        roll -= weights[n];
        if (0 >= roll)
            break;
    }

    auto pdf = weights[n];
    auto light_sample = sample_light(scene, intersection, scene.lights[n], ray,
                                     rng->sample_2d());
    auto probability = pdf * light_sample.probability;
    auto towards = ray.direction * -1;
    auto reflected =
        scatter(scene, intersection, towards, light_sample.direction);

    auto L = light_sample.value;
    if (not light_sample.specular) {
        L *= power_heuristic(probability, reflected.probability) / probability;
        if (not reflected.probability)
            return {{.probability = probability}, weights};
    }
    auto icos = (light_sample.direction).dot(intersection.normal);
    L *= reflected.value * std::abs(icos);

    return {scatter_sample{
                .value = L,
                .direction = light_sample.direction,
                .probability = probability,
                .specular = light_sample.specular,
            },
            weights};
}

std::pair<scatter_sample, std::vector<Float>>
sample_direct_lighting_uniform(const scene& scene, sample_sequence* rng,
                               const intersection& intersection,
                               const ray& ray) {
    if (scene.lights.empty())
        return {};

    std::vector<Float> weights;
    for (size_t i{}; i < scene.lights.size(); ++i)
        weights.push_back(1. / scene.lights.size());
    auto one_light =
        std::clamp<int>((int)(rng->sample_1d() * scene.lights.size()), 0,
                        scene.lights.size() - 1);
    auto pdf = 1.0f / scene.lights.size();

    auto light_sample = sample_light(
        scene, intersection, scene.lights[one_light], ray, rng->sample_2d());
    auto probability = pdf * light_sample.probability;

    auto towards = ray.direction * -1;
    auto reflected =
        scatter(scene, intersection, towards, light_sample.direction);
    if (not reflected.probability)
        return {{.probability = probability}, weights};

    auto icos = (light_sample.direction).dot(intersection.normal);
    auto L = light_sample.value;
    L *= reflected.value * std::abs(icos);
    L *= power_heuristic(probability, reflected.probability) / probability;

    return {{
                .value = L,
                .direction = light_sample.direction,
                .probability = probability,
            },
            weights};
}

light path_trace(const scene& scene, sample_sequence* rng, ray r,
                 int max_depth) {
    light beta{1};
    light L{};
    int depth{0};

    ray prev_ray{r};
    std::optional<intersection> previous_intersection;
    Float scattering_pdf{-1};
    std::vector<Float> light_sampler_pdfs;

    while (true) {
        auto intersection =
            intersect(scene.root, r, std::numeric_limits<Float>::max());
        if (not intersection) {
            L += beta * light{scene.film.global_radiance};
            break;
        }

        if (intersection->mat and is_emissive(intersection->mat)) {
            auto Le = emitted_light(*intersection);
            if (scattering_pdf < 0)
                L += beta * Le;
            else {
                for (size_t li{}; li < scene.lights.size(); ++li) {
                    auto& light = scene.lights[li];
                    auto area_light = std::get_if<node_light>(&light);
                    if (not area_light)
                        continue;
                    auto d = intersect(area_light->node, r,
                                       std::numeric_limits<Float>::infinity());
                    if (d and d->distance == intersection->distance) {
                        auto light_pdf =
                            sample_light(scene, *previous_intersection, light,
                                         prev_ray, {})
                                .probability *
                            light_sampler_pdfs[li];
                        Le *= power_heuristic(scattering_pdf, light_pdf);
                        L += beta * Le;
                        break;
                    }
                }
            }
        }

        if (max_depth <= depth)
            break;

        rng->skip_to(2 + 0 + depth * 7);
        auto [light, l_weights] =
            sample_direct_lighting_weighted(scene, rng, *intersection, r);
        light_sampler_pdfs = l_weights;
        if (0 < light.value.length_sq()) {
            L += beta * light.value;
        }
        rng->skip_to(2 + 3 + depth * 7);

        auto intersection_point = r.distance(intersection->distance);
        auto scattering = scatter(scene, rng, *intersection, r.direction * -1);
        rng->skip_to(2 + 6 + depth * 7);

        auto icos = scattering.direction.dot(intersection->normal);
        auto reflected =
            (scattering.value * std::abs(icos)) / scattering.probability;

        beta *= reflected;

        if (beta.length_sq() <= 0)
            break;
        if (std::isnan(beta.y))
            break;

        auto scatter_ray = offset_origin(
            *intersection, ray{intersection_point, scattering.direction});

        previous_intersection = intersection;
        auto bsdf_pdf = scatter(scene, *intersection, r.direction * -1,
                                scattering.direction)
                            .probability;

        scattering_pdf = scattering.specular ? -1 : bsdf_pdf;
        // scattering_pdf = scattering.specular ? -1 : scattering.probability;

        prev_ray = r;
        r = scatter_ray;
        depth++;

        auto best = std::max(beta.x, std::max(beta.y, beta.z));
        if (depth < 2 or best > 0.06)
            continue;
        auto q = std::clamp<Float>(1 - best, 0, 0.8);
        if (rng->sample_1d() < q)
            break;
        beta /= (1 - q);
    }

    return L;
}

vec3f light_to_rgb(light light) {
    return {std::clamp<Float>(gamma_correct(light.x), 0, 1),
            std::clamp<Float>(gamma_correct(light.y), 0, 1),
            std::clamp<Float>(gamma_correct(light.z), 0, 1)};
}

constexpr int ceil(Float f) {
    const int l = static_cast<int>(f);
    return (f > l) ? l + 1 : l;
}

Float gaussian(Float a, Float r, Float x2) {
    constexpr Float inv_pi = 0.318309886184;
    return a * inv_pi *
           std::max<Float>(0, std::exp(-(x2 * a)) - std::exp(-(r * r * a)));
}

struct image {
    image(vec2<int> resolution) : resolution(resolution) {
        light.resize(resolution.x * resolution.y);
        partial_image.resize(3 * resolution.y * resolution.x);
    }

    void set_supersampling(int ss) { supersampling = ss; }

    bool gaussian_filter{true};
    static constexpr Float alpha{3};
    static constexpr Float filter_radius{1.5};
    static constexpr int max_offset{ceil(filter_radius)};

    void add_sample(const vec2f& position, const light& value) {
        auto px =
            vec2<int>{(int)std::floor(position.x), (int)std::floor(position.y)};

        if (not gaussian_filter) {
            auto [x, y] = px;
            if (x < 0 or resolution.x - 1 < x or y < 0 or resolution.y - 1 < y)
                return;
            auto offset = (y * resolution.x + x);
            auto lock = std::lock_guard{update_mutex};
            light[offset] += value;
            auto [r, g, b] = light[offset] / supersampling;
            offset *= 3;
            partial_image[offset + 0] = r;
            partial_image[offset + 1] = g;
            partial_image[offset + 2] = b;
            return;
        }

        auto supersampling = this->supersampling.load();

        constexpr auto tile_size = max_offset * 2 + 1;
        ::light update[tile_size][tile_size]{};
        auto hlines = vec2<int>{std::max(-max_offset, -px.y),
                                std::min(max_offset, resolution.y - px.y)};
        auto vlines = vec2<int>{std::max(-max_offset, -px.x),
                                std::min(max_offset, resolution.x - px.x)};
        for (int yd{hlines.x}; yd < hlines.y; ++yd) {
            for (int xd{vlines.x}; xd < vlines.y; ++xd) {
                int y = px.y + yd;
                int x = px.x + xd;

                for (int c{}; c < 3; ++c) {
                    auto color_offset = vec2f{1 / (Float)3, 0} * (c - 1);
                    auto p =
                        vec2f{x + (Float)0.5, y + (Float)0.5} + color_offset;
                    auto d = (position - p).length_sq();
                    auto f = gaussian(alpha, 2, d);
                    update[yd + max_offset][xd + max_offset][c] += value[c] * f;
                }
            }
        }

        auto lock = std::lock_guard{update_mutex};
        for (int yd{hlines.x}; yd < hlines.y; ++yd) {
            for (int xd{vlines.x}; xd < vlines.y; ++xd) {
                int y = px.y + yd;
                int x = px.x + xd;
                auto offset = (y * resolution.x + x);
                light[offset] += update[yd + max_offset][xd + max_offset];
                auto [r, g, b] = light[offset] / supersampling;
                offset *= 3;
                partial_image[offset + 0] = r;
                partial_image[offset + 1] = g;
                partial_image[offset + 2] = b;
            }
        }
    }

    std::mutex update_mutex{};
    vec2<int> resolution;
    std::atomic<int> supersampling{0};
    std::atomic<bool> finished{false};
    std::vector<light> light;
    std::vector<float> partial_image;
};

void ipc_update_stream(ipc::tev* ipc, image* image, std::string ipc_name) {
    auto resolution = image->resolution;
    constexpr int chunk_size = 8;

    const auto chunks = (int)std::ceil(resolution.y / (float)chunk_size);
    std::vector<int> chunk_status(chunks, 1);
    chunk_status.resize(chunks);
    int current_chunk = 0;

    std::vector<float> update;

    while (true) {
        auto image_sample = image->supersampling.load();
        auto chunk_sample = chunk_status[current_chunk];
        if (image_sample <= chunk_sample) {
            if (image->finished)
                break;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            continue;
        }

        auto first = resolution.x * current_chunk * chunk_size * 3;
        auto last =
            std::min(resolution.x * (current_chunk + 1) * chunk_size * 3,
                     resolution.x * resolution.y * 3);
        auto size = last - first;
        auto lines = size / (resolution.x * 3);

        update.resize(size);
        {
            auto lock = std::lock_guard{image->update_mutex};
            std::memcpy(update.data(), image->partial_image.data() + first,
                        size * sizeof(float));
        }

        ipc->update_rgb_image(ipc_name, 0, current_chunk * chunk_size,
                              resolution.x, lines, update);

        chunk_status[current_chunk++] = image_sample;
        current_chunk %= chunks;
    }
}

int main(int argc, char** argv) {
    auto scene_path = [argc, argv]() -> std::optional<std::string> {
        for (auto i{1}; i < argc; ++i) {
            auto arg = std::string{argv[i]};
            if (arg.substr(0, 1) == "-")
                continue;
            return {arg};
        }
        return {};
    }();

    if (not scene_path) {
        fmt::print("missing scene path\n", *scene_path);
        exit(1);
    }

    auto cmd_arg = [argc,
                    argv](std::string name) -> std::pair<bool, std::string> {
        name = "-" + name;
        for (auto i{1}; i < argc; ++i) {
            auto arg = std::string{argv[i]};
            if (arg == name)
                return {true, ""};
            if (arg.substr(0, name.length() + 1) == name + "=")
                return {true, arg.substr(name.length() + 1)};
        }

        return {false, ""};
    };

    auto t0 = std::chrono::system_clock::now();
    fmt::print("loading scene: {}\n", *scene_path);
    auto scene = load_scene(*scene_path);
    auto resolution = scene.film.resolution;
    image image{resolution};
    auto dt = std::chrono::system_clock::now() - t0;
    fmt::print("[{:.1f}s] scene ready\n", dt.count() / 1000000000.,
               *scene_path);

    auto area_lights = collect_lights(&scene.root);
    for (auto& light : area_lights) scene.lights.push_back(light);
    fmt::print("lights({}) ready\n", scene.lights.size());

    auto debug = cmd_arg("debug").first;

    auto trace = [&scene, debug]() {
        if (debug)
            return debug_trace;
        switch (scene.film.method) {
        case integrator::path: return path_trace;
        case integrator::scatter: return scatter_trace;
        case integrator::brute_force: return naive_trace;
        }
    }();

    auto scene_sampler = [&scene]() {
        switch (scene.film.sampler) {
        case sampler::independent: return independent_sampler;
        case sampler::stratified: return stratified_sampler;
        case sampler::multi_stratified: return multi_stratified_sampler;
        }
    }();

    auto [tev_enabled, tev_host] = cmd_arg("tev");

    auto tev = std::optional<ipc::tev>{};
    auto ipc_worker = std::thread{};
    if (tev_enabled) {
        auto ipc_name =
            *scene_path +
            fmt::format(
                "-{}",
                std::chrono::system_clock::now().time_since_epoch().count() %
                    (1 << 16));
        try {
            if (not tev_host.empty())
                tev.emplace(tev_host);
            else
                tev.emplace();

            tev->create_rgb_image(ipc_name, resolution.x, resolution.y);
            ipc_worker =
                std::thread(ipc_update_stream, &*tev, &image, ipc_name);
        }
        catch (std::exception& err) {
            fmt::print("tev connection failed, image output only ({})\n",
                       err.what());
            tev.reset();
        }
    }

    std::unordered_map<int, std::unique_ptr<pixel_sampler>> samplers;
    for (int y = -1; y <= resolution.y; ++y) {
        for (int x = -1; x <= resolution.x; ++x) {
            auto px = vec2{x, y};
            samplers.insert({y * resolution.x + x,
                             scene_sampler(px, scene.film.supersampling)});
        }
    }
    fmt::print("sampler ready\n");

    t0 = std::chrono::system_clock::now();

    for (int s{}; s < scene.film.supersampling; ++s) {
        image.set_supersampling(s + 1);

#pragma omp parallel for schedule(monotonic : dynamic)
        for (int y = -1; y <= resolution.y; ++y) {
            for (int x = -1; x <= resolution.x; ++x) {

                auto& sampler = samplers.at(y * resolution.x + x);
                auto rng = sampler->nth_sample(s);

                auto px = vec2{x, y};
                auto jitter = rng->sample_2d();
                auto point = vec2f{px} + jitter;

                auto ray = scene.view->point(point);
                auto light = trace(scene, rng.get(), ray, scene.film.depth);

                if (std::isnan(light.x) or std::isinf(light.x) or
                    std::isnan(light.y) or std::isinf(light.y) or
                    std::isnan(light.z) or std::isinf(light.z))
                    continue;
                light.x = std::clamp<Float>(light.x, 0, 80);
                light.y = std::clamp<Float>(light.y, 0, 80);
                light.z = std::clamp<Float>(light.z, 0, 80);

                image.add_sample(point, light);
            }
        }
    }

    image.finished = true;
    dt = std::chrono::system_clock::now() - t0;
    fmt::print("[{:.1f}s] rendering finished\n", dt.count() / 1000000000.);

    std::vector<unsigned char> out;
    out.resize(4 * resolution.y * resolution.x);
    for (int px = 0; px < resolution.y * resolution.x; ++px) {
        auto light = image.light[px] / scene.film.supersampling;
        auto [r, g, b] = light_to_rgb(light);
        out[px * 4 + 0] = r * 255;
        out[px * 4 + 1] = g * 255;
        out[px * 4 + 2] = b * 255;
        out[px * 4 + 3] = 255;
    }
    sf::Image img;
    img.create((u32)(resolution.x), (u32)(resolution.y), out.data());
    img.saveToFile("out.png");

    if (ipc_worker.joinable())
        ipc_worker.join();
}

// coordinates:
// camera view covered by x and y (-1, 1)
// right-handed world coordinates
// x and y horizontal, z vertical, positive upwards

// todo:
// diffuse transmission is too bright
// light sampling from a surface with a diffuse coating is wrong
// stratified sampler still gets stuck
// perspective camera is a bit wrong
// convergence metering
// an empty white scene gets reconstructed badly
// some self-intersection issues apparent in zero-day and other scenes
// can be fudged with a min ray distance but should investigate
// bdpt
// add a --help
