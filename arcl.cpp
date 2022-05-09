#include "base.h"
#include "scene.h"
#include "rng.h"
#include "tev_ipc.h"

#include <array>
#include <queue>
#include <chrono>
#include <mutex>

#include <SFML/Graphics.hpp>
#include <cblas.h>

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

light sample_texture(const Float& constant, const intersection&) {
    return {constant};
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

light sample_texture(const texture& tex, const intersection& isect) {
    return std::visit(
        [&isect](auto& tex) { return sample_texture(tex, isect); }, tex);
}

Float sample_texture_1d(const texture& tex, const intersection& isect) {
    auto c = sample_texture(tex, isect);
    return (c.x + c.y + c.z) / 3;
}

bool same_normal_hemisphere(vec3f norm, vec3f v) { return 0 < norm.dot(v); }
bool same_shading_hemisphere(vec3f a, vec3f b) { return 0 < a.z * b.z; }

bool is_emissive(material* m) { return std::holds_alternative<emissive>(*m); }

light emitted_light(const intersection& intersection) {
    return sample_texture(std::get_if<emissive>(intersection.mat)->value,
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
    auto& [A, B, C] = *t;
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
        lights.push_back({node_instance{T, *node}, find_bounding_sphere(node)});
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
        lights.push_back(
            {node_instance{T, node->n}, find_bounding_sphere(node)});
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

bool bbox_ray_intersection(const bounding_box& bb, const ray& ray, Float d) {
    constexpr Float gamma3 = gamma(3);
    auto inv =
        vec3f{1 / ray.direction.x, 1 / ray.direction.y, 1 / ray.direction.z};

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

bounding_box node_bounds(const node_instance& i) { return i.bounds; }
bounding_box node_instance::transformed_bounds() {
    auto [min, max] = node_bounds(n);
    return bounding_box{} | T.point(min) | T.point(max) |
           T.point(vec3f{max.x, min.y, min.z}) |
           T.point(vec3f{min.x, max.y, min.z}) |
           T.point(vec3f{min.x, min.y, max.z}) |
           T.point(vec3f{min.x, max.y, max.z}) |
           T.point(vec3f{max.x, min.y, max.z}) |
           T.point(vec3f{max.x, max.y, min.z}) |
           T.point(vec3f{max.x, max.y, max.z});
};

bounding_box node_bounds(const bvh_mesh& n) { return n.bvh.nodes[0].bounds; }
bounding_box node_bounds(const node_bvh& n) { return n.bvh.nodes[0].bounds; }

bounding_box node_bounds(const node& n) {
    return std::visit([](const auto& n) { return node_bounds(*n); }, n.shape);
}

bounding_box node_bounds(const node_view& n) {
    return std::visit([](const auto& n) { return node_bounds(*n); }, n.shape);
}

std::optional<intersection> intersect(const node& n, const ray& r, Float max_t);
std::optional<intersection> intersect(const node_view& n, const ray& r,
                                      Float max_t);
std::optional<intersection> intersect(const triangle&, const ray&, Float max_t);
std::optional<intersection> intersect(const bvh_mesh&, const ray&, Float max_t);
std::optional<intersection> intersect(const node_view& n, const ray& r,
                                      Float max_t);
std::optional<intersection> intersect(const node_bvh&, const ray&, Float max_t);

std::optional<intersection> intersect(const node_instance& i, const ray& r,
                                      Float max_t) {
    auto Ti = i.T.inverse();
    auto ri = Ti(r);
    auto hit = intersect(i.n, ri, max_t);
    if (hit) {
        hit->normal = Ti.normal(hit->normal).normalized();
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
    if (i and n.material)
        i->mat = n.material;
    return i;
}

std::optional<intersection> intersect(const node_view& n, const ray& r,
                                      Float max_t) {
    auto i = std::visit(
        [&r, max_t](const auto& n) { return intersect(*n, r, max_t); },
        n.shape);
    if (i and n.material)
        i->mat = n.material;
    return i;
}

struct bvh_traversal {
    bvh_traversal(const BVH* bvh, const ray& r) : bvh(bvh), ray(r) {
        stack.push(0);
    }

    std::pair<int, int> next(Float bound) {
        while (not stack.empty()) {
            int n = stack.top();
            auto& node = bvh->nodes[n];
            stack.pop();

            if (!bbox_ray_intersection(node.bounds, ray, bound))
                continue;

            if (node.is_leaf())
                return {node.offset, node.offset + node.count};
            else {
                auto A = n + 1;
                auto B = node.offset;
                if ((bvh->nodes[A].bounds.centroid() - ray.origin)
                        .length_sq() >=
                    (bvh->nodes[B].bounds.centroid() - ray.origin)
                        .length_sq()) {
                    stack.push(A);
                    stack.push(B);
                }
                else {
                    stack.push(B);
                    stack.push(A);
                }
            }
        }

        return {0, 0};
    }

    operator bool() const { return not stack.empty(); }

private:
    const BVH* bvh;
    std::stack<int> stack;
    const ray& ray;
};

// woop et al 2013
std::optional<intersection> intersect(const indexed_triangle& tri,
                                      const ray& ray, Float max_t) {
    if (not bbox_ray_intersection(shape_bounds(tri), ray, max_t))
        return {};

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
    vec3f dAC = tA - tB;
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

        auto n = dpdu.cross(dpdv).normalized();

        return intersection{
            .distance = t,
            .normal = n,
            .uv = tri.mesh->normals.empty()
                      ? (uv[0] * U + uv[1] * V + uv[2] * W)
                      : (tri.mesh->tex_coords[tri.vertices.x] * U +
                         tri.mesh->tex_coords[tri.vertices.y] * V +
                         tri.mesh->tex_coords[tri.vertices.z] * W),
            .shading = {dpdu, dpdv},
        };
    }
}

std::optional<intersection> intersect(const triangle& T, const ray& ray,
                                      Float max_t) {
    indexed_mesh M{
        .vertices = {T.A, T.B, T.C},
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
    light value{0};
    vec3f direction;
    Float probability{};
    bool specular{false};
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

scatter_sample scatter(const scene&, sample_sequence* rng,
                       const lambertian& matte, const intersection& isect,
                       const vec3f& towards) {
    auto in = isect.shading.to_local(towards);
    auto away = sample_cosine_hemisphere(rng->sample_2d());
    if (in.z < 0)
        away.z *= -1;

    return scatter_sample{
        .value = sample_texture(matte.reflectance, isect) / π,
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
        .value = sample_texture(matte.reflectance, isect) / π,
        .direction = away,
        .probability = std::abs(angle_cos) / π,
    };
}

scatter_sample scatter(const scene&, sample_sequence*, const emissive&,
                       const intersection&, const vec3f&) {
    return scatter_sample{};
}

scatter_sample scatter(const scene&, const emissive&, const intersection&,
                       const vec3f&, const vec3f&) {
    return scatter_sample{};
}

scatter_sample blinn_phong_forward(const blinn_phong& material,
                                   const intersection& isect,
                                   const vec3f& towards, const vec3f& away) {
    auto angle_cos = away.z;
    auto probability = std::abs(angle_cos) / π;

    auto roughness = material.diffuse;
    auto glossiness = (Float)material.sharpness;
    light reflectance{sample_texture(material.diffuse_color, isect)};

    auto diffuse = reflectance * roughness * (1 / π);

    auto halfv = (away + isect.shading.to_local(towards)).normalized();
    auto specular_intensity = std::pow(std::max<Float>(halfv.z, 0), glossiness);

    auto energy_conservation = [](Float n) {
        return ((n + 2) * (n + 4)) / (8 * π * (std::pow<Float>(2, -n / 2) + 2));
    }(glossiness);

    auto specular = sample_texture(material.specular_color, isect) *
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

std::optional<vec3f> transmission_direction(const vec3f& in,
                                            const vec3f& normal,
                                            Float refraction_ratio) {
    auto cos_in = in.dot(normal);
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
    auto internal_refraction = sample_texture(material.refraction, isect);
    auto absorption = sample_texture(material.absorption, isect);
    auto transmission_rate = sample_texture(material.transmission, isect);

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
        return {.probability = {}};

    auto transmission = (light{1} - Fr) * transmission_rate;
    transmission *= refraction_ratio * refraction_ratio;

    return scatter_sample{.value = transmission / std::abs(away->z),
                          .direction = isect.shading.to_world(*away),
                          .probability = 1 - F,
                          .specular = true};
}

scatter_sample scatter(const scene&, const specular&, const intersection&,
                       const vec3f&, const vec3f& away) {
    return scatter_sample{
        .value = 0,
        .direction = away,
        .probability = 0,
    };
}

Float trowbridge_reitz_microfacet_area(Float roughness, Float a) {
    auto t = std::tan(a);
    auto t2 = t * t;
    if (std::isinf(t2))
        return 0;
    auto a2 = roughness * roughness;
    return a2 / (π * std::pow(std::cos(a), 4) * std::pow(a2 + t2, 2));
}

Float trowbridge_reitz_microfacet_G1(Float roughness, Float a) {
    auto t = std::tan(a);
    if (std::isinf(t))
        return 0;
    auto t2 = t * t;
    auto a2 = roughness * roughness;
    auto D = std::sqrt(std::max((Float)0, (1 + a2 * t2)));
    return 2 / (1 + D);
}

Float trowbridge_reitz_microfacet_lambda(Float roughness, Float a) {
    return (1 / trowbridge_reitz_microfacet_G1(roughness, a)) - 1;
}

Float trowbridge_reitz_microfacet_geometry(Float roughness, Float a, Float b) {
    auto A = trowbridge_reitz_microfacet_lambda(roughness, a);
    auto B = trowbridge_reitz_microfacet_lambda(roughness, b);
    return 1 / (1 + A + B);
}

// heitz 2018
vec3f trowbridge_reitz_sample(sample_sequence* rng, Float alpha, vec3f v) {
    auto vh = vec3f{alpha * v.x, alpha * v.y, v.z}.normalized();
    if (vh.z < 0)
        vh.z *= -1;

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
    return trowbridge_reitz_microfacet_G1(alpha, std::acos(v.z)) /
           std::abs(v.z) *
           trowbridge_reitz_microfacet_area(alpha, std::acos(n.z)) *
           std::abs(v.dot(n));
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

light glossy_reflection(const vec3f& towards, const vec3f& away,
                        Float roughness, light eta, light k) {
    auto halfv = (away + towards).normalized();
    auto a = std::acos(away.z);
    auto b = std::acos(towards.z);
    auto h = std::acos(halfv.z);

    auto F = fresnel_conductor(std::abs(towards.dot(halfv)), {1}, eta, k);

    auto D = trowbridge_reitz_microfacet_area(roughness, h);
    auto G = trowbridge_reitz_microfacet_geometry(roughness, a, b);

    // auto D = beckmann_microfacet_area(roughness, h);
    // auto G = beckmann_microfacet_geometry(roughness, a, b);

    return F * D * G / (4 * std::abs(away.z) * std::abs(towards.z));
}

scatter_sample scatter(const scene&, sample_sequence* rng,
                       const glossy& material, const intersection& isect,
                       const vec3f& towards) {
    // auto away = sample_cosine_hemisphere(rng->sample_2d());
    // auto angle_cos = away.z;
    // auto probability = std::abs(angle_cos) / π;

    auto in = isect.shading.to_local(towards).normalized();
    auto inside = in.z < 0;
    if (inside)
        in.z *= -1;

    auto facet = trowbridge_reitz_sample(
        rng, sample_texture_1d(material.roughness, isect), in);
    auto away = (in * -1) + facet * 2 * in.dot(facet);

    auto f = glossy_reflection(in, away,
                               sample_texture_1d(material.roughness, isect),
                               sample_texture(material.refraction, isect),
                               sample_texture(material.absorption, isect));
    auto probability =
        trowbridge_reitz_sample_pdf(
            sample_texture_1d(material.roughness, isect), facet, in) /
        (4 * std::abs(in.dot(facet)));

    if (inside)
        away.z *= -1;

    return scatter_sample{
        .value = f,
        .direction = isect.shading.to_world(away),
        .probability = probability,
    };
}

scatter_sample scatter(const scene&, const glossy& material,
                       const intersection& isect, const vec3f& towards,
                       const vec3f& out) {
    auto away = isect.shading.to_local(out).normalized();
    auto in = isect.shading.to_local(towards).normalized();
    auto facet = (away + in).normalized();
    auto probability =
        trowbridge_reitz_sample_pdf(
            sample_texture_1d(material.roughness, isect), facet, in) /
        (4 * std::abs(in.dot(facet)));
    auto f = same_shading_hemisphere(in, away)
                 ? glossy_reflection(
                       in, away, sample_texture_1d(material.roughness, isect),
                       sample_texture(material.refraction, isect),
                       sample_texture(material.absorption, isect))
                 : 0;
    return scatter_sample{
        .value = f,
        .direction = out,
        .probability = probability,
    };
}

scatter_sample scatter(const scene& scene, sample_sequence* rng,
                       const intersection& isect, const vec3f& towards);
scatter_sample scatter(const scene& scene, const intersection& isect,
                       const vec3f& towards, const vec3f& away);

scatter_sample scatter(const scene& scene, const mixed_material& mix,
                       const intersection& isect, const vec3f& towards,
                       const vec3f& away);

scatter_sample scatter(const scene& scene, sample_sequence* rng,
                       const mixed_material& mix, const intersection& isect,
                       const vec3f& towards);
scatter_sample scatter(const scene& scene, const mixed_material& mix,
                       const intersection& isect, const vec3f& towards,
                       const vec3f& away);

scatter_sample scatter(const scene& scene, const mixed_material& mix,
                       const intersection& isect, const vec3f& towards,
                       scatter_sample scattering,
                       std::optional<int> scattering_source) {
    vec3f away = scattering.direction;

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

        scattering.probability += s.probability * weight;
        scattering.value += s.value * weight;
    }

    scattering.probability /= total_weight;
    scattering.value /= total_weight;

    return scattering;
}

scatter_sample scatter(const scene& scene, const mixed_material& mix,
                       const intersection& isect, const vec3f& towards,
                       const vec3f& away) {
    return scatter(scene, mix, isect, towards, scatter_sample{{}, away, {}},
                   {});
}

scatter_sample scatter(const scene& scene, sample_sequence* rng,
                       const mixed_material& mix, const intersection& isect,
                       const vec3f& towards) {

    auto scattering_source = std::clamp<int>(
        (int)(rng->sample_1d() * mix.layers.size()), 0, mix.layers.size() - 1);

    auto scattering = std::visit(
        [&](const auto& m) { return scatter(scene, rng, m, isect, towards); },
        *mix.layers[scattering_source].second);
    scattering.probability *= mix.layers[scattering_source].first;
    scattering.value *= mix.layers[scattering_source].first;

    return scatter(scene, mix, isect, towards, scattering, scattering_source);
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

scatter_sample scatter(const scene& scene, sample_sequence* rng,
                       const intersection& isect, const vec3f& towards) {
    if (isect.mat)
        return std::visit(
            [&](const auto& material) {
                return scatter(scene, rng, material, isect, towards);
            },
            *isect.mat);
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
        if (not intersection)
            return light{scene.film.global_radiance} * beta;

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

Float power_heuristic(Float f, Float g) { return (f * f) / (f * f + g * g); }

scatter_sample sample_light(vec2f sample, const scene& scene,
                            const intersection& intersection,
                            const light_source& source, const ray& r) {
    auto intersection_point = r.distance(intersection.distance);

    auto& [light, light_bounds] = source;
    auto [light_direction, light_sample_pdf] =
        sample_sphere(sample, light_bounds, intersection_point);

    auto light_probability = 1 / (light_sample_pdf * scene.lights.size());

    auto light_ray =
        offset_origin(intersection, ray{.origin = intersection_point,
                                        .direction = light_direction});

    auto icos = (light_ray.direction).dot(intersection.normal);
    if (icos < 0)
        return {.probability = light_probability};

    auto visible =
        intersect(light, light_ray, std::numeric_limits<Float>::max());
    if (not visible)
        return {.probability = light_probability};

    auto occluded = intersect(scene.root, light_ray, visible->distance);
    if (occluded and occluded->distance < visible->distance)
        return {.probability = light_probability};

    auto towards = r.direction * -1;
    auto reflected = scatter(scene, intersection, towards, light_ray.direction);

    auto L = emitted_light(*visible);
    L *= reflected.value * icos;
    L *= power_heuristic(light_probability, reflected.probability) /
         light_probability;

    return scatter_sample{
        .value = L,
        .direction = light_ray.direction,
        .probability = light_probability,
    };
}

scatter_sample sample_direct_lighting(const scene& scene, sample_sequence* rng,
                                      const intersection& intersection,
                                      const ray& r) {
    if (scene.lights.empty())
        return {};
    auto one_light =
        std::clamp<int>((int)(rng->sample_1d() * scene.lights.size()), 0,
                        scene.lights.size() - 1);
    return sample_light(rng->sample_2d(), scene, intersection,
                        scene.lights[one_light], r);
}

light path_trace(const scene& scene, sample_sequence* rng, ray r,
                 int max_depth) {
    light beta{1};
    light L{};
    int depth{0};

    ray prev_ray{r};
    std::optional<intersection> previous_intersection;
    Float scattering_pdf{-1};

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
                    auto d = intersect(scene.lights[li].first, r,
                                       intersection->distance + gamma(32));
                    if (d and d->distance == intersection->distance) {
                        auto light_pdf =
                            sample_light({}, scene, *previous_intersection,
                                         scene.lights[li], prev_ray)
                                .probability;
                        Le *= power_heuristic(scattering_pdf, light_pdf);
                        L += beta * Le;
                        break;
                    }
                }
            }
        }

        if (max_depth <= depth)
            break;

        auto light = sample_direct_lighting(scene, rng, *intersection, r);
        if (0 < light.value.length_sq()) {
            L += beta * light.value;
        }
        rng->skip_to(2 + 3 + depth * 5);

        auto intersection_point = r.distance(intersection->distance);
        auto scattering = scatter(scene, rng, *intersection, r.direction * -1);
        rng->skip_to(2 + 5 + depth * 5);

        auto icos = scattering.direction.dot(intersection->normal);
        auto reflected =
            (scattering.value * std::abs(icos)) / scattering.probability;

        beta *= reflected;

        if (beta.length_sq() <= 0)
            break;
        if (std::isnan(beta.y))
            return L;

        auto scatter_ray = offset_origin(
            *intersection, ray{intersection_point, scattering.direction});

        previous_intersection = intersection;
        scattering_pdf = scattering.specular ? -1 : scattering.probability;
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

light light_trace(const scene& scene, sample_sequence* rng, ray r, int) {
    auto intersection =
        intersect(scene.root, r, std::numeric_limits<Float>::max());
    if (not intersection)
        return {scene.film.global_radiance};

    light L{};

    if (intersection->mat and is_emissive(intersection->mat))
        return emitted_light(*intersection);

    auto Ld = sample_direct_lighting(scene, rng, *intersection, r);
    return L + Ld.value;
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
        auto px = vec2<int>{position};

        if (not gaussian_filter) {
            auto [x, y] = px;
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
                    auto p = vec2f{x + (Float)0.5, y + (Float)0.5} +
                             vec2f{1 / (Float)3, 0} * (c - 1);
                    auto d = (position - p).length_sq();
                    auto f = gaussian(alpha, filter_radius, d);
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
    int supersampling;
    std::vector<light> light;
    std::vector<float> partial_image;
};

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
    fmt::print("scene({}) ready\n", *scene_path);

    scene.lights = collect_lights(&scene.root);
    fmt::print("lights({}) ready\n", scene.lights.size());

    auto trace = [&scene, debug]() {
        if (debug)
            return debug_trace;
        switch (scene.film.method) {
        case integrator::path: return path_trace;
        case integrator::light: return light_trace;
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

    auto tev = std::optional<ipc::tev>{};
    auto ipc_name =
        *scene_path +
        fmt::format(
            "-{}", std::chrono::system_clock::now().time_since_epoch().count() %
                       (1 << 16));
    if (arg_present("--tev")) {
        try {
            tev.emplace();
            tev->create_rgb_image(ipc_name, resolution.x, resolution.y);
        }
        catch (std::exception& err) {
            fmt::print("tev connection failed, image output only\n");
        }
    }

    std::unordered_map<int, std::unique_ptr<pixel_sampler>> samplers;
    for (int y = 0; y < resolution.y; ++y) {
        for (int x = 0; x < resolution.x; ++x) {
            auto px = vec2{x, y};
            samplers.insert({y * resolution.x + x,
                             scene_sampler(px, scene.film.supersampling)});
        }
    }
    fmt::print("sampler ready\n");

    image image{resolution};

    for (int s{}; s < scene.film.supersampling; ++s) {
        image.set_supersampling(s + 1);
#pragma omp parallel for schedule(monotonic : dynamic)
        for (int y = 0; y < resolution.y; ++y) {
            for (int x = 0; x < resolution.x; ++x) {

                auto& sampler = samplers.at(y * resolution.x + x);
                auto rng = sampler->nth_sample(s);

                auto px = vec2{x, y};
                auto jitter = rng->sample_2d();
                auto point = vec2f{px} + jitter;

                auto ray = scene.view.point(point);
                auto light = trace(scene, rng.get(), ray, scene.film.depth);

                if (std::isnan(light.x) or std::isnan(light.x) or
                    std::isnan(light.y) or std::isinf(light.y) or
                    std::isinf(light.z) or std::isinf(light.z))
                    continue;
                image.add_sample(point, light);
            }
        }

        if (tev)
            tev->update_rgb_image(ipc_name, 0, 0, resolution.x, resolution.y,
                                  image.partial_image);
    }

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
}

// coordinates:
// camera view covered by x and y (-1, 1)
// right-handed world coordinates
// x and y horizontal, z vertical, positive upwards

// todo:
// fixing mix materials
// convergence metering
// perspective camera
// interactive preview

// diffuse/glossy transmission
