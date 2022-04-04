#include "base.h"
#include <array>
#include <numeric>
#include <queue>

#include <SFML/Graphics.hpp>
#include <cblas.h>

struct triangle {
    vec3f A, B, C;
};

struct ray {
    vec3f origin;
    vec3f direction;
    vec3f inverse_direction{};
};

struct intersection {
    Float distance;
    vec3f normal;
};

struct BVH {
    bounding_box bounds;
    std::vector<int> overlap;
    std::shared_ptr<BVH> a, b;
};

struct bvh_mesh {
    std::vector<triangle> triangles;
    BVH bvh;
};

struct matrix_sq4 {
    Float m[4][4];

    matrix_sq4 matmul(const matrix_sq4& b) const {
        matrix_sq4 c{};
        auto A = reinterpret_cast<const Float(&)[16]>(m);
        auto B = reinterpret_cast<const Float(&)[16]>(b.m);
        auto C = reinterpret_cast<Float(&)[16]>(c.m);
        cblas_sgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, 4, 4, 4, 1., A,
                    4, B, 4, 1., C, 4);
        return c;
    }

    static matrix_sq4 identity() {
        auto m = matrix_sq4{};
        for (int i{}; i < 4; ++i) m.m[i][i] = 1;
        return m;
    }
};

struct transform {
    matrix_sq4 m{};

    transform compose(transform& with) { return {m.matmul(with.m)}; };

    vec3f point(const vec3f& p) const {
        auto mp = vec3f{
            p.x * m.m[0][0] + p.y * m.m[0][1] + p.z * m.m[0][2] + m.m[0][3],
            p.x * m.m[1][0] + p.y * m.m[1][1] + p.z * m.m[1][2] + m.m[1][3],
            p.x * m.m[2][0] + p.y * m.m[2][1] + p.z * m.m[2][2] + m.m[2][3]};
        auto w =
            p.x * m.m[3][0] + p.y * m.m[3][1] + p.z * m.m[3][2] + m.m[3][3];
        if (w == 1)
            return mp;
        else
            return mp / w;
    }

    vec3f vector(const vec3f& v) const {
        return vec3f{v.x * m.m[0][0] + v.y * m.m[0][1] + v.z * m.m[0][2],
                     v.x * m.m[1][0] + v.y * m.m[1][1] + v.z * m.m[1][2],
                     v.x * m.m[2][0] + v.y * m.m[2][1] + v.z * m.m[2][2]};
    }

    ray operator()(const ray& r) const {
        return ray{point(r.origin), vector(r.direction)};
    }

    static transform identity() { return {matrix_sq4::identity()}; }

    static transform translate(vec3f t) {
        auto m = matrix_sq4::identity();
        m.m[0][3] = t.x;
        m.m[1][3] = t.y;
        m.m[2][3] = t.z;
        return {m};
    }

    static transform scale(vec3f s) {
        auto m = matrix_sq4::identity();
        m.m[0][0] = s.x;
        m.m[1][1] = s.y;
        m.m[2][2] = s.z;
        return {m};
    }

    static transform rotate_z(Float a) {
        auto m = matrix_sq4::identity();
        auto cos_a = std::cos(-a);
        auto sin_a = std::sin(-a);
        m.m[0][0] = cos_a;
        m.m[0][1] = -sin_a;
        m.m[1][0] = sin_a;
        m.m[1][1] = cos_a;
        return {m};
    }

    static transform look_at(vec3f pos, vec3f at, vec3f up) {
        up = up.normalized();
        auto m = translate(pos).m;
        auto n = (at - pos).normalized();
        m.m[0][2] = n.x;
        m.m[1][2] = n.y;
        m.m[2][2] = n.z;
        auto x = up.cross(n).normalized();
        m.m[0][0] = x.x;
        m.m[1][0] = x.y;
        m.m[2][0] = x.z;
        auto y = x.cross(n);
        m.m[0][1] = y.x;
        m.m[1][1] = y.y;
        m.m[2][1] = y.z;
        return {m};
    }
};

struct camera {
    transform camera_to_world;
    transform raster_to_screen;
    transform raster_to_camera;
    transform screen_to_camera;

    ray point(vec2f px) {
        auto o = raster_to_camera.point({px.x, px.y, 0});
        return camera_to_world(ray{o, {0, 0, 1}});
    }
};

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

constexpr Float epsilon = std::numeric_limits<Float>::epsilon() * 0.5;
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

bounding_box shape_bounds(const triangle& V) {
    vec3f min{std::min(std::min(V.A.x, V.B.x), V.C.x),
              std::min(std::min(V.A.y, V.B.y), V.C.y),
              std::min(std::min(V.A.z, V.B.z), V.C.z)};
    vec3f max{std::max(std::max(V.A.x, V.B.x), V.C.x),
              std::max(std::max(V.A.y, V.B.y), V.C.y),
              std::max(std::max(V.A.z, V.B.z), V.C.z)};
    return {min, max};
}

std::pair<int, Float> partition_dimension(const std::vector<triangle>& mesh,
                                          const std::vector<int>& objs) {
    bounding_box all = {{10e10}, {-10e10}};
    for (auto index : objs) {
        auto& t = mesh[index];
        all = all | shape_bounds(t).centroid();
    }
    auto size = vec3f{all.max.x - all.min.x, all.max.y - all.min.y,
                      all.max.z - all.min.z};
    auto best = std::max(size.x, std::max(size.y, size.z));
    if (best == size.x)
        return {0, all.min.x + size.x / 2};
    else if (best == size.y)
        return {1, all.min.y + size.y / 2};
    else
        return {2, all.min.z + size.z / 2};
}

BVH build_bvh(const std::vector<triangle>& mesh, const std::vector<int>& objs) {
    BVH node;

    auto [dimension, cut] = partition_dimension(mesh, objs);
    node.bounds = {{10e10}, {-10e10}};
    for (auto index : objs) {
        auto& t = mesh[index];
        node.bounds = node.bounds | shape_bounds(t);
    }

    if (objs.size() <= 8) {
        node.overlap = objs;
        return node;
    }

    std::vector<int> L;
    std::vector<int> R;
    for (auto index : objs) {
        auto& t = mesh[index];
        bool above = shape_bounds(t).centroid()[dimension] < cut;
        if (above)
            L.push_back(index);
        else
            R.push_back(index);
    }

    if (0 < L.size()) {
        node.a = std::make_shared<BVH>();
        *(node.a) = build_bvh(mesh, L);
    }
    if (0 < R.size()) {
        node.b = std::make_shared<BVH>();
        *(node.b) = build_bvh(mesh, R);
    }

    return node;
}

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

    return intersection{(ray.direction * t).length(), n.normalized()};
}

std::optional<intersection> intersect(const bvh_mesh& m, ray ray) {
    auto nearest = std::numeric_limits<Float>::max();

    std::optional<intersection> result{};

    std::queue<const BVH*> node_queue;
    node_queue.push(&m.bvh);

    while (!node_queue.empty()) {
        auto next = node_queue.front();
        node_queue.pop();

        if (!bbox_ray_intersection(next->bounds, ray))
            continue;

        for (const auto& index : next->overlap) {
            auto intersection = intersect(m.triangles[index], ray);
            if (not intersection)
                continue;

            auto distance = intersection->distance;
            if (nearest <= distance) {
                continue;
            }

            nearest = distance;
            result = {intersection};
        }

        if (next->a)
            node_queue.push(&*next->a);
        if (next->b)
            node_queue.push(&*next->b);
    }

    return result;
}

int main() {

    // auto v = vec3f{2, 3, -1};
    // auto A = transform::translate({1, 2, 3});
    // fmt::print("Ap {}\n", A.point(v));
    // fmt::print("Av {}\n", A.vector(v));
    // auto B = transform::scale({-1, .5, 2});
    // fmt::print("Bp {}\n", B.point(v));
    // fmt::print("B(Ap) {}\n", B.point(A.point(v)));
    // fmt::print("(BA)p {}\n", B.compose(A).point(v));
    // fmt::print("A(Bp) {}\n", A.point(B.point(v)));
    // fmt::print("(AB)p {}\n", A.compose(B).point(v));
    // v = {(Float)std::sqrt(0.5), (Float)std::sqrt(0.5), 0};
    // auto R = transform::rotate_z(std::acos(-1) / 4);
    // fmt::print("Rv {}\n", R.vector(v));
    // auto C = transform::look_at({0, 0, 0}, {10, 0, 0}, {0, 0, 1});
    // fmt::print("Cpos {}\n", C.point({0, 0, 0}));
    // fmt::print("C at  {}\n", C.point({0, 0, 1}));
    // C = transform::look_at({-2, 0, 0}, {0, 0, 0}, {0, 0, 1});
    // fmt::print("Cpos {}\n", C.point({0, 0, 0}));
    // fmt::print("C at  {}\n", C.point({0, 0, 1}));
    // C = transform::look_at({0, 0, 2}, {0, 0, 0}, {1, 0, 0});
    // fmt::print("Cpos {}\n", C.point({0, 0, 0}));
    // fmt::print("C at  {}\n", C.point({0, 0, 1}));

    auto ply_mesh = load_ply("scene/bunny/bun_zipper.ply");
    // auto ply_mesh = load_ply("scene/dragon.ply");
    auto mesh = bvh_mesh{{}, {}};
    for (auto [A, B, C] : ply_mesh.triangles) {
        mesh.triangles.push_back(triangle{
            ply_mesh.vertices[A],
            ply_mesh.vertices[B],
            ply_mesh.vertices[C],
        });
    }
    std::vector<int> bvh_indices(mesh.triangles.size());
    std::iota(bvh_indices.begin(), bvh_indices.end(), 0);
    mesh.bvh = build_bvh(mesh.triangles, bvh_indices);
    fmt::print("load done\n");

    auto resolution = vec2<int>{600, 600};
    auto view = orthographic_camera(vec2f{resolution}, .1, {1., 1., 2.},
                                    {-0.03, 0.11, 0}, {0., 1, 0});

    std::vector<unsigned char> out;
    out.resize(4 * resolution.y * resolution.x);
    for (int y = 0; y < resolution.y; ++y) {
        for (int x = 0; x < resolution.x; ++x) {
            auto px = vec2{x, y};
            auto ray = view.point(vec2f{px});
            ray.direction = ray.direction.normalized();
            auto intersection = intersect(mesh, ray);
            u8 c = 0;
            if (intersection) {
                auto shade = intersection->normal.dot(ray.direction);
                c = 255 - ((shade + 1) / 2) * 255;
            }
            auto px_offset = 4 * (y * resolution.x + x);
            out[px_offset + 0] = c;
            out[px_offset + 1] = c;
            out[px_offset + 2] = c;
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
// intersection interface
// ray-triangle intersection
