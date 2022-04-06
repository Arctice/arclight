#include "base.h"
#include <array>
#include <numeric>
#include <queue>
#include <variant>

#include <SFML/Graphics.hpp>
#include <cblas.h>

struct ray {
    vec3f origin;
    vec3f direction;
};

struct intersection {
    Float distance;
    vec3f normal;
};

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
    matrix_sq4 i{};

    transform compose(const transform& with) const {
        return {m.matmul(with.m), with.i.matmul(i)};
    }

    transform inverse() const { return {i, m}; }

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

    vec3f normal(const vec3f& v) const {
        return vec3f{v.x * m.m[0][0] + v.y * m.m[1][0] + v.z * m.m[2][0],
                     v.x * m.m[0][1] + v.y * m.m[1][1] + v.z * m.m[2][1],
                     v.x * m.m[0][2] + v.y * m.m[1][2] + v.z * m.m[2][2]};
    }

    ray operator()(const ray& r) const {
        return ray{point(r.origin), vector(r.direction)};
    }

    static transform identity() {
        return {matrix_sq4::identity(), matrix_sq4::identity()};
    }

    static transform translate(vec3f t) {
        auto m = matrix_sq4::identity();
        m.m[0][3] = t.x;
        m.m[1][3] = t.y;
        m.m[2][3] = t.z;
        auto i = matrix_sq4::identity();
        i.m[0][3] = -t.x;
        i.m[1][3] = -t.y;
        i.m[2][3] = -t.z;
        return {m, i};
    }

    static transform scale(vec3f s) {
        auto m = matrix_sq4::identity();
        m.m[0][0] = s.x;
        m.m[1][1] = s.y;
        m.m[2][2] = s.z;
        auto i = matrix_sq4::identity();
        i.m[0][0] = 1 / s.x;
        i.m[1][1] = 1 / s.y;
        i.m[2][2] = 1 / s.z;
        return {m, i};
    }

    static transform rotate_z(Float a) {
        auto m = matrix_sq4::identity();
        auto cos_a = std::cos(-a);
        auto sin_a = std::sin(-a);
        m.m[0][0] = cos_a;
        m.m[0][1] = -sin_a;
        m.m[1][0] = sin_a;
        m.m[1][1] = cos_a;
        auto i = matrix_sq4::identity();
        cos_a = std::cos(a);
        sin_a = std::sin(a);
        i.m[0][0] = cos_a;
        i.m[0][1] = -sin_a;
        i.m[1][0] = sin_a;
        i.m[1][1] = cos_a;
        return {m, i};
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

struct BVH {
    bounding_box bounds;
    std::vector<int> overlap;
    std::unique_ptr<BVH> a, b;
};

struct bvh_mesh {
    indexed_mesh data;
    BVH bvh;
};

struct node_bvh;
struct node_instance;

using node =
    std::variant<std::unique_ptr<triangle>, std::unique_ptr<bvh_mesh>,
                 std::unique_ptr<node_bvh>, std::unique_ptr<node_instance>>;
using node_view = std::variant<const triangle*, const bvh_mesh*,
                               const node_bvh*, const node_instance*>;

bounding_box node_bounds(const node& n);
std::optional<intersection> intersect(const node& n, const ray& r);
bounding_box node_bounds(const node_view& n);
std::optional<intersection> intersect(const node_view& n, const ray& r);

struct node_bvh {
    std::vector<node> nodes;
    BVH bvh;
};

struct node_instance {
    node_instance(transform T, const node& n) : T(T) {
        this->n = std::visit([](auto& n) { return node_view{n.get()}; }, n);
    }
    transform T;
    node_view n;
};

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
    return intersection{
        i.T.vector(ri.origin + ri.direction * hit->distance).length(),
        i.T.normal(hit->normal).normalized()};
}

std::optional<intersection> intersect(const node_bvh&, const ray&);

std::optional<intersection> intersect(const node& n, const ray& r) {
    return std::visit([r](const auto& n) { return intersect(*n, r); }, n);
}

std::optional<intersection> intersect(const node_view& n, const ray& r) {
    return std::visit([r](const auto& n) { return intersect(*n, r); }, n);
}

template <class Fn> std::pair<int, Float>
partition_dimension(const std::vector<int>& objs, Fn&& bound) {
    bounding_box all = {{std::numeric_limits<Float>::max()},
                        {std::numeric_limits<Float>::lowest()}};
    for (auto index : objs) all = all | bound(index).centroid();

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

template <class Fn> BVH build_bvh(const std::vector<int>& objs, Fn&& bound) {
    BVH node;
    auto [dimension, cut] = partition_dimension(objs, bound);
    node.bounds = {{std::numeric_limits<Float>::max()},
                   {std::numeric_limits<Float>::lowest()}};
    for (auto index : objs) node.bounds = node.bounds | bound(index);

    if (objs.size() <= 8) {
        node.overlap = objs;
        return node;
    }

    std::vector<int> L;
    std::vector<int> R;
    for (auto index : objs) {
        bool above = bound(index).centroid()[dimension] < cut;
        if (above)
            L.push_back(index);
        else
            R.push_back(index);
    }

    if (not L.empty())
        node.a = std::make_unique<BVH>(build_bvh(L, bound));
    if (not R.empty())
        node.b = std::make_unique<BVH>(build_bvh(R, bound));

    return node;
}

template <class Fn> BVH build_bvh(const size_t size, Fn&& bound) {
    std::vector<int> indices(size);
    std::iota(indices.begin(), indices.end(), 0);
    return build_bvh(indices, bound);
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

struct camera {
    transform camera_to_world;
    transform raster_to_screen;
    transform raster_to_camera;
    transform screen_to_camera;

    ray point(vec2f px) {
        auto o = raster_to_camera.point({px.x, px.y, 0});
        return camera_to_world(ray{o, {0, 0, 1}});
        return ray{{0, 0, 0}, {1, 1, 1}};
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

struct scene {
    node root;
    camera view;
    std::vector<node> assets;
};

scene test_scene(vec2<int> resolution) {
    std::vector<node> assets;
    auto model = node{load_model("scene/bunny/bun_zipper.ply")};
    auto pair = [&model]() {
        auto pair = std::make_unique<node_bvh>();
        pair->nodes.push_back(instance(
            transform::scale({1.2}).compose(transform::translate({.1})),
            model));
        pair->nodes.push_back(instance(
            transform::scale({.8}).compose(transform::translate({-.1})),
            model));
        pair->bvh = build_bvh(pair->nodes.size(), [&pair](size_t n) {
            return node_bounds(pair->nodes[n]);
        });
        return node{std::move(pair)};
    }();

    auto a = std::make_unique<node_bvh>();
    a->nodes.push_back(instance(transform::scale({.7}), pair));
    a->nodes.push_back(instance(transform::translate({-0.1, -0.0, -.0}), pair));
    a->nodes.push_back(instance(transform::rotate_z(0.2), pair));
    a->nodes.push_back(instance(transform::rotate_z(0.4), pair));

    a->nodes.push_back(std::make_unique<triangle>(
        triangle{{-0.1, 0, -0.1}, {-0.1, 0, 0.1}, {0.1, 0, -0.1}}));
    a->nodes.push_back(std::make_unique<triangle>(
        triangle{{0.1, 0, -0.1}, {-0.1, 0, 0.1}, {0.1, 0, 0.1}}));

    a->bvh = build_bvh(a->nodes.size(),
                       [&a](size_t n) { return node_bounds(a->nodes[n]); });
    auto view =
        orthographic_camera(vec2f{resolution}, .2, vec3f{1., 1., 2.} * 4,
                            {-0.03, 0.11, 0.}, {0., 1, 0.});

    assets.push_back(std::move(model));
    assets.push_back(std::move(pair));
    return {std::move(a), view, std::move(assets)};
}

int main() {
    auto resolution = vec2<int>{400, 400};
    auto scene = test_scene(resolution);

    std::vector<unsigned char> out;
    out.resize(4 * resolution.y * resolution.x);
    for (int y = 0; y < resolution.y; ++y) {
        for (int x = 0; x < resolution.x; ++x) {
            auto px = vec2{x, y};
            auto ray = scene.view.point(vec2f{px});
            ray.direction = ray.direction.normalized();
            auto intersection = intersect(scene.root, ray);
            u8 c = 0;
            if (intersection) {
                auto shade = intersection->normal.dot(ray.direction);
                c = 255 - ((shade + 1) / 2) * 255;
                // auto shade = intersection->distance;
                // c = 255 * (shade) / 1;
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
// ray-triangle intersection
