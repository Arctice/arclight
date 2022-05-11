#pragma once
#include "base.h"
#include <variant>
#include <numeric>
#include <stack>

#include <SFML/Graphics.hpp>

using light = vec3f;


struct uv_texture;
struct checkerboard_texture;
struct product_texture;
struct image_texture;
using texture =
    std::variant<Float, light, uv_texture, checkerboard_texture, product_texture, image_texture>;

struct uv_texture {};

struct checkerboard_texture {};

struct product_texture {
    std::shared_ptr<texture> A;
    std::shared_ptr<texture> B;
};

struct image_texture {
    sf::Image* img;
};

struct emissive {
    texture value{light{1}};
};

struct lambertian {
    texture reflectance{light{0.5}};
};

struct specular {
    texture transmission;
    texture refraction;
    texture absorption;
};

struct glossy {
    texture roughness;
    texture refraction;
    texture absorption;
    texture transmission;
};

struct blinn_phong {
    Float sharpness;
    Float diffuse;
    texture diffuse_color;
    texture specular_color;
};

struct coated;
using material =
    std::variant<emissive, lambertian, specular, glossy, blinn_phong, coated>;

struct coated {
    texture weight;
    const material* coat;
    const material* base;
};

struct ray {
    vec3f origin;
    vec3f direction;

    vec3f distance(Float t) const { return origin + direction * t; }
};

struct shading_frame {
    vec3f dpdu, dpdv;
    vec3f normal;

    vec3f to_local(vec3f) const;
    vec3f to_world(vec3f) const;
};

struct intersection {
    Float distance;
    vec3f normal;
    vec2f uv;
    shading_frame shading;
    material* mat{};
};

struct BVH_tree {
    bounding_box bounds;
    std::vector<int> overlap;
    std::unique_ptr<BVH_tree> a, b;
};

struct BVH {
    struct node {
        bounding_box bounds;
        int offset{-1};
        int count{-1};
        bool is_leaf() const { return 0 <= count; }
    };

    std::vector<node> nodes;
};

struct bvh_mesh {
    indexed_mesh data;
    BVH bvh;
};

struct node_bvh;
struct node_instance;

template <template <class> class Wrapper> struct node_body {
    std::variant<Wrapper<triangle>, Wrapper<bvh_mesh>, Wrapper<node_bvh>,
                 Wrapper<node_instance>>
        shape;
    material* material{};
};

template <class T> using node_wrapper = std::unique_ptr<T>;
using node = node_body<node_wrapper>;
template <class T> using node_view_wrapper = const T*;
using node_view = node_body<node_view_wrapper>;

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

template <class Fn>
BVH_tree build_bvh(const std::vector<int>& objs, Fn&& bound) {
    BVH_tree node;
    auto [dimension, cut] = partition_dimension(objs, bound);
    node.bounds = {{std::numeric_limits<Float>::max()},
                   {std::numeric_limits<Float>::lowest()}};
    for (auto index : objs) node.bounds = node.bounds | bound(index);

    if (objs.size() <= 16) {
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
        node.a = std::make_unique<BVH_tree>(build_bvh(L, bound));
    if (not R.empty())
        node.b = std::make_unique<BVH_tree>(build_bvh(R, bound));

    return node;
}

template <class FnIndexCallback>
inline BVH flatten_bvh(const BVH_tree& tree, FnIndexCallback&& index_callback) {
    BVH bvh;
    std::stack<std::pair<int, const BVH_tree*>> stack;
    stack.push({-1, &tree});
    size_t object_count{0};

    while (not stack.empty()) {
        auto [parent, tree_node] = stack.top();
        stack.pop();
        if (tree_node == nullptr)
            continue;

        int node_index = bvh.nodes.size();
        if (0 <= parent)
            bvh.nodes[parent].offset = node_index;
        bvh.nodes.push_back({tree_node->bounds});

        auto& node = bvh.nodes.back();
        if (0 < tree_node->overlap.size()) {
            node.offset = object_count;
            node.count = tree_node->overlap.size();
            for (auto i : tree_node->overlap) {
                index_callback(i);
                object_count++;
            }
        }
        else {
            stack.push({node_index, tree_node->b.get()});
            stack.push({-1, tree_node->a.get()});
        }
    }
    return bvh;
}

template <class FnBoundary, class FnIndexCallback>
BVH build_bvh(const size_t size, FnBoundary&& bound,
              FnIndexCallback&& index_callback) {
    std::vector<int> indices(size);
    std::iota(indices.begin(), indices.end(), 0);
    return flatten_bvh(build_bvh(indices, bound), index_callback);
}

bounding_box node_bounds(const node& n);
bounding_box node_bounds(const node_view& n);

struct matrix_sq4 {
    Float m[4][4];

    matrix_sq4 matmul(const matrix_sq4& b) const;

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

    static transform rotate_x(Float a) {
        auto m = matrix_sq4::identity();
        auto cos_a = std::cos(-a);
        auto sin_a = std::sin(-a);
        m.m[1][1] = cos_a;
        m.m[1][2] = -sin_a;
        m.m[2][1] = sin_a;
        m.m[2][2] = cos_a;
        auto i = matrix_sq4::identity();
        i.m[1][1] = cos_a;
        i.m[2][1] = -sin_a;
        i.m[1][2] = sin_a;
        i.m[2][2] = cos_a;
        return {m, i};
    }

    static transform rotate_y(Float a) {
        auto m = matrix_sq4::identity();
        auto cos_a = std::cos(-a);
        auto sin_a = std::sin(-a);
        m.m[0][0] = cos_a;
        m.m[0][2] = -sin_a;
        m.m[2][0] = sin_a;
        m.m[2][2] = cos_a;
        auto i = matrix_sq4::identity();
        i.m[0][0] = cos_a;
        i.m[0][2] = sin_a;
        i.m[2][0] = -sin_a;
        i.m[2][2] = cos_a;
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
        i.m[0][0] = cos_a;
        i.m[0][1] = sin_a;
        i.m[1][0] = -sin_a;
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
                           vec3f towards, vec3f up);

struct node_bvh {
    std::vector<node> nodes;
    BVH bvh;
};

inline node_view make_node_view(const node& n) {
    auto view = std::visit([](auto& s) { return node_view{s.get()}; }, n.shape);
    view.material = n.material;
    return view;
}

struct node_instance {
    bounding_box transformed_bounds();

    node_instance(transform T, const node_view& n)
        : T(T), n(n), bounds(transformed_bounds()) {}
    node_instance(transform T, const node& n)
        : T(T), n(make_node_view(n)), bounds(transformed_bounds()) {}
    const transform T;
    node_view n;
    const bounding_box bounds;
};

std::unique_ptr<node_instance> instance(const transform& T, const node& n);

enum class integrator {
    brute_force,
    light,
    scatter,
    path,
};

enum class sampler {
    independent,
    stratified,
    multi_stratified,
};

struct film {
    integrator method;
    sampler sampler;
    vec2<int> resolution;
    int supersampling;
    int depth;
    vec3f global_radiance;
};

struct bounding_sphere {
    vec3f centre;
    Float radius;
};

using light_source = std::pair<node_instance, bounding_sphere>;

struct scene {
    film film;
    node root;
    camera view;
    std::vector<std::unique_ptr<material>> materials;
    std::vector<light_source> lights;
    std::vector<node> assets;
    std::unordered_map<std::string, sf::Image> images;
};

std::unique_ptr<bvh_mesh> load_model(std::string path);

scene load_scene(std::string path);
