#include "scene.h"
#include "toml.hpp"
#include <iostream>
#include <typeinfo>
#include <filesystem>
#include <forward_list>

template <class T, class N> auto parse_vec2(const toml::node_view<N>& v) {
    return vec2<T>{*v[0].template value<T>(), *v[1].template value<T>()};
}

template <class T, class N> auto parse_vec3(const toml::node_view<N>& v) {
    if (v.is_number())
        return vec3<T>{*v.template value<T>()};
    if (not v[0].is_number()) {
        fmt::print("err: couldn't parse vector at {}\n", v.node()->source());
    }
    return vec3<T>{*v[0].template value<T>(), *v[1].template value<T>(),
                   *v[2].template value<T>()};
}

struct scene_load_context {
    toml::table* description{};
    std::filesystem::path scene_root;
    std::unordered_map<std::string, node> nodes;
    std::unordered_map<std::string, node*> models;
    std::unordered_map<std::string, std::unique_ptr<material>> materials;
    std::unordered_map<std::string, texture> textures;
    std::unordered_map<std::string, sf::Image> images;
};

// https://github.com/google/ion/blob/master/ion/math/matrixutils.cc
matrix_sq4 matrix_inverse(const matrix_sq4& M) {
    auto& m = M.m;
    // For 4x4 do not compute the adjugate as the transpose of the cofactor
    // matrix, because this results in extra work. Several calculations can be
    // shared across the sub-determinants.
    //
    // This approach is explained in David Eberly's Geometric Tools book,
    // excerpted here:
    //   http://www.geometrictools.com/Documentation/LaplaceExpansionTheorem.pdf
    const Float s0 = m[0][0] * m[1][1] - m[1][0] * m[0][1];
    const Float s1 = m[0][0] * m[1][2] - m[1][0] * m[0][2];
    const Float s2 = m[0][0] * m[1][3] - m[1][0] * m[0][3];

    const Float s3 = m[0][1] * m[1][2] - m[1][1] * m[0][2];
    const Float s4 = m[0][1] * m[1][3] - m[1][1] * m[0][3];
    const Float s5 = m[0][2] * m[1][3] - m[1][2] * m[0][3];

    const Float c0 = m[2][0] * m[3][1] - m[3][0] * m[2][1];
    const Float c1 = m[2][0] * m[3][2] - m[3][0] * m[2][2];
    const Float c2 = m[2][0] * m[3][3] - m[3][0] * m[2][3];

    const Float c3 = m[2][1] * m[3][2] - m[3][1] * m[2][2];
    const Float c4 = m[2][1] * m[3][3] - m[3][1] * m[2][3];
    const Float c5 = m[2][2] * m[3][3] - m[3][2] * m[2][3];

    auto determinant =
        s0 * c5 - s1 * c4 + s2 * c3 + s3 * c2 - s4 * c1 + s5 * c0;
    auto s = 1 / determinant;

    return matrix_sq4{{{s * (m[1][1] * c5 - m[1][2] * c4 + m[1][3] * c3),
                        s * (-m[0][1] * c5 + m[0][2] * c4 - m[0][3] * c3),
                        s * (m[3][1] * s5 - m[3][2] * s4 + m[3][3] * s3),
                        s * (-m[2][1] * s5 + m[2][2] * s4 - m[2][3] * s3)},
                       {s * (-m[1][0] * c5 + m[1][2] * c2 - m[1][3] * c1),
                        s * (m[0][0] * c5 - m[0][2] * c2 + m[0][3] * c1),
                        s * (-m[3][0] * s5 + m[3][2] * s2 - m[3][3] * s1),
                        s * (m[2][0] * s5 - m[2][2] * s2 + m[2][3] * s1)},
                       {s * (m[1][0] * c4 - m[1][1] * c2 + m[1][3] * c0),
                        s * (-m[0][0] * c4 + m[0][1] * c2 - m[0][3] * c0),
                        s * (m[3][0] * s4 - m[3][1] * s2 + m[3][3] * s0),
                        s * (-m[2][0] * s4 + m[2][1] * s2 - m[2][3] * s0)},
                       {s * (-m[1][0] * c3 + m[1][1] * c1 - m[1][2] * c0),
                        s * (m[0][0] * c3 - m[0][1] * c1 + m[0][2] * c0),
                        s * (-m[3][0] * s3 + m[3][1] * s1 - m[3][2] * s0),
                        s * (m[2][0] * s3 - m[2][1] * s1 + m[2][2] * s0)}}};
}

transform parse_transform(const toml::array& ts) {
    auto T = transform::identity();
    for (auto& e : ts) {
        auto step = *e.as_array();
        auto name = step[0].value<std::string>();

        const auto& value = toml::node_view(step[1]);
        if (name == "translate") {
            T = transform::translate(parse_vec3<Float>(value)).compose(T);
        }
        else if (name == "scale") {
            T = transform::scale(parse_vec3<Float>(value)).compose(T);
        }
        else if (name == "rotate-x") {
            T = transform::rotate_x(*value.value<Float>()).compose(T);
        }
        else if (name == "rotate-y") {
            T = transform::rotate_y(*value.value<Float>()).compose(T);
        }
        else if (name == "rotate-z") {
            T = transform::rotate_z(*value.value<Float>()).compose(T);
        }
        else if (name == "matrix") {
            matrix_sq4 mx;
            auto arr = *value.as_array();
            for (int n{}; n < 16; ++n) {
                auto v = arr[n].value<Float>();
                mx.m[n % 4][n / 4] = *v;
            }
            T = transform{mx, matrix_inverse(mx)}.compose(T);
        }
        else
            throw std::runtime_error(fmt::format("bad transform {}", *name));
    }

    return T;
}

template <class N>
texture parse_texture(scene_load_context& context, const toml::node_view<N>& v);

void parse_texture_definition(scene_load_context& context,
                              const std::string& name, const toml::table& v) {
    auto type = *v["type"].template value<std::string>();

    if (type == "image") {
        auto file = **v["file"].as_string();
        if (not context.images.contains(file)) {
            context.images[file].loadFromFile(context.scene_root /
                                              std::filesystem::path{file});
        }
        context.textures[name] = image_texture{&context.images.at(file)};
    }
    else if (type == "product") {
        auto A{std::make_shared<texture>(Float{1})};
        *A = parse_texture(context, v["A"]);

        auto B{std::make_shared<texture>(Float{1})};
        *B = parse_texture(context, v["B"]);
        context.textures[name] = product_texture{std::move(A), std::move(B)};
    }
    else if (type == "uv") {
        context.textures[name] = uv_texture{};
    }
    else if (type == "checkerboard") {
        context.textures[name] = checkerboard_texture{};
    }
    else if (type == "constant") {
        context.textures[name] =
            constant_texture{parse_vec3<Float>(v["color"])};
    }
    else
        throw std::runtime_error(fmt::format("bad texture type {}", type));
}

template <class N> texture parse_texture(scene_load_context& context,
                                         const toml::node_view<N>& v) {
    if (v.is_string()) {
        auto name = **v.as_string();
        try {
            return context.textures.at(name);
        }
        catch (const std::out_of_range&) {
            auto textures = context.description->at("texture").as_table();
            if (textures and textures->contains(name)) {
                parse_texture_definition(context, name,
                                         *textures->at(name).as_table());
                assert(context.textures.contains(name));
                return context.textures.at(name);
            }
            else {
                fmt::print("err: unknown texture {}\n", **v.as_string());
                return checkerboard_texture{};
            }
        }
    }
    else if (v.is_array())
        return parse_vec3<Float>(v);
    else if (v.is_number())
        return *v.template value<Float>();
    else {
        fmt::print("err: couldn't parse texture at {}\n", v.node()->source());
        return vec3f{0};
    }
}

texture fix_refraction(texture eta) {
    if (auto r = std::get_if<Float>(&eta); r) {
        if (*r <= 1.0)
            *r = 1.01;
    }
    else if (auto l = std::get_if<light>(&eta); l) {
        for (int d{}; d < 3; ++d) {
            if ((*l)[d] <= 1.0)
                (*l)[d] = 1.01;
        }
    }
    else if (auto c = std::get_if<constant_texture>(&eta); c) {
        for (int d{}; d < 3; ++d) {
            if (c->v[d] <= 1.0)
                c->v[d] = 1.01;
        }
    }
    return eta;
}

material* parse_material(scene_load_context& context,
                         const toml::table& material_table,
                         const std::string& name, const toml::table& nt) {

    if (context.materials.contains(name))
        return context.materials.at(name).get();

    auto type = *nt["type"].value<std::string>();

    if (type == "lambertian") {
        context.materials[name] = std::make_unique<material>(lambertian{
            .reflectance = parse_texture(context, nt["reflectance"])});
    }

    else if (type == "translucent") {
        context.materials[name] = std::make_unique<material>(translucent{
            .reflectance = parse_texture(context, nt["reflectance"]),
            .transmission = parse_texture(context, nt["transmission"])});
    }

    else if (type == "specular") {
        auto refraction = parse_texture(context, nt["refraction"]);
        context.materials[name] = std::make_unique<material>(specular{
            .refraction = fix_refraction(refraction),
            .absorption = parse_texture(context, nt["absorption"]),
            .transmission = parse_texture(context, nt["transmission"]),
        });
    }

    else if (type == "glossy") {
        auto refraction = parse_texture(context, nt["refraction"]);
        context.materials[name] = std::make_unique<material>(glossy{
            .refraction = fix_refraction(refraction),
            .absorption = parse_texture(context, nt["absorption"]),
            .transmission = parse_texture(context, nt["transmission"]),
            .roughness = parse_texture(context, nt["roughness"]),
        });
    }

    else if (type == "blinn-phong") {
        context.materials[name] = std::make_unique<material>(blinn_phong{
            .sharpness = *nt["sharpness"].value<Float>(),
            .diffuse = *nt["diffuse"].value<Float>(),
            .diffuse_color = parse_texture(context, nt["diffuse_color"]),
            .specular_color = parse_texture(context, nt["specular_color"]),
        });
    }

    else if (type == "emissive") {
        context.materials[name] = std::make_unique<material>(
            emissive{.value = parse_texture(context, nt["light"])});
    }

    else if (type == "coat") {
        coated coat{};
        assert(nt.contains("coat"));
        assert(nt.contains("base"));
        assert(nt.contains("weight"));
        auto coat_name = nt["coat"].value<std::string>();
        auto base_name = nt["base"].value<std::string>();
        for (auto& name : {coat_name, base_name}) {
            if (not context.materials.contains(*name))
                parse_material(context, material_table, *name,
                               *material_table[*name].as_table());
            assert(context.materials.contains(*name));
        }

        coat.weight = parse_texture(context, nt["weight"]);
        coat.coat = context.materials.at(*coat_name).get();
        coat.base = context.materials.at(*base_name).get();
        context.materials[name] = std::make_unique<material>(std::move(coat));
    }

    else if (type == "alpha") {
        alpha alpha{};
        assert(nt.contains("map"));
        assert(nt.contains("base"));
        auto base = nt["base"].value<std::string>();
        if (not context.materials.contains(*base))
            parse_material(context, material_table, *base,
                           *material_table[*base].as_table());
        assert(context.materials.contains(*base));
        alpha.map = parse_texture(context, nt["map"]);
        alpha.base = context.materials.at(*base).get();
        context.materials[name] = std::make_unique<material>(std::move(alpha));
    }

    else
        throw std::runtime_error(fmt::format("bad material type {}", type));

    assert(context.materials.contains(name));
    return context.materials.at(name).get();
}

std::optional<node> parse_node(scene_load_context&, const toml::table&);

node* load_node_lazy(scene_load_context& context, std::string id) {
    auto node_table = (*context.description)["node"].as_table();
    auto model_table = (*context.description)["model"].as_table();

    if (context.nodes.contains(id)) {
        return &context.nodes.at(id);
    }

    else if (node_table and id.starts_with("node.")) {
        auto node_name = id.substr(5);
        auto vt = (*node_table)[node_name].as_table();
        auto n = parse_node(context, *vt);
        if (n) {
            context.nodes[id] = std::move(*n);
            return &context.nodes.at(id);
        }
        else
            return {};
    }

    else if (model_table and id.starts_with("model.")) {
        auto model_id = id.substr(6);
        auto vt = (*model_table)[model_id].as_table();
        auto path = (*vt)["file"].value<std::string>();
        if (not path) {
            fmt::print("bad model path: {}", id);
            return {};
        }

        if (context.models.contains(*path))
            return context.models.at(*path);
        auto full_path = context.scene_root.c_str() + *path;
        context.nodes[id] = {load_model(full_path)};
        auto model_view = &context.nodes.at(id);
        context.models[*path] = model_view;
        return model_view;
    }

    else {
        fmt::print("err: couldn't locate node {}\n", id);
        return {};
    }
}

std::optional<node> parse_node(scene_load_context& context,
                               const toml::table& nt) {
    material* material{};
    if (nt.contains("material")) {
        auto name = *nt["material"].value<std::string>();
        if (context.materials.contains(name))
            material = context.materials.at(name).get();
        else
            fmt::print("err: unknown material {}\n", name);
    }

    if (nt.contains("group")) {
        std::vector<node> nodes;
        for (auto& [k, v] : nt) {
            if (v.is_table()) {
                auto n = parse_node(context, *v.as_table());
                if (n)
                    nodes.push_back(std::move(*n));
            }
        }

        if (nodes.empty())
            return {};

        auto group = std::make_unique<node_bvh>();
        group->bvh = build_bvh(
            nodes.size(), [&nodes](size_t n) { return node_bounds(nodes[n]); },
            [&nodes, &group](size_t index) {
                group->nodes.push_back(std::move(nodes[index]));
            });

        return {node{{std::move(group)}, material}};
    }

    else if (nt.contains("instance")) {
        auto transform = transform::identity();
        if (nt.contains("transform") and nt["transform"].is_array())
            transform = parse_transform(*nt.at("transform").as_array());
        auto instance_of = *nt["instance"].value<std::string>();
        auto n = load_node_lazy(context, instance_of);
        if (n)
            return {node{instance(transform, *n), material}};
        else
            return {};
    }

    else if (nt.contains("triangle")) {
        auto t = std::make_unique<triangle>();
        auto vx = *nt["triangle"].as_array();
        t->A = parse_vec3<Float>(toml::node_view<toml::node>(vx[0]));
        t->B = parse_vec3<Float>(toml::node_view<toml::node>(vx[1]));
        t->C = parse_vec3<Float>(toml::node_view<toml::node>(vx[2]));
        if (nt.contains("uv")) {
            auto uv = *nt["uv"].as_array();
            t->uv[0] = parse_vec2<Float>(toml::node_view<toml::node>(uv[0]));
            t->uv[1] = parse_vec2<Float>(toml::node_view<toml::node>(uv[1]));
            t->uv[2] = parse_vec2<Float>(toml::node_view<toml::node>(uv[2]));
        }
        return {node{std::move(t), material}};
    }

    else {
        fmt::print("err: couldn't parse world node\n", nt.source());
        return {};
    }
}

light_source parse_light(const scene_load_context&, const toml::table& nt) {
    if (not nt.contains("type")) {
        fmt::print("err: couldn't parse light node\n", nt.source());
        return {};
    }

    auto type = *nt["type"].value<std::string>();

    if (type == "distant") {
        auto from = parse_vec3<Float>(nt["from"]);
        auto to = parse_vec3<Float>(nt["to"]);
        auto towards = (from - to).normalized();
        auto value = parse_vec3<Float>(nt["value"]);
        return {light_source{distant_light{towards, value}}};
    }

    else {
        fmt::print("err: couldn't parse light node\n", nt.source());
        return {};
    }
}

scene load_scene(std::string path) {
    scene_load_context context;
    context.scene_root = std::filesystem::path{path}.remove_filename();

    auto config = toml::parse_file(path);
    context.description = &config;

    film film{};
    auto method = config["film"]["method"].value_or<std::string>("path");
    if (method == "path")
        film.method = integrator::path;
    else if (method == "scatter")
        film.method = integrator::scatter;
    else if (method == "light")
        film.method = integrator::light;
    else if (method == "brute-force")
        film.method = integrator::brute_force;
    else
        throw std::runtime_error(
            fmt::format("bad integration method {}", method));

    auto sampler =
        config["film"]["sampler"].value_or<std::string>("independent");
    if (sampler == "independent")
        film.sampler = sampler::independent;
    else if (sampler == "stratified")
        film.sampler = sampler::stratified;
    else if (sampler == "multi-stratified")
        film.sampler = sampler::multi_stratified;
    else
        throw std::runtime_error(fmt::format("bad sampler {}", sampler));

    film.resolution = parse_vec2<int>(config["film"]["resolution"]);
    film.supersampling = config["film"]["supersampling"].value_or(8);
    film.depth = config["film"]["depth"].value_or(6);
    film.global_radiance = parse_vec3<Float>(config["film"]["global_radiance"]);

    auto cam_config = config["camera"];
    auto cam_type = *cam_config["type"].value_exact<std::string>();
    auto cam_scale = *cam_config["scale"].value<Float>();
    auto cam_pos = parse_vec3<Float>(cam_config["position"]);
    auto cam_towards = parse_vec3<Float>(cam_config["towards"]);
    auto cam_up = parse_vec3<Float>(cam_config["up"]);

    std::unique_ptr<camera> view;
    if (cam_type == "orthographic")
        view = std::make_unique<orthographic_camera>(
            vec2f{film.resolution}, cam_scale, cam_pos, cam_towards, cam_up);
    else if (cam_type == "perspective")
        view = std::make_unique<perspective_camera>(
            vec2f{film.resolution}, cam_scale, cam_pos, cam_towards, cam_up);
    else
        throw std::runtime_error(fmt::format("bad camera type {}", cam_type));

    auto texture_table = config["texture"].as_table();
    if (texture_table)
        for (auto [name, v] : *texture_table) {
            auto vt = v.as_table();
            if (not vt)
                continue;
            parse_texture_definition(context, name.data(), *vt);
        }

    auto& materials = context.materials;
    auto material_table = config["material"].as_table();
    if (material_table)
        for (auto [name, v] : *material_table) {
            auto vt = v.as_table();
            if (not vt)
                continue;
            parse_material(context, *material_table, name.data(), *vt);
        }

    std::vector<light_source> lights;
    auto light = config["light"].as_array();
    if (light) {
        for (auto& l : *light)
            lights.push_back(parse_light(context, *l.as_table()));
    }

    std::vector<node> root_nodes;
    auto world = *config["world"].as_array();
    for (auto& x : world) {
        auto n = parse_node(context, *x.as_table());
        if (n)
            root_nodes.push_back(std::move(*n));
    }

    auto root = std::make_unique<node_bvh>();
    root->bvh = build_bvh(
        root_nodes.size(),
        [&root_nodes](size_t n) { return node_bounds(root_nodes[n]); },
        [&root, &root_nodes](size_t index) {
            root->nodes.push_back(std::move(root_nodes[index]));
        });

    std::vector<node> assets;
    for (auto& [name, node] : context.nodes) assets.push_back(std::move(node));

    std::vector<std::unique_ptr<material>> mats_vec;
    for (auto& [name, mat] : materials) mats_vec.push_back(std::move(mat));

    return scene{
        .film = film,
        .root = {std::move(root)},
        .view = std::move(view),
        .materials = std::move(mats_vec),
        .lights = std::move(lights),
        .assets = std::move(assets),
        .images = std::move(context.images),
    };
}
