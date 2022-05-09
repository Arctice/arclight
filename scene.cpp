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
    if(not v[0].is_number()){
        fmt::print("warn: couldn't parse vector at {}\n", v.node()->source());
    }
    return vec3<T>{*v[0].template value<T>(), *v[1].template value<T>(),
                   *v[2].template value<T>()};
}

struct scene_load_context {
    std::filesystem::path scene_root;
    std::unordered_map<std::string, node> nodes;
    std::unordered_map<std::string, std::unique_ptr<material>> materials;
    std::unordered_map<std::string, texture> textures;
    std::unordered_map<std::string, sf::Image> images;
};

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
            T = transform::scale(*value.value<Float>()).compose(T);
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
    }

    return T;
}

void parse_texture_definition(scene_load_context& context,
                              const std::string& name, const toml::table& v) {
    auto type = *v["type"].template value<std::string>();

    if (type == "image") {
        auto file = **v["file"].as_string();
        context.images[name].loadFromFile(context.scene_root /
                                          std::filesystem::path{file});
        context.textures[name] = image_texture{&context.images[name]};
    }
    else if (type == "uv") {
        context.textures[name] = uv_texture{};
    }
    else if (type == "checkerboard") {
        context.textures[name] = checkerboard_texture{};
    }
    else {
        throw std::runtime_error(fmt::format("bad texture type {}", type));
    }
}

template <class N> texture parse_texture(scene_load_context& context,
                                         const toml::node_view<N>& v) {
    if (v.is_string()) {
        try {
            return context.textures.at(**v.as_string());
        }
        catch (const std::out_of_range&) {
            throw std::runtime_error(
                fmt::format("missing texture {}", **v.as_string()));
        }
    }
    else if (v.is_array())
        return parse_vec3<Float>(v);
    else
        return *v.template value<Float>();
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

    else if (type == "specular") {
        context.materials[name] = std::make_unique<material>(specular{
            .transmission = parse_texture(context, nt["transmission"]),
            .refraction = parse_texture(context, nt["refraction"]),
            .absorption = parse_texture(context, nt["absorption"]),
        });
    }

    else if (type == "glossy") {
        context.materials[name] = std::make_unique<material>(glossy{
            .roughness = parse_texture(context, nt["roughness"]),
            .refraction = parse_texture(context, nt["refraction"]),
            .absorption = parse_texture(context, nt["absorption"]),
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

    else if (type == "mix") {
        mixed_material mix{};

        auto layers = *nt.at("layers").as_array();
        for (auto& e : layers) {
            auto layer = *e.as_array();
            auto name = layer[0].value<std::string>();
            auto weight = *toml::node_view(layer[1]).value<Float>();

            if (not context.materials.contains(*name))
                parse_material(context, material_table, *name,
                               *material_table[*name].as_table());

            mix.layers.push_back({weight, context.materials.at(*name).get()});
        }

        context.materials[name] = std::make_unique<material>(std::move(mix));
    }

    else
        throw std::runtime_error(fmt::format("bad material type {}", type));

    return context.materials.at(name).get();
}

node parse_node(const scene_load_context& context, const toml::table& nt) {

    material* material{};
    if (nt.contains("material")) {
        material =
            context.materials.at(*nt["material"].value<std::string>()).get();
    }

    if (nt.contains("group")) {
        std::vector<node> nodes;

        for (auto& [k, v] : nt) {
            if (v.is_table()) {
                nodes.push_back(parse_node(context, *v.as_table()));
            }
        }

        auto group = std::make_unique<node_bvh>();
        group->bvh = build_bvh(
            nodes.size(), [&nodes](size_t n) { return node_bounds(nodes[n]); },
            [&nodes, &group](size_t index) {
                group->nodes.push_back(std::move(nodes[index]));
            });

        return {{std::move(group)}, material};
    }

    else if (nt.contains("instance")) {
        auto transform = transform::identity();
        if (nt.contains("transform") and nt["transform"].is_array())
            transform = parse_transform(*nt.at("transform").as_array());
        auto instance_of = *nt["instance"].value<std::string>();
        assert(context.nodes.contains(instance_of));
        return {instance(transform, context.nodes.at(instance_of)), material};
    }

    else if (nt.contains("triangle")) {
        auto t = std::make_unique<triangle>();
        auto vx = *nt["triangle"].as_array();
        t->A = parse_vec3<Float>(toml::node_view<toml::node>(vx[0]));
        t->B = parse_vec3<Float>(toml::node_view<toml::node>(vx[1]));
        t->C = parse_vec3<Float>(toml::node_view<toml::node>(vx[2]));
        return {std::move(t), material};
    }

    return {};
}

scene load_scene(std::string path) {
    scene_load_context context;
    context.scene_root = std::filesystem::path{path}.remove_filename();

    auto config = toml::parse_file(path);

    film film{};
    auto method = config["film"]["method"].value_or<std::string>("path");
    if (method == "path")
        film.method = integrator::path;
    else if (method == "scatter")
        film.method = integrator::scatter;
    else if (method == "light")
        film.method = integrator::light;
    else if (method == "brute_force")
        film.method = integrator::brute_force;
    else
        throw std::runtime_error(
            fmt::format("bad integration method {}", method));

    auto sampler =
        config["film"]["sampler"].value_or<std::string>("stratified");
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
    film.global_radiance = config["film"]["global_radiance"].value_or<Float>(0);

    auto cam_config = config["camera"];
    auto cam_type = *cam_config["type"].value_exact<std::string>();
    auto cam_scale = *cam_config["scale"].value<Float>();
    auto cam_pos = parse_vec3<Float>(cam_config["position"]);
    auto cam_towards = parse_vec3<Float>(cam_config["towards"]);
    auto cam_up = parse_vec3<Float>(cam_config["up"]);

    auto view = orthographic_camera(vec2f{film.resolution}, cam_scale, cam_pos,
                                    cam_towards, cam_up);

    auto& nodes = context.nodes;
    auto& materials = context.materials;

    auto model_table = config["model"].as_table();
    if (model_table)
        for (auto [k, v] : *model_table) {
            auto vt = v.as_table();
            if (not vt)
                continue;

            auto path = (*vt)["file"].value<std::string>();
            if (not path)
                continue;

            auto full_path = context.scene_root.c_str() + *path;
            auto m = load_model(full_path);
            nodes["model." + std::string(k.data())] = node{std::move(m)};
        }

    auto texture_table = config["texture"].as_table();
    if (texture_table)
        for (auto [name, v] : *texture_table) {
            auto vt = v.as_table();
            if (not vt)
                continue;
            parse_texture_definition(context, name.data(), *vt);
        }

    auto material_table = config["material"].as_table();
    if (material_table)
        for (auto [name, v] : *material_table) {
            auto vt = v.as_table();
            if (not vt)
                continue;
            parse_material(context, *material_table, name.data(), *vt);
        }

    auto node_table = config["node"].as_table();
    if (node_table)
        for (auto& [k, v] : *config["node"].as_table()) {
            auto vt = v.as_table();
            if (not vt)
                continue;

            auto name = "node." + std::string(k.data());
            nodes[name] = parse_node(context, *vt);
        }

    std::vector<node> root_nodes;
    auto world = *config["world"].as_array();
    for (auto& x : world)
        root_nodes.push_back(parse_node(context, *x.as_table()));

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
        .root = node{std::move(root)},
        .view = camera{view},
        .materials = std::move(mats_vec),
        .assets = std::move(assets),
        .images = std::move(context.images),
    };
}
