#include "scene.h"
#include "toml.hpp"
#include <iostream>
#include <typeinfo>
#include <filesystem>

template <class T> auto parse_vec2(const toml::node_view<toml::node>& v) {
    return vec2<T>{*v[0].value<T>(), *v[1].value<T>()};
}

template <class T> auto parse_vec3(const toml::node_view<toml::node>& v) {
    return vec3<T>{*v[0].value<T>(), *v[1].value<T>(), *v[2].value<T>()};
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

node parse_node(
    const std::unordered_map<std::string, node>& nodes,
    const std::unordered_map<std::string, std::unique_ptr<material>>& mats,
    const toml::table& nt) {

    material* material{};
    if (nt.contains("material")){
        material = mats.at(*nt["material"].value<std::string>()).get();
    }

    if (nt.contains("group")) {
        auto group = std::make_unique<node_bvh>();

        for (auto& [k, v] : nt) {
            if (v.is_table()) {
                group->nodes.push_back(parse_node(nodes, mats, *v.as_table()));
            }
        }

        group->bvh = build_bvh(group->nodes.size(), [&group](size_t n) {
            return node_bounds(group->nodes[n]);
        });

        return {{std::move(group)}, material};
    }

    else if (nt.contains("instance")) {
        auto transform = transform::identity();
        if (nt.contains("transform") and nt["transform"].is_array())
            transform = parse_transform(*nt.at("transform").as_array());
        auto instance_of = *nt["instance"].value<std::string>();
        return {instance(transform, nodes.at(instance_of)), material};
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
    auto scene_root = std::filesystem::path{path}.remove_filename();

    auto config = toml::parse_file(path);

    film scene_film{};
    auto method = config["film"]["method"].value_or<std::string>("path");
    if (method == "path")
        scene_film.method = integrator::path;
    else if (method == "brute_force")
        scene_film.method = integrator::brute_force;
    else
        throw std::runtime_error(fmt::format("bad integration method {}", method));

    scene_film.resolution = parse_vec2<int>(config["film"]["resolution"]);
    scene_film.supersampling = config["film"]["supersampling"].value_or(8);
    scene_film.depth = config["film"]["depth"].value_or(6);
    scene_film.global_radiance = config["film"]["global_radiance"].value_or<Float>(1);

    auto cam_config = config["camera"];
    auto cam_type = *cam_config["type"].value_exact<std::string>();
    auto cam_scale = *cam_config["scale"].value<Float>();
    auto cam_pos = parse_vec3<Float>(cam_config["position"]);
    auto cam_towards = parse_vec3<Float>(cam_config["towards"]);
    auto cam_up = parse_vec3<Float>(cam_config["up"]);

    auto view = orthographic_camera(vec2f{scene_film.resolution}, cam_scale,
                                    cam_pos, cam_towards, cam_up);

    std::unordered_map<std::string, node> nodes;

    auto model_table = config["model"].as_table();
    if (model_table)
        for (auto [k, v] : *model_table) {
            auto vt = v.as_table();
            if (not vt)
                continue;

            auto path = (*vt)["file"].value<std::string>();
            if (not path)
                continue;

            auto full_path = scene_root.c_str() + *path;
            auto m = load_model(full_path);
            nodes["model." + std::string(k.data())] = node{std::move(m)};
        }

    std::unordered_map<std::string, std::unique_ptr<material>> materials;
    auto material_table = config["material"].as_table();
    if (material_table)
        for (auto [k, v] : *material_table) {

            auto vt = v.as_table();
            if (not vt)
                continue;

            auto m = material{
                .light = {},
                .reflectance = {0.5},
            };
            if (vt->contains("light"))
                m.light = parse_vec3<Float>((*vt)["light"]);
            if (vt->contains("reflectance"))
                m.reflectance = parse_vec3<Float>((*vt)["reflectance"]);
            materials[std::string(k.data())] = std::make_unique<material>(m);
        }

    auto node_table = config["node"].as_table();
    if (node_table)
        for (auto& [k, v] : *config["node"].as_table()) {
            auto vt = v.as_table();
            if (not vt)
                continue;

            auto name = "node." + std::string(k.data());
            nodes[name] = parse_node(nodes, materials, *vt);
        }

    auto root = std::make_unique<node_bvh>();

    auto world = *config["world"].as_array();
    for (auto& x : world) {
        root->nodes.push_back(parse_node(nodes, materials, *x.as_table()));
    }

    fmt::print("world bvh... \n");
    root->bvh = build_bvh(root->nodes.size(), [&root](size_t n) {
        return node_bounds(root->nodes[n]);
    });

    std::vector<node> assets;
    for (auto& [name, node] : nodes) assets.push_back(std::move(node));

    std::vector<std::unique_ptr<material>> mats_vec;
    for (auto& [name, mat] : materials) mats_vec.push_back(std::move(mat));

    return scene{scene_film, node{std::move(root)}, camera{view},
                 std::move(assets), std::move(mats_vec)};
}
