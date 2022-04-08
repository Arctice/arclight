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
        else if (name == "rotate-z") {
            T = transform::rotate_z(*value.value<Float>()).compose(T);
        }
    }

    return T;
}

node parse_node(const std::unordered_map<std::string, node>& nodes,
                const toml::table& nt) {
    if (nt.contains("group")) {
        auto group = std::make_unique<node_bvh>();

        for (auto& [k, v] : nt) {
            if (v.is_table()) {
                group->nodes.push_back(parse_node(nodes, *v.as_table()));
            }
        }

        group->bvh = build_bvh(group->nodes.size(), [&group](size_t n) {
            return node_bounds(group->nodes[n]);
        });

        return group;
    }

    else if (nt.contains("instance")) {
        auto transform = transform::identity();
        if (nt.contains("transform") and nt["transform"].is_array())
            transform = parse_transform(*nt.at("transform").as_array());
        auto instance_of = *nt["instance"].value<std::string>();
        return instance(transform, nodes.at(instance_of));
    }

    else if (nt.contains("triangle")) {
        auto t = std::make_unique<triangle>();
        auto vx = *nt["triangle"].as_array();
        t->A = parse_vec3<Float>(toml::node_view<toml::node>(vx[0]));
        t->B = parse_vec3<Float>(toml::node_view<toml::node>(vx[1]));
        t->C = parse_vec3<Float>(toml::node_view<toml::node>(vx[2]));
        return t;
    }

    return {};
}

scene load_scene(std::string path) {
    auto scene_root = std::filesystem::path{path}.remove_filename();

    auto config = toml::parse_file(path);

    auto resolution = parse_vec2<int>(config["film"]["resolution"]);
    auto supersampling = config["film"]["supersampling"].value_or(8);
    auto depth = config["film"]["depth"].value_or(6);

    auto cam_config = config["camera"];
    auto cam_type = *cam_config["type"].value_exact<std::string>();
    auto cam_scale = *cam_config["scale"].value<Float>();
    auto cam_pos = parse_vec3<Float>(cam_config["position"]);
    auto cam_towards = parse_vec3<Float>(cam_config["towards"]);
    auto cam_up = parse_vec3<Float>(cam_config["up"]);

    auto view = orthographic_camera(vec2f{resolution}, cam_scale, cam_pos,
                                    cam_towards, cam_up);

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

    auto node_table = config["node"].as_table();
    if (node_table)
        for (auto& [k, v] : *config["node"].as_table()) {
            auto vt = v.as_table();
            if (not vt)
                continue;

            auto name = "node." + std::string(k.data());
            nodes[name] = parse_node(nodes, *vt);
        }

    auto root = std::make_unique<node_bvh>();

    auto world = *config["world"].as_array();
    for (auto& x : world) {
        root->nodes.push_back(parse_node(nodes, *x.as_table()));
    }

    fmt::print("world bvh... \n");
    root->bvh = build_bvh(root->nodes.size(), [&root](size_t n) {
        return node_bounds(root->nodes[n]);
    });

    std::vector<node> assets;
    for (auto& [name, node] : nodes) assets.push_back(std::move(node));

    return {{resolution, supersampling, depth},
            std::move(root),
            view,
            std::move(assets)};
}
