#include "base.h"

#define TINYPLY_IMPLEMENTATION
#include "ply.h"
#include <fstream>

indexed_mesh load_ply(std::string path, bool describe) {
    try {
        std::vector<uint8_t> byte_buffer;
        std::ifstream stream{path, std::ios::binary};

        if (!stream || stream.fail())
            throw std::runtime_error("stream failed to open " + path);

        ply::PlyFile ply;
        ply.parse_header(stream);

        auto comments = ply.get_comments();
        auto info = ply.get_info();

        std::shared_ptr<ply::PlyData> vertices, normals, tex_coords, faces;

        vertices =
            ply.request_properties_from_element("vertex", {"x", "y", "z"});

        try {
            normals = ply.request_properties_from_element("vertex",
                                                          {"nx", "ny", "nz"});
        }
        catch (const std::exception & e) {}

        try {
            tex_coords =
                ply.request_properties_from_element("vertex", {"u", "v"});
        }
        catch (const std::exception & e) {}

        faces = ply.request_properties_from_element("face", {"vertex_indices"});

        ply.read(stream);

        if (describe) {
            fmt::print("ply: {}\n", ply.is_binary_file() ? "binary" : "ascii");
            for (const auto& e : ply.get_elements()) {
                fmt::print("ply: element {} {}\n", e.name, e.size);
                for (const auto& p : e.properties) {
                    fmt::print("  {} :: {} {}\n", p.name,
                               ply::PropertyTable[p.propertyType].str,
                               p.isList ? ply::PropertyTable[p.listType].str
                                        : "");
                }
            }
            if (vertices)
                fmt::print("vertices: {}\n", vertices->count);
            if (normals)
                fmt::print("normals: {}\n", normals->count);
            if (tex_coords)
                fmt::print("tex_coords: {}\n", tex_coords->count);
            if (faces)
                fmt::print("faces: {}\n", faces->count);
        }

        indexed_mesh data;
        data.vertices.resize(vertices->count);
        if (normals)
            data.normals.resize(normals->count);
        if (tex_coords)
            data.tex_coords.resize(tex_coords->count);
        data.triangles.resize(faces->count);

        if (vertices->t == ply::Type::FLOAT32) {
            size_t size = vertices->buffer.size_bytes();
            std::memcpy(data.vertices.data(), vertices->buffer.get(), size);
        }
        else
            throw std::runtime_error("bad vertex type " +
                                     ply::PropertyTable.at(vertices->t).str);

        if (normals) {
            if (normals->t == ply::Type::FLOAT32) {
                size_t size = normals->buffer.size_bytes();
                std::memcpy(data.normals.data(), normals->buffer.get(), size);
            }
            else
                throw std::runtime_error("bad normal type " +
                                         ply::PropertyTable.at(normals->t).str);
        }

        if (tex_coords) {
            if (tex_coords->t == ply::Type::FLOAT32) {
                size_t size = tex_coords->buffer.size_bytes();
                std::memcpy(data.tex_coords.data(), tex_coords->buffer.get(),
                            size);
            }
            else
                throw std::runtime_error(
                    "bad tex_coord type " +
                    ply::PropertyTable.at(tex_coords->t).str);
        }

        if (faces->t == ply::Type::INT32) {
            size_t size = faces->buffer.size_bytes();
            std::memcpy(data.triangles.data(), faces->buffer.get(), size);
        }
        else
            throw std::runtime_error("bad face type " +
                                     ply::PropertyTable.at(vertices->t).str);

        return data;
    }
    catch (const std::exception& e) {
        fmt::print("ply error: {}\n", e.what());
    }

    return {};
}
