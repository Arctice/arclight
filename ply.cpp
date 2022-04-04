#include "base.h"

#define TINYPLY_IMPLEMENTATION
#include "ply.h"
#include <fstream>


mesh load_ply(std::string path, bool describe) {
    try {
        std::vector<uint8_t> byte_buffer;
        std::ifstream stream{path, std::ios::binary};

        if (!stream || stream.fail())
            throw std::runtime_error("stream failed to open " + path);

        ply::PlyFile ply;
        ply.parse_header(stream);

        auto comments = ply.get_comments();
        auto info = ply.get_info();

        auto vertices =
            ply.request_properties_from_element("vertex", {"x", "y", "z"});
        auto faces =
            ply.request_properties_from_element("face", {"vertex_indices"}, 3);

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
            if (faces)
                fmt::print("faces: {}\n", faces->count);
        }

        mesh data;
        data.vertices.resize(vertices->count);
        data.triangles.resize(faces->count);

        if (vertices->t == ply::Type::FLOAT32) {
            size_t size = vertices->buffer.size_bytes();
            std::memcpy(data.vertices.data(), vertices->buffer.get(), size);
        }
        else
            throw std::runtime_error("bad vertex type " +
                                     ply::PropertyTable.at(vertices->t).str);

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

