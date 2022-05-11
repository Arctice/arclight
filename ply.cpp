#include "base.h"

#include <cassert>
#include "miniply.h"

indexed_mesh load_ply(std::string path) {

    miniply::PLYReader ply{path.c_str()};
    if (!ply.valid()) {
        fmt::print("invalid ply file {}\n", path);
    }

    indexed_mesh mesh;

    while (ply.has_element()) {
        u32 idxs[3];
        if (ply.element_is("vertex") and ply.load_element() &&
            ply.find_pos(idxs)) {
            auto size = ply.num_rows();
            mesh.vertices.resize(size);
            ply.extract_properties(idxs, 3, miniply::PLYPropertyType::Float,
                                   mesh.vertices.data());

            if (ply.find_texcoord(idxs)) {
                mesh.tex_coords.resize(size);
                ply.extract_properties(idxs, 2, miniply::PLYPropertyType::Float,
                                       mesh.tex_coords.data());
            }

            if (ply.find_normal(idxs)) {
                mesh.normals.resize(size);
                ply.extract_properties(idxs, 3, miniply::PLYPropertyType::Float,
                                       mesh.normals.data());
            }
        }

        else if (ply.element_is("face") and ply.load_element() &&
                 ply.find_indices(idxs)) {
            if (not ply.requires_triangulation(idxs[0])) {
                auto size = ply.num_rows();
                mesh.triangles.resize(size);
                ply.extract_list_property(idxs[0],
                                          miniply::PLYPropertyType::Int,
                                          mesh.triangles.data());
            }
            else {
                auto size = ply.num_triangles(idxs[0]);
                mesh.triangles.resize(size);
                ply.extract_triangles(
                    idxs[0], (float*)mesh.vertices.data(), mesh.vertices.size(),
                    miniply::PLYPropertyType::Int, mesh.triangles.data());
            }
        }

        ply.next_element();
    }

    return mesh;
}
