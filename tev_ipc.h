#pragma once
#include "base.h"

namespace ipc {

struct tev {
    tev(std::string host = "localhost");
    ~tev();

    void create_rgb_image(const std::string& name, i32 width, i32 height);
    void update_rgb_image(const std::string& name, i32 x, i32 y, i32 width,
                          i32 height, const std::vector<float>& data);

private:
    struct impl;
    std::unique_ptr<impl> pimpl;
};

}
