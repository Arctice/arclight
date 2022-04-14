#include "tev_ipc.h"
#include <asio.hpp>

using namespace std;

namespace ipc {

struct tev::impl {
    asio::io_context io;
    asio::ip::tcp::socket socket{io};

    impl() {
        asio::ip::tcp::resolver resolver{io};
        auto endpoint = resolver.resolve("127.0.0.1", "14158");
        asio::connect(socket, endpoint);
    }

    void send(const std::vector<char>& data) {
        asio::write(socket, asio::buffer(data));
    }
};

tev::tev() : pimpl(std::make_unique<impl>()) {}
tev::~tev() {}

class OStream {
public:
    OStream(std::vector<char>& data) : mData{data} {
        *this << (uint32_t)0;
    }

    template <typename T> OStream& operator<<(const std::vector<T>& var) {
        for (auto&& elem : var) { *this << elem; }
        return *this;
    }

    OStream& operator<<(const std::string& var) {
        for (auto&& character : var) { *this << character; }
        *this << '\0';
        return *this;
    }

    OStream& operator<<(bool var) {
        if (mData.size() < mIdx + 1) {
            mData.resize(mIdx + 1);
        }

        mData[mIdx] = var ? 1 : 0;
        ++mIdx;
        updateSize();
        return *this;
    }

    template <typename T> OStream& operator<<(T var) {
        if (mData.size() < mIdx + sizeof(T)) {
            mData.resize(mIdx + sizeof(T));
        }

        *(T*)&mData[mIdx] = var;
        mIdx += sizeof(T);
        updateSize();
        return *this;
    }

private:
    void updateSize() { *((uint32_t*)mData.data()) = (uint32_t)mIdx; }

    std::vector<char>& mData;
    size_t mIdx = 0;
};

void tev::create_rgb_image(const std::string& name, i32 width, i32 height) {
    std::vector<char> data;
    OStream payload{data};
    payload << char(4);
    payload << false; // grab focus
    payload << name;
    payload << width << height;
    payload << 3; // channels
    payload << std::vector<std::string>{"R", "G", "B"};
    pimpl->send(data);
}

void tev::update_rgb_image(const std::string& name, i32 x, i32 y, i32 width,
                           i32 height, const std::vector<float>& image) {

    i32 channel_count = 3;
    vector<std::string> channels{"R", "G", "B"};
    vector<i64> offsets{0, 1, 2};
    vector<i64> strides{3, 3, 3};

    std::vector<char> data;
    OStream payload{data};
    payload << (char)6;
    payload << false; // grab focus
    payload << name;
    payload << channel_count;
    payload << channels;
    payload << x << y << width << height;
    payload << offsets;
    payload << strides;
    payload << image;
    pimpl->send(data);
}

}
