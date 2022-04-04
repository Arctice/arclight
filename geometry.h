#pragma once
#include <cmath>
#include <ostream>
#include <type_traits>

template <typename T> class vec3 {
public:
    T x, y, z;

    vec3() : x(0), y(0), z(0) {}
    vec3(T a) : x(a), y(a), z(a) {}
    vec3(T vx, T vy, T zy) : x(vx), y(vy), z(zy) {}

    vec3 operator+(const vec3& rhs) const;
    vec3& operator+=(const vec3& rhs);
    vec3 operator-(const vec3& rhs) const;
    vec3& operator-=(const vec3& rhs);

    template <typename V> vec3 operator*(const V& rhs) const;
    template <typename V> vec3& operator*=(const V& rhs);
    template <typename V> vec3 operator/(const V& rhs) const;
    template <typename V> vec3& operator/=(const V& rhs);

    template <typename V> vec3 operator*(const vec3<V>& rhs) const;
    template <typename V> vec3& operator*=(const vec3<V>& rhs);
    template <typename V> vec3 operator/(const vec3<V>& rhs) const;
    template <typename V> vec3& operator/=(const vec3<V>& rhs);

    bool operator==(const vec3& rhs) const;
    bool operator!=(const vec3& rhs) const;

    template <typename V> explicit operator vec3<V>() const;

    inline T& operator[](size_t d) { return reinterpret_cast<T*>(this)[d]; }
    inline const T& operator[](size_t d) const {
        return reinterpret_cast<const T*>(this)[d];
    }

    T length_sq() const;
    T length() const;

    vec3<T> normalized() const;

    T dot(const vec3&) const;
    vec3 cross(const vec3&) const;
};

template <typename T> vec3<T> vec3<T>::normalized() const {
    T r = std::sqrt(this->length_sq());
    return vec3(x / r, y / r, z / r);
}

template <typename T> T vec3<T>::dot(const vec3& rhs) const {
    return x * rhs.x + y * rhs.y + z * rhs.z;
}

template <typename T> vec3<T> vec3<T>::cross(const vec3& rhs) const {
    return vec3{(y * rhs.z) - (z * rhs.y), (z * rhs.x) - (x * rhs.z),
                (x * rhs.y) - (y * rhs.x)};
}

template <typename T> vec3<T> vec3<T>::operator-(const vec3<T>& rhs) const {
    return vec3<T>(x - rhs.x, y - rhs.y, z - rhs.z);
}

template <typename T> vec3<T> vec3<T>::operator+(const vec3<T>& rhs) const {
    return vec3<T>(x + rhs.x, y + rhs.y, z + rhs.z);
}

template <typename T> vec3<T>& vec3<T>::operator-=(const vec3<T>& rhs) {
    vec3<T> r = (*this) - rhs;
    (*this) = r;
    return *this;
}

template <typename T> template <typename V>
vec3<T> vec3<T>::operator*(const V& rhs) const {
    return vec3<T>(x * rhs, y * rhs, z * rhs);
}

template <typename T> template <typename V>
vec3<T>& vec3<T>::operator*=(const V& rhs) {
    x *= rhs;
    y *= rhs;
    z *= rhs;
    return *this;
}

template <typename T> template <typename V>
vec3<T> vec3<T>::operator/(const V& rhs) const {
    return vec3<T>(x / rhs, y / rhs, z / rhs);
}

template <typename T> template <typename V>
vec3<T>& vec3<T>::operator/=(const V& rhs) {
    x /= rhs;
    y /= rhs;
    z /= rhs;
    return *this;
}

template <typename T> template <typename V>
vec3<T> vec3<T>::operator*(const vec3<V>& rhs) const {
    return vec3<T>(x * rhs.x, y * rhs.y, z * rhs.z);
}

template <typename T> template <typename V>
vec3<T>& vec3<T>::operator*=(const vec3<V>& rhs) {
    x *= rhs.x;
    y *= rhs.y;
    z *= rhs.z;
    return *this;
}

template <typename T> template <typename V>
vec3<T> vec3<T>::operator/(const vec3<V>& rhs) const {
    return vec3<T>(x / rhs.x, y / rhs.y, z / rhs.z);
}

template <typename T> template <typename V>
vec3<T>& vec3<T>::operator/=(const vec3<V>& rhs) {
    x /= rhs.x;
    y /= rhs.y;
    z /= rhs.z;
    return *this;
}

template <typename T> vec3<T>& vec3<T>::operator+=(const vec3<T>& rhs) {
    (*this) = (*this) + rhs;
    return *this;
}

template <typename T> bool vec3<T>::operator==(const vec3<T>& rhs) const {
    return (rhs.x == x) && (rhs.y == y) && (rhs.z == z);
}

template <typename T> bool vec3<T>::operator!=(const vec3<T>& rhs) const {
    return !(*this == rhs);
}

template <typename T> template <typename V> vec3<T>::operator vec3<V>() const {
    return vec3<V>{static_cast<V>(x), static_cast<V>(y), static_cast<V>(z)};
}

template <typename T> T vec3<T>::length_sq() const {
    return x * x + y * y + z * z;
}

template <typename T> T vec3<T>::length() const {
    return std::sqrt(this->length_sq());
}

template <typename T>
std::ostream& operator<<(std::ostream& stream, const vec3<T>& V) {
    return stream << "<" << V.x << ", " << V.y << ", " << V.z << ">";
}

template <typename T> class vec2 {
public:
    T x, y;

    vec2() : x(0), y(0) {}
    vec2(T vx, T vy) : x(vx), y(vy) {}

    vec2 operator+(const vec2& rhs) const;
    vec2& operator+=(const vec2& rhs);
    vec2 operator-(const vec2& rhs) const;
    vec2& operator-=(const vec2& rhs);

    template <typename V> vec2 operator*(const V& rhs) const;
    template <typename V> vec2& operator*=(const V& rhs);

    template <typename V> vec2 operator*(const vec2<V>& rhs) const;
    template <typename V> vec2& operator*=(const vec2<V>& rhs);
    template <typename V> vec2 operator/(const vec2<V>& rhs) const;
    template <typename V> vec2& operator/=(const vec2<V>& rhs);

    bool operator==(const vec2& rhs) const;
    bool operator!=(const vec2& rhs) const;
    bool operator<(const vec2& rhs) const;

    template <typename V> explicit operator vec2<V>() const;

    T length_sq() const;
    T length() const;

    vec2<T> normalized() const;
};

template <typename T> vec2<T> vec2<T>::normalized() const {
    auto r = std::sqrt(this->length_sq());
    return vec2(x / r, y / r);
}

template <typename T> vec2<T> vec2<T>::operator-(const vec2<T>& rhs) const {
    return vec2<T>(x - rhs.x, y - rhs.y);
}

template <typename T> vec2<T> vec2<T>::operator+(const vec2<T>& rhs) const {
    return vec2<T>(x + rhs.x, y + rhs.y);
}

template <typename T> vec2<T>& vec2<T>::operator-=(const vec2<T>& rhs) {
    vec2<T> r = (*this) - rhs;
    (*this) = r;
    return *this;
}

template <typename T> template <typename V>
vec2<T> vec2<T>::operator*(const V& rhs) const {
    return vec2<T>(x * rhs, y * rhs);
}

template <typename T> template <typename V>
vec2<T>& vec2<T>::operator*=(const V& rhs) {
    x *= rhs;
    y *= rhs;
    return *this;
}

template <typename T> template <typename V>
vec2<T> vec2<T>::operator*(const vec2<V>& rhs) const {
    return vec2<T>(x * rhs.x, y * rhs.y);
}

template <typename T> template <typename V>
vec2<T>& vec2<T>::operator*=(const vec2<V>& rhs) {
    x *= rhs.x;
    y *= rhs.y;
    return *this;
}

template <typename T> template <typename V>
vec2<T> vec2<T>::operator/(const vec2<V>& rhs) const {
    return vec2<T>(x / rhs.x, y / rhs.y);
}

template <typename T> template <typename V>
vec2<T>& vec2<T>::operator/=(const vec2<V>& rhs) {
    x /= rhs.x;
    y /= rhs.y;
    return *this;
}

template <typename T> vec2<T>& vec2<T>::operator+=(const vec2<T>& rhs) {
    (*this) = (*this) + rhs;
    return *this;
}

template <typename T> bool vec2<T>::operator==(const vec2<T>& rhs) const {
    return (rhs.x == x) && (rhs.y == y);
}

template <typename T> bool vec2<T>::operator!=(const vec2<T>& rhs) const {
    return !(*this == rhs);
}

template <typename T> bool vec2<T>::operator<(const vec2<T>& rhs) const {
    if (x > rhs.x)
        return false;
    else if (x < rhs.x)
        return true;
    else if (y < rhs.y)
        return true;
    else
        return false;
}

template <typename T> template <typename V> vec2<T>::operator vec2<V>() const {
    return vec2<V>{static_cast<V>(x), static_cast<V>(y)};
}

template <typename T> T vec2<T>::length_sq() const { return x * x + y * y; }

template <typename T> T vec2<T>::length() const {
    return sqrt(this->length_sq());
}

struct colour {
    float r, g, b, a;
};

template <typename T>
std::ostream& operator<<(std::ostream& stream, const vec2<T>& V) {
    return stream << "<" << V.x << ", " << V.y << ">";
}

template <class T, class V> bool box_bound(T box_min, T box_max, V point) {
    return point.x > box_min.x && point.y > box_min.y && point.x < box_max.x &&
           point.y < box_max.y;
}
