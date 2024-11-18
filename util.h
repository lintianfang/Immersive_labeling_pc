#pragma once


#include <cgv/math/quaternion.h>
#include <cgv/math/fvec.h>
#include <cmath>
#include <memory>
#include <functional>

//types in cgv namespace
using vec3 = cgv::vec3;
using quat = cgv::quat;
using dquat = cgv::dquat;
using box3 = cgv::box3;


inline void tesselate_icosahedron(const float a, cgv::math::fvec<float, 3>* points, int* triangles) {
    const float phi = (1.0 + sqrt(5.0)) / 2.0;

    points[0] = cgv::math::fvec<float, 3>(-1.0, -phi, 0.0);
    points[1] = cgv::math::fvec<float, 3>(-1.0, phi, 0.0);
    points[2] = cgv::math::fvec<float, 3>(1.0, phi, 0.0);
    points[3] = cgv::math::fvec<float, 3>(1.0, -phi, 0.0);

    points[4] = cgv::math::fvec<float, 3>(0.0, -1.0, -phi);
    points[5] = cgv::math::fvec<float, 3>(0.0, -1.0, phi);
    points[6] = cgv::math::fvec<float, 3>(0.0, 1.0, phi);
    points[7] = cgv::math::fvec<float, 3>(0.0, 1.0, -phi);

    points[8] = cgv::math::fvec<float, 3>(-phi, 0.0, -1.0);
    points[9] =  cgv::math::fvec<float, 3>(phi, 0.0, -1.0);
    points[10] = cgv::math::fvec<float, 3>(phi, 0.0, 1.0);
    points[11] = cgv::math::fvec<float, 3>(-phi, 0.0, 1.0);
    
    //adjacent to inner rectangles
    for (int i = 0; i < 3; ++i) {
        int off = 12 * i;
        triangles[off + 0] = i * 4; triangles[off + 1] = i * 4 + 3; triangles[off + 2] = (i * 4 + 4) % 12;
        triangles[off + 3] = i * 4 + 3; triangles[off + 4] = i * 4; triangles[off + 5] = (i * 4 + 5) % 12;
        triangles[off + 6] = i * 4 + 1; triangles[off + 7] = i * 4 + 2; triangles[off + 8] = (i * 4 + 6) % 12;
        triangles[off + 9] = i * 4 + 2; triangles[off + 10] = i * 4 + 1; triangles[off + 11] = (i * 4 + 7) % 12;
    }

    for (int i = 0; i < 2; ++i) {
        int off = 12 * 3 + i * 3*4;
        triangles[off] = (2 * i) % 4; triangles[off+1] = 4+(2*i %4); triangles[off+2] = 8+ (2 * i % 4);
        triangles[off+3] = (2 * i) % 4; triangles[off+4] = 4+((1+2 * i) % 4); triangles[off+5] = 8+((3+2*i) % 4);

        triangles[off+6] = (3 + 2 * i) % 4; triangles[off+7] = 4+(2*i)%4; triangles[off+8] = 8+(1+i*2);
        triangles[off+9] = (3 + 2 * i) % 4; triangles[off+10] = 4+((1 + 2 * i) % 4); triangles[off+11] = 8+((2+i*2)%4);
    }
}

inline const std::array<int, 60>& icosahedron_triangles() {
    static std::array<int, 60> triangles;
    static bool computed = false;
    if (!computed) {
        //adjacent to inner rectangles
        for (int i = 0; i < 3; ++i) {
            int off = 12 * i;
            triangles[off + 0] = i * 4; triangles[off + 1] = i * 4 + 3; triangles[off + 2] = (i * 4 + 4) % 12;
            triangles[off + 3] = i * 4 + 3; triangles[off + 4] = i * 4; triangles[off + 5] = (i * 4 + 5) % 12;
            triangles[off + 6] = i * 4 + 1; triangles[off + 7] = i * 4 + 2; triangles[off + 8] = (i * 4 + 6) % 12;
            triangles[off + 9] = i * 4 + 2; triangles[off + 10] = i * 4 + 1; triangles[off + 11] = (i * 4 + 7) % 12;
        }

        for (int i = 0; i < 2; ++i) {
            int off = 12 * 3 + i * 3 * 4;
            triangles[off] = (2 * i) % 4; triangles[off + 1] = 4 + (2 * i % 4); triangles[off + 2] = 8 + (2 * i % 4);
            triangles[off + 3] = (2 * i) % 4; triangles[off + 4] = 4 + ((1 + 2 * i) % 4); triangles[off + 5] = 8 + ((3 + 2 * i) % 4);

            triangles[off + 6] = (3 + 2 * i) % 4; triangles[off + 7] = 4 + (2 * i) % 4; triangles[off + 8] = 8 + (1 + i * 2);
            triangles[off + 9] = (3 + 2 * i) % 4; triangles[off + 10] = 4 + ((1 + 2 * i) % 4); triangles[off + 11] = 8 + ((2 + i * 2) % 4);
        }
        computed = true;
    }
    return triangles;
}

inline const std::array<cgv::math::fvec<float, 3>, 12>& icosahedron_points() {
    static std::array<cgv::math::fvec<float, 3>, 12> points;
    static bool computed = false;

    if (!computed) {
        const float phi = (1.0 + sqrt(5.0)) / 2.0;

        points[0] = cgv::math::fvec<float, 3>(-1.0, -phi, 0.0);
        points[1] = cgv::math::fvec<float, 3>(-1.0, phi, 0.0);
        points[2] = cgv::math::fvec<float, 3>(1.0, phi, 0.0);
        points[3] = cgv::math::fvec<float, 3>(1.0, -phi, 0.0);

        points[4] = cgv::math::fvec<float, 3>(0.0, -1.0, -phi);
        points[5] = cgv::math::fvec<float, 3>(0.0, -1.0, phi);
        points[6] = cgv::math::fvec<float, 3>(0.0, 1.0, phi);
        points[7] = cgv::math::fvec<float, 3>(0.0, 1.0, -phi);

        points[8] = cgv::math::fvec<float, 3>(-phi, 0.0, -1.0);
        points[9] = cgv::math::fvec<float, 3>(phi, 0.0, -1.0);
        points[10] = cgv::math::fvec<float, 3>(phi, 0.0, 1.0);
        points[11] = cgv::math::fvec<float, 3>(-phi, 0.0, 1.0);
        computed = true;
    }
    return points;
}

/// a hash generator for vec3
struct vec3_hash {
    size_t operator()(const cgv::math::fvec<float, 3>& p) const{
        size_t h = std::hash<float>{}(p.x());
        h << (sizeof(size_t) * 2);
        h ^= std::hash<float>{}(p.y());
        h << (sizeof(size_t) * 2);
        h ^= std::hash<float>{}(p.z());
        return h;
    }
};

/// creates a geodesic sphere from an icosahedron
/// @param[in] subdivisions : number of subdivisions
/// @param[out] points : storage for sphere's points
/// @param[out] triangles : point indices
inline void tesselate_geodesic_sphere(int subdivisions, std::vector<cgv::math::fvec<float, 3>>& points, std::vector<unsigned>& triangles)
{
    points.resize(icosahedron_points().size());
    triangles.resize(icosahedron_triangles().size());
    memcpy(points.data(), icosahedron_points().data(), icosahedron_points().size() * sizeof(cgv::math::fvec<float, 3>));
    memcpy(triangles.data(), icosahedron_triangles().data(), icosahedron_triangles().size() * sizeof(unsigned));

    auto& hash_fx = [](const cgv::math::fvec<float, 3>& p) {
        size_t h = std::hash<float>{}(p.x());
        h << (sizeof(size_t) * 2);
        h ^= std::hash<float>{}(p.y());
        h << (sizeof(size_t) * 2);
        h ^= std::hash<float>{}(p.z());
        return h;
    };

    std::unordered_map<cgv::math::fvec<float, 3>, unsigned, vec3_hash> points_to_index;

    auto& get_point_index = [&](const cgv::math::fvec<float, 3>& p) {
        auto p_it = points_to_index.find(p);
        if (p_it != points_to_index.end()) {
            return p_it->second;
        }
        else {
            unsigned ix = (unsigned)points.size();
            points.push_back(p);
            points_to_index[p] = ix;
            return ix;
        }
    };

    //subdivide
    while (subdivisions > 0) {
        std::vector<unsigned> new_triangles;
        int nr_triangle_points = triangles.size();
        for (int i = 0; i < nr_triangle_points; i += 3) {
            int a_ix = triangles[i];
            int b_ix = triangles[i + 1];
            int c_ix = triangles[i + 2];

            auto dab = 0.5f * (points[a_ix] + points[b_ix]);
            auto dbc = 0.5f * (points[b_ix] + points[c_ix]);
            auto dca = 0.5f * (points[c_ix] + points[a_ix]);

            int dab_ix = get_point_index(dab);
            int dbc_ix = get_point_index(dbc);
            int dca_ix = get_point_index(dca);

            new_triangles.push_back(a_ix);
            new_triangles.push_back(dab_ix);
            new_triangles.push_back(dca_ix);

            new_triangles.push_back(b_ix);
            new_triangles.push_back(dbc_ix);
            new_triangles.push_back(dab_ix);

            new_triangles.push_back(c_ix);
            new_triangles.push_back(dca_ix);
            new_triangles.push_back(dbc_ix);

            new_triangles.push_back(dab_ix);
            new_triangles.push_back(dbc_ix);
            new_triangles.push_back(dca_ix);
        }

        //project everything onto a sphere
        for (auto& p : points) {
            p = cgv::math::normalize(p);
        }

        triangles.swap(new_triangles);
        --subdivisions;
        points_to_index.clear();
    }
}


template<typename GUIProvider>
inline void make_editor_element(GUIProvider& prov,cgv::math::fvec<float,3>& pnt, bool& state, const std::string& node_name) {
    if (prov.begin_tree_node(node_name, state)) {
        prov.add_member_control(&prov, "x", pnt.x(), "value_slider", "min=-10.0;max=10.0;log=false;ticks=true");
        prov.add_member_control(&prov, "y", pnt.y(), "value_slider", "min=-10.0;max=10.0;log=false;ticks=true");
        prov.add_member_control(&prov, "z", pnt.z(), "value_slider", "min=-10.0;max=10.0;log=false;ticks=true");
        prov.end_tree_node(state);
    }

}

template <typename T>
inline quat to_quat(const T& roll, const T& pitch, const T& yaw)
{
    T cy = cos(yaw * 0.5);
    T sy = sin(yaw * 0.5);
    T cp = cos(pitch * 0.5);
    T sp = sin(pitch * 0.5);
    T cr = cos(roll * 0.5);
    T sr = sin(roll * 0.5);

    quat q;
    q.w() = cr * cp * cy + sr * sp * sy;
    q.x() = sr * cp * cy - cr * sp * sy;
    q.y() = cr * sp * cy + sr * cp * sy;
    q.z() = cr * cp * sy - sr * sp * cy;

    return q;
}

template <typename T>
inline quat euler_angels_to_quat(const cgv::math::fvec<T,3>& euler)
{
    return to_quat(euler.x(),euler.y(),euler.z());
}


inline vec3 to_euler_angels(const dquat& q) {
    vec3 angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1.0 - 2 * (q.x() * q.x() + q.y() * q.y());
    angles.x() = (float) std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
    if (std::abs(sinp) >= 1.0)
        angles.y() = (float) std::copysign(M_PI / 2.0, sinp);
    else
        angles.y() = (float) std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    angles.z() = (float) std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

namespace glutil {
    static std::unordered_map< GLenum, std::string> gl_error_to_string = {
    {GL_NO_ERROR, "GL_NO_ERROR"},
    {GL_INVALID_VALUE, "GL_INVALID_VALUE"},
    {GL_INVALID_ENUM, "GL_INVALID_ENUM"},
    {GL_INVALID_OPERATION, "GL_INVALID_OPERATION"},
    {GL_INVALID_FRAMEBUFFER_OPERATION, "GL_INVALID_FRAMEBUFFER_OPERATION"},
    {GL_OUT_OF_MEMORY, "GL_OUT_OF_MEMORY"}
    };

    inline const std::string& get_glError_as_string() {
        static const std::string unknown_err_msg = "unknown error";
        GLenum err = glGetError();
        auto it = gl_error_to_string.find(err);
        if (it != gl_error_to_string.end()) {
            return it->second;
        }
        return unknown_err_msg;
    }
}

