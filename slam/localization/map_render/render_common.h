#ifndef __RENDER_COMMON_H
#define __RENDER_COMMON_H

#include <boost/functional/hash.hpp>
#include <unordered_map>
#include <unordered_set>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>

const double PointResolution    = 0.02;
const double InvPointResolution = 1.0 / PointResolution;
const double VoxelResolution    = 0.2;
const double InvVoxelResolution = 1.0 / VoxelResolution;
const double MinDepth = 2.0;
const double MaxDepth = 80.0;

class hash_point
{
public:
    int64_t x;
    int64_t y;
    int64_t z;
    hash_point() : x(0), y(0), z(0) {}
    hash_point(int64_t x, int64_t y, int64_t z) : x(x), y(y), z(z) {}

    bool operator==(hash_point const& other) const
    {
        return x == other.x && y == other.y && z == other.z;
    }

    friend std::size_t hash_value(hash_point const& p)
    {
        std::size_t seed = 0;
        boost::hash_combine(seed, p.x);
        boost::hash_combine(seed, p.y);
        boost::hash_combine(seed, p.z);

        return seed;
    }
};

class hash_uv
{
public:
    int u;
    int v;
    hash_uv() : u(0), v(0) {}
    hash_uv(int u, int v) : u(u), v(v) {}

    bool operator==(hash_uv const& other) const
    {
        return u == other.u && v == other.v;
    }

    friend std::size_t hash_value(hash_uv const& p)
    {
        std::size_t seed = 0;
        boost::hash_combine(seed, p.u);
        boost::hash_combine(seed, p.v);

        return seed;
    }
};

namespace std {
    template<> struct hash<::hash_point> : boost::hash<::hash_point> {};
    template<> struct hash<::hash_uv> : boost::hash<::hash_uv> {};
}

template <typename data_type = int64_t, typename T = void *>
struct hash_map_3d
{
    using hash_3d_T = std::unordered_map<hash_point, T>;
    hash_3d_T m_map_3d_hash_map;
    void insert(const data_type &x, const data_type &y, const data_type &z, const T &target)
    {
        m_map_3d_hash_map[hash_point(x, y, z)] = target;
    }

    int if_exist(const data_type &x, const data_type &y, const data_type &z)
    {
        if(m_map_3d_hash_map.find(hash_point(x, y, z)) == m_map_3d_hash_map.end())
        {
            return 0;
        }
        return 1;
    }

    bool empty()
    {
        return m_map_3d_hash_map.empty();
    }

    size_t size()
    {
        return m_map_3d_hash_map.size();
    }

    void clear()
    {
        m_map_3d_hash_map.clear();
    }
};

struct ColmapPoint2d
{
    double x;
    double y;
    uint64_t point3d_id;
};

struct ColmapTrack
{
    uint32_t image_id {0};
    uint32_t point2d_id {0};
};

struct ColmapCamera
{
    uint32_t camera_id;
    int model_id;
    uint64_t width;
    uint64_t height;
    std::vector<double> params;
};

struct ColmapImage
{
    uint32_t camera_id;
    uint64_t image_id;
    Eigen::Quaterniond quat;
    Eigen::Vector3d trans;
    std::string image_name;
    std::vector<ColmapPoint2d> points2d;
    cv::Mat image;
};

struct ColmapPoint3d
{
    uint64_t point_id;
    Eigen::Vector3d pose;
    Eigen::Vector3d color;
    std::vector<ColmapTrack> tracks;
};

namespace Render {

struct PointType
{
    PointType() {}
    PointType(const Eigen::Matrix<float, 3, 1> &pos) : m_pos(pos) {}
    Eigen::Matrix<float, 3, 1> m_pos   {0, 0, 0};
    Eigen::Matrix<float, 3, 1> m_rgb   {0, 0, 0};
    float                      m_score {0};
    float                      m_depth {100.0};
    std::vector<ColmapTrack>   m_tracks;
};

struct Voxel
{
    std::vector<PointType>      m_pts;
    std::unordered_set<int32_t> m_set;
    int32_t                     m_hits  {0};
    double                      m_depth {100.0};
    int64_t                     m_cx    {0};
    int64_t                     m_cy    {0};
    int64_t                     m_cz    {0};
};

using VoxelPtr = std::shared_ptr<Voxel>;

struct PixelType
{
    cv::Vec3b   bgr;
    VoxelPtr    voxel;
    int         idx;
    double      depth {-1.0};
    double      u     { 0.0};
    double      v     { 0.0};
    float       score { 0.0};
    ColmapTrack track;
};

using PixelPtr = std::shared_ptr<PixelType>;

struct VoxelPixel
{
    double                  depth {-1.0};
    std::vector<PixelPtr>   pixel;
};

inline void update_voxel_rgb(PixelPtr &pixel)
{
    if (pixel->score < 0.1) {
        return;
    }

    // add visible image track
    if (pixel->track.image_id > 0) {
        pixel->voxel->m_pts[pixel->idx].m_tracks.emplace_back(pixel->track);
    }

    if (pixel->depth > (pixel->voxel->m_pts[pixel->idx].m_depth * 1.5)) {
        return;
    }

    if (pixel->depth < pixel->voxel->m_pts[pixel->idx].m_depth) {
        pixel->voxel->m_pts[pixel->idx].m_depth = pixel->depth;
    }

    const double &score1 = pixel->voxel->m_pts[pixel->idx].m_score;
    const double &score2 = pixel->score;
    pixel->voxel->m_pts[pixel->idx].m_rgb[0] = (pixel->voxel->m_pts[pixel->idx].m_rgb[0] * score1 + pixel->bgr(2) * score2) / (score1 + score2);
    pixel->voxel->m_pts[pixel->idx].m_rgb[1] = (pixel->voxel->m_pts[pixel->idx].m_rgb[1] * score1 + pixel->bgr(1) * score2) / (score1 + score2);
    pixel->voxel->m_pts[pixel->idx].m_rgb[2] = (pixel->voxel->m_pts[pixel->idx].m_rgb[2] * score1 + pixel->bgr(0) * score2) / (score1 + score2);
    pixel->voxel->m_pts[pixel->idx].m_score  = std::max(score1, score2);
}

}

#endif // __RENDER_COMMON_H