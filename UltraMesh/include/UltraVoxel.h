#pragma once
#include "C:/Eigen3.3.7/Eigen/Dense"

//#include "UltraMeshExportImport.h"
#include <set>
#include <array>

typedef double Bounds[6];

#define EPSILON6 1.0E-6
#define VOXEL_FLAG_EMPTY 0
#define VOXEL_FLAG_THIN 1
#define VOXEL_FLAG_THICK 2
#define VOXEL_FLAG_TRANSIENT 4




class UltraFace;
class UltraVertex;

struct UltraVoxel
{
    long int ancestor = 0;
    int layer = 0;
    double depth = 0.0;
    char flag = 0;

};


/* ============================================================================================== */

class  VoxelVolume
{
public:
    VoxelVolume(const Bounds& bounds, const double resolution);
    ~VoxelVolume();
    void CalcSurfaceLayer(const std::vector<UltraFace>& faces, const std::vector<UltraVertex>& vertices);
    int CalcSecond(bool diagonal);
    int CalcLayer(const int neighbourLayer, const int newLayer, bool diagonal);
    void CalcOutside();
    void CalcDepth(bool diagonal);
    void Render(const int cost, std::vector< Eigen::Vector3i>& ijks);
    void ClassifyByDepth(const double fromDepth, const double toDepth, char flag);
    void RenderByFlag(char flag, std::vector< Eigen::Vector3i>& ijks);
    void AdjustNeighboringFlags();
    void Dilate(const int layer, const char flag, const int layers);
    std::set<std::array<int, 3>> Border() { return m_border; }
    std::set<std::array<int, 3>> Outside() { return m_outside; }

private:
    void XYZ2IJK(const Eigen::Vector3d& xyz, Eigen::Vector3i& ijk);
    void IJK2XYZ(const Eigen::Vector3i& ijk, Eigen::Vector3d& xyz);
    void DrawLine(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const int cost,
        const int faceIdx);
    void DrawLine(const Eigen::Vector3i& p1, const Eigen::Vector3i& p2, const int cost,
        const int faceIdx);
    void DrawTrig(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3, const int cost,
        const int faceIdx);
    void DrawTrig(const Eigen::Vector3i& p1, const Eigen::Vector3i& p2, const Eigen::Vector3i& p3, const int cost,
        const int faceIdx);
    void SetDepth(Eigen::Vector3i ijk, bool diagonal);
    void FloodFill(const Eigen::Vector3i seed, const int& fromValue, const int& toValue, int depth);
    void FloodFillRays(const Eigen::Vector3i seed, const int& fromValue, const int& toValue, int depth);
    bool FindSeed(const int seedValue, const int neighborValue, Eigen::Vector3i& ijk);
    bool HasNeighbor(const int& i, const int& j, const int& k, const int& layer, bool diagonal);
    bool HasNeighbor(const int& i, const int& j, const int& k, const char& flags, bool diagonal);
    bool FindLayer(const int& fromI, const int& fromJ, const int& fromK, const int dirI, const int dirJ, const int dirK, const int targetLayer);
    double m_brickSize = 0;
    Bounds m_bounds = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    Eigen::Vector3i m_size = { 0, 0, 0 };
    Eigen::Vector3d m_margins = { 0.0, 0.0, 0.0 };
    std::vector<std::vector<std::vector<UltraVoxel>>> m_voxels;
    std::set<std::array<int,3>> m_border;
    std::vector<Eigen::Vector3d> m_faceNormals;
    std::set<std::array<int, 3>> m_outside;

};
