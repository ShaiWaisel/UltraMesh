#pragma once
#include "C:/Eigen3.3.7/Eigen/Dense"

#include "UltraMeshExportImport.h"
#include <set>
#include <array>

typedef double Bounds[6];

#define EPSILON6 1.0E-6


class UltraFace;
class UltraVertex;

class Voxel
{
public:
    Voxel();
    ~Voxel();
    long int ancestor = 0;
    int cost = 0;
    double depth = 0.0;

private:

};


/* ============================================================================================== */

class ULTRAMESH_API VoxelVolume
{
public:
    VoxelVolume(const Bounds& bounds, const double resolution);
    ~VoxelVolume();
    void CalcBorder(const std::vector<UltraFace>& faces, const std::vector<UltraVertex>& vertices);
    void CalcSecond();
    bool CalcLayer(const int neighbourLayer, const int newLayer);
    void CalcOutside();
    void CalcDepth();
    void Render(const int cost, std::vector< Eigen::Vector3i>& ijks);
    void RenderByDepth(const double fromDepth, const double toDepth, std::vector< Eigen::Vector3i>& ijks);
    std::set<std::array<int, 3>> Border() { return m_border; }
    std::set<std::array<int, 3>> Outside() { return m_outside; }

private:
    void XYZ2IJK(const Eigen::Vector3d& xyz, Eigen::Vector3i& ijk);
    void IJK2XYZ(const Eigen::Vector3i& ijk, Eigen::Vector3d& xyz);
    void DrawLine(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const int cost,
        const int faceIdx);
    void DrawTrig(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3, const int cost,
        const int faceIdx);
    void SetDepth(Eigen::Vector3i ijk);
    void FloodFill(const const Eigen::Vector3i& seed, const int& fromValue, const int& toValue, int depth);
    bool FindSeed(const int seedValue, const int neighborValue, Eigen::Vector3i& ijk);
    bool HasNeighbor(const int i, const int j, const int k, const int cost);
    double m_brickSize = 0;
    Bounds m_bounds = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    Eigen::Vector3i m_size = { 0, 0, 0 };
    Eigen::Vector3d m_margins = { 0.0, 0.0, 0.0 };
    std::vector<std::vector<std::vector<Voxel>>> m_voxels;
    std::set<std::array<int,3>> m_border;
    std::vector<Eigen::Vector3d> m_faceNormals;
    std::vector<Eigen::Vector3d> m_faceCenters;
    std::set<std::array<int, 3>> m_outside;

};
