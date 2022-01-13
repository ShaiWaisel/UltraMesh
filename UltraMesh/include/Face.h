#pragma once

#include <set>
#include <math.h>
#include "UltraVoxel.h"

#define FLAG_RESET 0
#define FLAG_MARK 1
#define FLAG_OBSOLETE 2

#include "UltraMeshExportImport.h"

class UltraVertex;

class ULTRAMESH_API UltraFace 
{
public:
	UltraFace(int idx1, int idx2, int idx3);
	void CalcPlane(Eigen::Vector3d& p1, Eigen::Vector3d& p2, Eigen::Vector3d& p3);
	double Dist(Eigen::Vector3d& p);
    void CalcPlane(std::vector<UltraVertex>& vertices);
	Eigen::Vector3i Vertices() { return m_vertices; }
	Eigen::Vector3i Edges() { return m_edges; }
	Eigen::Vector4d GetPlane() { return m_plane; }
	Eigen::Vector3d Normal() { return { m_plane(0), m_plane(1), m_plane(2) }; }
	void SetStatus(int status) { m_status = status; }
	int Status() { return m_status; }
	bool IntersectWithRay(std::vector<UltraVertex>& vertices, Eigen::Vector3d& origin, 
		Eigen::Vector3d& direction, Eigen::Vector3d& hit);
    bool RayIntersectsTriangleDist(
        const std::vector<UltraVertex>& vertices,
        const Eigen::Vector3d& rayOrigin,
        const Eigen::Vector3d& rayVector,
        Eigen::Vector3d& outIntersectionPoint);
    bool IsPointInside(const std::vector<UltraVertex>& vertices, const Eigen::Vector3d& point, Eigen::Vector3d& projectedPoint);
    double DistPointPlane(const Eigen::Vector3d& point);
    double ClampDistPoint(const std::vector<UltraVertex>& vertices, const Eigen::Vector3d& point, Eigen::Vector3d& closestPoint);
    bool MaxDistToSkeleton(const std::vector<UltraVertex>& vertices, const Eigen::Vector3d& point, const Eigen::Vector3d& direction,
        const double maxDist, double& distance);
    void SetColors(const std::vector<UltraVertex>& vertices, const double redYellow, const double yellowGreen);


	Eigen::Vector3i m_vertices = { 0, 0, 0 };
	Eigen::Vector3i m_edges = { 0, 0, 0 };
	Eigen::Vector4d m_plane = { 0.0, 0.0, 0.0, 0.0 };
    double m_colors[3][3] = { {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0} };
	double m_area = 0.0;
	int m_status = FLAG_RESET;
};


