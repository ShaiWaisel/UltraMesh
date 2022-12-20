#pragma once

#include <vector>
#include <list>
#include "vertex.h"
#include "Edge.h"
//#include "UltraMeshExportImport.h"


class Bucket
{
public:
    Bucket() { ; }
    Bucket(Bounds bounds, int idxX, int idxY, int idxZ);
    bool IsInside(Eigen::Vector3d point);
    std::set<int> m_triangles;
    bool IntersectWithRay(Eigen::Vector3d& origin, Eigen::Vector3d& direction);
    Bounds m_bounds = { DBL_MAX, -DBL_MAX, DBL_MAX, -DBL_MAX, DBL_MAX, -DBL_MAX };
    Eigen::Vector3i m_gridLocation = { 0, 0, 0 };
};

class Zpolyline
{
public:
    Zpolyline(const double z) { m_Z = z; };
    bool AddSegment(const double x1, const double y1, const double x2, const double y2);
    bool Closed() { return m_closed; }
    void StartPoint(int idx, Eigen::Vector3d& p);
    void EndPoint(int idx, Eigen::Vector3d& p);
    int NSegments() { return (int)m_segments.size(); }
    double Length();
    double Z() { return m_Z; }

private:
    double m_Z = 0.0;
    bool m_closed = false;
    std::list<std::pair<std::pair<double, double>, std::pair<double, double>>> m_segments;
};




class UltraMesh
{
public:
	UltraMesh();
    UltraMesh(const UltraMesh& other);
    UltraMesh& operator=(const UltraMesh& other);

	void MapEdges();
	void CalcFaces();
	void CalcNormals(bool byArea);
	void CalcBuckets();
    void CalcCurvature();
	Bounds* CalcBounds();
	std::vector<UltraFace> IntersectWithRay(Eigen::Vector3d& origin, Eigen::Vector3d& direction);
    void Smooth();
    void Transform(Eigen::Affine3d mat);
    void OffsetBySkeleton(double maxOffset);
    void CalcColors(const double redYellow, const double yellowGreen);
    void SaveAsVRML(const std::wstring fileName);
    void CalcThickness(const UltraMesh& otherMesh);
    bool CalcMinimas(std::vector<Eigen::Vector3d>& minimas, std::vector<Eigen::Vector3d>& normals);
    bool CalcSkeleton(double minDistBetweenSkeletonPoints, std::vector<Eigen::Vector3d>& skeleton, std::vector<Eigen::Vector3d>& normals);
    void GetNearestNeighbours(Eigen::Vector3d seed, double radius, std::vector<UltraVertex>& neighbours);
    bool Slice(std::vector<std::pair<double, std::vector<Zpolyline>>>& slices);
    void AlignToMinZ();




	std::vector<UltraVertex> m_vertices;
	std::vector<UltraEdge> m_edges;
	std::vector<UltraFace> m_faces;
	Bounds m_bounds;
	std::vector<Bucket> m_buckets;

};

