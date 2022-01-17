#pragma once

#include <vector>
#include "vertex.h"
#include "Edge.h"
#include "UltraMeshExportImport.h"




class ULTRAMESH_API Bucket
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


class ULTRAMESH_API UltraMesh
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
    void OffsetBySkeleton(double maxOffset);
    void CalcColors(const double redYellow, const double yellowGreen);
    void SaveAsVRML(const std::wstring fileName);
    void CalcThickness(const UltraMesh& otherMesh);
    bool CalcMinimas(std::vector<Eigen::Vector3d>& minimas, std::vector<Eigen::Vector3d>& normals);
    bool CalcSkeleton(double minDistBetweenSkeletonPoints, std::vector<Eigen::Vector3d>& skeleton, std::vector<Eigen::Vector3d>& normals);



	std::vector<UltraVertex> m_vertices;
	std::vector<UltraEdge> m_edges;
	std::vector<UltraFace> m_faces;
	Bounds m_bounds;
	std::vector<Bucket> m_buckets;

};

