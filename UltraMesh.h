#pragma once

#include <vector>
#include "vertex.h"
#include "Edge.h"
#include "UltraMeshExportImport.h"


typedef double Bounds[6];
#define EPSILON6 1.0E-6


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
	void MapEdges();
	void CalcFaces();
	void CalcNormals(bool byArea);
	void CalcBuckets();
	Bounds* CalcBounds();
	std::vector<UltraFace> IntersectWithRay(Eigen::Vector3d& origin, Eigen::Vector3d& direction);
    void Smooth();
    void OffsetBySkeleton(double maxOffset);


	std::vector<UltraVertex> m_vertices;
	std::vector<UltraEdge> m_edges;
	std::vector<UltraFace> m_faces;
	Bounds m_bounds;
	std::vector<Bucket> m_buckets;

};

