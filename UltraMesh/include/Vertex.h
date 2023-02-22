#pragma once

#include "Face.h"
//#include "UltraMeshExportImport.h"


class UltraEdge;

class UltraVertex
{
public:
	UltraVertex(Eigen::Vector3d position);
	void MoveTo(Eigen::Vector3d& newPosition);
	void AddEdgeIndex(int index);
	Eigen::Vector3d Position() { return m_position; }
	std::vector<int> Edges() { return m_connectedEdges; }
	int Edge(int index);
	void SetStatus(int status) { m_status = status; }
    void CalcNormal(std::vector<UltraFace>& faces, std::vector<UltraEdge>& edges, bool byArea);
	int Status() { return m_status; }
    void CalcCurvature(std::vector<UltraFace>& faces, std::vector<UltraEdge>& edges, std::vector<UltraVertex>& vertices);
    bool MaxDistToSkeleton(Eigen::Vector3d point, const Eigen::Vector3d& direction, double& distance);
    void MoveToSkeleton(std::vector<UltraFace>& faces, std::vector<UltraEdge>& edges, std::vector<UltraVertex>& vertices, double direction, double maxDistance);



	int m_index = -1;
	Eigen::Vector3d m_position = { 0.0, 0.0, 0.0 };
	Eigen::Vector3d m_normal = { 0.0, 0.0, 0.0 };
    Eigen::Vector3d m_shadowPosition = { 0.0, 0.0, 0.0 };
    
    std::vector<int> m_connectedEdges;
	double m_curvature = 0.0;
	double m_thickness = 0.0;
    double m_limit = 0.0;
	int m_status = FLAG_RESET;
};

