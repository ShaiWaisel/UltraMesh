#pragma once

#include "Vertex.h"
#include "UltraMeshExportImport.h"



class ULTRAMESH_API UltraEdge
{
public:
	UltraEdge(int idxV1, int idxV2, int idxFace);
	void SetTwin(int idxTwin) { m_idxTwin = idxTwin; }
	int Twin() { return m_idxTwin; }
	int Face() { return m_idxFace; }
	void SetStatus(int status) { m_status = status; }
	int Status() { return m_status; }
	int Vertex(int id) { return (id == 1) ? m_idxV1 : m_idxV2; }
	double Dihedral() { return m_dihedral; }
    void CalcVector(std::vector<UltraVertex>& vertices);
    bool MaxDistToSkeleton(const std::vector<UltraVertex>& vertices, Eigen::Vector3d point, const Eigen::Vector3d& direction, double& distance);

	int m_idxV1 = -1;
	int m_idxV2 = -1;
	int m_idxFace = -1;
	int m_idxTwin = -1;
    Eigen::Vector3d m_vector = { 0.0, 0.0, 0.0 };
	double m_dihedral = 0.0;
	int m_status = FLAG_RESET;

};

