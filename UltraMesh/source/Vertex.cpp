#include "../../UltraMesh/include/Vertex.h"
#include "../../UltraMesh/include/Edge.h"



UltraVertex::UltraVertex(Eigen::Vector3d position)
{
	m_position = position;
	m_connectedEdges.clear();
}


void UltraVertex::AddEdgeIndex(int index)
{
	m_connectedEdges.push_back(index);
}

void UltraVertex::MoveTo(Eigen::Vector3d& newPosition)
{
	m_position = newPosition;
}


int UltraVertex::Edge(int index)
{
    return m_connectedEdges[index];
}

void UltraVertex::CalcNormal(std::vector<UltraFace>& faces, std::vector<UltraEdge>& edges, bool byArea)
{
    m_normal.Zero();
    for (auto edgeI : m_connectedEdges)			// Loop over touching faces
    {
        auto& edge = edges[edgeI];						// Touching face
        Eigen::Vector3d norm = faces[edge.m_idxFace].Normal();		// Touching face norm
        bool unique = true;
        for (auto edgeJ = m_connectedEdges.begin(); *edgeJ != edgeI; edgeJ++)
        {
            auto edge2 = edges[*edgeJ];						// Touching face
            Eigen::Vector3d norm2 = faces[edge2.m_idxFace].Normal();
            unique &= (norm.dot(norm2) < 0.99999);
        }
        if (byArea)
            norm *= faces[edge.m_idxFace].m_area;					// Normal is weighted by each touching face area
        if (unique)
            m_normal += norm;
    }

    m_normal.normalize();													// Keep as 1 unit length

}

void UltraVertex::CalcCurvature(std::vector<UltraFace>& faces, std::vector<UltraEdge>& edges, std::vector<UltraVertex>& vertices)
{
    double sumAngDiffs = 0.0;
    double sumDotSpokes = 0.0;
    for (size_t i = 0; i < m_connectedEdges.size(); i++)                 // Loop over touching faces
    {
        UltraEdge* edge = &edges[m_connectedEdges[i]];                       // Extract touhing edge ("spoke") and its twin
        UltraEdge* edgeT = &edges[edge->Twin()];
        UltraFace* f1 = &faces[edge->Face()];                                             // Two planes attached with the said edge
        UltraFace* f2 = &faces[edgeT->Face()];
        Eigen::Vector3d v1 = f1->m_plane.block<3, 1>(0, 0);
        v1.normalize();
        Eigen::Vector3d v2 = f2->m_plane.block<3, 1>(0, 0);
        v2.normalize();
        double ang = acos(v1.dot(v2));
        if (!isnan(ang))
        {
            sumAngDiffs += ang;                                                                                    // Accumulate angles between touching faces
            v2 = edge->m_vector;
            v2.normalize();
            double dotSpoke = (m_normal.dot(v2));                                             // Normalized spoke vector projected on m_normal
            sumDotSpokes += dotSpoke;                                                                      // Accumulate spokes projected on m_normal
        }
    }
    // sumAngDiffs can vary from 0.0 (all faces lay on the same plane) to ~2*PI (for very sharp cone tip)
    // sumDotSpokes is used for it's sign whether convex or concave
    if (sumDotSpokes != 0.0)
         m_curvature = std::max(-1.0, std::min(1.0, -sumAngDiffs * 0.5 / M_PI * sumDotSpokes / abs(sumDotSpokes)));
}

bool UltraVertex::MaxDistToSkeleton(Eigen::Vector3d point, const Eigen::Vector3d& direction, double& distance)
{
    double B = 2 * direction[0] * (m_position[0] - point[0]) +
        2 * direction[1] * (m_position[1] - point[1]) +
        2 * direction[2] * (m_position[2] - point[2]);
    if (B == 0.0)
        return 0.0;
    double C = (point - m_position).squaredNorm();
    double t = C / B;
    //point = point + Eigen::Vector3d( t*direction[0], t*direction[1], t*direction[2]  );
    distance = t;
    return (t>0);
}

void UltraVertex::MoveToSkeleton(std::vector<UltraFace>& faces, std::vector<UltraEdge>& edges, std::vector<UltraVertex>& vertices, double direction, double maxDistance)
{
    Eigen::Vector3d normal = m_normal * direction;
}




