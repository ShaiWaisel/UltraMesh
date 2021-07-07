#include "Edge.h"

UltraEdge::UltraEdge(int idxV1, int idxV2, int idxFace) 
{ 
	m_idxV1 = idxV1; 
	m_idxV2 = idxV2; 
	m_idxFace = idxFace; 
}

void UltraEdge::CalcVector(std::vector<UltraVertex>& vertices)
{
    m_vector = vertices[m_idxV2].m_position - vertices[m_idxV1].m_position;
}


