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


bool UltraEdge::MaxDistToSkeleton(const std::vector<UltraVertex>& vertices, Eigen::Vector3d point, const Eigen::Vector3d& direction, double& distance)
{
 #define MATLAB_DEBUG 0
    Eigen::Vector3d edgeP1 = vertices[m_idxV1].m_position;
    Eigen::Vector3d edgeP2 = vertices[m_idxV2].m_position;
    double edgeLength = (edgeP2 - edgeP1).norm();
    Eigen::Vector3d K = { edgeP2[1]*edgeP1[2] - edgeP1[1]*edgeP1[2] - edgeP2[2]*edgeP1[1] + edgeP1[2]*edgeP1[1],
        edgeP2[2]*edgeP1[0] - edgeP1[2]*edgeP1[0] - edgeP2[0]*edgeP1[2] + edgeP1[0]*edgeP1[2],
        edgeP2[0]*edgeP1[1] - edgeP1[0]*edgeP1[1] - edgeP2[1]*edgeP1[0] + edgeP1[1]*edgeP1[0] };

    double eqA = direction[2] * direction[2] * (-edgeP2[1] + edgeP1[1]) *(-edgeP2[1] + edgeP1[1]) +
        2 * direction[1] * direction[2] * (-edgeP2[1] + edgeP1[1])*(edgeP2[2] - edgeP1[2]) +
        direction[1] * direction[1] * (edgeP2[2] - edgeP1[2]) *(edgeP2[2] - edgeP1[2]) +
        direction[0] * direction[0] * (-edgeP2[2] + edgeP1[2]) *(-edgeP2[2] + edgeP1[2]) +
        2 * direction[2] * direction[0] * (-edgeP2[2] + edgeP1[2])*(edgeP2[0] - edgeP1[0]) +
        direction[2] * direction[2] * (edgeP2[0] - edgeP1[0])*(edgeP2[0] - edgeP1[0]) +
        direction[1] * direction[1] * (-edgeP2[0] + edgeP1[0]) *(-edgeP2[0] + edgeP1[0]) +
        2 * direction[0] * direction[1] * (-edgeP2[0] + edgeP1[0])*(edgeP2[1] - edgeP1[1]) +
        direction[0] * direction[0] * (edgeP2[1] - edgeP1[1]) *(edgeP2[1] - edgeP1[1]);

    double eqB = 2 * point[2] * direction[2] * (-edgeP2[1] + edgeP1[1]) *(-edgeP2[1] + edgeP1[1]) +
        2 * (point[2] * direction[1] + point[1] * direction[2])*(-edgeP2[1] + edgeP1[1])*(edgeP2[2] - edgeP1[2]) +
        2 * point[1] * direction[1] * (edgeP2[2] - edgeP1[2])*(edgeP2[2] - edgeP1[2]) +
        2 * K[0] * (direction[2] * (-edgeP2[1] + edgeP1[1]) + direction[1] * (edgeP2[2] - edgeP1[2])) +
        2 * point[0] * direction[0] * (-edgeP2[2] + edgeP1[2])*(-edgeP2[2] + edgeP1[2]) +
        2 * (point[0] * direction[2] + point[2] * direction[0])*(-edgeP2[2] + edgeP1[2])*(edgeP2[0] - edgeP1[0]) +
        2 * point[2] * direction[2] * (edgeP2[0] - edgeP1[0])*(edgeP2[0] - edgeP1[0]) +
        2 * K[1] * (direction[0] * (-edgeP2[2] + edgeP1[2]) + direction[2] * (edgeP2[0] - edgeP1[0])) +
        2 * point[1] * direction[1] * (-edgeP2[0] + edgeP1[0])*(-edgeP2[0] + edgeP1[0]) +
        2 * (point[1] * direction[0] + point[0] * direction[1])*(-edgeP2[0] + edgeP1[0])*(edgeP2[1] - edgeP1[1]) +
        2 * point[0] * direction[0] * (edgeP2[1] - edgeP1[1])*(edgeP2[1] - edgeP1[1]) +
        2 * K[2] * (direction[1] * (-edgeP2[0] + edgeP1[0]) + direction[0] * (edgeP2[1] - edgeP1[1]));

    double eqC = point[2] * point[2] * (-edgeP2[1] + edgeP1[1])*(-edgeP2[1] + edgeP1[1]) +
        2 * point[2] * point[1] * (-edgeP2[1] + edgeP1[1])*(edgeP2[2] - edgeP1[2]) +
        point[1] * point[1] * (edgeP2[2] - edgeP1[2])*(edgeP2[2] - edgeP1[2]) +
        2 * K[0] * (point[2] * (-edgeP2[1] + edgeP1[1]) + point[1] * (edgeP2[2] - edgeP1[2])) + K[0] * K[0] +
        point[0] * point[0] * (-edgeP2[2] + edgeP1[2])*(-edgeP2[2] + edgeP1[2]) +
        2 * point[0] * point[2] * (-edgeP2[2] + edgeP1[2])*(edgeP2[0] - edgeP1[0]) +
        point[2] * point[2] * (edgeP2[0] - edgeP1[0])*(edgeP2[0] - edgeP1[0]) +
        2 * K[1] * (point[0] * (-edgeP2[2] + edgeP1[2]) + point[2] * (edgeP2[0] - edgeP1[0])) + K[1] * K[1] +
        point[1] * point[1] * (-edgeP2[0] + edgeP1[0])*(-edgeP2[0] + edgeP1[0]) +
        2 * point[1] * point[0] * (-edgeP2[0] + edgeP1[0])*(edgeP2[1] - edgeP1[1]) +
        point[0] * point[0] * (edgeP2[1] - edgeP1[1])*(edgeP2[1] - edgeP1[1]) +
        2 * K[2] * (point[1] * (-edgeP2[0] + edgeP1[0]) + point[0] * (edgeP2[1] - edgeP1[1])) + K[2] * K[2];

    eqA = eqA / edgeLength;
    eqB = eqB / edgeLength;
    eqC = eqC / edgeLength;
    eqA = eqA - 1;
    double dist, T1, T2;
    if (eqA == 0)
    {
        dist = ((edgeP2 - edgeP1).cross(edgeP1 - point)).norm() / (edgeP2 - edgeP1).norm();
        T1 = dist / 2.0;
        T2 = dist / 2.0;
    }
    else
    {
        double det = (eqB * eqB - 4 * eqA * eqC);
        if (det < 0)
            return false;
        T1 = (-eqB + sqrt(det)) / (2 * eqA);
        T2 = (-eqB - sqrt(det)) / (2 * eqA);
    }
    Eigen::Vector3d P1 = point + T1 * direction;
    Eigen::Vector3d P2 = point + T2 * direction;
    double finalT = -1.0;
    if (T1 > 0.0)
    {
        Eigen::Vector3d AB = (edgeP2 - edgeP1);
        AB.normalize();
        Eigen::Vector3d AP = (P1 - edgeP1);
        AP.normalize();
        Eigen::Vector3d PB = (edgeP2 - P1);
        PB.normalize();
        finalT = (AP.dot(AB) > 0.0) && (PB.dot(AB) > 0.0) ? T1 : -1.0;
    }
    if (T2 > 0.0)
    {
        Eigen::Vector3d AB = (edgeP2 - edgeP1);
        AB.normalize();
        Eigen::Vector3d AP = (P2 - edgeP1);
        AP.normalize();
        Eigen::Vector3d PB = (edgeP2 - P2);
        PB.normalize();
        finalT = (AP.dot(AB) > 0.0) && (PB.dot(AB) > 0.0) ? T2 : -1.0;
    }
#if MATLAB_DEBUG
    printf("\nclose all; clear all; \n");
    printf("V1 = [%f %f %f];\n", vertices[m_idxV1].m_position[0], vertices[m_idxV1].m_position[1], vertices[m_idxV1].m_position[2]);
    printf("V2 = [%f %f %f];\n", vertices[m_idxV2].m_position[0], vertices[m_idxV2].m_position[1], vertices[m_idxV2].m_position[2]);
    printf("V=[V1;V2];\n");
    printf("P=[%f %f %f];\n", point[0], point[1], point[2]);
    printf("D=[%f %f %f];\n", direction[0], direction[1], direction[2]);
    printf("t=%f;\n", finalT);
    Eigen::Vector3d proposed = point = point + finalT * direction;
    printf("Proposed=[%f %f %f];\n", proposed[0], proposed[1], proposed[2]);
    printf("figure, axis equal;hold on;\n");
    printf("plot3(V(:,1), V(:,2), V(:,3));\n");
    printf("scatter3(P(1), P(2), P(3),'o','r', 'filled');\n");
    printf("plot3([P(1) P(1)+D(1)*5], [P(2) P(2)+D(2)*5], [P(3) P(3)+D(3)*5],'g');\n");
    printf("scatter3(Proposed(1), Proposed(2), Proposed(3),'*','b');\n");
#endif

    if (finalT >= 0.0)
    {
        Eigen::Vector3d proposed = { point[0] + direction[0] * finalT, point[1] + direction[1] * finalT, point[2] + direction[2] * finalT };
        Eigen::Vector3d newVec = (proposed - point);
        newVec.normalize();
        if (direction.dot(newVec) < 0.0)
            return false;
        //point = point + finalT * direction;
        distance = finalT;
    }
    return (finalT >= 0.0);
}
