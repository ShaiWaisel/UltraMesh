#include "../../UltraMesh/include/UltraVoxel.h"
#include "../../UltraMesh/include/Vertex.h"

#define MAX_RECURSION_DEPTH 700



VoxelVolume::VoxelVolume(const Bounds& bounds, const double resolution)
{
    printf("Initiating volume...");
    m_brickSize = resolution;
    m_margins = { (trunc((bounds[1] - bounds[0] + resolution) / resolution) * resolution - (bounds[1] - bounds[0])) / 2.0,
        (trunc((bounds[3] - bounds[2] + resolution) / resolution) * resolution - (bounds[3] - bounds[2])) / 2.0,
        (trunc((bounds[5] - bounds[4] + resolution) / resolution) * resolution - (bounds[5] - bounds[4])) / 2.0 };
    m_bounds[0] = bounds[0] - m_margins[0];
    m_bounds[1] = bounds[1] + m_margins[0];
    m_bounds[2] = bounds[2] - m_margins[1];
    m_bounds[3] = bounds[3] + m_margins[1];
    m_bounds[4] = bounds[4] - m_margins[2];
    m_bounds[5] = bounds[5] + m_margins[2];
    m_size = { (int)trunc((m_bounds[1] - m_bounds[0]) / resolution),
    (int)trunc((m_bounds[3] - m_bounds[2]) / resolution),
    (int)trunc((m_bounds[5] - m_bounds[4]) / resolution)};
    m_voxels.resize(m_size[0] + 2);
    for (int i = 0; i <= m_size[0] + 1; i++)
    {
        m_voxels[i].resize(m_size[1] + 2);
        for (int j = 0; j <= m_size[1] + 1; j++)
        {
            m_voxels[i][j].resize(m_size[2] + 2);
            for (int k = 0; k <= m_size[2] + 1; k++)
            {
                m_voxels[i][j][k].layer = 0;
                m_voxels[i][j][k].flag = VOXEL_FLAG_EMPTY;
            }

        }

    }
    printf("%d X %d X %d\n", m_size[0], m_size[1], m_size[2]);
}

VoxelVolume::~VoxelVolume()
{
}

void VoxelVolume::XYZ2IJK(const Eigen::Vector3d& xyz, Eigen::Vector3i& ijk)
{
    double invSize = 1.0 / m_brickSize;
    ijk[0] = (int)trunc((xyz[0] - m_bounds[0] - m_margins[0]) * invSize) + 1;
    ijk[1] = (int)trunc((xyz[1] - m_bounds[2] - m_margins[1]) * invSize) + 1;
    ijk[2] = (int)trunc((xyz[2] - m_bounds[4] - m_margins[2]) * invSize) + 1;
}

void VoxelVolume::IJK2XYZ(const Eigen::Vector3i& ijk, Eigen::Vector3d& xyz)
{
    xyz[0] = m_bounds[0] + m_margins[0] + (ijk[0] + 1.5) * m_brickSize;
    xyz[1] = m_bounds[2] + m_margins[1] + (ijk[1] + 1.5) * m_brickSize;
    xyz[2] = m_bounds[4] + m_margins[2] + (ijk[2] + 1.5) * m_brickSize;
}

void VoxelVolume::DrawLine(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const int cost, const int faceIdx)
{
    Eigen::Vector3i ip, ip1, ip2;
    XYZ2IJK(p1, ip1);
    XYZ2IJK(p2, ip2);
    int N = (int)round((p2 - p1).norm() / (m_brickSize * 0.2)) + 1;
   
    for (int i = 0; i < N + 1; i++)
    {
        double factor = (double)i / N;
        Eigen::Vector3d fp = p1 + factor * (p2 - p1);
        XYZ2IJK(fp, ip);
        m_voxels[ip[0]][ip[1]][ip[2]].layer = cost;
        m_voxels[ip[0]][ip[1]][ip[2]].ancestor = faceIdx;
        m_border.insert({ ip[0], ip[1], ip[2] });
        continue;
        Eigen::Vector3i ijk = { (int)trunc(ip1[0] + (ip2[0] - ip1[0]) * factor) + 1,
        (int)trunc(ip1[1] + (ip2[1] - ip1[1]) * factor) + 1,
        (int)trunc(ip1[2] + (ip2[2] - ip1[2]) * factor) + 1};
        m_voxels[ijk[0]][ijk[1]][ijk[2]].layer = cost;
        m_voxels[ijk[0]][ijk[1]][ijk[2]].ancestor = faceIdx;
        m_border.insert({ ijk[0], ijk[1], ijk[2] });
    }
}

void VoxelVolume::DrawLine(const Eigen::Vector3i& p1, const Eigen::Vector3i& p2, const int cost, const int faceIdx)
{
    int dx = abs(p1[0] - p2[0]), dy = abs(p1[1] - p2[1]), dz = abs(p1[2] - p2[2]);
    Eigen::Vector3i ip;
    int iDir = 0;
    if ((dx >= dy) && (dx >= dz))
    {                              // Main axis X
        iDir = (p2[0] >= p1[0]) ? 1 : -1;
        for (int iX = p1[0]; iX * iDir <= p2[0] * iDir; iX+= iDir)
        {
            double factor = (p1[0] == p2[0]) ? 1.0 : (double)(iX - p1[0]) / (p2[0] - p1[0]);
            ip << (int)(iX, p1[1] + factor * (p2[1] - p1[1]), p1[2] + factor * (p2[2] - p1[2]));
            m_voxels[ip[0]][ip[1]][ip[2]].layer = cost;
            m_voxels[ip[0]][ip[1]][ip[2]].ancestor = faceIdx;
            m_border.insert({ ip[0], ip[1], ip[2] });
        }
    }
    else 
        if ((dy >= dx) && (dy >= dz))
        {                      // Main axis Y
            iDir = (p2[1] >= p1[1]) ? 1 : -1;
            for (int iY = p1[1]; iY * iDir <= p2[1] * iDir; iY+= iDir)
            {
                double factor = (p1[1] == p2[1]) ? 1.0 : (double)(iY - p1[1]) / (p2[1] - p1[1]);
                ip << (int)(p1[0] + factor * (p2[0] - p1[0]), iY, p1[2] + factor * (p2[2] - p1[2]));
                m_voxels[ip[0]][ip[1]][ip[2]].layer = cost;
                m_voxels[ip[0]][ip[1]][ip[2]].ancestor = faceIdx;
                m_border.insert({ ip[0], ip[1], ip[2] });
            }
        }
        else
        {                      // Main axis Z
            iDir = (p2[2] >= p1[2]) ? 1 : -1;

            for (int iZ = p1[2]; iZ * iDir <= p2[2] * iDir; iZ += iDir)
            {
                double factor = (p1[2] == p2[2]) ? 1.0 : (double)(iZ - p1[2]) / (p2[2] - p1[2]);
                ip << (int)(p1[0] + factor * (p2[0] - p1[0]), p1[1] + factor * (p2[1] - p1[1]), iZ);
                m_voxels[ip[0]][ip[1]][ip[2]].layer = cost;
                m_voxels[ip[0]][ip[1]][ip[2]].ancestor = faceIdx;
                m_border.insert({ ip[0], ip[1], ip[2] });
            }
        }

 }


void VoxelVolume::DrawTrig(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, const Eigen::Vector3d& p3, const int cost,
    const int faceIdx)
{
    const double len1 = (p2 - p1).norm();
    const double len2 = (p3 - p2).norm();
    const double len3 = (p1 - p3).norm();
    const int N = (int)trunc(std::max(std::max(len1, len2), len3) / (m_brickSize)) + 1;
    Eigen::Vector3d pA, pB, pC;
    if ((len1 >= len2) && (len1 >= len3))
    {
        pA = p1;
        pB = p2; 
        pC = p3;
    }
    else   if ((len2 >= len1) && (len2 >= len3))
    {
        pA = p2;
        pB = p3;
        pC = p1;
    }
    else
    {
        pA = p3;
        pB = p1;
        pC = p2;
    }

    for (int i = 0; i < N + 1; i++)
    {
        double factor = (double)i / N;
        Eigen::Vector3d fp1 = pA + factor * (pB - pA);
        Eigen::Vector3d fp2 = pA + factor * (pC - pA);
        DrawLine(fp1, fp2, cost, faceIdx);
    }
    Eigen::Vector3d normal = (pB - pA).cross(pC - pA);
    normal.normalize();
    Eigen::Vector3d center = (pA + pB + pC) / 3.0;
    m_faceNormals.push_back(normal);
 }

void VoxelVolume::DrawTrig(const Eigen::Vector3i& p1, const Eigen::Vector3i& p2, const Eigen::Vector3i& p3, const int cost,
    const int faceIdx)
{
    const int len1 = (p2 - p1).norm();
    const int len2 = (p3 - p2).norm();
    const int len3 = (p1 - p3).norm();
    Eigen::Vector3i pA, pB, pC;
    if ((len1 >= len2) && (len1 >= len3))
    {
        pA = p1;
        pB = p2;
        pC = p3;
    }
    else   if ((len2 >= len1) && (len2 >= len3))
    {
        pA = p2;
        pB = p3;
        pC = p1;
    }
    else
    {
        pA = p3;
        pB = p1;
        pC = p2;
    }

    int dx = abs(pA[0] - pB[0]), dy = abs(pA[1] - pB[1]), dz = abs(pA[2] - pB[2]);
    Eigen::Vector3i pAB, pACB;
    int iDir = 0;
    if ((dx >= dy) && (dx >= dz))
    {
        // slice along X
        iDir = (pB[0] >= pA[0]) ? 1 : -1;
         for (int iX = pA[0]; iX * iDir <= pB[0] * iDir; iX += iDir)
         {
             double factorAB = (pB[0] == pA[0]) ? 1.0 : (double)(iX - pA[0]) / (pB[0] - pA[0]);
             double factorACB = 0.0;
            pAB << (int)(iX, pA[1] + factorAB * (pB[1] - pA[1]), pA[2] + factorAB * (pB[2] - pA[2]));
            if (iX*iDir <= pC[0]*iDir)
            {
                factorACB = (pC[0] == pA[0]) ? 1.0 : (double)(iX - pA[0]) / (pC[0] - pA[0]);
                pACB << (int)(iX, pA[1] + factorACB * (pC[1] - pA[1]), pA[2] + factorACB * (pC[2] - pA[2]));
            }
            else
            {
                factorACB = (pB[0] == pC[0]) ? 1.0 : (double)(iX - pC[0]) / (pB[0] - pC[0]);
                pACB << (int)(iX, pC[1] + factorACB * (pB[1] - pC[1]), pC[2] + factorACB * (pB[2] - pC[2]));
            }
            DrawLine(pAB, pACB, cost, faceIdx);
        }

    }
    else if ((dy >= dx) && (dy >= dz))
    {
        // slice along Y
        iDir = (pB[1] >= pA[1]) ? 1 : -1;
        for (int iY = pA[1]; iY * iDir <= pB[1] * iDir; iY += iDir)
        {
            double factorAB = (pB[1] == pA[1]) ? 1.0 : (double)(iY - pA[1]) / (pB[1] - pA[1]);
            double factorACB = 0.0;
            pAB << (int)(pA[0] + factorAB * (pB[0] - pA[0]), iY, pA[2] + factorAB * (pB[2] - pA[2]));
            if (iY*iDir <= pC[1]*iDir)
            {
                factorACB = (pC[1] == pA[1]) ? 1.0 : (double)(iY - pA[1]) / (pC[1] - pA[1]);
                pACB << (int)(pA[0] + factorACB * (pC[0] - pA[0]), iY, pA[2] + factorACB * (pC[2] - pA[2]));
            }
            else
            {
                factorACB = (pB[1] == pC[1]) ? 1.0 : (double)(iY - pC[1]) / (pB[1] - pC[1]);
                pACB << (int)(pC[0] + factorACB * (pB[0] - pC[0]), iY, pC[2] + factorACB * (pB[2] - pC[2]));
            }
            DrawLine(pAB, pACB, cost, faceIdx);
        }

    }   
    else
    {
        // slice along Z
        iDir = (pB[2] >= pA[2]) ? 1 : -1;
        for (int iZ = pA[2]; iZ * iDir <= pB[2] * iDir; iZ+= iDir)
        {
            double factorAB = (pB[2] == pA[2]) ? 1.0 : (double)(iZ - pA[2]) / (pB[2] - pA[2]);
            double factorACB = 0.0;
            pAB << (int)(pA[0] + factorAB * (pB[0] - pA[0]), pA[1] + factorAB * (pB[1] - pA[1]), iZ);
            if (iZ*iDir <= pC[2] * iDir)
            {
                factorACB = (pC[2] == pA[2]) ? 1.0 : (double)(iZ - pA[2]) / (pC[2] - pA[2]);
                pACB << (int)(pA[0] + factorACB * (pC[0] - pA[0]), pA[1] + factorACB * (pC[1] - pA[1]), iZ);
            }
            else
            {
                factorACB = (pB[2] == pC[2]) ? 1.0 : (double)(iZ - pC[2]) / (pB[2] - pC[2]);
                pACB << (int)(pC[0] + factorACB * (pB[0] - pC[0]), pC[1] + factorACB * (pB[1] - pC[1]), iZ);
            }
            DrawLine(pAB, pACB, cost, faceIdx);
        }

    }



}



void VoxelVolume::CalcSurfaceLayer(const std::vector<UltraFace>& faces, const std::vector<UltraVertex>& vertices)
{
    for (int iFace = 0; iFace < faces.size(); iFace++)
    {
        //Eigen::Vector3i p1, p2, p3;
        //XYZ2IJK(vertices[faces[iFace].m_vertices[0]].m_position, p1);
        //XYZ2IJK(vertices[faces[iFace].m_vertices[1]].m_position, p2);
        //XYZ2IJK(vertices[faces[iFace].m_vertices[2]].m_position, p3);
        //DrawTrig(p1, p2, p3, 1, iFace);

        DrawTrig(vertices[faces[iFace].m_vertices[0]].m_position,
            vertices[faces[iFace].m_vertices[1]].m_position,
            vertices[faces[iFace].m_vertices[2]].m_position, 1, iFace);
        if ((iFace % 877) == 0)
        printf("\rBorder: %3.1f%%", (double)iFace / faces.size()*100.0);
    }
    printf("\rBorder: 100.0%%\n");
}

int VoxelVolume::CalcSecond(bool diagonal)
{
    Eigen::Vector3i ijk;
    int newVoxels = 0;
    for (int i = 1; i <= m_size[0]; i++)
        for (int j = 1; j <= m_size[1]; j++)
            for (int k = 1; k <= m_size[2]; k++)
            {
                bool found = (m_voxels[i][j][k].layer == 0);
                if (found)
                {
                    ijk[0] = -1;
                    if (m_voxels[i + 1][j][k].layer == 1)
                        ijk = { i + 1, j, k };
                    else if (m_voxels[i - 1][j][k].layer == 1)
                        ijk = { i - 1, j, k };
                    else if (m_voxels[i][j + 1][k].layer == 1)
                        ijk = { i, j + 1, k };
                    else if (m_voxels[i][j - 1][k].layer == 1)
                        ijk = { i, j - 1, k };
                    else if (m_voxels[i][j][k + 1].layer == 1)
                        ijk = { i, j, k + 1 };
                    else if (m_voxels[i][j][k - 1].layer == 1)
                        ijk = { i, j, k - 1 };
                    if (diagonal)
                    {
                        if (m_voxels[i + 1][j + 1][k].layer == 1)
                            ijk = { i + 1, j + 1, k };
                        else if (m_voxels[i - 1][j + 1][k].layer == 1)
                            ijk = { i - 1, j + 1, k };
                        else if (m_voxels[i + 1][j - 1][k].layer == 1)
                            ijk = { i + 1, j - 1, k };
                        else if (m_voxels[i - 1][j - 1][k].layer == 1)
                            ijk = { i - 1, j - 1, k };
                        else if (m_voxels[i + 1][j][k + 1].layer == 1)
                            ijk = { i + 1, j, k + 1 };
                        else if (m_voxels[i - 1][j][k + 1].layer == 1)
                            ijk = { i - 1, j, k + 1 };
                        else if (m_voxels[i][j + 1][k + 1].layer == 1)
                            ijk = { i, j + 1, k + 1 };
                        else if (m_voxels[i][j - 1][k + 1].layer == 1)
                            ijk = { i, j - 1, k + 1 };
                        else if (m_voxels[i + 1][j][k - 1].layer == 1)
                            ijk = { i + 1, j, k - 1 };
                        else if (m_voxels[i - 1][j][k - 1].layer == 1)
                            ijk = { i - 1, j, k - 1 };
                        else if (m_voxels[i][j + 1][k - 1].layer == 1)
                            ijk = { i, j + 1, k - 1 };
                        else if (m_voxels[i][j - 1][k - 1].layer == 1)
                            ijk = { i, j - 1, k - 1 };

                    }
                   
                    found = (ijk[0] >= 0);
                    if (!found)
                        continue;
                    m_voxels[i][j][k].layer = -1; // assume outside
                    //   if cant find border in all directions then must be outside therefore discarded
                    if (!FindLayer(i, j, k, -1, 0, 0, 1))
                        continue;
                    if (!FindLayer(i, j, k, 1, 0, 0, 1))
                        continue;
                    if (!FindLayer(i, j, k, 0, -1, 0, 1))
                        continue;
                    if (!FindLayer(i, j, k, 0, 1, 0, 1))
                        continue;
                    if (!FindLayer(i, j, k, 0, 0, -1, 1))
                        continue;
                    if (!FindLayer(i, j, k, 0, 0, 1, 1))
                        continue;
                    // passed all tests therefore considered to be inside
                    // optional geometric filter, fails on very big triangles, TBC

  /*                  int faceIdx = m_voxels[ijk[0]][ijk[1]][ijk[2]].ancestor;
                    Eigen::Vector3d normal = m_faceNormals[faceIdx];
                    Eigen::Vector3d center = m_faceCenters[faceIdx];
                    Eigen::Vector3d xyz;
                    IJK2XYZ({ i,j,k }, xyz);
                    Eigen::Vector3d dir = (xyz - center);
                    dir.normalize();
                    if (dir.dot(normal) > 0.0)
                        continue;
*/
                    m_voxels[i][j][k].ancestor = m_voxels[ijk[0]][ijk[1]][ijk[2]].ancestor;
                    m_voxels[i][j][k].layer = 2;
                    newVoxels++;
  
                }
            }
  
    for (int i = 0; i < m_size[0]; i++)
        for (int j = 0; j < m_size[1]; j++)
            for (int k = 0; k < m_size[2]; k++)
            {
                if ((m_voxels[i][j][k].layer == 1) && (!HasNeighbor(i, j, k, 2, false)))
                    m_voxels[i][j][k].layer = 0;
            }

    return newVoxels;
}

int VoxelVolume::CalcLayer(const int neighbourLayer, const int newLayer, bool diagonal)
{
    Eigen::Vector3i ijk;
    int newVoxels = 0;

    for (int i = 1; i < m_size[0]-1; i++)
        for (int j = 1; j < m_size[1]-1; j++)
            for (int k = 1; k < m_size[2]-1; k++)
            {
                bool modify = (m_voxels[i][j][k].layer == 0);
                if (modify)
                {
                    ijk[0] = -1;
                    if (m_voxels[i + 1][j][k].layer == neighbourLayer)
                        ijk = { i + 1, j, k };
                    else if (m_voxels[i - 1][j][k].layer == neighbourLayer)
                        ijk = { i - 1, j, k };
                    else if (m_voxels[i][j + 1][k].layer == neighbourLayer)
                        ijk = { i, j + 1, k };
                    else if (m_voxels[i][j - 1][k].layer == neighbourLayer)
                        ijk = { i, j - 1, k };
                    else if (m_voxels[i][j][k + 1].layer == neighbourLayer)
                        ijk = { i, j, k + 1 };
                    else if (m_voxels[i][j][k - 1].layer == neighbourLayer)
                        ijk = { i, j, k - 1 };
                    if (diagonal)
                    {
                        if (m_voxels[i + 1][j + 1][k].layer == neighbourLayer)
                            ijk = { i + 1, j + 1, k };
                        else if (m_voxels[i - 1][j + 1][k].layer == neighbourLayer)
                            ijk = { i - 1, j + 1, k };
                        else if (m_voxels[i + 1][j - 1][k].layer == neighbourLayer)
                            ijk = { i + 1, j - 1, k };
                        else if (m_voxels[i - 1][j - 1][k].layer == neighbourLayer)
                            ijk = { i - 1, j - 1, k };
                        else if (m_voxels[i + 1][j][k + 1].layer == neighbourLayer)
                            ijk = { i + 1, j, k + 1 };
                        else if (m_voxels[i - 1][j][k + 1].layer == neighbourLayer)
                            ijk = { i - 1, j, k + 1 };
                        else if (m_voxels[i][j + 1][k + 1].layer == neighbourLayer)
                            ijk = { i, j + 1, k + 1 };
                        else if (m_voxels[i][j - 1][k + 1].layer == neighbourLayer)
                            ijk = { i, j - 1, k + 1 };
                        else if (m_voxels[i + 1][j][k - 1].layer == neighbourLayer)
                            ijk = { i + 1, j, k - 1 };
                        else if (m_voxels[i - 1][j][k - 1].layer == neighbourLayer)
                            ijk = { i - 1, j, k - 1 };
                        else if (m_voxels[i][j + 1][k - 1].layer == neighbourLayer)
                            ijk = { i, j + 1, k - 1 };
                        else if (m_voxels[i][j - 1][k - 1].layer == neighbourLayer)
                            ijk = { i, j - 1, k - 1 };

                    }
                    modify = (ijk[0] >= 0);
                    if (modify)
                    {
                        m_voxels[i][j][k].layer = newLayer;
                        m_voxels[i][j][k].ancestor = m_voxels[ijk[0]][ijk[1]][ijk[2]].ancestor;
                        newVoxels++;
                    }
                }
            }
    return newVoxels;
}


void VoxelVolume::Render(const int cost, std::vector< Eigen::Vector3i>& ijks)
{
    ijks.resize(0);
    for (int i = 1; i < m_size[0]+1; i++)
        for (int j = 1; j < m_size[1]+1; j++)
            for (int k = 1; k < m_size[2]+1; k++)
                if (m_voxels[i][j][k].layer == cost)
                    ijks.push_back({ i, j, k });

}

void VoxelVolume::ClassifyByDepth(const double fromDepth, const double toDepth, char flag)
{
    for (int i = 1; i <= m_size[0]; i++)
        for (int j = 1; j <= m_size[1]; j++)
            for (int k = 1; k <= m_size[2]; k++)
                if ((m_voxels[i][j][k].layer == 1) && (m_voxels[i][j][k].depth >= fromDepth) && (m_voxels[i][j][k].depth <= toDepth))
                    m_voxels[i][j][k].flag = flag;

}

void VoxelVolume::RenderByFlag(char flag, std::vector< Eigen::Vector3i>& ijks)
{
    ijks.resize(0);
    for (int i = 1; i <= m_size[0]; i++)
        for (int j = 1; j <= m_size[1]; j++)
            for (int k = 1; k <= m_size[2]; k++)
                if (m_voxels[i][j][k].flag == flag)
                    ijks.push_back({ i, j, k });

}


void VoxelVolume::CalcDepth(bool diagonal)
{
    for (int i = 1; i <= m_size[0] ; i++)
        for (int j = 1; j <= m_size[1]; j++)
            for (int k = 1; k <= m_size[2]; k++)
                if ((m_voxels[i][j][k].layer == 1) /*&& (HasNeighbor(i,j,k,2, diagonal))*/ )
                    SetDepth({ i, j, k }, diagonal);
                else  m_voxels[i][j][k].depth = -1;
                  
                    

}

void VoxelVolume::SetDepth(Eigen::Vector3i ijk, bool diagonal)
{
    int currentCost = m_voxels[ijk[0]][ijk[1]][ijk[2]].layer;
    int nextCost = currentCost;
    Eigen::Vector3i nextVoxel = ijk;
    bool progress = false;
    do
    {
        progress = false;

        nextCost = m_voxels[nextVoxel[0] - 1][nextVoxel[1]][nextVoxel[2]].layer;
        if (nextCost > currentCost)
        {
            currentCost = nextCost;
            progress = true;
            nextVoxel[0]--;
            continue;
        }
 
        nextCost = m_voxels[nextVoxel[0] + 1][nextVoxel[1]][nextVoxel[2]].layer;
        if (nextCost > currentCost)
        {
            currentCost = nextCost;
            progress = true;
            nextVoxel[0]++;
            continue;
        }

        nextCost = m_voxels[nextVoxel[0]][nextVoxel[1] - 1][nextVoxel[2]].layer;
        if (nextCost > currentCost)
        {
            currentCost = nextCost;
            progress = true;
            nextVoxel[1]--;
            continue;
        }

        nextCost = m_voxels[nextVoxel[0]][nextVoxel[1] + 1][nextVoxel[2]].layer;
        if (nextCost > currentCost)
        {
            currentCost = nextCost;
            progress = true;
            nextVoxel[1]++;
            continue;
        }

        nextCost = m_voxels[nextVoxel[0]][nextVoxel[1]][nextVoxel[2] - 1].layer;
        if (nextCost > currentCost)
        {
            currentCost = nextCost;
            progress = true;
            nextVoxel[2]--;
            continue;
        }

        nextCost = m_voxels[nextVoxel[0]][nextVoxel[1]][nextVoxel[2] + 1].layer;
        if (nextCost > currentCost)
        {
            currentCost = nextCost;
            progress = true;
            nextVoxel[2]++;
            continue;
        }

        //// check diagonal neighbors
        if (diagonal)
        {
            nextCost = m_voxels[nextVoxel[0] - 1][nextVoxel[1] - 1][nextVoxel[2] - 1].layer;
            if (nextCost > currentCost)
            {
                currentCost = nextCost;
                progress = true;
                nextVoxel[0]--;
                nextVoxel[1]--;
                nextVoxel[2]--;
                continue;
            }

            nextCost = m_voxels[nextVoxel[0] + 1][nextVoxel[1] - 1][nextVoxel[2] - 1].layer;
            if (nextCost > currentCost)
            {
                currentCost = nextCost;
                progress = true;
                nextVoxel[0]++;
                nextVoxel[1]--;
                nextVoxel[2]--;
                continue;
            }

            nextCost = m_voxels[nextVoxel[0] - 1][nextVoxel[1] + 1][nextVoxel[2] - 1].layer;
            if (nextCost > currentCost)
            {
                currentCost = nextCost;
                progress = true;
                nextVoxel[0]--;
                nextVoxel[1]++;
                nextVoxel[2]--;
                continue;
            }

            nextCost = m_voxels[nextVoxel[0] + 1][nextVoxel[1] + 1][nextVoxel[2] - 1].layer;
            if (nextCost > currentCost)
            {
                currentCost = nextCost;
                progress = true;
                nextVoxel[0]++;
                nextVoxel[1]++;
                nextVoxel[2]--;
                continue;
            }

            nextCost = m_voxels[nextVoxel[0] - 1][nextVoxel[1] - 1][nextVoxel[2] + 1].layer;
            if (nextCost > currentCost)
            {
                currentCost = nextCost;
                progress = true;
                nextVoxel[0]--;
                nextVoxel[1]--;
                nextVoxel[2]++;
                continue;
            }

            nextCost = m_voxels[nextVoxel[0] + 1][nextVoxel[1] - 1][nextVoxel[2] + 1].layer;
            if (nextCost > currentCost)
            {
                currentCost = nextCost;
                progress = true;
                nextVoxel[0]++;
                nextVoxel[1]--;
                nextVoxel[2]++;
                continue;
            }

            nextCost = m_voxels[nextVoxel[0] - 1][nextVoxel[1] + 1][nextVoxel[2] + 1].layer;
            if (nextCost > currentCost)
            {
                currentCost = nextCost;
                progress = true;
                nextVoxel[0]--;
                nextVoxel[1]++;
                nextVoxel[2]++;
                continue;
            }

            nextCost = m_voxels[nextVoxel[0] + 1][nextVoxel[1] + 1][nextVoxel[2] + 1].layer;
            if (nextCost > currentCost)
            {
                currentCost = nextCost;
                progress = true;
                nextVoxel[0]++;
                nextVoxel[1]++;
                nextVoxel[2]++;
                continue;
            }

            nextCost = m_voxels[nextVoxel[0] - 1][nextVoxel[1]][nextVoxel[2] - 1].layer;
            if (nextCost > currentCost)
            {
                currentCost = nextCost;
                progress = true;
                nextVoxel[0]--;
                nextVoxel[2]--;
                continue;
            }

            nextCost = m_voxels[nextVoxel[0] + 1][nextVoxel[1]][nextVoxel[2] - 1].layer;
            if (nextCost > currentCost)
            {
                currentCost = nextCost;
                progress = true;
                nextVoxel[0]++;
                nextVoxel[2]--;
                continue;
            }

            nextCost = m_voxels[nextVoxel[0]][nextVoxel[1] - 1][nextVoxel[2] - 1].layer;
            if (nextCost > currentCost)
            {
                currentCost = nextCost;
                progress = true;
                nextVoxel[1]--;
                nextVoxel[2]--;
                continue;
            }

            nextCost = m_voxels[nextVoxel[0]][nextVoxel[1] + 1][nextVoxel[2] - 1].layer;
            if (nextCost > currentCost)
            {
                currentCost = nextCost;
                progress = true;
                nextVoxel[1]++;
                nextVoxel[2]--;
                continue;
            }

            nextCost = m_voxels[nextVoxel[0] - 1][nextVoxel[1]][nextVoxel[2] + 1].layer;
            if (nextCost > currentCost)
            {
                currentCost = nextCost;
                progress = true;
                nextVoxel[0]--;
                nextVoxel[2]++;
                continue;
            }

            nextCost = m_voxels[nextVoxel[0] + 1][nextVoxel[1]][nextVoxel[2] + 1].layer;
            if (nextCost > currentCost)
            {
                currentCost = nextCost;
                progress = true;
                nextVoxel[0]++;
                nextVoxel[2]++;
                continue;
            }

            nextCost = m_voxels[nextVoxel[0]][nextVoxel[1] - 1][nextVoxel[2] + 1].layer;
            if (nextCost > currentCost)
            {
                currentCost = nextCost;
                progress = true;
                nextVoxel[1]--;
                nextVoxel[2]++;
                continue;
            }

            nextCost = m_voxels[nextVoxel[0]][nextVoxel[1] + 1][nextVoxel[2] + 1].layer;
            if (nextCost > currentCost)
            {
                currentCost = nextCost;
                progress = true;
                nextVoxel[1]++;
                nextVoxel[2]++;
                continue;
            }

        }
        progress &= (nextVoxel[0] > 0) && (nextVoxel[0] < m_size[0]) &&
            (nextVoxel[1] > 0) && (nextVoxel[1] < m_size[1]) &&
            (nextVoxel[2] > 0) && (nextVoxel[2] < m_size[2]);

    } while (progress);
    Eigen::Vector3d source, destination;
    IJK2XYZ(ijk, source);
    IJK2XYZ(nextVoxel, destination);
    m_voxels[ijk[0]][ijk[1]][ijk[2]].depth = (destination - source).norm();

}


void VoxelVolume::FloodFill(const Eigen::Vector3i seed, const int& fromValue, const int& toValue, int depth)
{
    if (m_voxels[seed[0]][seed[1]][seed[2]].layer == fromValue)
    {
        m_voxels[seed[0]][seed[1]][seed[2]].layer = toValue;
        m_outside.insert({ seed[0], seed[1], seed[2] });
        if (depth > MAX_RECURSION_DEPTH)
            return;

        if (seed[0] > 0)
        {
            if (m_voxels[seed[0] - 1][seed[1]][seed[2]].layer == fromValue)
                FloodFill({ seed[0] - 1, seed[1], seed[2] }, fromValue, toValue, depth + 1);
        }

        if (seed[0] < m_size[0] - 1)
        {
            if (m_voxels[seed[0] + 1][seed[1]][seed[2]].layer == fromValue)
                FloodFill({ seed[0] + 1, seed[1], seed[2] }, fromValue, toValue, depth + 1);
        }

        if (seed[1] > 0)
        {
            if (m_voxels[seed[0]][seed[1] - 1][seed[2]].layer == fromValue)
                FloodFill({ seed[0], seed[1] - 1, seed[2] }, fromValue, toValue, depth + 1);
        }

        if (seed[1] < m_size[1] - 1)
        {
            if (m_voxels[seed[0]][seed[1] + 1][seed[2]].layer == fromValue)
                FloodFill({ seed[0], seed[1] + 1, seed[2] }, fromValue, toValue, depth + 1);
        }

        if (seed[2] > 0)
        {
            if (m_voxels[seed[0]][seed[1]][seed[2] - 1].layer == fromValue)
                FloodFill({ seed[0], seed[1], seed[2] - 1 }, fromValue, toValue, depth + 1);
        }

        if (seed[2] < m_size[2] - 1)
        {
            if (m_voxels[seed[0]][seed[1]][seed[2] + 1].layer == fromValue)
                FloodFill({ seed[0], seed[1], seed[2] + 1 }, fromValue, toValue, depth + 1);
        }
    }

 }

void VoxelVolume::FloodFillRays(const Eigen::Vector3i seed, const int& fromValue, const int& toValue, int depth)
{
    if (m_voxels[seed[0]][seed[1]][seed[2]].layer == fromValue)
    {
        m_voxels[seed[0]][seed[1]][seed[2]].layer = toValue;
        bool done = false;
        int step = 1;
        while ((!done) && (seed[0] - step > 0))
        {
            done = (m_voxels[seed[0] - step][seed[1]][seed[2]].layer != fromValue);
            if (!done)
            {
                m_voxels[seed[0] - step][seed[1]][seed[2]].layer = toValue;
                m_outside.insert({ seed[0], seed[1], seed[2] });
            }
            step++;
        }

        step = 1;
        done = false;
        while ((!done) && (seed[0] + step < m_size[0] - 1)  )
        {
            done = (m_voxels[seed[0] + step][seed[1]][seed[2]].layer != fromValue);
            if (!done)
            {
                m_voxels[seed[0] + step][seed[1]][seed[2]].layer = toValue;
                m_outside.insert({ seed[0] + step, seed[1], seed[2] });
            }
            step++;
        }

        step = 1;
        done = false;
        while ((!done) && (seed[1] - step > 0))
        {
            done = (m_voxels[seed[0]][seed[1] - step][seed[2]].layer != fromValue);
            if (!done)
            {
                m_voxels[seed[0]][seed[1] - step][seed[2]].layer = toValue;
                m_outside.insert({ seed[0], seed[1] - step, seed[2] });
            }
            step++;
        }

        step = 1;
        done = false;
        while ((!done) && (seed[1] + step < m_size[1] - 1))
        {
            done = (m_voxels[seed[0]][seed[1] + step][seed[2]].layer != fromValue);
            if (!done)
            {
                m_voxels[seed[0]][seed[1] + step][seed[2]].layer = toValue;
                m_outside.insert({ seed[0], seed[1] + step, seed[2] });
            }
            step++;
        }
 
        step = 1;
        done = false;
        while ((!done) && (seed[2] - step > 0))
        {
            done = (m_voxels[seed[0]][seed[1]][seed[2] - step].layer != fromValue);
            if (!done)
            {
                m_voxels[seed[0]][seed[1]][seed[2] - step].layer = toValue;
                m_outside.insert({ seed[0], seed[1], seed[2] - step });
            }
            step++;
        }

        step = 1;
        done = false;
        while ((!done) && (seed[2] + step < m_size[2] - 1))
        {
            done = (m_voxels[seed[0]][seed[1]][seed[2] + step].layer != fromValue);
            if (!done)
            {
                m_voxels[seed[0]][seed[1]][seed[2] + step].layer = toValue;
                m_outside.insert({ seed[0], seed[1], seed[2] + step });
            }
            step++;
        }
    }

}

bool VoxelVolume::FindSeed(const int seedValue, const int neighborValue, Eigen::Vector3i& ijk)
{
    bool found = false;
    for (int i = 1; i < m_size[0] - 1; i++)
        for (int j = 1; j < m_size[1] - 1; j++)
            for (int k = 1; k < m_size[2] - 1; k++)
            {
                found = ((m_voxels[i][j][k].layer == seedValue) &&
                    ((m_voxels[i + 1][j][k].layer == neighborValue) ||
                    (m_voxels[i - 1][j][k].layer == neighborValue) ||
                    (m_voxels[i][j + 1][k].layer == neighborValue) ||
                    (m_voxels[i][j - 1][k].layer == neighborValue) ||
                    (m_voxels[i][j][k + 1].layer == neighborValue) ||
                    (m_voxels[i][j][k - 1].layer == neighborValue)));
                if (found)
                {
                    ijk[0] = i;
                    ijk[1] = j;
                    ijk[2] = k;
                    return true;
                }
            }
    return false;
}

void VoxelVolume::CalcOutside()
{


    FloodFillRays(Eigen::Vector3i{ 1,1,1 }, 0, -1, 0);
    FloodFillRays(Eigen::Vector3i{ m_size[0] - 1, m_size[1] - 1,m_size[2] - 1 }, 0, -1, 0);
    Eigen::Vector3i ijk;
    while (FindSeed(0, -1, ijk))
    {
        FloodFillRays(ijk, 0, -1, 0);

    }
}


bool VoxelVolume::HasNeighbor(const int& i, const int& j, const int& k, const int& layer, bool diagonal)
{
    if (m_voxels[i - 1][j][k].layer == layer)
        return true;
    if (m_voxels[i][j - 1][k].layer == layer)
        return true;
    if (m_voxels[i][j][k - 1].layer == layer)
        return true;
    if (m_voxels[i + 1][j][k].layer == layer)
        return true;
    if (m_voxels[i][j + 1][k].layer == layer)
        return true;
    if (m_voxels[i][j][k + 1].layer == layer)
        return true;

    if (diagonal)
    {
        if (m_voxels[i - 1][j - 1][k - 1].layer == layer)
            return true;
        if (m_voxels[i + 1][j - 1][k - 1].layer == layer)
            return true;
        if (m_voxels[i - 1][j + 1][k - 1].layer == layer)
            return true;
        if (m_voxels[i + 1][j + 1][k - 1].layer == layer)
            return true;
        if (m_voxels[i - 1][j - 1][k + 1].layer == layer)
            return true;
        if (m_voxels[i + 1][j - 1][k + 1].layer == layer)
            return true;
        if (m_voxels[i - 1][j + 1][k + 1].layer == layer)
            return true;
        if (m_voxels[i + 1][j + 1][k + 1].layer == layer)
            return true;

        if (m_voxels[i - 1][j][k - 1].layer == layer)
            return true;
        if (m_voxels[i + 1][j][k - 1].layer == layer)
            return true;
        if (m_voxels[i][j - 1][k - 1].layer == layer)
            return true;
        if (m_voxels[i][j + 1][k - 1].layer == layer)
            return true;
        if (m_voxels[i - 1][j][k + 1].layer == layer)
            return true;
        if (m_voxels[i + 1][j][k + 1].layer == layer)
            return true;
        if (m_voxels[i][j - 1][k + 1].layer == layer)
            return true;
        if (m_voxels[i][j + 1][k + 1].layer == layer)
            return true;
    }

    return false;
}


bool VoxelVolume::HasNeighbor(const int& i, const int& j, const int& k, const char& flags, bool diagonal)
{
    if (m_voxels[i - 1][j][k].flag == flags)
        return true;
    if (m_voxels[i][j - 1][k].flag == flags)
        return true;
    if (m_voxels[i][j][k - 1].flag == flags)
        return true;
    if (m_voxels[i + 1][j][k].flag == flags)
        return true;
    if (m_voxels[i][j + 1][k].flag == flags)
        return true;
    if (m_voxels[i][j][k + 1].flag == flags)
        return true;

    if (diagonal)
    {
        if (m_voxels[i - 1][j - 1][k - 1].flag & flags)
            return true;
        if (m_voxels[i + 1][j - 1][k - 1].flag & flags)
            return true;
        if (m_voxels[i - 1][j + 1][k - 1].flag & flags)
            return true;
        if (m_voxels[i + 1][j + 1][k - 1].flag & flags)
            return true;
        if (m_voxels[i - 1][j - 1][k + 1].flag & flags)
            return true;
        if (m_voxels[i + 1][j - 1][k + 1].flag & flags)
            return true;
        if (m_voxels[i - 1][j + 1][k + 1].flag & flags)
            return true;
        if (m_voxels[i + 1][j + 1][k + 1].flag & flags)
            return true;

        if (m_voxels[i - 1][j][k - 1].flag & flags)
            return true;
        if (m_voxels[i + 1][j][k - 1].flag & flags)
            return true;
        if (m_voxels[i][j - 1][k - 1].flag & flags)
            return true;
        if (m_voxels[i][j + 1][k - 1].flag & flags)
            return true;
        if (m_voxels[i - 1][j][k + 1].flag & flags)
            return true;
        if (m_voxels[i + 1][j][k + 1].flag & flags)
            return true;
        if (m_voxels[i][j - 1][k + 1].flag & flags)
            return true;
        if (m_voxels[i][j + 1][k + 1].flag & flags)
            return true;
    }

    return false;
}

bool VoxelVolume::FindLayer(const int& fromI, const int& fromJ, const int& fromK, const int dirI, const int dirJ, const int dirK, const int targetLayer)
{
    bool found = false;
    Eigen::Vector3i ijk = { fromI, fromJ, fromK };
    while ((!found) && (ijk[0] >= 0) && (ijk[0] <= m_size[0]) &&
        (ijk[1] >= 0) && (ijk[1] <= m_size[1]) &&
        (ijk[2] >= 0) && (ijk[2] <= m_size[2]))
    {
        found = (m_voxels[ijk[0]][ijk[1]][ijk[2]].layer == targetLayer);
        ijk[0] += dirI;
        ijk[1] += dirJ;
        ijk[2] += dirK;
    }
    return found;
}



void VoxelVolume::AdjustNeighboringFlags()
{
    for (int i = 1; i <= m_size[0]; i++)
        for (int j = 1; j <= m_size[1]; j++)
            for (int k = 1; k <= m_size[2]; k++)
            {
                if ((m_voxels[i][j][k].layer == 1) && (m_voxels[i][j][k].flag == VOXEL_FLAG_THICK))
                {
                    if (!HasNeighbor(i, j, k, VOXEL_FLAG_THICK, false))
                        m_voxels[i][j][k].flag = VOXEL_FLAG_THIN;

                }
            }

    char flags = VOXEL_FLAG_THICK | VOXEL_FLAG_TRANSIENT;
    for (int i = 1; i <= m_size[0]; i++)
        for (int j = 1; j <= m_size[1]; j++)
            for (int k = 1; k <= m_size[2]; k++)
            {
                if (m_voxels[i][j][k].flag == VOXEL_FLAG_THIN)
                {
                      if ((m_voxels[i][j][k].layer == 1) && (const char)(HasNeighbor(i,j,k,flags, true))  )
                          m_voxels[i][j][k].flag = VOXEL_FLAG_TRANSIENT + 100;
                }
            }
    for (int i = 1; i <= m_size[0] ; i++)
        for (int j = 1; j <= m_size[1]; j++)
            for (int k = 1; k <= m_size[2]; k++)
            {
                if (m_voxels[i][j][k].flag > 100)
                    m_voxels[i][j][k].flag -= 100;

            }
}

void VoxelVolume::Dilate(const int layer, const char flag, const int layers)
{
    for (int i = 0; i < layers; i++)
    {
        for (int i = 1; i <= m_size[0]; i++)
            for (int j = 1; j <= m_size[1]; j++)
                for (int k = 1; k <= m_size[2]; k++)
                {
                    if ((m_voxels[i][j][k].layer == 0) && (HasNeighbor(i, j, k, (const char)flag, false)))
                    {
                        m_voxels[i][j][k].flag = flag + 100;
                        m_voxels[i][j][k].layer = layer;
                    }
                }
        for (int i = 1; i <= m_size[0]; i++)
            for (int j = 1; j <= m_size[1]; j++)
                for (int k = 1; k <= m_size[2]; k++)
                {
                    if (m_voxels[i][j][k].flag > 100)
                        m_voxels[i][j][k].flag -= 100;

                }
    }
}

