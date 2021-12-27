#include "UltraVoxel.h"
#include "Vertex.h"

#define MAX_RECURSION_DEPTH 700

Voxel::Voxel()
{
}

Voxel::~Voxel()
{
}


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
    m_voxels.resize(m_size[0]);
    for (int i = 0; i < m_size[0]; i++)
    {
        m_voxels[i].resize(m_size[1]);
        for (int j = 0; j < m_size[1]; j++)
        {
            m_voxels[i][j].resize(m_size[2]);
            for (int k = 0; k < m_size[2]; k++)
                m_voxels[i][j][k].cost = 0;

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
    ijk[0] = (int)trunc((xyz[0] - m_bounds[0] - m_margins[0]) * invSize);
    ijk[1] = (int)trunc((xyz[1] - m_bounds[2] - m_margins[1]) * invSize);
    ijk[2] = (int)trunc((xyz[2] - m_bounds[4] - m_margins[2]) * invSize);
}

void VoxelVolume::IJK2XYZ(const Eigen::Vector3i& ijk, Eigen::Vector3d& xyz)
{
    xyz[0] = m_bounds[0] + m_margins[0] + (ijk[0] + 0.5) * m_brickSize;
    xyz[1] = m_bounds[2] + m_margins[1] + (ijk[1] + 0.5) * m_brickSize;
    xyz[2] = m_bounds[4] + m_margins[2] + (ijk[2] + 0.5) * m_brickSize;
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
        m_voxels[ip[0]][ip[1]][ip[2]].cost = cost;
        m_voxels[ip[0]][ip[1]][ip[2]].ancestor = faceIdx;
        m_border.insert({ ip[0], ip[1], ip[2] });
        continue;
        Eigen::Vector3i ijk = { (int)trunc(ip1[0] + (ip2[0] - ip1[0]) * factor),
        (int)trunc(ip1[1] + (ip2[1] - ip1[1]) * factor),
        (int)trunc(ip1[2] + (ip2[2] - ip1[2]) * factor) };
        m_voxels[ijk[0]][ijk[1]][ijk[2]].cost = cost;
        m_voxels[ijk[0]][ijk[1]][ijk[2]].ancestor = faceIdx;
        m_border.insert({ ijk[0], ijk[1], ijk[2] });
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
    m_faceCenters.push_back(center);
    m_faceNormals.push_back(normal);
    //DrawLine(p1, p2, cost, faceIdx);
    //DrawLine(p2, p3, cost, faceIdx);
    //DrawLine(p3, p1, cost, faceIdx);
}


void VoxelVolume::CalcBorder(const std::vector<UltraFace>& faces, const std::vector<UltraVertex>& vertices)
{
    for (int iFace = 0; iFace < faces.size(); iFace++)
    {
        DrawTrig(vertices[faces[iFace].m_vertices[0]].m_position,
            vertices[faces[iFace].m_vertices[1]].m_position,
            vertices[faces[iFace].m_vertices[2]].m_position, 1, iFace);
        if ((iFace % 877) == 0)
        printf("\rBorder: %3.1f%%", (double)iFace / faces.size()*100.0);
    }
    printf("\rBorder: 100.0%%\n");
}

void VoxelVolume::CalcSecond()
{
    Eigen::Vector3i ijk;
    for (int i = 1; i < m_size[0] - 1; i++)
        for (int j = 1; j < m_size[1] - 1; j++)
            for (int k = 1; k < m_size[2] - 1; k++)
            {
                bool found = (m_voxels[i][j][k].cost == 0);
                if (found)
                {
                    ijk[0] = -1;
                    if (m_voxels[i + 1][j][k].cost == 1)
                        ijk = { i + 1, j, k };
                    else if (m_voxels[i - 1][j][k].cost == 1)
                        ijk = { i - 1, j, k };
                    else if (m_voxels[i][j + 1][k].cost == 1)
                        ijk = { i, j + 1, k };
                    else if (m_voxels[i][j - 1][k].cost == 1)
                        ijk = { i, j - 1, k };
                    else if (m_voxels[i][j][k + 1].cost == 1)
                        ijk = { i, j, k + 1 };
                    else if (m_voxels[i][j][k - 1].cost == 1)
                        ijk = { i, j, k - 1 };
                    found = (ijk[0] >= 0);
                    if (!found)
                        continue;
                    int faceIdx = m_voxels[ijk[0]][ijk[1]][ijk[2]].ancestor;
                    Eigen::Vector3d normal = m_faceNormals[faceIdx];
                    Eigen::Vector3d center = m_faceCenters[faceIdx];
                    Eigen::Vector3d xyz;
                    IJK2XYZ({ i,j,k }, xyz);
                    Eigen::Vector3d dir = (xyz - center);
                    dir.normalize();
                    m_voxels[i][j][k].cost = (dir.dot(normal) > 0.0) ? -1 : 2;
                    if (m_voxels[i][j][k].cost > 0)
                        m_voxels[i][j][k].ancestor = m_voxels[ijk[0]][ijk[1]][ijk[2]].ancestor;
                }
            }
}

bool VoxelVolume::CalcLayer(const int neighbourLayer, const int newLayer)
{
    Eigen::Vector3i ijk;
    bool found = false;
    for (int i = 1; i < m_size[0] - 1; i++)
        for (int j = 1; j < m_size[1] - 1; j++)
            for (int k = 1; k < m_size[2] - 1; k++)
            {
                bool modify = (m_voxels[i][j][k].cost == 0);
                if (modify)
                {
                    ijk[0] = -1;
                    if (m_voxels[i + 1][j][k].cost == neighbourLayer)
                        ijk = { i + 1, j, k };
                    else if (m_voxels[i - 1][j][k].cost == neighbourLayer)
                        ijk = { i - 1, j, k };
                    else if (m_voxels[i][j + 1][k].cost == neighbourLayer)
                        ijk = { i, j + 1, k };
                    else if (m_voxels[i][j - 1][k].cost == neighbourLayer)
                        ijk = { i, j - 1, k };
                    else if (m_voxels[i][j][k + 1].cost == neighbourLayer)
                        ijk = { i, j, k + 1 };
                    else if (m_voxels[i][j][k - 1].cost == neighbourLayer)
                        ijk = { i, j, k - 1 };
                    modify = (ijk[0] >= 0);
                    if (modify)
                    {
                        found = true;
                        m_voxels[i][j][k].cost = newLayer;
                        m_voxels[i][j][k].ancestor = m_voxels[ijk[0]][ijk[1]][ijk[2]].ancestor;
                    }
                }
            }
    return found;
}


void VoxelVolume::Render(const int cost, std::vector< Eigen::Vector3i>& ijks)
{
    ijks.resize(0);
    for (int i = 0; i < m_size[0]; i++)
        for (int j = 0; j < m_size[1]; j++)
            for (int k = 0; k < m_size[2]; k++)
                if (m_voxels[i][j][k].cost == cost)
                    ijks.push_back({ i, j, k });

}

void VoxelVolume::RenderByDepth(const double fromDepth, const double toDepth, std::vector< Eigen::Vector3i>& ijks)
{
    ijks.resize(0);
    for (int i = 0; i < m_size[0]; i++)
        for (int j = 0; j < m_size[1]; j++)
            for (int k = 0; k < m_size[2]; k++)
                if ((m_voxels[i][j][k].cost == 1) && (m_voxels[i][j][k].depth >= fromDepth) && (m_voxels[i][j][k].depth <= toDepth))
                    ijks.push_back({ i, j, k });

}



void VoxelVolume::CalcDepth()
{
    for (int i = 0; i < m_size[0]; i++)
        for (int j = 0; j < m_size[1]; j++)
            for (int k = 0; k < m_size[2]; k++)
                if ((m_voxels[i][j][k].cost == 1) && (HasNeighbor(i,j,k,2)) )
                    SetDepth({ i, j, k });
                else  m_voxels[i][j][k].depth = -1;
                  
                    

}

void VoxelVolume::SetDepth(Eigen::Vector3i ijk)
{
    int currentCost = m_voxels[ijk[0]][ijk[1]][ijk[2]].cost;
    int nextCost = currentCost;
    Eigen::Vector3i nextVoxel = ijk;
    bool progress = false;
    do
    {
        progress = false;
        bool canLeft = (nextVoxel[0] > 0);
        bool canRight = (nextVoxel[0] < m_size[0] - 1);
        bool canFront = (nextVoxel[1] > 0);
        bool canBack = (nextVoxel[1] < m_size[1] - 1);
        bool canDown = (nextVoxel[2] > 0);
        bool canUp = (nextVoxel[2] < m_size[2] - 1);

        bool canLeftFrontBottom = (nextVoxel[0] > 0) && (nextVoxel[1] > 0) && (nextVoxel[2] > 0);
        bool canRightFrontBottom = (nextVoxel[0] < m_size[0] - 1) && (nextVoxel[1] > 0) && (nextVoxel[2] > 0);
        bool canLeftBackBottom = (nextVoxel[0] > 0) && (nextVoxel[1] < m_size[1] - 1) && (nextVoxel[2] > 0);
        bool canRightBackBottom = (nextVoxel[0] < m_size[0] - 1) && (nextVoxel[1] < m_size[1] - 1) && (nextVoxel[2] > 0);
        bool canLeftFrontTop = (nextVoxel[0] > 0) && (nextVoxel[1] > 0) && (nextVoxel[2] < m_size[2] - 1);
        bool canRightFrontTop = (nextVoxel[0] < m_size[0] - 1) && (nextVoxel[1] > 0) && (nextVoxel[2] < m_size[2] - 1);
        bool canLeftBackTop = (nextVoxel[0] > 0) && (nextVoxel[1] < m_size[1] - 1) && (nextVoxel[2] < m_size[2] - 1);
        bool canRightBackTop = (nextVoxel[0] < m_size[0] - 1) && (nextVoxel[1] < m_size[1] - 1) && (nextVoxel[2] < m_size[2] - 1);

        bool canLeftBottom = (nextVoxel[0] > 0)  && (nextVoxel[2] > 0);
        bool canRightBottom = (nextVoxel[0] < m_size[0] - 1)  && (nextVoxel[2] > 0);
        bool canBackBottom = (nextVoxel[1] < m_size[1] - 1) && (nextVoxel[2] > 0);
        bool canFrontBottom = (nextVoxel[1] < m_size[1] - 1) && (nextVoxel[2] > 0);
        bool canLeftTop = (nextVoxel[0] > 0) && (nextVoxel[2] < m_size[2] - 1);
        bool canRightTop = (nextVoxel[0] < m_size[0] - 1) && (nextVoxel[2] < m_size[2] - 1);
        bool canBackTop = (nextVoxel[1] < m_size[1] - 1) && (nextVoxel[2] < m_size[2] - 1);
        bool canFrontTop = (nextVoxel[1] < m_size[1] - 1) && (nextVoxel[2] < m_size[2] - 1);
        if (canLeft)
        {
            nextCost = m_voxels[nextVoxel[0] - 1][nextVoxel[1]][nextVoxel[2]].cost;
            if (nextCost > currentCost)
            {
                currentCost = nextCost;
                progress = true;
                nextVoxel[0]--;
                continue;
            }
        }
        if (canRight)
        {
            nextCost = m_voxels[nextVoxel[0] + 1][nextVoxel[1]][nextVoxel[2]].cost;
            if (nextCost > currentCost)
            {
                currentCost = nextCost;
                progress = true;
                nextVoxel[0]++;
                continue;
            }
        }
        if (canFront)
        {
            nextCost = m_voxels[nextVoxel[0]][nextVoxel[1] - 1][nextVoxel[2]].cost;
            if (nextCost > currentCost)
            {
                currentCost = nextCost;
                progress = true;
                nextVoxel[1]--;
                continue;
            }
        }
        if (canBack)
        {
            nextCost = m_voxels[nextVoxel[0]][nextVoxel[1] + 1][nextVoxel[2]].cost;
            if (nextCost > currentCost)
            {
                currentCost = nextCost;
                progress = true;
                nextVoxel[1]++;
                continue;
            }
        }
        if (canDown)
        {
            nextCost = m_voxels[nextVoxel[0]][nextVoxel[1]][nextVoxel[2] - 1].cost;
            if (nextCost > currentCost)
            {
                currentCost = nextCost;
                progress = true;
                nextVoxel[2]--;
                continue;
            }
        }
        if (canUp)
        {
            nextCost = m_voxels[nextVoxel[0]][nextVoxel[1]][nextVoxel[2] + 1].cost;
            if (nextCost > currentCost)
            {
                currentCost = nextCost;
                progress = true;
                nextVoxel[2]++;
                continue;
            }
        }

        /*
        if (canLeftFrontBottom)
        {
            nextCost = m_voxels[nextVoxel[0] - 1][nextVoxel[1] - 1][nextVoxel[2] - 1].cost;
            if (nextCost > currentCost)
            {
                currentCost = nextCost;
                progress = true;
                nextVoxel[0]--;
                nextVoxel[1]--;
                nextVoxel[2]--;
                continue;
            }
        }
        if (canRightFrontBottom)
        {
            nextCost = m_voxels[nextVoxel[0] + 1][nextVoxel[1] - 1][nextVoxel[2] - 1].cost;
            if (nextCost > currentCost)
            {
                currentCost = nextCost;
                progress = true;
                nextVoxel[0]++;
                nextVoxel[1]--;
                nextVoxel[2]--;
                continue;
            }
        }
        if (canLeftBackBottom)
        {
            nextCost = m_voxels[nextVoxel[0] - 1][nextVoxel[1] + 1][nextVoxel[2] - 1].cost;
            if (nextCost > currentCost)
            {
                currentCost = nextCost;
                progress = true;
                nextVoxel[0]--;
                nextVoxel[1]++;
                nextVoxel[2]--;
                continue;
            }
        }
        if (canRightBackBottom)
        {
            nextCost = m_voxels[nextVoxel[0] + 1][nextVoxel[1] + 1][nextVoxel[2] - 1].cost;
            if (nextCost > currentCost)
            {
                currentCost = nextCost;
                progress = true;
                nextVoxel[0]++;
                nextVoxel[1]++;
                nextVoxel[2]--;
                continue;
            }
        }
        if (canLeftFrontTop)
        {
            nextCost = m_voxels[nextVoxel[0] - 1][nextVoxel[1] - 1][nextVoxel[2] + 1].cost;
            if (nextCost > currentCost)
            {
                currentCost = nextCost;
                progress = true;
                nextVoxel[0]--;
                nextVoxel[1]--;
                nextVoxel[2]++;
                continue;
            }
        }
        if (canRightFrontTop)
        {
            nextCost = m_voxels[nextVoxel[0] + 1][nextVoxel[1] - 1][nextVoxel[2] + 1].cost;
            if (nextCost > currentCost)
            {
                currentCost = nextCost;
                progress = true;
                nextVoxel[0]++;
                nextVoxel[1]--;
                nextVoxel[2]++;
                continue;
            }
        }
        if (canLeftBackTop)
        {
            nextCost = m_voxels[nextVoxel[0] - 1][nextVoxel[1] + 1][nextVoxel[2] + 1].cost;
            if (nextCost > currentCost)
            {
                currentCost = nextCost;
                progress = true;
                nextVoxel[0]--;
                nextVoxel[1]++;
                nextVoxel[2]++;
                continue;
            }
        }
        if (canRightBackTop)
        {
            nextCost = m_voxels[nextVoxel[0] + 1][nextVoxel[1] + 1][nextVoxel[2] + 1].cost;
            if (nextCost > currentCost)
            {
                currentCost = nextCost;
                progress = true;
                nextVoxel[0]++;
                nextVoxel[1]++;
                nextVoxel[2]++;
                continue;
            }
        }
        if (canLeftBottom)
        {
            nextCost = m_voxels[nextVoxel[0] - 1][nextVoxel[1]][nextVoxel[2] - 1].cost;
            if (nextCost > currentCost)
            {
                currentCost = nextCost;
                progress = true;
                nextVoxel[0]--;
                nextVoxel[2]--;
                continue;
            }
        }
*/
    }   while (progress);
    Eigen::Vector3d source, destination;
    IJK2XYZ(ijk, source);
    IJK2XYZ(nextVoxel, destination);
    m_voxels[ijk[0]][ijk[1]][ijk[2]].depth = (destination - source).norm();

}


void VoxelVolume::FloodFill(const Eigen::Vector3i& seed, const int& fromValue, const int& toValue, int depth)
{
    if (m_voxels[seed[0]][seed[1]][seed[2]].cost == fromValue)
    {
        m_voxels[seed[0]][seed[1]][seed[2]].cost = toValue;
        m_outside.insert({ seed[0], seed[1], seed[2] });
        if (depth > MAX_RECURSION_DEPTH)
            return;

        if (seed[0] > 0)
            FloodFill({ seed[0] - 1, seed[1], seed[2] }, fromValue, toValue, depth + 1);

        if (seed[0] < m_size[0] - 1)
            FloodFill({ seed[0] + 1, seed[1], seed[2] }, fromValue, toValue, depth + 1);

        if (seed[1] > 0)
            FloodFill({ seed[0], seed[1] - 1, seed[2] }, fromValue, toValue, depth + 1);

        if (seed[1] < m_size[1] - 1)
            FloodFill({ seed[0], seed[1] + 1, seed[2] }, fromValue, toValue, depth + 1);

        if (seed[2] > 0)
            FloodFill({ seed[0], seed[1], seed[2] - 1 }, fromValue, toValue, depth + 1);

        if (seed[2] < m_size[2] - 1)
            FloodFill({ seed[0], seed[1], seed[2] + 1 }, fromValue, toValue, depth + 1);
    }

}

bool VoxelVolume::FindSeed(const int seedValue, const int neighborValue, Eigen::Vector3i& ijk)
{
    bool found = false;
    for (int i = 1; i < m_size[0] - 1; i++)
        for (int j = 1; j < m_size[1] - 1; j++)
            for (int k = 1; k < m_size[2] - 1; k++)
            {
                found = ((m_voxels[i][j][k].cost == seedValue) &&
                    ((m_voxels[i + 1][j][k].cost == neighborValue) ||
                    (m_voxels[i - 1][j][k].cost == neighborValue) ||
                    (m_voxels[i][j + 1][k].cost == neighborValue) ||
                    (m_voxels[i][j - 1][k].cost == neighborValue) ||
                    (m_voxels[i][j][k + 1].cost == neighborValue) ||
                    (m_voxels[i][j][k - 1].cost == neighborValue)));
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
    FloodFill(Eigen::Vector3i{ 0,0,0 }, 0, -1, 0);
    FloodFill(Eigen::Vector3i{ m_size[0] - 1, m_size[1] - 1,m_size[2] - 1 }, 0, -1, 0);
    Eigen::Vector3i ijk;
    while (FindSeed(0, -1, ijk))
    {
        FloodFill(ijk, 0, -1, 0);

    }
}


bool VoxelVolume::HasNeighbor(const int i, const int j, const int k, const int cost)
{
    if ((i > 0) && (m_voxels[i - 1][j][k].cost == cost))
        return true;
    if ((j > 0) && (m_voxels[i][j - 1][k].cost == cost))
        return true;
    if ((k > 0) && (m_voxels[i][j][k - 1].cost == cost))
        return true;
    if ((i < m_size[0] - 1) && (m_voxels[i + 1][j][k].cost == cost))
        return true;
    if ((j < m_size[1] - 1) && (m_voxels[i][j + 1][k].cost == cost))
        return true;
    if ((k < m_size[2] - 1) && (m_voxels[i][j][k + 1].cost == cost))
        return true;
/*
    if ((i > 0) && (j > 0) && (k > 0) && (m_voxels[i - 1][j - 1][k - 1].cost == cost))
        return true;
    if ((i < m_size[0] - 1) && (j > 0) && (k > 0) && (m_voxels[i + 1][j - 1][k - 1].cost == cost))
        return true;
    if ((i > 0) && (j < m_size[1] - 1) && (k > 0) && (m_voxels[i - 1][j + 1][k - 1].cost == cost))
        return true;
    if ((i < m_size[0] - 1) && (j < m_size[1] - 1) && (k > 0) && (m_voxels[i + 1][j + 1][k - 1].cost == cost))
        return true;
    if ((i > 0) && (j > 0) && (k < m_size[2] - 1) && (m_voxels[i - 1][j - 1][k + 1].cost == cost))
        return true;
    if ((i < m_size[0] - 1) && (j > 0) && (k < m_size[2] - 1) && (m_voxels[i + 1][j - 1][k + 1].cost == cost))
        return true;
    if ((i > 0) && (j < m_size[1] - 1) && (k < m_size[2] - 1) && (m_voxels[i - 1][j + 1][k + 1].cost == cost))
        return true;
    if ((i < m_size[0] - 1) && (j < m_size[1] - 1) && (k < m_size[2] - 1) && (m_voxels[i + 1][j + 1][k + 1].cost == cost))
        return true;
  */
    return false;
}

