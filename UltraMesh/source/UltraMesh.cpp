#include "../../UltraMesh/include/UltraMesh.h"
#include <iostream>
#include <fstream>
#include <filesystem>


typedef struct VVF
{
public:
	VVF() { ; }
	VVF(int v1, int v2, int f, int i)
	{
		m_v1 = v1;
		m_v2 = v2;
		m_f = f;
		m_i = i;
	}
	int m_v1;
	int m_v2;
	int m_f;
	int m_i;
}VVF;

bool Zpolyline::AddSegment(const double x1, const double y1, const double x2, const double y2)
{
    if (m_segments.size() == 0)
    {
        m_segments.push_back(std::make_pair(std::make_pair(x1, y1), std::make_pair(x2, y2)));
        return true;
    }
    if (m_closed)
        return false;
    Eigen::Vector2d p1 = { x1, y1 };
    Eigen::Vector2d p2 = { x2, y2 };
    Eigen::Vector2d head = {m_segments.begin()->first.first, m_segments.begin()->first.second };
    Eigen::Vector2d tail = { m_segments.back().second.first, m_segments.back().second.second };
    if ((p1 - tail).squaredNorm() < EPSILON4)
    {
        m_segments.push_back(std::make_pair(std::make_pair(x1, y1), std::make_pair(x2, y2)));
        m_closed = ((Eigen::Vector2d{ x2, y2 } - head).squaredNorm() < EPSILON6);
        return true;
    }
    if ((Eigen::Vector2d{ x2, y2 } - head).squaredNorm() < EPSILON4)
    {
        m_segments.push_front(std::make_pair(std::make_pair(x1, y1), std::make_pair(x2, y2)));
        m_closed = ((Eigen::Vector2d{ x1, y1 } - tail).squaredNorm() < EPSILON6);
        return true;
    }
    return false;
}

void Zpolyline::StartPoint(int idx, Eigen::Vector3d& p)
{
    std::list<std::pair<std::pair<double, double>, std::pair<double, double>>>::iterator it = std::next(m_segments.begin(), idx);
    p[0] = it->first.first;
    p[1] = it->first.second;
    p[2] = m_Z;
};

void Zpolyline::EndPoint(int idx, Eigen::Vector3d& p)
{
    std::list<std::pair<std::pair<double, double>, std::pair<double, double>>>::iterator it = std::next(m_segments.begin(), idx);
    p[0] = it->second.first;
    p[1] = it->second.second;
    p[2] = m_Z;
};

double Zpolyline::Length()
{
    double length = 0.0;
    for (auto& segment : m_segments)
        length += sqrt((segment.second.first - segment.first.first) * (segment.second.first - segment.first.first) +
        (segment.second.second - segment.first.second) * (segment.second.second - segment.first.second));
    return length;
}

UltraMesh::UltraMesh()
{
	m_vertices.clear();
	m_faces.clear();
	m_edges.clear();
}


UltraMesh::UltraMesh(const UltraMesh& other)
{
    if (this != &other)
    {
        this->m_vertices = other.m_vertices;
        this->m_edges = other.m_edges;
        this->m_faces = other.m_faces;
        this->m_buckets = other.m_buckets;
        this->CalcBounds();
    }
 }

UltraMesh& UltraMesh::operator=(const UltraMesh& other)
{
    if (this != &other)
    {
        this->m_vertices = other.m_vertices;
        this->m_edges = other.m_edges;
        this->m_faces = other.m_faces;
        this->m_buckets = other.m_buckets;
        this->CalcBounds();
    }
    return *this;
}


void UltraMesh::MapEdges()
{
	bool verbose = false;
	m_edges.clear();
	for (int idx = 0; idx < (int)m_faces.size(); idx++)
	{
		m_edges.push_back(UltraEdge(m_faces[idx].m_vertices[0], m_faces[idx].m_vertices[1], (int)idx));
		m_edges.push_back(UltraEdge(m_faces[idx].m_vertices[1], m_faces[idx].m_vertices[2], (int)idx));
		m_edges.push_back(UltraEdge(m_faces[idx].m_vertices[2], m_faces[idx].m_vertices[0], (int)idx));
        m_faces[idx].m_edges[0] = idx * 3;
        m_faces[idx].m_edges[1] = idx * 3 + 1;
        m_faces[idx].m_edges[2] = idx * 3 + 2;
    }
	std::vector <VVF> map1;							// Maps all Edges V1->V2 (direct)
	std::vector <VVF> map2;							// Maps all Edges V2->V1 (reversed)

	map1.resize(m_faces.size() * 3);								// Memory upfront allocation
	map2.resize(m_faces.size() * 3);
	
	int mapIndex = 0;

	// The algorithm runs over all Faces and builds direct and reversed map vectors
	for (int iFace = 0; iFace < m_faces.size(); iFace++)
	{
		UltraFace face = m_faces[iFace];
		map1[mapIndex] = VVF(face.m_vertices[0], face.m_vertices[1], iFace, mapIndex);
		map2[mapIndex] = VVF(face.m_vertices[1], face.m_vertices[0], iFace, mapIndex);
		mapIndex++;
		map1[mapIndex] = VVF(face.m_vertices[1], face.m_vertices[2], iFace, mapIndex);
		map2[mapIndex] = VVF(face.m_vertices[2], face.m_vertices[1], iFace, mapIndex);
		mapIndex++;
		map1[mapIndex] = VVF(face.m_vertices[2], face.m_vertices[0], iFace, mapIndex);
		map2[mapIndex] = VVF(face.m_vertices[0], face.m_vertices[2], iFace, mapIndex);
		mapIndex++;
	}
	//Sorting both maps in ascending order v1 then v2
	sort(map1.begin(), map1.end(), [](const VVF v1, const VVF v2)
	{
		if (v1.m_v1 == v2.m_v1)
			return v1.m_v2 < v2.m_v2;
		else
			return v1.m_v1 < v2.m_v1;
	});
	sort(map2.begin(), map2.end(), [](const VVF v1, const VVF v2)
	{
		if (v1.m_v1 == v2.m_v1)
			return v1.m_v2 < v2.m_v2;
		else
			return v1.m_v1 < v2.m_v1;
	});
	// As both maps are sorted, every line consists of two related faces of
	// both direct and reversed Edges, therefore twin index i2 can be set
	for (int iMap = 0; iMap < map1.size(); iMap++)
	{
		m_edges[map1[iMap].m_i].SetTwin(map2[iMap].m_i);
		m_edges[map2[iMap].m_i].SetTwin(map1[iMap].m_i);
	}

    for (int edgeIdx = 0; edgeIdx < m_edges.size(); edgeIdx++)
    {
        UltraEdge& edge = m_edges[edgeIdx];
        m_vertices[edge.m_idxV1].m_connectedEdges.push_back((int)edgeIdx);
    }
    for (auto& edge : m_edges)
        edge.CalcVector(m_vertices);


}

void UltraMesh::CalcFaces()
{
	for (auto& face : m_faces)
		face.CalcPlane(m_vertices[face.m_vertices[0]].m_position,
			m_vertices[face.m_vertices[1]].m_position,
			m_vertices[face.m_vertices[2]].m_position);
}

void UltraMesh::CalcNormals(bool byArea)
{
	for (auto& vertex : m_vertices)
	{
        vertex.CalcNormal(m_faces, m_edges, byArea);
	}
}

void UltraMesh::CalcCurvature()
{
    for (UltraVertex& vertex : m_vertices)
        vertex.CalcCurvature(m_faces, m_edges, m_vertices);
}

Bounds* UltraMesh::CalcBounds()
{
	for (int i = 0; i < 6; i++)
		m_bounds[i] = (i % 2 == 0) ? DBL_MAX : -DBL_MAX;

	for (auto& vertex : m_vertices)
	{
		m_bounds[0] = std::min(m_bounds[0], vertex.m_position[0]);
		m_bounds[1] = std::max(m_bounds[1], vertex.m_position[0]);
		m_bounds[2] = std::min(m_bounds[2], vertex.m_position[1]);
		m_bounds[3] = std::max(m_bounds[3], vertex.m_position[1]);
		m_bounds[4] = std::min(m_bounds[4], vertex.m_position[2]);
		m_bounds[5] = std::max(m_bounds[5], vertex.m_position[2]);
	}
	return &m_bounds;
}

void UltraMesh::CalcBuckets()
{
	// get mesh bounding box
	CalcBounds();
	// calculate optimal bucket size
	int numberOfFaces = (int)m_faces.size();
	int maxRow =  (int)(sqrt(sqrt(numberOfFaces)));

	// calculate cells size and repetitions per axis
	double cellSize = std::max((m_bounds[1] - m_bounds[0]) / maxRow,
		std::max((m_bounds[3] - m_bounds[2]) / maxRow, (m_bounds[5] - m_bounds[4]) / maxRow)) + 1;

	int NX = (int)((m_bounds[1] - m_bounds[0]) / cellSize + 1);
	int NY = (int)((m_bounds[3] - m_bounds[2]) / cellSize + 1);
	int NZ = (int)((m_bounds[5] - m_bounds[4]) / cellSize + 1);

	double cellX = (m_bounds[1] - m_bounds[0]) / NX;
	double cellY = (m_bounds[3] - m_bounds[2]) / NY;
	double cellZ = (m_bounds[5] - m_bounds[4]) / NZ;

	// create array of buckets
	Bounds newBounds;
	for (int i = 0; i < NZ; i++)
	{
		newBounds[4] = m_bounds[4] + i * cellZ - EPSILON6;
		newBounds[5] = newBounds[4] + cellZ + 2 * EPSILON6;
		for (int j = 0; j < NY; j++)
		{
			newBounds[2] = m_bounds[2] + j * cellY - EPSILON6;
			newBounds[3] = newBounds[2] + cellY + 2 * EPSILON6;
			for (int k = 0; k < NX; k++)
			{
				newBounds[0] = m_bounds[0] + k * cellX - EPSILON6;
				newBounds[1] = newBounds[0] + cellX + 2 * EPSILON6;
				m_buckets.push_back(Bucket(newBounds, k, j, i));
			}
		}
	}

	// add triangles to buckets. Since triangles are way smaller than the bucket size, we
	// can add triangles to neighboring buckets 
	// TBD add triangles to buckets if one of the triangle edges cuts corner in the bucket

	for (int iF = 0; iF < m_faces.size(); iF++)
	{
		UltraFace face = m_faces[iF];
		for (int iV = 0; iV < 3; iV++)
		{
			UltraVertex vertex = m_vertices[face.Vertices()[iV]];
			int indexX = (int)(std::max(0.0, vertex.Position()[0] - m_bounds[0] - EPSILON6) / cellX);
			int indexY = (int)(std::max(0.0, vertex.Position()[1] - m_bounds[2] - EPSILON6) / cellY);
			int indexZ = (int)(std::max(0.0, vertex.Position()[2] - m_bounds[4] - EPSILON6) / cellZ);

			int stride = indexZ * NX * NY + indexY * NX + indexX;

			Bucket& bucket = m_buckets[stride];
			if (bucket.IsInside(vertex.m_position))
				bucket.m_triangles.insert(iF);
		}
	}
}

std::vector<UltraFace> UltraMesh::IntersectWithRay(Eigen::Vector3d& origin, Eigen::Vector3d& direction)
{
	std::vector<UltraFace> result;
	direction.normalize();
    Bucket whole(m_bounds, -1, -1, -1);
    if (whole.IntersectWithRay(origin, direction))
    {
        std::vector<Bucket> intersected;
        for (size_t idx = 0; idx < m_buckets.size(); idx++)
        {
            Bucket bucket = m_buckets[idx];
            if ((bucket.m_triangles.size() > 0) && (bucket.IntersectWithRay(origin, direction)))
            {
                intersected.push_back(bucket);
            }
        }
        //printf("Ray from %f %f %f to %f %f %f\n", origin[0], origin[1], origin[2],
        //	direction[0], direction[1], direction[2]);
        int counter = 0;
        for (auto bucket : intersected)
        {
            //printf("Bucket %d %d %d\n", bucket.m_gridLocation[0], bucket.m_gridLocation[1], bucket.m_gridLocation[2]);
            for (int i = 0; i < (int)bucket.m_triangles.size(); i++)
            {
                std::set<int>::iterator it = bucket.m_triangles.begin();
                std::advance(it, i);
                UltraFace triangle = m_faces[*it];
                Eigen::Vector3d hit = { 0.0, 0.0, 0.0 };
                if (triangle.IntersectWithRay(m_vertices, origin, direction, hit))
                {
                    counter++;
                    //printf("%3d) intersect face %6d at: %f %f %f\n ", counter, *it, hit[0], hit[1], hit[2]);
                    result.push_back(m_faces[*it]);
                }
            }

        }
    }
	return result;
}

void UltraMesh::Smooth()
{
    // Non weighted averaging
    for (size_t i = 0; i < m_vertices.size(); i++)
    {
        UltraVertex* vertex = &m_vertices[i];
        Eigen::Vector3d target = { 0.0, 0.0, 0.0 };
        for (size_t j = 0; j < vertex->m_connectedEdges.size(); j++)
        {
            target += m_vertices[m_edges[vertex->m_connectedEdges[j]].m_idxV2].m_position;
        }
        vertex->m_shadowPosition = target / vertex->m_connectedEdges.size();
    }
    // move distpos to pos
    for (size_t i = 0; i < m_vertices.size(); i++)
    {
        m_vertices[i].m_thickness = (m_vertices[i].m_position - m_vertices[i].m_shadowPosition).norm();
        m_vertices[i].m_position = m_vertices[i].m_shadowPosition;
    }
    // calc new edges vector lengths
    for (size_t i = 0; i < m_edges.size(); i++)
    {
        m_edges[i].CalcVector(m_vertices);
    }
    // calc new faces planes and areas
    for (size_t i = 0; i < m_faces.size(); i++)
    {
        m_faces[i].CalcPlane(m_vertices);
    }
    // calc each vertex normal and local curvature
    for (size_t i = 0; i < m_vertices.size(); i++)
    {
        m_vertices[i].CalcNormal(m_faces, m_edges, false);
    }
}


// "Adaptive offset" function
/*
void UltraMesh::OffsetBySkeleton(double maxOffset)
{
    // set direction Normal multiplier
    bool inwards = (maxOffset < 0.0);
    maxOffset = abs(maxOffset);
    double direction = (inwards) ? -1.0 : 1.0;								// bool to double factor

    // A simple performance booster mechanism - divide all faces into 3D array of containers
    // ("buckets") for more efficient search.
    // Nbuckets size is set by thumb rule as the fourth root of the number of vertices.
    int Nbuckets = std::max(2, int(sqrt(sqrt(m_vertices.size()))));

    // The buckets array (Nbuckets*Nbuckets*Nbuckets) is arranged as a vector.
    // TBD check for more robust implementation such as KDtree
    std::vector <Bucket> buckets;

    // Each bucket is overlapping its neighbors by half of maxDisplacement. This bucketMargin is needed for
    // checking "proposed" vertices positions (m_dispos) that are initially set to maxDisplacement.
    double bucketMargins = maxOffset * 0.5;

    // Each bucket size "a", excluding margins, is set by dividing the total vertices bounding box by Nbuckets
    double ax = (m_bounds[1] - m_bounds[0]) / Nbuckets;
    double ay = (m_bounds[3] - m_bounds[2]) / Nbuckets;
    double az = (m_bounds[5] - m_bounds[4]) / Nbuckets;

    // Loop over entire volume and set VolumeOfInterest for each bucket, then push it into buckets.
    for (int i = 0; i < Nbuckets; i++)
    {
        for (int j = 0; j < Nbuckets; j++)
        {
            for (int k = 0; k < Nbuckets; k++)
            {
                Bucket bucket = Bucket();
                Bounds box = { m_bounds[0] + k * ax - bucketMargins, m_bounds[0] + (k + 1) * ax + bucketMargins,
                    m_bounds[2] + j * ay - bucketMargins, m_bounds[2] + (j + 1) * ay + bucketMargins,
                    m_bounds[4] + i * az - bucketMargins, m_bounds[4] + (i + 1) * az + bucketMargins };
                memcpy(&bucket.m_bounds[0], &box[0], sizeof(Bounds));
                buckets.push_back(bucket);
            }
        }
    }
    // buckets are ready, hence push faces into relevant buckets (each face can appear in more than one bucket)
    for (size_t i = 0; i < m_faces.size(); i++)
    {
        // found variable counts the bucket insertions. it's value shall always be in range 1..3
        int found = 0;
        int faceVs[3] = { m_faces[i].m_vertices[0], m_faces[i].m_vertices[1],m_faces[i].m_vertices[2] };
        for (size_t j = 0; j < 3; j++)
        {
            // After three vertices are considered in buckets, no need for further checking.
            if (found > 2)
                break;
            // Each face can be recognized by 1-3 buckets, depending on each vertex location

            for (size_t k = 0; k < buckets.size(); k++)
            {
                if ((m_vertices[faceVs[j]].m_position[0] >= buckets[k].m_bounds[0]) &&
                    (m_vertices[faceVs[j]].m_position[0] <= buckets[k].m_bounds[1]) &&
                    (m_vertices[faceVs[j]].m_position[1] >= buckets[k].m_bounds[2]) &&
                    (m_vertices[faceVs[j]].m_position[1] <= buckets[k].m_bounds[3]) &&
                    (m_vertices[faceVs[j]].m_position[2] >= buckets[k].m_bounds[4]) &&
                    (m_vertices[faceVs[j]].m_position[2] <= buckets[k].m_bounds[5]))
                {
                    // found inside
                    found++;
                    buckets[k].m_triangles.insert((int)i);
#ifdef DEBUG_MODE
                    isInside = true;
#endif
                }
            }
        }
#ifdef DEBUG_MODE
        if (!isInside)
        {
            // should not happed
            printf("Face %d not in any bucket\n", (int)i);
        }
#endif
    }

    // Main vertex displacement processing:
    // For each vertex, check all nearby faces for distance. In case of vertex displacement is overruled by a closer
    // face "F", then a new equilibrium point is calculated, where the displacement equals the closest distance to
    // the said face F.

    double stepOffset = maxOffset/1 ;
    for (int iStep = 0; iStep < 1; iStep++)
    {
        for (size_t i = 0; i < m_vertices.size(); i++)
        {
            // progress message every (big prime number) loops.
            if (i % 1000 == 0)																			// saves output time
                printf("\rProcessing...%3.1f%%", double(i) / m_vertices.size()*100.0);
            // vertex to be displaced

            UltraVertex* vertex = &m_vertices[i];

            // set default displacement to maximum, considering normal direction and in/out factor.
            vertex->m_shadowPosition = vertex->m_position + vertex->m_normal * direction * stepOffset;
            //if (abs(vertex->m_position[0]) > 9.99)
            //    continue;

            // eliminate infinite loop in case of a bad vertex
            if (vertex->m_normal.norm() == 0)
            {
                vertex->m_limit = 0.0;
                continue;
            }
            // distance limit from original point
            vertex->m_limit = stepOffset;


            // Provided that vertex will be moved, check for overruling by looping over buckets and iterating on
            // nearby faces.
            for (size_t j = 0; j < buckets.size(); j++)
            {
                if ((m_vertices[i].m_shadowPosition[0] < buckets[j].m_bounds[0]) ||
                    (m_vertices[i].m_shadowPosition[0] > buckets[j].m_bounds[1]) ||
                    (m_vertices[i].m_shadowPosition[1] < buckets[j].m_bounds[2]) ||
                    (m_vertices[i].m_shadowPosition[1] > buckets[j].m_bounds[3]) ||
                    (m_vertices[i].m_shadowPosition[2] < buckets[j].m_bounds[4]) ||
                    (m_vertices[i].m_shadowPosition[2] > buckets[j].m_bounds[5]))
                    continue; // outside bucket
                // Iterate over relevant faces
                std::set<int>::iterator it = buckets[j].m_triangles.begin();

                // As long as there are nearby triangles to check
                while (it != buckets[j].m_triangles.end())
                {
                    // check nearby triangle "face"
                    UltraFace* face = &m_faces[*it];
                    // Exclude  triangles touching (neighboring of) "vertex"
                    if ((face->m_vertices[0] != vertex->m_index) && (face->m_vertices[1] != vertex->m_index) && (face->m_vertices[2] != vertex->m_index))
                    {
                        // Check minimal distance from proposed new vertex position to either of face's vertices
                        Eigen::Vector3d TestPoint = vertex->m_position + vertex->m_normal * direction * vertex->m_limit;
                        Eigen::Vector3d v2p, P;
                        double minDist = face->ClampDistPoint(m_vertices, TestPoint, P);                    // In case minimal distance is closer to "face", yet significant, perform
                        // a further investigation
                        if ((vertex->m_limit > 1.0E-5) && (minDist < vertex->m_limit)) // no point at limit < machine resolution (10u)
                        {
                            // set closest vertex as point P
                            // Minimal distance is closer than current limit - hence update limit to the point of equilibrium
                            if (minDist < vertex->m_limit)
                            {
                                v2p = P - vertex->m_position;
                                double len = v2p.norm();
                                v2p.normalize();
                                double cosAng = abs(v2p.dot(vertex->m_normal * direction));
                                bool modified = false;
                                // Calculate point of equilibrium
                                if (cosAng > 0.0)
                                {
                                    // Point of equilibrium in case the overruling face is not perpendicular to vertex normal
                                    double newLimit = std::min(vertex->m_limit, (len * 0.4999 / cosAng)); // notch below 0.5 to eliminate infinite loop on roundings.
                                    if (newLimit < vertex->m_limit - EPSILON6)
                                    {
                                        vertex->m_limit = newLimit;
                                        modified = true;
                                    }
                                }
                                else
                                {
                                    if ((vertex->m_position - P).norm() > 1.0E-5)
                                    {
                                        // Point of equilibrium in case the overruling face is perpendicular to vertex normal
                                        double newLimit = ((vertex->m_position + P) * 0.5).norm();
                                        if (newLimit < vertex->m_limit - EPSILON6)
                                        {
                                            vertex->m_limit = newLimit;
                                            modified = true;
                                        }
                                    }
                                }
#if DEBUG_MODE
                                Eigen::Vector3d minP = vertex->m_pos + vertex->m_normal * vertex->m_limit;
                                double check1 = (vertex->m_pos - minP).norm();
                                double check2 = (P - minP).norm();
                                if (abs(check1 - check2) > maxCheckDiff)
                                {
                                    maxCheckDiff = abs(check1 - check2);
                                    printf("check1=%f, check2=%f  Diff=%f\n", check1, check2, maxCheckDiff);
                                }
#endif
                                // Restart iterations for other overruling faces
                                if (modified)
                                    it = buckets[j].m_triangles.begin();
                            }
                        }
                    }
                    it++;
                }
            }
            // Iterations completed, m_limit is set to optimum hence m_dispos is set.
            vertex->m_shadowPosition = vertex->m_position + direction * vertex->m_normal * vertex->m_limit;
        }
        printf("\rProcessing...%3.1f%%\n", 100.0);
        // Vertex repositioning completed, updating FlexiVertex internal variables
        for (size_t i = 0; i < m_vertices.size(); i++)
        {
            UltraVertex* vertex = &m_vertices[i];
            vertex->m_position = vertex->m_shadowPosition;
        }

        // Recalculate edges vectors after vertices were moved
        for (size_t i = 0; i < m_edges.size(); i++)
        {
            m_edges[i].CalcVector(m_vertices);
        }

        // Recalculate faces planes after vertices were moved
        for (size_t i = 0; i < m_faces.size(); i++)
        {
            m_faces[i].CalcPlane(m_vertices[m_faces[i].m_vertices[0]].m_position,
                m_vertices[m_faces[i].m_vertices[1]].m_position,
                m_vertices[m_faces[i].m_vertices[2]].m_position);
        }

        // Recalculate vertices normals after vertices were moved
        for (size_t i = 0; i < m_vertices.size(); i++)
        {
            m_vertices[i].CalcNormal(m_faces, m_edges, false);
        }
    }

}

*/

void UltraMesh::OffsetBySkeleton(double maxOffset)
{
    // set direction Normal multiplier
    bool inwards = (maxOffset < 0.0);
    maxOffset = abs(maxOffset);
    double direction = (inwards) ? -1.0 : 1.0;								// bool to double factor
 
    // A simple performance booster mechanism - divide all faces into 3D array of containers
    // ("buckets") for more efficient search. 
    // Nbuckets size is set by thumb rule as the fourth root of the number of vertices.
    int Nbuckets = std::max(2, int(sqrt(sqrt(m_vertices.size())))) * 2;
    printf("Number of buckets: %d\n", Nbuckets);

    // The buckets array (Nbuckets*Nbuckets*Nbuckets) is arranged as a vector. 
    // TBD check for more robust implementation such as KDtree
    std::vector <Bucket> buckets;

    // Each bucket is overlapping its neighbors by half of maxDisplacement. This bucketMargin is needed for
    // checking "proposed" vertices positions (m_dispos) that are initially set to maxDisplacement.
    double bucketMargins = maxOffset * 1.2;

    // Each bucket size "a", excluding margins, is set by dividing the total vertices bounding box by Nbuckets
    double ax = (m_bounds[1] - m_bounds[0]) / Nbuckets;
    double ay = (m_bounds[3] - m_bounds[2]) / Nbuckets;
    double az = (m_bounds[5] - m_bounds[4]) / Nbuckets;

    // Loop over entire volume and set VolumeOfInterest for each bucket, then push it into buckets.
    for (int i = 0; i < Nbuckets; i++)
    {
        for (int j = 0; j < Nbuckets; j++)
        {
            for (int k = 0; k < Nbuckets; k++)
            {
                Bucket bucket = Bucket();
                Bounds box = { m_bounds[0] + k * ax - bucketMargins, m_bounds[0] + (k + 1) * ax + bucketMargins,
                    m_bounds[2] + j * ay - bucketMargins, m_bounds[2] + (j + 1) * ay + bucketMargins,
                    m_bounds[4] + i * az - bucketMargins, m_bounds[4] + (i + 1) * az + bucketMargins };
                memcpy(&bucket.m_bounds[0], &box[0], sizeof(Bounds));
                buckets.push_back(bucket);
            }
        }
    }
    // buckets are ready, hence push faces into relevant buckets (each face can appear in more than one bucket)
    for (size_t i = 0; i < m_faces.size(); i++)
    {
        // found variable counts the bucket insertions. it's value shall always be in range 1..3
        int found = 0;
        int faceVs[3] = { m_faces[i].m_vertices[0], m_faces[i].m_vertices[1],m_faces[i].m_vertices[2] };
        for (size_t j = 0; j < 3; j++)
        {
            // After three vertices are considered in buckets, no need for further checking.
            if (found > 2)
                break;
            // Each face can be recognized by 1-3 buckets, depending on each vertex location

            for (size_t k = 0; k < buckets.size(); k++)
            {
                if ((m_vertices[faceVs[j]].m_position[0] >= buckets[k].m_bounds[0]) &&
                    (m_vertices[faceVs[j]].m_position[0] <= buckets[k].m_bounds[1]) &&
                    (m_vertices[faceVs[j]].m_position[1] >= buckets[k].m_bounds[2]) &&
                    (m_vertices[faceVs[j]].m_position[1] <= buckets[k].m_bounds[3]) &&
                    (m_vertices[faceVs[j]].m_position[2] >= buckets[k].m_bounds[4]) &&
                    (m_vertices[faceVs[j]].m_position[2] <= buckets[k].m_bounds[5]))
                {
                    // found inside
                    found++;
                    buckets[k].m_triangles.insert((int)i);
#ifdef DEBUG_MODE
                    isInside = true;
#endif
                }
            }
        }
#ifdef DEBUG_MODE
        if (!isInside)
        {
            // should not happed
            printf("Face %d not in any bucket\n", (int)i);
        }
#endif
    }

    // Main vertex displacement processing:
    // For each vertex, check all nearby faces for distance. In case of vertex displacement is overruled by a closer
    // face "F", then a new equilibrium point is calculated, where the displacement equals the closest distance to 
    // the said face F.

    double stepOffset = maxOffset/1 ;
    for (int iStep = 0; iStep < 1; iStep++)
    {
        for (size_t vertexIdx = 0; vertexIdx < m_vertices.size(); vertexIdx++)
        {
            // progress message every (big prime number) loops.
            if (vertexIdx % 87 == 0)																			// saves output time
                printf("\rProcessing...%3.1f%%", double(vertexIdx) / m_vertices.size()*100.0);
            // vertex to be displaced

            UltraVertex* vertex = &m_vertices[vertexIdx];
            double x = vertex->m_position[0];
           double maxDist = maxOffset*1.2, dist = 0.0;
           double maxDist2 = 2.0 * maxDist;
            Eigen::Vector3d position = vertex->Position();// +direction * vertex->m_normal * abs(vertex->m_curvature) / (1 - abs(vertex->m_curvature));

            // Provided that vertex will be moved, check for overruling by looping over buckets and iterating on
            // nearby faces.
            for (size_t bucketIdx = 0; bucketIdx < buckets.size(); bucketIdx++)
            {
                if ((m_vertices[vertexIdx].m_position[0] + maxOffset < buckets[bucketIdx].m_bounds[0]) ||
                    (m_vertices[vertexIdx].m_position[0] - maxOffset > buckets[bucketIdx].m_bounds[1]) ||
                    (m_vertices[vertexIdx].m_position[1] + maxOffset < buckets[bucketIdx].m_bounds[2]) ||
                    (m_vertices[vertexIdx].m_position[1] - maxOffset > buckets[bucketIdx].m_bounds[3]) ||
                    (m_vertices[vertexIdx].m_position[2] + maxOffset < buckets[bucketIdx].m_bounds[4]) ||
                    (m_vertices[vertexIdx].m_position[2] - maxOffset > buckets[bucketIdx].m_bounds[5]))
                    continue; // outside bucket
                // Iterate over relevant faces
                std::set<int>::iterator bucketIt = buckets[bucketIdx].m_triangles.begin();

                // As long as there are nearby triangles to check
                while (bucketIt != buckets[bucketIdx].m_triangles.end())
                {
                    // check nearby triangle "face"
                    UltraFace* face = &m_faces[*bucketIt];
                    //if (direction.dot(m_plane.block<3, 1>(0, 0)) > 0.0)
 //    return false;

                    if ((face->m_vertices[0] == vertexIdx) || (face->m_vertices[1] == vertexIdx) || (face->m_vertices[2] == vertexIdx))
                    {
                        bucketIt++;
                        continue;
                    }
                    if (face->MaxDistToSkeleton(m_vertices, position, vertex->m_normal * direction, maxDist2, dist))
                    {
                        maxDist = std::min(maxDist, dist);
                    }
                    else
                    {
                        for (int edgeIdx = 0; edgeIdx < 3; edgeIdx++)
                        {
                            UltraEdge* edge = &m_edges[face->m_edges[edgeIdx]];
                            if(edge->MaxDistToSkeleton(m_vertices, position, vertex->m_normal * direction, maxDist2, dist))
                            {
                                maxDist = std::min(maxDist, dist);
                            }
                        }
                    }
                    for (int vertIdx = 0; vertIdx < 3; vertIdx++)
                    {
                        if (m_vertices[face->m_vertices[vertIdx]].MaxDistToSkeleton(position, vertex->m_normal * direction, dist))
                        {
                            maxDist = std::min(maxDist, dist);
                        }

                    }

                    bucketIt++;
                }
            }
            // Iterations completed, m_limit is set to optimum hence m_dispos is set.
            vertex->m_shadowPosition = position + direction * vertex->m_normal * maxDist;// *sqrt(1 + vertex->m_curvature);
            if (isnan(vertex->m_shadowPosition[0]))
            {
                printf("NaN detected vertex %zd \n", vertexIdx);
            }
        }
        printf("\rProcessing...%3.1f%%\n", 100.0);
        // Vertex repositioning completed, updating FlexiVertex internal variables
        for (size_t i = 0; i < m_vertices.size(); i++)
        {
            UltraVertex* vertex = &m_vertices[i];
            vertex->m_thickness = (vertex->m_position - vertex->m_shadowPosition).norm();
            vertex->m_position = vertex->m_shadowPosition;
        }

        // Recalculate edges vectors after vertices were moved
        for (size_t i = 0; i < m_edges.size(); i++)
        {
            m_edges[i].CalcVector(m_vertices);
        }

        // Recalculate faces planes after vertices were moved
        for (size_t i = 0; i < m_faces.size(); i++)
        {
            m_faces[i].CalcPlane(m_vertices[m_faces[i].m_vertices[0]].m_position,
                m_vertices[m_faces[i].m_vertices[1]].m_position,
                m_vertices[m_faces[i].m_vertices[2]].m_position);
        }

        // Recalculate vertices normals after vertices were moved
        for (size_t i = 0; i < m_vertices.size(); i++)
        {
            m_vertices[i].CalcNormal(m_faces, m_edges, false);
        }
    }

}




Bucket::Bucket(Bounds bounds, int idxX, int idxY, int idxZ)
{
	memcpy(&m_bounds[0], &bounds[0], sizeof(Bounds));
	m_gridLocation = { idxX, idxY, idxZ };
}

bool Bucket::IsInside(Eigen::Vector3d point)
{
	return !(point[0] < m_bounds[0] || point[0] > m_bounds[1] ||
		point[1] < m_bounds[2] || point[1] > m_bounds[3] ||
		point[2] < m_bounds[4] || point[2] > m_bounds[5]);
}

bool Bucket::IntersectWithRay(Eigen::Vector3d& origin, Eigen::Vector3d& direction)
{
	Eigen::Vector3d invDir = { 1.0 / direction[0], 1.0 / direction[1], 1.0 / direction[2] };
	Eigen::Vector3i sign = { invDir[0] < 0, invDir[1] < 0, invDir[2] < 0 };

	double tmin, tmax, tymin, tymax, tzmin, tzmax;

	tmin = (m_bounds[sign[0]] - origin[0]) * invDir[0];
	tmax = (m_bounds[1 - sign[0]] - origin[0]) * invDir[0];
	tymin = (m_bounds[2 + sign[1]] - origin[1]) * invDir[1];
	tymax = (m_bounds[3 - sign[1]] - origin[1]) * invDir[1];

	if ((tmin > tymax) || (tymin > tmax))
		return false;
	if (tymin > tmin)
		tmin = tymin;
	if (tymax < tmax)
		tmax = tymax;

	tzmin = (m_bounds[4 + sign[2]] - origin[2]) * invDir[2];
	tzmax = (m_bounds[5 - sign[2]] - origin[2]) * invDir[2];

	if ((tmin > tzmax) || (tzmin > tmax))
		return false;
	if (tzmin > tmin)
		tmin = tzmin;
	if (tzmax < tmax)
		tmax = tzmax;
		return true;
}

void UltraMesh::CalcColors(const double redYellow, const double yellowGreen)
{
    for (int repeats = 0; repeats < 1; repeats++)
    {
        int idx = 0;
        for (auto& face : m_faces)
        {
            face.SetColors(m_vertices, redYellow, yellowGreen);
            idx++;
        }
    }

}

void UltraMesh::SaveAsVRML(const std::wstring fileName)
{
    std::ofstream myfile;
    myfile.open(std::filesystem::path(fileName));
    myfile << "#VRML V2.0 utf8\n#'Mesh by Shai Waisel'\nShape\n{\n";
    myfile << "\tgeometry IndexedFaceSet\n\t{\n";
    myfile << "\t\tcoord Coordinate\n\t\t{\n";
    myfile << "\t\t\tpoint\n\t\t\t[\n";
    for (auto& face : m_faces)
    {
        for (int i=0; i<3; i++)
        myfile << "\t\t\t" << m_vertices[face.m_vertices[i]].m_position[0] << "\t" << 
            m_vertices[face.m_vertices[i]].m_position[1] << "\t" << 
            m_vertices[face.m_vertices[i]].m_position[2] << "\n";
    }
    myfile << "\t\t\t]\n\t\t}\n";
    myfile << "\t\tcolorPerVertex TRUE\n";
    myfile << "\t\tcolor Color\n\t\t{\n";
    myfile << "\t\t\tcolor\n\t\t\t[\n";
    for (auto& face : m_faces)
    {
        for (int i = 0; i < 3; i++)
            myfile << "\t\t\t" << face.m_colors[i][0] << "\t" <<
            face.m_colors[i][1] << "\t" <<
            face.m_colors[i][2] << "\n";
    }
    myfile << "\t\t\t]\n\t\t}\n";
    myfile << "\t\tcoordIndex\n\t\t[\n";
    int counter = 0;
    for (auto& face : m_faces)
    {
        myfile << "\t\t" << counter << "\t" << counter+1 << "\t" << counter+2 << "-1\n";
        counter += 3;
    }
    myfile << "\t\t]\n\t}\n";

    myfile << "}\n";
    myfile.close();

}


void UltraMesh::CalcThickness(const UltraMesh& otherMesh)
{
    for (int idx = 0; idx < m_vertices.size(); idx++)
    {
        m_vertices[idx].m_thickness = (m_vertices[idx].m_position - otherMesh.m_vertices[idx].m_position).norm();
    }
}

bool UltraMesh::CalcMinimas(std::vector<Eigen::Vector3d>& minimas, std::vector<Eigen::Vector3d>& normals)
{
    for (auto& vertex : m_vertices)
    {
        if (vertex.m_normal[2] < -0.2)
        {
            bool lowest = true;
            for (auto& vIdx : vertex.Edges())
            {
                Eigen::Vector3d otherP = m_vertices[m_edges[vIdx].m_idxV2].m_position;
                if (otherP[2] < vertex.m_position[2] + EPSILON6)
                {
                    lowest = false;
                    break;
                }
            }
            if (lowest)
            {
                minimas.push_back(vertex.m_position);
                normals.push_back(vertex.m_normal);
            }
        }
    }

    return true;
}

bool UltraMesh::CalcSkeleton( double minDistBetweenSkeletonPoints, std::vector<Eigen::Vector3d>& skeleton, std::vector<Eigen::Vector3d>& normals)
{
    for (auto& edge : m_edges)
    {
        if (edge.Status() == FLAG_RESET)
        {

            Eigen::Vector3d normal1 = m_faces[edge.m_idxFace].Normal();
            if (normal1[2] < 0.0)
            {
                Eigen::Vector3d normal2 = m_faces[m_edges[edge.Twin()].m_idxFace].Normal();
                if (normal2[2] < 0.0)
                {
                    Eigen::Vector3d p1 = m_vertices[edge.m_idxV1].m_position;
                    Eigen::Vector3d p2 = m_vertices[edge.m_idxV2].m_position;
                    Eigen::Vector3d n1 = m_vertices[edge.m_idxV1].m_normal;
                    Eigen::Vector3d n2 = m_vertices[edge.m_idxV2].m_normal;
                    Eigen::Vector3d p3 = { p2[0], p2[1], p2[2] - 1.0 };
                    Eigen::Vector3d v1 = p2 - p1;
                    Eigen::Vector3d v2 = p3 - p1;
                    Eigen::Vector3d n = v1.cross(v2);
                    double nd1 = normal1.dot(n);
                    double nd2 = normal2.dot(n);
                    if (nd1*nd2 < 0.0)
                    {
                        skeleton.push_back(m_vertices[edge.m_idxV1].m_position);
                        skeleton.push_back(m_vertices[edge.m_idxV2].m_position);
                        normals.push_back(m_vertices[edge.m_idxV1].m_normal);
                        normals.push_back(m_vertices[edge.m_idxV2].m_normal);
                        double len = sqrt((p1[0] - p2[0])*(p1[0] - p2[0]) + (p1[1] - p2[1])*(p1[1] - p2[1]));
                        if (len > minDistBetweenSkeletonPoints)
                        {
                            int middles = (int)round(len / minDistBetweenSkeletonPoints) + 1;
                            double step = len / middles;
                            for (int i = 1; i < middles; i++)
                            {
                                double t = (double)i / middles;
                                Eigen::Vector3d p = p1 + (p2 - p1) * t;
                                Eigen::Vector3d n = n1 + (n2 - n1) * t;
                                n.normalize();
                                skeleton.push_back(p);
                                normals.push_back(n);
                            }
                        }
                        edge.SetStatus(FLAG_MARK);
                        m_edges[edge.Twin()].SetStatus(FLAG_MARK);
                    }
                }
            }
        }
    }
    for (auto& edge : m_edges)
        edge.SetStatus(FLAG_RESET);
    return true;
}

void UltraMesh::GetNearestNeighbours(Eigen::Vector3d seed, double radius, std::vector<UltraVertex>& neighbours)
{
    double sqRad = radius * radius;
    neighbours.reserve(m_vertices.size());
    for (auto& vertex : m_vertices)
    {
        if ((vertex.m_position - seed).squaredNorm() <= sqRad)
            neighbours.push_back(vertex);
    }
}


bool Pinch(Eigen::Vector3d p1, Eigen::Vector3d p2, const double z, double (&segment)[4])
{
    if (abs(p1[2] - z) < EPSILON6)
        p1[2] += EPSILON5;
    if (abs(p2[2] - z) < EPSILON6)
        p2[2] += EPSILON5;
    if (abs(p1[2] - p2[2]) < EPSILON6)
        return false; // line parallel to the pinching plane
    if ((abs(p1[2] - z) < EPSILON6) && (abs(p2[2] - z) < EPSILON6))
        return false; // two points on the pinching plane
    if ((p1[2] - z) * (p2[2] - z) > 0.0)
        return false; // two points above or below the pinching plane
    double factor = (z - p1[2]) / (p2[2] - p1[2]);
    double p[2];
    p[0] = p1[0] + factor * (p2[0] - p1[0]);
    p[1] = p1[1] + factor * (p2[1] - p1[1]);
    if (p2[2] < p1[2])
    {                               // pinching downwards
        segment[0] = p[0];
        segment[1] = p[1];
    }
    else
    {                              // pinching upwards
        segment[2] = p[0];
        segment[3] = p[1];
    }
    return ((abs(segment[2] - segment[0]) > EPSILON3) || (abs(segment[3] - segment[1]) > EPSILON3));
}

void UltraMesh::Transform(Eigen::Affine3d mat)
{
    for (auto& vertex : m_vertices)
    {
        Eigen::Vector4d point = { vertex.m_position[0], vertex.m_position[1], vertex.m_position[2], 1.0};
        point = mat * point;
        for (int i=0; i<3; i++)
            vertex.m_position[i] = point[i];
    }
}

bool UltraMesh::Slice(std::vector<std::pair<double, std::vector<Zpolyline>>>& slices)
{
    typedef std::pair<std::pair<double, double>, std::pair<double, double>> Segment;
    typedef std::vector<Segment> Mikado;

    std::vector<Mikado> soup;
    std::vector<double> zVals;
    soup.reserve(slices.size());
    zVals.reserve(slices.size());
    for (auto& slice : slices)
    {
        Mikado mikado ;
        soup.push_back(mikado);
        zVals.push_back(slice.first);
    }
    for (auto& face : m_faces)
    {
        double faceZmin = std::min(std::min(m_vertices[face.m_vertices[0]].m_position[2], m_vertices[face.m_vertices[1]].m_position[2]), m_vertices[face.m_vertices[2]].m_position[2]);
        double faceZmax = std::max(std::max(m_vertices[face.m_vertices[0]].m_position[2], m_vertices[face.m_vertices[1]].m_position[2]), m_vertices[face.m_vertices[2]].m_position[2]);
        int low = (int)(std::lower_bound(zVals.begin(), zVals.end(), faceZmin) - zVals.begin());
        int high = (int)(std::upper_bound(zVals.begin(), zVals.end(), faceZmax) - zVals.begin());
        for (int idx = low; idx < high; idx++)
        {
            double segment[4] = { 0.0, 0.0, 0.0, 0.0 }; // constant Z
            int pinches = 
                (int)Pinch(m_vertices[face.m_vertices[0]].m_position, m_vertices[face.m_vertices[1]].m_position, zVals[idx], segment) +
                (int)Pinch(m_vertices[face.m_vertices[1]].m_position, m_vertices[face.m_vertices[2]].m_position, zVals[idx], segment) +
                (int)Pinch(m_vertices[face.m_vertices[2]].m_position, m_vertices[face.m_vertices[0]].m_position, zVals[idx], segment);
            if (pinches == 2)
            {
                Segment newSegment = { {segment[0], segment[1]}, {segment[2], segment[3]} };
                soup[idx].push_back(newSegment);
            }
        }
    }
    
    // loop over all slices
    for (int layerIdx = 0; layerIdx < (int)zVals.size(); layerIdx++)
    {
        // loop over all segments in slice idx
        Mikado& mikado = soup[layerIdx];
        bool chained = false;
        while (mikado.size() > 0)
        {
            for (int segmentIdx = 0; segmentIdx < (int)mikado.size(); segmentIdx++)
            {
                auto& segment = mikado[segmentIdx];
                chained = false;
                for (int polyIdx = 0; polyIdx < slices[layerIdx].second.size(); polyIdx++)
                {
                    if (slices[layerIdx].second[polyIdx].AddSegment(segment.first.first, segment.first.second, segment.second.first, segment.second.second))
                    {
                        chained = true;
                        mikado.erase(mikado.begin() + segmentIdx);
                        break;
                    }
                }
                if (chained)
                    break;
            }
            if ((!chained) && (mikado.size() > 0))
            {
                Zpolyline polyLine(zVals[layerIdx]);
                polyLine.AddSegment(mikado[0].first.first, mikado[0].first.second, mikado[0].second.first, mikado[0].second.second);
                slices[layerIdx].second.push_back(polyLine);
                chained = true;
                mikado.erase(mikado.begin());
            }

        }

        for (int polyIdx = (int)slices[layerIdx].second.size() - 1; polyIdx > 0; polyIdx--)
        {
            auto& polyline = slices[layerIdx].second[polyIdx];
            if (polyline.Length() < 0.1)
                slices[layerIdx].second.erase(slices[layerIdx].second.begin() + polyIdx);
        }
    }

    return true;
}

void UltraMesh::AlignToMinZ()
{
    UltraMesh testMesh(*this);
    double accurateThetaX = 0.0, accurateThetaY = 0.0;
    testMesh.CalcBounds();

    Eigen::Affine3d transMat = Eigen::Affine3d::Identity();
    double dx = (testMesh.m_bounds[0] + testMesh.m_bounds[1]) * 0.5;
    double dy = (testMesh.m_bounds[2] + testMesh.m_bounds[3]) * 0.5;
    double dz = (testMesh.m_bounds[4] + testMesh.m_bounds[5]) * 0.5;
    transMat.translation() = Eigen::Vector3d(-dx, -dy, -dz);
    testMesh.Transform(transMat);
    double angFrom = -M_PI_2;
    double angTo = M_PI_2;
    double angStep = (angTo - angFrom) / 10;

    double theta = angFrom;
    while (angStep > 1.0E-6)
    {
        std::vector<double> heights;
        theta = angFrom - angStep;
        while (theta < angTo)
        {
            theta = theta + angStep;
            Eigen::Affine3d aff = Eigen::Affine3d::Identity();
            aff.rotate(Eigen::AngleAxisd(theta, Eigen::Vector3d(1, 0, 0)));
            testMesh.Transform(aff);
            testMesh.CalcBounds();
            heights.push_back(testMesh.m_bounds[5] - testMesh.m_bounds[4]);
            aff = aff.inverse();
            testMesh.Transform(aff);
        }
        int minIdx = (int)(std::min_element(heights.begin(), heights.end()) - heights.begin() + 1);
        if (minIdx < heights.size())
            angTo = angFrom + (minIdx + 1) * angStep;
        else
            angTo += angStep;
        if (minIdx > 1)
            angFrom += (minIdx - 1) * angStep;
        else
            angFrom -= angStep;
        angStep = (angTo - angFrom) / 10.0;
        printf("X: idx %d ang from %1.8f to %1.8f step %1.8f\n", minIdx, angFrom, angTo, angStep);
    }

    accurateThetaX = (angFrom + angTo) * 0.5;
    Eigen::Affine3d aff = Eigen::Affine3d::Identity();
    aff.rotate(Eigen::AngleAxisd(accurateThetaX, Eigen::Vector3d(1, 0, 0)));
    testMesh.Transform(aff);
    Transform(aff);
    angFrom = -M_PI_2;
    angTo = M_PI_2;
    angStep = (angTo - angFrom) / 10;

    theta = angFrom;
    while (angStep > 1.0E-6)
    {
        std::vector<double> heights;
        theta = angFrom - angStep;
        while (theta < angTo)
        {
            theta = theta + angStep;
            Eigen::Affine3d aff = Eigen::Affine3d::Identity();
            aff.rotate(Eigen::AngleAxisd(theta, Eigen::Vector3d(0, 1, 0)));
            testMesh.Transform(aff);
            testMesh.CalcBounds();
            heights.push_back(testMesh.m_bounds[5] - testMesh.m_bounds[4]);
            aff = aff.inverse();
            testMesh.Transform(aff);
        }
        int minIdx = (int)(std::min_element(heights.begin(), heights.end()) - heights.begin() + 1);
        if (minIdx < heights.size())
            angTo = angFrom + (minIdx + 1) * angStep;
        else
            angTo += angStep;
        if (minIdx > 1)
            angFrom += (minIdx - 1) * angStep;
        else
            angFrom -= angStep;
        angStep = (angTo - angFrom) / 10.0;
        printf("Y: idx %d ang from %1.8f to %1.8f step %1.8f\n", minIdx, angFrom, angTo, angStep);
    }

    accurateThetaY = (angFrom + angTo) * 0.5;
    printf("Final rotation angle: X %f Y %f\n", accurateThetaX *180.0 / M_PI, accurateThetaY *180.0 / M_PI);
    aff = Eigen::Affine3d::Identity();
    //aff.rotate(Eigen::AngleAxisd(accurateThetaX, Eigen::Vector3d(1, 0, 0)));
    aff.rotate(Eigen::AngleAxisd(accurateThetaY, Eigen::Vector3d(0, 1, 0)));
    //aff.translate(Eigen::Vector3d(dx, dy, dz));
    Transform(aff);

}






