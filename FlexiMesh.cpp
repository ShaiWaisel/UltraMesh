%FlexiMesh.cpp
#include "pch.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <chrono>

#include "FlexiMesh.h"
#include "Simplifier.h"
#include "FlexiMeshDisplacement.h"



typedef struct
{
	std::set <size_t> data;
	Bounds bounds;
}Bucket;

typedef struct
{
	std::vector <size_t> data;
	Bounds bounds;
}VBucket;

#define TIME_INTERVAL(end, start) double(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()) / 1000.0


FlexiVertex::FlexiVertex(const int index, const double x, const double y, const double z)
{
	m_index = index;
	m_pos[0] = x;
	m_pos[1] = y;
	m_pos[2] = z;
	m_uv[0] = 0.0;
	m_uv[1] = 0.0;
}

FlexiVertex::FlexiVertex(const int index, const double x, const double y, const double z, const double u, const double v)
{
	m_index = index;
	m_pos[0] = x;
	m_pos[1] = y;
	m_pos[2] = z;
	m_uv[0] = u;
	m_uv[1] = v;
}

FlexiVertex::FlexiVertex(const int index, const double x, const double y, const double z,
	const double nx, const double ny, const double nz,
	double const u, double const v)
{
	m_index = index;
	m_pos[0] = x;
	m_pos[1] = y;
	m_pos[2] = z;
	m_normal[0] = nx;
	m_normal[1] = ny;
	m_normal[2] = nz;
	m_uv[0] = u;
	m_uv[1] = v;
}


void FlexiVertex::AddEdge(const int edgeID)
{
	m_connectedEdges.push_back(edgeID);
}

void FlexiVertex::CalcNormal(const FacesVec& faces, const EdgesVec& edges, bool byArea)
{
	m_normal.Zero();
	for (size_t i = 0; i < m_connectedEdges.size(); i++)			// Loop over touching faces
	{
		auto edge = edges[m_connectedEdges[i]];						// Touching face
		Eigen::Vector3d norm = { 0.0, 0.0, 0.0 };
		faces[edge.m_face].GetNormal(norm);								// Touching face norm
		bool unique = true;
		for (size_t j = 0; j < i; j++)
		{
			auto edge2 = edges[m_connectedEdges[j]];						// Touching face
			Eigen::Vector3d norm2 = { 0.0, 0.0, 0.0 };
			faces[edge2.m_face].GetNormal(norm2);								// Touching face norm
			double dot = norm.dot(norm2);
			unique &= (dot < 0.999) && (dot > -0.999);
		}
		if (byArea)
			norm *= faces[edge.m_face].m_area;					// Normal is weighted by each touching face area
		if (unique)
			m_normal += norm;
	}

	m_normal.normalize();													// Keep as 1 unit length
	if (m_normal.norm() == 0.0)
	{
		printf("Bad vertex %d, connected edges=%d\n", m_index, (int)m_connectedEdges.size());
	}
}

bool CompareApprox(
	const VertsVec& lhs, const VertsVec& rhs,
	double epsilon)
{
	if (lhs.size() != rhs.size())
		return false;

	for (size_t vix = 0; vix < lhs.size(); ++vix)
		if (!CompareApprox(lhs[vix], rhs[vix], epsilon))
			return false;

	return true; // joyous day!
}

FlexiEdge::FlexiEdge(int ID, int V1, int V2, int face)
{
	m_index = ID;
	m_V1 = V1;
	m_V2 = V2;
	m_face = face;
}

void FlexiEdge::CalcVector(const VertsVec& vertices)
{
	m_vector = (vertices[m_V2].m_pos - vertices[m_V1].m_pos);
}


FlexiFace::FlexiFace(int index, int V1, int V2, int V3, int E1, int E2, int E3)
{
	m_index = index;
	m_V1 = V1;
	m_V2 = V2;
	m_V3 = V3;
	m_E1 = E1;
	m_E2 = E2;
	m_E3 = E3;
	m_ancestorID = -1;
}

void FlexiFace::CalcPlane(const VertsVec& vertices, const EdgesVec& edges)
{
	double a1 = vertices[m_V2].m_pos[0] - vertices[m_V1].m_pos[0];
	double b1 = vertices[m_V2].m_pos[1] - vertices[m_V1].m_pos[1];
	double c1 = vertices[m_V2].m_pos[2] - vertices[m_V1].m_pos[2];
	double a2 = vertices[m_V3].m_pos[0] - vertices[m_V1].m_pos[0];
	double b2 = vertices[m_V3].m_pos[1] - vertices[m_V1].m_pos[1];
	double c2 = vertices[m_V3].m_pos[2] - vertices[m_V1].m_pos[2];
	m_plane[0] = b1 * c2 - b2 * c1;
	m_plane[1] = a2 * c1 - a1 * c2;
	m_plane[2] = a1 * b2 - b1 * a2;
	double len = sqrt(m_plane[0] * m_plane[0] + m_plane[1] * m_plane[1] + m_plane[2] * m_plane[2]);
	bool success = (len > 0.0);
	if (success)
	{
		len = 1.0 / len;
		m_plane[0] *= len;
		m_plane[1] *= len;
		m_plane[2] *= len;
		m_plane[3] = (-m_plane[0] * vertices[m_V1].m_pos[0] -
			m_plane[1] * vertices[m_V1].m_pos[1] -
			m_plane[2] * vertices[m_V1].m_pos[2]);
		double p = edges[m_E1].m_vector.norm() + edges[m_E2].m_vector.norm() + edges[m_E3].m_vector.norm();
		m_area = sqrt(p * (p - edges[m_E1].m_vector.norm()) * (p - edges[m_E2].m_vector.norm()) * (p - edges[m_E3].m_vector.norm()));
	}
}

bool FlexiFace::RayIntersectsTriangleDist(
	const VertsVec& vertices,
	const Eigen::Vector3d& rayOrigin,
	const Eigen::Vector3d& rayVector,
	Eigen::Vector3d& outIntersectionPoint)
{
	const double EPSILON = 0.0000001;	// TBD: Gather epsilons from wherever are...

	Eigen::Vector3d vertex0 = vertices[m_V1].m_pos;
	Eigen::Vector3d vertex1 = vertices[m_V2].m_pos;
	Eigen::Vector3d vertex2 = vertices[m_V3].m_pos;
	Eigen::Vector3d edge1, edge2, h, s, q;
	double a, f, u, v;
	edge1 = vertex1 - vertex0;
	edge2 = vertex2 - vertex0;
	h = rayVector.cross(edge2);
	a = edge1.dot(h);
	if (a > -EPSILON && a < EPSILON)
		return false;    // This ray is parallel to this triangle.
	f = 1.0 / a;
	s = rayOrigin - vertex0;
	u = f * s.dot(h);
	if (u < -EPSILON || u > 1.0 + EPSILON)
		return false;
	q = s.cross(edge1);
	v = f * rayVector.dot(q);
	if (v < -EPSILON || (u + v) > 1.0 + EPSILON)
		return false;
	// At this stage we can compute t to find out where the intersection point is on the line.
	double t = f * edge2.dot(q);
	if (t > EPSILON) // ray intersection
	{
		outIntersectionPoint = rayOrigin + rayVector * t;
		return true;
	}
	else // This means that there is a line intersection but not a ray intersection.
		return false;
}

bool FlexiFace::CalcUV(const VertsVec& vertices, const Eigen::Vector3d& point, Eigen::Vector2d& barycentric)
{
	const double EPSILON = EPSILON_MICRO;
	Eigen::Vector3d vertex0 = vertices[m_V1].m_pos;
	Eigen::Vector3d vertex1 = vertices[m_V2].m_pos;
	Eigen::Vector3d vertex2 = vertices[m_V3].m_pos;
	Eigen::Vector3d perpendicular = { -m_plane[0], -m_plane[1], -m_plane[2] };
	Eigen::Vector3d edge1, edge2, normEdge1, normEdge2, normS, h, s, q;
	double a, f, u, v;
	edge1 = vertex1 - vertex0;
	edge2 = vertex2 - vertex0;
	normEdge1 = edge1.normalized();
	normEdge2 = edge2.normalized();
	h = perpendicular.cross(normEdge2);
	a = normEdge1.dot(h);
	if (a > -EPSILON && a < EPSILON)
		return false;    // This ray is parallel to this triangle.
	f = 1.0 / a;
	s = point - vertex0;
	normS = s.normalized();
	u = f * normS.dot(h);
	u = std::max(0.0, std::min(u, 1.0));
	if (u < -EPSILON || u > 1.0 + EPSILON)
		return false;
	q = s.cross(normEdge1);
	v = f * perpendicular.dot(q);
	v = std::max(0.0, std::min(v, 1.0));
	if (v < -EPSILON || (u + v) > 1.0 + EPSILON)
		return false;
	// At this stage we can compute t to find out where the intersection point is on the line.
	double t = f * edge2.dot(q);
	if (abs(t) < EPSILON)
	{
		const Eigen::Vector3d v0 = vertex1 - vertex0;
		const Eigen::Vector3d v1 = vertex2 - vertex0;
		double d00 = v0.dot(v0);
		double d01 = v0.dot(v1);
		double d11 = v1.dot(v1);
		double denom = d00 * d11 - d01 * d01;

		if (denom != 0.0) // calculate relative barycentric re projected triangle
		{
			const Eigen::Vector3d v2 = point - vertex0;
			double d20 = v2.dot(v0);
			double d21 = v2.dot(v1);

			double v = (d11 * d20 - d01 * d21) / denom;
			double w = (d00 * d21 - d01 * d20) / denom;
			barycentric[0] = 1 - v - w;
			barycentric[1] = v;
			return true;
		}
		else
			return false;
	}
	else // point is far from triangle plane.
		return false;

}

void FlexiFace::UVinterpolator(const VertsVec& vertices, const Eigen::Vector2d& baryCoords, Eigen::Vector2d& uv)
{
	double lastBary = 1 - baryCoords[0] - baryCoords[1];
	uv[0] = vertices[m_V1].m_uv[0] * baryCoords[0] + vertices[m_V2].m_uv[0] * baryCoords[1] + vertices[m_V3].m_uv[0] * lastBary;
	uv[1] = vertices[m_V1].m_uv[1] * baryCoords[0] + vertices[m_V2].m_uv[1] * baryCoords[1] + vertices[m_V3].m_uv[1] * lastBary;
}

void FlexiFace::UVinterpolator(const Eigen::Vector2d& baryCoords, Eigen::Vector2d& worldBarycentric)
{
	double lastBary = 1 - baryCoords[0] - baryCoords[1];
	worldBarycentric = m_UV1 * baryCoords[0] +
		m_UV2 * baryCoords[1] +
		m_UV3 * lastBary;
}




double FlexiFace::MaxEnclosedRad(Eigen::Vector3d& P1, Eigen::Vector3d& P2, Eigen::Vector3d P3)
{
	// return the radius of the maximal enclosed circle in a given triangle
	Eigen::Vector3d V12 = P2 - P1;
	Eigen::Vector3d V23 = P3 - P2;
	Eigen::Vector3d V31 = P1 - P3;
	double len12 = V12.norm();
	double len23 = V23.norm();
	double len31 = V31.norm();
	double halfPerimeter = (len12 + len23 + len31) * 0.5;
	if (halfPerimeter > 0.0)
	{
		double area = sqrt(halfPerimeter * (halfPerimeter - len12) * (halfPerimeter - len23) * (halfPerimeter - len31));
		return area / halfPerimeter;
	}
	else return (-1.0);
}

void FlexiFace::InOffset(double distance, Eigen::Vector3d& P1, Eigen::Vector3d& P2, Eigen::Vector3d& P3)
{
	Eigen::Vector3d VP = { 0.0, 0.0, 0.0 }, V1 = { 0.0, 0.0, 0.0 }, V2 = { 0.0, 0.0, 0.0 };
	double dot = 0.0, halfAng = 0.0, len = 0.0;
	// calculate half angle @ V1
	Eigen::Vector3d OP1 = P1;
	Eigen::Vector3d OP2 = P2;
	Eigen::Vector3d OP3 = P3;

	V1 = (OP2 - OP1);
	V2 = (OP3 - OP1);
	V1.normalize();
	V2.normalize();
	dot = V1.x() * V2.x() + V1.y() * V2.y() + V1.z() * V2.z();
	halfAng = 0.5 * acos(dot / sqrt(V1.squaredNorm() * V2.squaredNorm()));
	len = distance / sin(halfAng);
	VP = Eigen::Vector3d(V1 + V2);
	VP.normalize();
	P1 = OP1 + VP * len;

	// calculate half angle @ V2
	V1 = (OP3 - OP2);
	V2 = (OP1 - OP2);
	V1.normalize();
	V2.normalize();
	dot = V1.x() * V2.x() + V1.y() * V2.y() + V1.z() * V2.z();
	halfAng = 0.5 * acos(dot / sqrt(V1.squaredNorm() * V2.squaredNorm()));
	len = distance / sin(halfAng);
	VP = Eigen::Vector3d(V1 + V2);
	VP.normalize();
	P2 = OP2 + VP * len;

	// calculate half angle @ V1
	V1 = (OP1 - OP3);
	V2 = (OP2 - OP3);
	V1.normalize();
	V2.normalize();
	dot = V1.x() * V2.x() + V1.y() * V2.y() + V1.z() * V2.z();
	halfAng = 0.5 * acos(dot / sqrt(V1.squaredNorm() * V2.squaredNorm()));
	len = distance / sin(halfAng);
	VP = Eigen::Vector3d(V1 + V2);
	VP.normalize();
	P3 = OP3 + VP * len;
}

FlexiMesh::FlexiMesh(const Vertices& vertices, const Faces& faces, const UVs & uvs)
{
	m_vertices.resize(vertices.size());
	m_faces.resize(faces.size());
	m_edges.resize(m_faces.size() * 3);
	if (uvs.empty())
	{
		for (int i = 0; i < (int)vertices.size(); i++)
		{
			m_vertices[i] = FlexiVertex(i, vertices[i][0], vertices[i][1], vertices[i][2]);
		}
	}
	else
	{
		for (int i = 0; i < (int)vertices.size(); i++)
		{
			m_vertices[i] = FlexiVertex(i, vertices[i][0], vertices[i][1], vertices[i][2], uvs[i][0], uvs[i][1]);
		}
	}
	for (int i = 0; i < (int)faces.size(); i++)
	{
		int edgeIdx = i * 3;
		m_faces[i] = FlexiFace(i, faces[i][0], faces[i][1], faces[i][2], edgeIdx, edgeIdx + 1, edgeIdx + 2);
		m_edges[edgeIdx] = FlexiEdge(edgeIdx, faces[i][0], faces[i][1], i);
		m_vertices[faces[i][0]].AddEdge(edgeIdx);
		edgeIdx++;
		m_edges[edgeIdx] = FlexiEdge(edgeIdx, faces[i][1], faces[i][2], i);
		m_vertices[faces[i][1]].AddEdge(edgeIdx);
		edgeIdx++;
		m_edges[edgeIdx] = FlexiEdge(edgeIdx, faces[i][2], faces[i][0], i);
		m_vertices[faces[i][2]].AddEdge(edgeIdx);
		m_faces[i].CalcPlane(m_vertices, m_edges);
		m_faces[i].m_UV1[0] = uvs[m_faces[i].m_V1][0];
		m_faces[i].m_UV1[1] = uvs[m_faces[i].m_V1][1];
		m_faces[i].m_UV2[0] = uvs[m_faces[i].m_V2][0];
		m_faces[i].m_UV2[1] = uvs[m_faces[i].m_V2][1];
		m_faces[i].m_UV3[0] = uvs[m_faces[i].m_V3][0];
		m_faces[i].m_UV3[1] = uvs[m_faces[i].m_V3][1];
	}
	MapEdges();
}

FlexiMesh::FlexiMesh(FlexiMesh&& other)
{
	*this = std::move(other);
}

FlexiMesh& FlexiMesh::operator=(FlexiMesh&& other)
{
	m_vertices = std::move(other.m_vertices);
	m_faces = std::move(other.m_faces);
	m_edges = std::move(other.m_edges);

	return *this;
}

bool FlexiMesh::IsManifold()
{
	bool isManifold = true;
	for (FlexiEdge& edge : m_edges)
	{
		if (edge.m_twin < 0)
		{
			isManifold = false;
			break;
		}
	}
	return isManifold;
}

void FlexiMesh::CopyFacesUV(const FlexiMesh& fromMesh)
{
	if (fromMesh.m_faces.size() != m_faces.size()) {
		PRINT_ERROR("Attempting to copy UV information between meshes of different sizes.");
		throw std::runtime_error("Face count mismatch.");
	}

	for (size_t idx = 0; idx < fromMesh.m_faces.size(); idx++) {
		m_faces[idx].m_UV1 = fromMesh.m_faces[idx].m_UV1;
		m_faces[idx].m_UV2 = fromMesh.m_faces[idx].m_UV2;
		m_faces[idx].m_UV3 = fromMesh.m_faces[idx].m_UV3;
	}
}


void FlexiMesh::CopySelectFacesUV(
	const FlexiMesh& fromMesh,
	const std::vector<size_t>& selection)
{
	if (selection.size() > m_faces.size()) {
		PRINT_ERROR("Attempting to copy UV information from too many faces.");
		throw std::runtime_error("Face count mismatch.");
	}

	for (size_t idx = 0; idx < selection.size(); idx++) {
		size_t srcFaceIx = selection[idx];

		m_faces[idx].m_UV1 = fromMesh.m_faces[srcFaceIx].m_UV1;
		m_faces[idx].m_UV2 = fromMesh.m_faces[srcFaceIx].m_UV2;
		m_faces[idx].m_UV3 = fromMesh.m_faces[srcFaceIx].m_UV3;
	}
}

void FlexiMesh::MapEdges()
{
	using HalfEdgeContainer = std::unordered_map<
		std::pair<int, int>, // v1 -> v2
		size_t, // index in edges vector.
		boost::hash<std::pair<int, int>>
	>;
	HalfEdgeContainer edgeLocator;
	m_edges.clear();
	m_edges.reserve(m_faces.size() * 3);

	for (size_t faceIx = 0; faceIx < m_faces.size(); ++faceIx) {
		std::array<int, 3> faceVertIxs = { m_faces[faceIx].m_V1, m_faces[faceIx].m_V2, m_faces[faceIx].m_V3 };

		int lastEdge = (int)m_edges.size();
		m_faces[faceIx].m_E1 = lastEdge;
		m_faces[faceIx].m_E2 = lastEdge + 1;
		m_faces[faceIx].m_E3 = lastEdge + 2;

		for (unsigned vix = 0; vix < 3; ++vix) {
			int vert = faceVertIxs[vix];
			int nextVert = faceVertIxs[(vix + 1) % 3];

			int edgeID = (int)m_edges.size();
			m_edges.emplace_back(edgeID, vert, nextVert, (int)faceIx);

			std::pair<int, int> twinVerts{ nextVert, vert };
			HalfEdgeContainer::iterator twinIt;
			if ((twinIt = edgeLocator.find(twinVerts)) != edgeLocator.end()) {
				m_edges[twinIt->second].SetTwin(edgeID);
				m_edges[edgeID].SetTwin((int)twinIt->second);
				edgeLocator.erase(twinIt);
			}
			else {
				edgeLocator[{vert, nextVert}] = edgeID;
			}
		}
	}
	for (size_t i = 0; i < m_edges.size(); i++)
	{
		m_edges[i].CalcVector(m_vertices);
	}
	for (size_t i = 0; i < m_vertices.size(); i++)
	{
		m_vertices[i].CalcNormal(m_faces, m_edges, false);
	}
}


FlexiMesh::~FlexiMesh()
{
	m_faces.clear();
	m_edges.clear();
	m_vertices.clear();
}

void FlexiMesh::MoveVertexTo(int vertexIdx, double toX, double toY, double toZ)
{
	FlexiVertex* vertex = &m_vertices[vertexIdx];
	vertex->m_pos[0] = toX;
	vertex->m_pos[1] = toY;
	vertex->m_pos[2] = toZ;
	for (size_t i = 0; i < vertex->m_connectedEdges.size(); i++)
		m_edges[vertex->m_connectedEdges[i]].CalcVector(m_vertices);
	for (size_t i = 0; i < vertex->m_connectedEdges.size(); i++)
		m_faces[m_edges[vertex->m_connectedEdges[i]].m_face].CalcPlane(m_vertices, m_edges);
}

void FlexiMesh::Smooth()
{
	// Non weighted averaging
	for (size_t i = 0; i < m_vertices.size(); i++)
	{
		FlexiVertex* vertex = &m_vertices[i];
		Eigen::Vector3d target = { 0.0, 0.0, 0.0 };
		for (size_t j = 0; j < vertex->m_connectedEdges.size(); j++)
		{
			target += m_vertices[m_edges[vertex->m_connectedEdges[j]].m_V2].m_pos;
		}
		vertex->m_dispos = target / vertex->m_connectedEdges.size();
	}
	// move distpos to pos
	for (size_t i = 0; i < m_vertices.size(); i++)
	{
		m_vertices[i].m_pos = m_vertices[i].m_dispos;
	}
	// calc new edges vector lengths
	for (size_t i = 0; i < m_edges.size(); i++)
	{
		m_edges[i].CalcVector(m_vertices);
	}
	// calc new faces planes and areas
	for (size_t i = 0; i < m_faces.size(); i++)
	{
		m_faces[i].CalcPlane(m_vertices, m_edges);
	}
	// calc each vertex normal and local curvature
	for (size_t i = 0; i < m_vertices.size(); i++)
	{
		m_vertices[i].CalcNormal(m_faces, m_edges, false);
	}
}

// "Adaptive offset" function

void FlexiMesh::LimitByDistances(Bounds bounds, bool inwards, double maxDisplacement)
{
	auto start = std::chrono::high_resolution_clock::now();
	auto end = start;

	// set direction Normal multiplier
	double direction = (inwards) ? -1.0 : 1.0;								// bool to double factor
#if DEBUG_MODE
	double maxCheckDiff = 0;
#endif

	// A simple performance booster mechanism - divide all faces into 3D array of containers
	// ("buckets") for more efficient search. 
	// Nbuckets size is set by thumb rule as the fourth root of the number of vertices.
	int Nbuckets = std::max(2, int(sqrt(sqrt(m_vertices.size()))));

	// The buckets array (Nbuckets*Nbuckets*Nbuckets) is arranged as a vector. 
	// TBD check for more robust implementation such as KDtree
	std::vector <Bucket> buckets;

	// Each bucket is overlapping its neighbors by half of maxDisplacement. This bucketMargin is needed for
	// checking "proposed" vertices positions (m_dispos) that are initially set to maxDisplacement.
	double bucketMargins = maxDisplacement * 0.5;

	// Each bucket size "a", excluding margins, is set by dividing the total vertices bounding box by Nbuckets
	double ax = (bounds[1] - bounds[0]) / Nbuckets;
	double ay = (bounds[3] - bounds[2]) / Nbuckets;
	double az = (bounds[5] - bounds[4]) / Nbuckets;

	// Loop over entire volume and set VolumeOfInterest for each bucket, then push it into buckets.
	for (int i = 0; i < Nbuckets; i++)
	{
		for (int j = 0; j < Nbuckets; j++)
		{
			for (int k = 0; k < Nbuckets; k++)
			{
				Bucket bucket = Bucket();
				Bounds box = { bounds[0] + k * ax - bucketMargins, bounds[0] + (k + 1) * ax + bucketMargins,
					bounds[2] + j * ay - bucketMargins, bounds[2] + (j + 1) * ay + bucketMargins,
					bounds[4] + i * az - bucketMargins, bounds[4] + (i + 1) * az + bucketMargins };
				memcpy(&bucket.bounds[0], &box[0], sizeof(Bounds));
				buckets.push_back(bucket);
			}
		}
	}
	// buckets are ready, hence push faces into relevant buckets (each face can appear in more than one bucket)
	for (size_t i = 0; i < m_faces.size(); i++)
	{
#ifdef DEBUG_MODE
		bool isInside = false;
#endif
		// found variable counts the bucket insertions. it's value shall always be in range 1..3
		int found = 0;
		int faceVs[3] = { m_faces[i].m_V1, m_faces[i].m_V2,m_faces[i].m_V3 };
		for (size_t j = 0; j < 3; j++)
		{
			// After three vertices are considered in buckets, no need for further checking.
			if (found > 2)
				break;
			// Each face can be recognized by 1-3 buckets, depending on each vertex location

			for (size_t k = 0; k < buckets.size(); k++)
			{
				if ((m_vertices[faceVs[j]].m_pos[0] >= buckets[k].bounds[0]) &&
					(m_vertices[faceVs[j]].m_pos[0] <= buckets[k].bounds[1]) &&
					(m_vertices[faceVs[j]].m_pos[1] >= buckets[k].bounds[2]) &&
					(m_vertices[faceVs[j]].m_pos[1] <= buckets[k].bounds[3]) &&
					(m_vertices[faceVs[j]].m_pos[2] >= buckets[k].bounds[4]) &&
					(m_vertices[faceVs[j]].m_pos[2] <= buckets[k].bounds[5]))
				{
					// found inside
					found++;
					buckets[k].data.insert((int)i);
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

	for (size_t i = 0; i < m_vertices.size(); i++)
	{
		// progress message every (big prime number) loops.
		if (i % 877 == 0)																			// saves output time
			printf("\rProcessing...%3.1f%%", double(i) / m_vertices.size()*100.0);
		// vertex to be displaced

		FlexiVertex* vertex = &m_vertices[i];

		// set default displacement to maximum, considering normal direction and in/out factor.
		vertex->m_dispos = vertex->m_pos + vertex->m_normal * direction * maxDisplacement;

		// eliminate infinite loop in case of a bad vertex
		if (vertex->m_normal.norm() == 0)
		{
			vertex->m_limit = 0.0;
			continue;
		}
		// distance limit from original point
		vertex->m_limit = maxDisplacement;


		// Provided that vertex will be moved, check for overruling by looping over buckets and iterating on
		// nearby faces.
		for (size_t j = 0; j < buckets.size(); j++)
		{
			if ((m_vertices[i].m_dispos[0] < buckets[j].bounds[0]) ||
				(m_vertices[i].m_dispos[0] > buckets[j].bounds[1]) ||
				(m_vertices[i].m_dispos[1] < buckets[j].bounds[2]) ||
				(m_vertices[i].m_dispos[1] > buckets[j].bounds[3]) ||
				(m_vertices[i].m_dispos[2] < buckets[j].bounds[4]) ||
				(m_vertices[i].m_dispos[2] > buckets[j].bounds[5]))
				continue; // outside bucket
			// Iterate over relevant faces
			std::set<size_t>::iterator it = buckets[j].data.begin();

			// As long as there are nearby triangles to check
			while (it != buckets[j].data.end())
			{
				// check nearby triangle "face"
				FlexiFace* face = &m_faces[*it];
				// Exclude  triangles touching (neighboring of) "vertex" 
				if ((face->m_V1 != vertex->m_index) && (face->m_V2 != vertex->m_index) && (face->m_V3 != vertex->m_index))
				{
					// Check minimal distance from proposed new vertex position to either of face's vertices
					Eigen::Vector3d TestPoint = vertex->m_pos + vertex->m_normal * direction * vertex->m_limit;
					double dist1 = (m_vertices[face->m_V1].m_pos - TestPoint).norm();
					double dist2 = (m_vertices[face->m_V2].m_pos - TestPoint).norm();
					double dist3 = (m_vertices[face->m_V3].m_pos - TestPoint).norm();
					double minDist = std::min(dist1, std::min(dist2, dist3));
					// In case minimal distance is closer to "face", yet significant, perform 
					// a further investigation
					if ((vertex->m_limit > 1.0E-5) && (minDist < vertex->m_limit)) // no point at limit < machine resolution (10u)
					{
						// set closest vertex as point P
						Eigen::Vector3d v2p, P;
						if (dist1 == minDist)
							P = m_vertices[face->m_V1].m_pos;
						else if (dist2 == minDist)
							P = m_vertices[face->m_V2].m_pos;
						else
							P = m_vertices[face->m_V3].m_pos;


						// Extract "face" normal direction from plane equation
						Eigen::Vector3d faceNormal = -direction * face->m_plane.block<3, 1>(0, 0);

						// Check for point closer than closest vertex (testPoint projection on face "face") as 
						// the closest point
						Eigen::Vector3d faceTouchingPoint = { 0.0, 0.0, 0.0 };
						Eigen::Vector3d FTP = { 0.0, 0.0, 0.0 };;
						if (face->RayIntersectsTriangleDist(m_vertices, vertex->m_dispos, faceNormal, faceTouchingPoint))
						{
							// closest point is the projected testPoint on face (faceTouchingPoint)
							double disTouch = (faceTouchingPoint - TestPoint).norm();
							// Update minimal distance
							if (disTouch < minDist)
							{
								P = faceTouchingPoint;
								minDist = disTouch;
							}
						}
						// Minimal distance is closer than current limit - hence update limit to the point of equilibrium
						if (minDist < vertex->m_limit)
						{
							v2p = P - vertex->m_pos;
							double len = v2p.norm();
							v2p.normalize();
							double cosAng = v2p.dot(vertex->m_normal * direction);
							bool modified = false;
							// Calculate point of equilibrium
							if (abs(cosAng) > 0.0)
							{
								// Point of equilibrium in case the overruling face is not perpendicular to vertex normal 
								vertex->m_limit = std::min(vertex->m_limit, (len * 0.4999 / cosAng)); // notch below 0.5 to eliminate infinite loop on roundings.
								modified = true;
							}
							else
							{
								if ((vertex->m_pos - P).norm() > 1.0E-5)
								{
									// Point of equilibrium in case the overruling face is perpendicular to vertex normal
									vertex->m_limit = ((vertex->m_pos + P) * 0.5).norm();
									modified = true;
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
								it = buckets[j].data.begin();
						}
					}
				}
				it++;
			}
		}
		// Iterations completed, m_limit is set to optimum hence m_dispos is set.
		vertex->m_dispos = vertex->m_pos + direction * vertex->m_normal * vertex->m_limit;
	}
	printf("\rProcessing...%3.1f%%\n", 100.0);
	// Vertex repositioning completed, updating FlexiVertex internal variables
	for (size_t i = 0; i < m_vertices.size(); i++)
	{
		FlexiVertex* vertex = &m_vertices[i];
		vertex->m_pos = vertex->m_dispos;
	}

	// Recalculate edges vectors after vertices were moved
	for (size_t i = 0; i < m_edges.size(); i++)
	{
		m_edges[i].CalcVector(m_vertices);
	}

	// Recalculate faces planes after vertices were moved
	for (size_t i = 0; i < m_faces.size(); i++)
	{
		m_faces[i].CalcPlane(m_vertices, m_edges);
	}

	// Recalculate vertices normals after vertices were moved
	for (size_t i = 0; i < m_vertices.size(); i++)
	{
		m_vertices[i].CalcNormal(m_faces, m_edges, false);
	}
	end = std::chrono::high_resolution_clock::now();
	printf(" Limit mesh created in: %3.3f seconds \n\n", TIME_INTERVAL(end, start));
}

bool FlexiFace::ValidTriangle(Eigen::Vector3d& P1, Eigen::Vector3d& P2, Eigen::Vector3d& P3)
{
	// return true if triangle is valid (i.e. area > 0)
	double area = (P1 - P2).cross(P1 - P3).norm();
	return (area > 0.0);
}


void FlexiFace::TessellateStrip(Eigen::Vector3d fromLeft, Eigen::Vector3d toLeft,
	Eigen::Vector3d fromRight, Eigen::Vector3d toRight, double stepSize, Vertices& vertices, Faces& faces)
{
	/***************************************************************************************************/
	/*   TessellateStrip cuts a triangle or trapezoid into a series of continuous triangles.           */
	/*   A step size is calculated per each of the main path definers (Left and Right)                 */
	/*                                                                                                 */
	/*   The function loops through the path definers, each loop shortens the longer path definer      */
	/*   by a single step and output one tessellated, smaller than stepSize, triangle.                 */
	/*                                                                                                 */
	/*   In some cases the Left path definer may be zero length, providing a set of two triangles.     */
	/*                                                                                                 */
	/*								stepL                                                                     */
	/*                    +-------+                                                                    */
	/*                nextLeft                                                                         */
	/*	fromLeft  +-------+-------+ toLeft                                                             */
	/*				  |\     / \     /                                                                     */
	/*				  |  \  |   \   /                  PROGRESS >>>>>>>                                    */
	/*				  |    \/    \ /                                                                       */
	/*  fromRight +-----+-----+ toRight                                                                */
	/*              nextRight                                                                          */
	/*                  +-----+                                                                        */
	/*                   stepR                                                                         */
	/*                                                                                                 */
	/***************************************************************************************************/	// Difference vectors
	if (toRight.isApprox(toLeft, EPSILON_NANO))
		toRight = toLeft;
	if (fromRight.isApprox(fromLeft, EPSILON_NANO))
		fromRight = fromLeft;

	Eigen::Vector3d stripL = (toLeft - fromLeft);
	Eigen::Vector3d stripR = (toRight - fromRight);
	// Local vars
	double lengthL = stripL.norm();
	double lengthR = stripR.norm();
	if ((lengthL > EPSILON_NANO) || (lengthR > EPSILON_NANO))
	{
		// Direction
		stripL.normalize();
		stripR.normalize();
		// Calculate L and R path finders by integer, providing edges shorter or equal to stepSize
		size_t index = vertices.size();
		int iSteps = int(lengthL / stepSize + 1);
		double stepL = lengthL / double(iSteps);
		iSteps = int(lengthR / stepSize + 1);
		double stepR = lengthR / double(iSteps);
		// Next step containers
		Eigen::Vector3d nextLeft = { 0.0, 0.0, 0.0 };
		Eigen::Vector3d nextRight = { 0.0, 0.0, 0.0 };
		while ((lengthL > EPSILON_NANO) || (lengthR > EPSILON_NANO))
		{
			// still way to go: calculate next stop
			nextLeft = fromLeft + stripL * std::min(lengthL, stepL);
			nextRight = fromRight + stripR * std::min(lengthR, stepR);
			if (nextLeft.isApprox(toLeft, EPSILON_NANO))
				nextLeft = toLeft;
			if (nextRight.isApprox(toRight, EPSILON_NANO))
				nextRight = toRight;
			bool validTriangle = false;
			if (lengthL >= lengthR)
			{
				// Left is longer, therefore deduct a triangle based on Left
				validTriangle = ValidTriangle(fromLeft, fromRight, nextLeft);
				if (validTriangle)
				{
					vertices.push_back({ fromLeft[0], fromLeft[1], fromLeft[2] });
					vertices.push_back({ fromRight[0], fromRight[1], fromRight[2] });
					vertices.push_back({ nextLeft[0], nextLeft[1], nextLeft[2] });
					lengthL -= stepL;
					fromLeft = nextLeft;
				}
			}
			else
			{
				// Right is longer, therefore deduct a triangle based on Right
				validTriangle = ValidTriangle(fromRight, nextRight, fromLeft);
				if (validTriangle)
				{
					vertices.push_back({ fromRight[0], fromRight[1], fromRight[2] });
					vertices.push_back({ nextRight[0], nextRight[1], nextRight[2] });
					vertices.push_back({ fromLeft[0], fromLeft[1], fromLeft[2] });
					lengthR -= stepR;
					fromRight = nextRight;
				}
			}
			if (!validTriangle)
			{
				if (fromRight.isApprox(fromLeft, EPSILON_NANO))
				{
					if ((lengthL > EPSILON_NANO) && (lengthR > EPSILON_NANO))
					{
						validTriangle = true;
						vertices.push_back({ fromRight[0], fromRight[1], fromRight[2] });
						vertices.push_back({ nextRight[0], nextRight[1], nextRight[2] });
						vertices.push_back({ nextLeft[0], nextLeft[1], nextLeft[2] });
					}
					else
					{
						lengthR = 0.0;
						lengthL = 0.0;
					}
				}				if (toRight.isApprox(toLeft, EPSILON_NANO))
				{
					if ((lengthL > EPSILON_NANO) && (lengthR > EPSILON_NANO))
					{
						validTriangle = true;
						vertices.push_back({ fromLeft[0], fromLeft[1], fromLeft[2] });
						vertices.push_back({ fromRight[0], fromRight[1], fromRight[2] });
						vertices.push_back({ toRight[0], toRight[1], toRight[2] });
					}
					else
					{
						lengthR = 0.0;
						lengthL = 0.0;
					}
				}
				if (validTriangle)
				{
					lengthR -= stepR;
					fromRight = nextRight;
					lengthL -= stepL;
					fromLeft = nextLeft;
				}
			}			if (validTriangle)
			{
				faces.push_back({ (int)index, (int)index + 1, (int)index + 2 });
				index += 3;
			}
		}
	}
}

void FlexiFace::TessellateEdge(Eigen::Vector3d& fromLeft, Eigen::Vector3d& s1Left, Eigen::Vector3d& s2Left, Eigen::Vector3d& toLeft,
	Eigen::Vector3d& fromRight, Eigen::Vector3d& toRight, double stepSize, Vertices& vertices, Faces& faces)
{
	/***************************************************************************************************/
	/*   TessellateEdge called by EtssellateOutsideIn, tessellate three regions between an edge and    */
	/*   it's offset value:                                                                            */
	/*                                                                                                 */
	/*   1. From starting point P1, opening a triangle from P1 to the end of the angle bisector s1Left */
	/*      and to the relevant step point on the edge - s1Right                                       */
	/*                                                                                                 */
	/*   2. From s1Left and s1Right to s2Left and s2Right, respectively                                */
	/*                                                                                                 */
	/*   3. From s2Left and s2Right to P2                                                              */
	/*                                                                                                 */
	/*   Tessellation of all three regions is performed by calls to TessellateStrip                    */
	/*                                                                                                 */
	/*                                                                                                 */
	/*                                         s1Left            s2Left                                */
	/*                                          +-----------------+                                    */
	/*                             angle     /                      \     angle                        */
	/*                           bisector /                           \ bisector                       */
	/*                                 /                                \                              */
	/*                              /                                     \                            */
	/*                           +------+------+------+------+------+------+                           */
	/*                         P1                     |             |       P2                         */
	/*                                              s1Right        s2Right                             */
	/*                                                                                                 */
	/***************************************************************************************************/

	// Original edge vector
	Eigen::Vector3d right = (toRight - fromRight);
	double lengthR = right.norm();
	right.normalize();
	// native number of steps to conclude edge ("right") in a step smaller or equal to stepSize
	int iSteps = int(lengthR / stepSize + 1);
	double step = lengthR / double(iSteps);
	// Two bisector vectors
	double angBisectorLength1 = (s1Left - fromLeft).norm();
	double angBisectorLength2 = (toLeft - s2Left).norm();
	//Relative influence on step index selection along right
	double angBisectorInfluence = (angBisectorLength1 + angBisectorLength2) / lengthR;
	if (iSteps > 1)
	{
		// Edge is long enough to be tessellated
		// Calculate stop indices along "right" edge for s1Right and s2Right
		int iStop1 = std::min(iSteps / 2, std::max(1, int(trunc(iSteps * angBisectorInfluence * (angBisectorLength1 / (angBisectorLength1 + angBisectorLength2))))));
		Eigen::Vector3d s1Right = fromRight + right * iStop1 * step;
		int iStop2 = std::min(iSteps / 2, std::max(1, int(trunc(iSteps * angBisectorInfluence * (angBisectorLength2 / (angBisectorLength1 + angBisectorLength2))))));
		Eigen::Vector3d s2Right = toRight - right * iStop2 * step;
		// Call TessellateStrip for ramp-up, middle edge and ramp-down
		TessellateStrip(fromLeft, s1Left, fromRight, s1Right, stepSize, vertices, faces);
		TessellateStrip(s1Left, s2Left, s1Right, s2Right, stepSize, vertices, faces);
		TessellateStrip(s2Left, toLeft, s2Right, toRight, stepSize, vertices, faces);
	}
	else
	{
		// edge is too short for a three stage tessellation, one is enough.
		TessellateStrip(s2Left, toRight, s1Left, fromRight, stepSize, vertices, faces);
	}
}


void FlexiFace::TessellateSpiral(Eigen::Vector3d P1, Eigen::Vector3d P2, Eigen::Vector3d P3,
	double stepSize, Vertices& vertices, Faces& faces)
{
	/***************************************************************************************************/
	/*   TessellateTriangle is a recursive call in order to divide any triangle to similar,            */
	/*   preferrably sharp angled triangles whom edge size shall not be bigger than stepSize variable. */
	/*                                                                                                 */
	/*   After calculating the edges length the operation can continue in either of three options:     */
	/*   1. At lest two edges must be divided since they are longer than stepSize:                     */
	/*      a. Reduce the triangle area by "strip"                                                     */
	/*      b. Recursively call TessellateTriangle using the reduced triangle                          */
	/*   2. One edge must be divided since it is longer than stepSize, triangle is a one-strip:        */
	/*      a. Strip the triangle                                                                      */
	/*   3. All three edges are shorter or equal stepSize:                                             */
	/*      b. Output the triangle                                                                     */
	/*                                                                                                 */
	/*                                     P3                                                          */
	/*                                    /  \                                                         */
	/*                                   /    \                                                        */
	/*                                  /      \                                                       */
	/*                        nextFrom /        \ nextTo                                               */
	/*                            P1' +----------+ P2'                                                 */
	/*                               /   strip    \                                                    */
	/*                              +--------------+                                                   */
	/*                            P1                P2                                                 */
	/*                                                                                                 */
	/***************************************************************************************************/
	Eigen::Vector3d nextFrom = { 0.0, 0.0, 0.0 };
	Eigen::Vector3d nextTo = { 0.0, 0.0, 0.0 };
	double len12 = (P2 - P1).norm();
	double len23 = (P3 - P2).norm();
	double len31 = (P1 - P3).norm();
	size_t ns = faces.size();
	if ((floor(len12 / stepSize) + floor(len23 / stepSize) + floor(len31 / stepSize)) > 1)
	{
		// triangle is big enough for recursive tessellation
		bool justAstrip = false;
		nextFrom = (P3 - P1);
		double length = nextFrom.norm();
		if (length <= stepSize)
		{
			nextFrom = P3;
			nextTo = P2;
			justAstrip = true;
		}
		else
		{
			int iSteps = int(length / stepSize + 1);
			double step = length / double(iSteps);
			nextFrom.normalize();
			nextFrom = P1 + nextFrom * step;
			nextTo = (P3 - P2);
			length = nextTo.norm();
			if (length <= stepSize)
			{
				nextFrom = P1;
				nextTo = P3;
				justAstrip = true;
			}
			else
			{
				iSteps = int(length / stepSize + 1);
				step = length / double(iSteps);
				nextTo.normalize();
				nextTo = P2 + nextTo * step;
			}

		}
		TessellateStrip(nextFrom, nextTo, P1, P2, stepSize, vertices, faces);
		if (!justAstrip)
			TessellateSpiral(nextTo, P3, nextFrom, stepSize, vertices, faces);
	}
	else
	{
		// triangle is one strip size
		if ((len12 >= stepSize) && (len23 <= stepSize) && (len31 <= stepSize))
		{
			TessellateStrip(P3, P3, P1, P2, stepSize, vertices, faces);
		}
		else if ((len23 >= stepSize) && (len12 <= stepSize) && (len31 <= stepSize))
		{
			TessellateStrip(P1, P1, P2, P3, stepSize, vertices, faces);
		}
		else if ((len31 >= stepSize) && (len12 <= stepSize) && (len23 <= stepSize))
		{
			TessellateStrip(P2, P2, P3, P1, stepSize, vertices, faces);
		}
		else
		{
			// triangle is too small from being tessellated
			if (ValidTriangle(P1, P2, P3))
			{
				size_t index = vertices.size();
				vertices.push_back({ P1[0], P1[1], P1[2] });
				vertices.push_back({ P2[0], P2[1], P2[2] });
				vertices.push_back({ P3[0], P3[1], P3[2] });
				faces.push_back({ (int)index, (int)index + 1, (int)index + 2 });
			}
			else
				printf("Invalid Triangle!\n");
		}
	}
}

void FlexiFace::TessellateOutsideIn(Eigen::Vector3d P1, Eigen::Vector3d P2, Eigen::Vector3d P3,
	double stepSize, Vertices& vertices, Faces& faces)
{
	/***************************************************************************************************/
	/*   TessellateOutsideIn is a recursive call in order to divide any triangle to similar,           */
	/*   preferrably sharp angled triangles whom edge size shall not be bigger than stepSize variable. */
	/*                                                                                                 */
	/*   After calculating the edges length the operation can continue in either of two options:       */
	/*                                                                                                 */
	/*   1. Triangle is big enough to create a smaller (offset by stepSize) triangle. This is checked  */
	/*      by comparing enclosed maximal circle R to be > setpSize.                                   */
	/*      When this is the case, three angle bisectors define the three offset vertices P1i, P2i and */
	/*      P3i. There are nine strips created, three per edge:                                        */
	/*      a. A strip from first vertex to offset vertex and it's projection on edge (P1->P1i)        */
	/*      b. A strip between two offset vertices (P1i -> P2i)                                        */
	/*      b. A strip between the second offset vertex to the last vertex (P2i->P2)                   */
	/*                                                                                                 */
	/*   2. Triangle is small than enclosed circle R=stepSize, therefore divide it to it's center      */
	/*                                                                                                 */
	/*                                     P3                                                          */
	/*                                    /| \                                                         */
	/*                                   / +  \                                                        */
	/*                                  / P3i  \                                                       */
	/*                                 /P1i  P2i\                                                      */
	/*                                /--+----+--\                                                     */
	/*                               /   strip    \                                                    */
	/*                              +--------------+                                                   */
	/*                            P1                P2                                                 */
	/*                                                                                                 */
	/***************************************************************************************************/
	if (MaxEnclosedRad(P1, P2, P3) > stepSize)
	{
		Eigen::Vector3d P1i = P1;
		Eigen::Vector3d P2i = P2;
		Eigen::Vector3d P3i = P3;
		InOffset(stepSize * 0.99, P1i, P2i, P3i);
		TessellateEdge(P1, P1i, P2i, P2, P1, P2, stepSize, vertices, faces);
		TessellateEdge(P2, P2i, P3i, P3, P2, P3, stepSize, vertices, faces);
		TessellateEdge(P3, P3i, P1i, P1, P3, P1, stepSize, vertices, faces);
		TessellateOutsideIn(P1i, P2i, P3i, stepSize, vertices, faces);
	}
	else
	{
		Eigen::Vector3d Pi = (P1 + P2 + P3) / 3.0;
		TessellateEdge(P1, Pi, Pi, P2, P1, P2, stepSize, vertices, faces);
		TessellateEdge(P2, Pi, Pi, P3, P2, P3, stepSize, vertices, faces);
		TessellateEdge(P3, Pi, Pi, P1, P3, P1, stepSize, vertices, faces);
	}
}

#define CHECK_INSIDE_UV_TRIG 0
bool FlexiFace::CalcRoughness(const Displacement::DisplacementSpec& disp, double tolerance)
{
	if (!&disp)
		return false;
	double imageW = disp.m_disp.GetWidth();
	double imageH = disp.m_disp.GetHeight();

	FlexiVertex::BaryCoords_t baryCoords[3] = {
		{m_UV1[0], m_UV1[1]},
		{m_UV2[0], m_UV2[1]},
		{m_UV3[0], m_UV3[1]} };

	// Map UV to get pixel value from the preprocessed displacement image.
	// "Z" value is derived from pixel value in displacement image

	double verticesUV[VERTS_PER_FACE][3];
	for (size_t idx = 0; idx < VERTS_PER_FACE; idx++)
	{
		double wrapAwareU = Displacement::ApplyWrap(baryCoords[idx][0], disp.wrapS);
		double wrapAwareV = Displacement::ApplyWrap(baryCoords[idx][1], disp.wrapT);
		verticesUV[idx][0] = ((imageW - 1) * wrapAwareU);
		verticesUV[idx][1] = ((imageH - 1) * (1 - wrapAwareV));
		verticesUV[idx][2] = disp.m_disp.Get((int)verticesUV[idx][0], (int)verticesUV[idx][1]);
	}

	// calculate triangle area in the UV world. If area is small then the 3D comparison is
	// calculated by distance point-line instead of distance point-plane
	double maxSqrDistFromPlane = 0.0;
	double sqrTolerance = tolerance * tolerance;
	bool overTolerance = false;

	double area = 0.5 * (verticesUV[0][0] * (verticesUV[1][1] - verticesUV[2][1]) +
		verticesUV[1][0] * (verticesUV[2][1] - verticesUV[0][1]) +
		verticesUV[2][0] * (verticesUV[0][1] - verticesUV[1][1]));

	// absolute area value since triangle in UV may flip direction
	if (abs(area) < 1.0)
	{
		// UV triangle is getting across edges => simulated as a 3D line

		// select the longest edge in the UV world
		Eigen::Vector3d vlen1 = { verticesUV[1][0] - verticesUV[0][0], verticesUV[1][1] - verticesUV[0][1], verticesUV[1][2] - verticesUV[0][2] };
		Eigen::Vector3d vlen2 = { verticesUV[2][0] - verticesUV[0][0], verticesUV[2][1] - verticesUV[0][1], verticesUV[2][2] - verticesUV[0][2] };
		Eigen::Vector3d vlen3 = { verticesUV[2][0] - verticesUV[1][0], verticesUV[2][1] - verticesUV[1][1], verticesUV[2][2] - verticesUV[1][2] };

		double len1 = vlen1.norm();
		double len2 = vlen2.norm();
		double len3 = vlen3.norm();
		Eigen::Vector3d head = { 0.0, 0.0, 0.0 };
		Eigen::Vector3d tail = { 0.0, 0.0, 0.0 };
		// set head and tail values according to the longest edge
		if ((len1 >= len2) && (len1 >= len3))
		{
			head[0] = verticesUV[0][0];
			head[1] = verticesUV[0][1];
			tail[0] = verticesUV[1][0];
			tail[1] = verticesUV[1][1];
		}
		else if ((len2 >= len1) && (len2 >= len3))
		{
			head[0] = verticesUV[0][0];
			head[1] = verticesUV[0][1];
			tail[0] = verticesUV[2][0];
			tail[1] = verticesUV[2][1];
		}
		else
		{
			head[0] = verticesUV[1][0];
			head[1] = verticesUV[1][1];
			tail[0] = verticesUV[2][0];
			tail[1] = verticesUV[2][1];
		}
		// set the Z value according to the displacement map
		head[2] = disp.m_disp.Get((int)head[0], (int)head[1]);
		tail[2] = disp.m_disp.Get((int)tail[0], (int)tail[1]);
		Eigen::Vector3d dir = (head - tail);
		double length = dir.norm();
		// in case of short edge - considered ad a calm
		if (length < 2)
			return false;
		dir.normalize();
		// loop along the edge and set rough condition as met. loop index is DOUBLE
		for (double s = 0.0; s < length; s++)
		{
			Eigen::Vector3d point = tail + s * dir;
			point[2] = disp.m_disp.Get((int)point[0], (int)point[1]);
			Eigen::Vector3d v = point - tail;
			double t = v.dot(dir);
			Eigen::Vector3d proj = tail + t * dir;
			maxSqrDistFromPlane = std::max(maxSqrDistFromPlane, (proj - point).squaredNorm());
			overTolerance = (maxSqrDistFromPlane > sqrTolerance);
			if (overTolerance)
				break;
		}

	}
	else
	{
		// UV triangle has a significant area
		// Calc plane equation in UV dimension
		double plane[4];
		double a1 = verticesUV[1][0] - verticesUV[0][0];
		double b1 = verticesUV[1][1] - verticesUV[0][1];
		double c1 = verticesUV[1][2] - verticesUV[0][2];
		double a2 = verticesUV[2][0] - verticesUV[0][0];
		double b2 = verticesUV[2][1] - verticesUV[0][1];
		double c2 = verticesUV[2][2] - verticesUV[0][2];
		plane[0] = b1 * c2 - b2 * c1;
		plane[1] = a2 * c1 - a1 * c2;
		plane[2] = a1 * b2 - b1 * a2;
		double len = (plane[0] * plane[0] + plane[1] * plane[1] + plane[2] * plane[2]);
		if (len == 0.0)
			return false; // Triangle cannot be divided
		len = sqrt(1.0 / len);
		plane[0] *= len;
		plane[1] *= len;
		plane[2] *= len;
		plane[3] = (-plane[0] * verticesUV[0][0] -
			plane[1] * verticesUV[0][1] -
			plane[2] * verticesUV[0][2]);
		double denSqr = plane[0] * plane[0] + plane[1] * plane[1] + plane[2] * plane[2];


		// Get triangle silhouette bounding box in image coordinates
		int bBox[4] = { (int)std::min(std::min(verticesUV[0][0],  verticesUV[1][0]), verticesUV[2][0]),
		(int)std::min(std::min(verticesUV[0][1], verticesUV[1][1]), verticesUV[2][1]) ,
		(int)std::max(std::max(verticesUV[0][0], verticesUV[1][0]), verticesUV[2][0]) ,
		(int)std::max(std::max(verticesUV[0][1], verticesUV[1][1]), verticesUV[2][1]) };
		if ((bBox[2] - bBox[0])*(bBox[3] - bBox[1]) < 1)
			return false;

		// find maximal distance from plane within in-triangle displacements
		for (double xPos = bBox[0]; xPos < bBox[2]; xPos++)
		{
			for (double yPos = bBox[1]; yPos < bBox[3]; yPos++)
			{
				// check if point is inside triangle
				bool IsInside = true;
				// the area check gives a more accurate measurement, however for performance reasons we
				// may consider the bounding rectangle pixels as "good enough" criteria.
				// use barycentric coordinates solution
#if CHECK_INSIDE_UV_TRIG
				double S = 1 / (2 * area)*
					(verticesUV[0][1] * verticesUV[2][0] - verticesUV[0][0] * verticesUV[2][1] +
					(verticesUV[2][1] - verticesUV[0][1])*xPos + (verticesUV[0][0] - verticesUV[2][0])*yPos);
				double T = 1 / (2 * area)*
					(verticesUV[0][0] * verticesUV[1][1] - verticesUV[0][1] * verticesUV[1][0] +
					(verticesUV[0][1] - verticesUV[1][1])*xPos + (verticesUV[1][0] - verticesUV[0][0])*yPos);
				double ST = 1 - S - T;
				IsInside = ((S * T > 0) && (S * ST > 0));    // triangle direction does not matter
#endif
				if (IsInside)
				{
					double zValue = disp.m_disp.Get((int)xPos, (int)yPos);
					double point[3] = { xPos, yPos, zValue };
					double numSqrt = std::abs(plane[0] * point[0] + plane[1] * point[1] + plane[2] * point[2] + plane[3]);
					double num = numSqrt * numSqrt;
					// update the max distance from plane 
					maxSqrDistFromPlane = std::max(maxSqrDistFromPlane, num / denSqr);
					// if rough then stop checking
					overTolerance = (maxSqrDistFromPlane > sqrTolerance);
				}
				if (overTolerance)
					break;
			}
			if (overTolerance)
				break;
		}
	}

	return (maxSqrDistFromPlane > sqrTolerance);
}

bool CompareApprox(const FlexiMesh& lhs, const FlexiMesh& rhs, double vertEpsilon)
{
	return CompareApprox(lhs.m_vertices, rhs.m_vertices, vertEpsilon) && (lhs.m_faces == lhs.m_faces)
		&& (lhs.m_edges == rhs.m_edges);
}

