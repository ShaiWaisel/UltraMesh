#include "Face.h"
#include "Edge.h"

UltraFace::UltraFace(int idx1, int idx2, int idx3)
{
	m_vertices = { idx1,  idx2, idx3 };
};


void UltraFace::CalcPlane(Eigen::Vector3d& p1, Eigen::Vector3d& p2, Eigen::Vector3d& p3)
{
	double a1 = p2[0] - p1[0];
	double b1 = p2[1] - p1[1];
	double c1 = p2[2] - p1[2];
	double a2 = p3[0] - p1[0];
	double b2 = p3[1] - p1[1];
	double c2 = p3[2] - p1[2];
	m_plane[0] = b1 * c2 - b2 * c1;
	m_plane[1] = a2 * c1 - a1 * c2;
	m_plane[2] = a1 * b2 - b1 * a2;
	double len = sqrt(m_plane[0] * m_plane[0] + m_plane[1] * m_plane[1] + m_plane[2] * m_plane[2]);
	if (len > 0)
	{
		m_plane[0] /= len;
		m_plane[1] /= len;
		m_plane[2] /= len;
		m_plane[3] = (-m_plane[0] * p1[0] - m_plane[1] * p1[1] - m_plane[2] * p1[2]);
		Eigen::Vector3d e1 = p2 - p1;
		Eigen::Vector3d e2 = p3 - p2;
		Eigen::Vector3d e3 = p1 - p3;

		double p = (e1.norm() + e2.norm() + e3.norm())*.5;
		m_area = sqrt(p * (p - e1.norm()) * (p - e2.norm()) * (p - e3.norm()));
	}
}

void UltraFace::CalcPlane(std::vector<UltraVertex>& vertices)
{
    CalcPlane(vertices[m_vertices[0]].m_position, vertices[m_vertices[1]].m_position, vertices[m_vertices[2]].m_position);
}


double UltraFace::Dist(Eigen::Vector3d& p)
{
	double d = fabs(m_plane[0] * p[0] + m_plane[1] * p[1] + m_plane[2] * p[2] + m_plane[3]);
	double e = sqrt(m_plane[0] * m_plane[0] + m_plane[1] * m_plane[1] + m_plane[2] * m_plane[2]);
	return d / e;

}

bool UltraFace::IntersectWithRay(std::vector<UltraVertex>& vertices, Eigen::Vector3d& origin,
	Eigen::Vector3d& direction, Eigen::Vector3d& hit)
{
	const double EPSILON = 0.0000001;	// TBD: Gather epsilons from wherever are...

	Eigen::Vector3d vertex0 = vertices[m_vertices[0]].Position();
	Eigen::Vector3d vertex1 = vertices[m_vertices[1]].Position();
	Eigen::Vector3d vertex2 = vertices[m_vertices[2]].Position();
	Eigen::Vector3d edge1, edge2, h, s, q;
	double a, f, u, v;
	edge1 = vertex1 - vertex0;
	edge2 = vertex2 - vertex0;
	h = direction.cross(edge2);
	a = edge1.dot(h);
	if (a > -EPSILON && a < EPSILON)
		return false;    // This ray is parallel to this triangle.
	f = 1.0 / a;
	s = origin - vertex0;
	u = f * s.dot(h);
	if (u < -EPSILON || u > 1.0 + EPSILON)
		return false;
	q = s.cross(edge1);
	v = f * direction.dot(q);
	if (v < -EPSILON || (u + v) > 1.0 + EPSILON)
		return false;
	// At this stage we can compute t to find out where the intersection point is on the line.
	double t = f * edge2.dot(q);
	if (t > EPSILON) // ray intersection
	{
		hit = origin + direction * t;
		return true;
	}
	else // This means that there is a line intersection but not a ray intersection.
		return false;

}

bool UltraFace::RayIntersectsTriangleDist(
    const std::vector<UltraVertex>& vertices,
    const Eigen::Vector3d& rayOrigin,
    const Eigen::Vector3d& rayVector,
    Eigen::Vector3d& outIntersectionPoint)
{
    const double EPSILON = 0.0000001;	// TBD: Gather epsilons from wherever are...

    Eigen::Vector3d vertex0 = vertices[m_vertices[0]].m_position;
    Eigen::Vector3d vertex1 = vertices[m_vertices[1]].m_position;
    Eigen::Vector3d vertex2 = vertices[m_vertices[2]].m_position;
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


