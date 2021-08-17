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

bool UltraFace::IsPointInside(const std::vector<UltraVertex>& vertices, const Eigen::Vector3d& point, Eigen::Vector3d& projectedPoint)
{
    // https://math.stackexchange.com/questions/544946/determine-if-projection-of-3d-point-onto-plane-is-within-a-triangle
    Eigen::Vector3d u = (vertices[m_vertices[1]].m_position - vertices[m_vertices[0]].m_position);
    Eigen::Vector3d v = (vertices[m_vertices[2]].m_position - vertices[m_vertices[0]].m_position);
    Eigen::Vector3d n = m_plane.block<3, 1>(0, 0);
    n = u.cross(v);
    Eigen::Vector3d w = (point - vertices[m_vertices[0]].m_position);
    double gamma = ((u.cross(w)).dot(n)) / n.dot(n);
    double beta = ((w.cross(v)).dot(n)) / n.dot(n);
    double alpha = 1.0 - gamma - beta;
    projectedPoint = alpha * vertices[m_vertices[0]].m_position + beta * vertices[m_vertices[1]].m_position + gamma * vertices[m_vertices[2]].m_position;
    return (alpha >= 0.0) && (alpha <= 1.0) && (beta >= 0.0) && (beta <= 1.0) && (gamma >= 0.0) && (gamma <= 1.0);
}

double UltraFace::DistPointPlane(const Eigen::Vector3d& point)
{
    return abs(m_plane[0] * point[0] + m_plane[1] * point[1] + m_plane[2] * point[2] + m_plane[3]);
}

double UltraFace::ClampDistPoint(const std::vector<UltraVertex>& vertices, const Eigen::Vector3d& point, Eigen::Vector3d& closestPoint)
{
    if (IsPointInside(vertices, point, closestPoint))
    {
        return DistPointPlane(point);
    }
    double dist1 = (vertices[m_vertices[0]].m_position - point).norm();
    double dist2 = (vertices[m_vertices[1]].m_position - point).norm();
    double dist3 = (vertices[m_vertices[2]].m_position - point).norm();
    if (dist1 < dist2)
    {
        if (dist1 < dist3)
        {
            closestPoint = vertices[m_vertices[0]].m_position;
            return dist1;
        }
        else
        {
            closestPoint = vertices[m_vertices[2]].m_position;
            return dist3;
        }
    }
    else
    {
        if (dist2 < dist3)
        {
            closestPoint = vertices[m_vertices[1]].m_position;
            return dist2;
        }
        else
        {
            closestPoint = vertices[m_vertices[2]].m_position;
            return dist3;
        }
    }
}

bool UltraFace::MaxDistToSkeleton(const std::vector<UltraVertex>& vertices, Eigen::Vector3d point, const Eigen::Vector3d& direction, double& distance)
{
#define MATLAB_DEBUG 0

    double dir = m_plane[0] * point[0] + m_plane[1] * point[1] + m_plane[2] * point[2] + m_plane[3];
    double K1 = m_plane[0] * direction[0] + m_plane[1] * direction[1] + m_plane[2] * direction[2];
    double K2 = m_plane[0] * point[0] + m_plane[1] * point[1] + m_plane[2] * point[2] + m_plane[3];
    double t = K2 / (1 - K1);
    if (t < 0.0)
        t = -K2 / (1 + K1);
    if (t <= 1.0E-3)
        return false;
    
    Eigen::Vector3d proposed = { point[0] + direction[0] * t, point[1] + direction[1] * t, point[2] + direction[2] * t };
    Eigen::Vector3d newVec = (proposed - point);
    newVec.normalize();
    if (direction.dot(newVec) < 0.0)
        return false;
    Eigen::Vector3d projected = { 0.0, 0.0, 0.0 };
    bool isInside = (IsPointInside(vertices, proposed, projected));
#if MATLAB_DEBUG
    printf("\nclose all; clear all; \n");
    printf("V1 = [%f %f %f];\n", vertices[m_vertices[0]].m_position[0], vertices[m_vertices[0]].m_position[1], vertices[m_vertices[0]].m_position[2]);
    printf("V2 = [%f %f %f];\n", vertices[m_vertices[1]].m_position[0], vertices[m_vertices[1]].m_position[1], vertices[m_vertices[1]].m_position[2]);
    printf("V3 = [%f %f %f];\n", vertices[m_vertices[2]].m_position[0], vertices[m_vertices[2]].m_position[1], vertices[m_vertices[2]].m_position[2]);
    printf("V=[V1;V2;V3;V1];\n");
    printf("P=[%f %f %f];\n", point[0], point[1], point[2]);
    printf("D=[%f %f %f];\n", direction[0], direction[1], direction[2]);
    printf("t=%f;\n", t);
    printf("Projected=[%f %f %f];\n", projected[0], projected[1], projected[2]);
    printf("Proposed=[%f %f %f];\n", proposed[0], proposed[1], proposed[2]);
    printf("figure, axis equal;hold on;\n");
    printf("plot3(V(:,1), V(:,2), V(:,3));\n");
    printf("scatter3(P(1), P(2), P(3),'o','r', 'filled');\n");
    printf("plot3([P(1) P(1)+D(1)*5], [P(2) P(2)+D(2)*5], [P(3) P(3)+D(3)*5],'g');\n");
    printf("scatter3(Proposed(1), Proposed(2), Proposed(3),'*','b');\n");
    printf("scatter3(Projected(1), Projected(2), Projected(3),'+','b');\n");
#endif
    //point = proposed;
    if (isInside)
    {
         distance = t;
    }

    return isInside;
}





