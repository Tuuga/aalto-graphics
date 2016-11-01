#include "objects.hpp"

#include "hit.hpp"
#include "VecUtils.h"

#include <cassert>

using namespace std;
using namespace FW;

Object3D* Group::operator[](int i) const {
	assert(i >= 0 && i < size());
	return objects_[i].get();
}

void Group::insert(Object3D* o) {
	assert(o);
	objects_.emplace_back(o);
}

bool Group::intersect(const Ray& r, Hit& h, float tmin) const {
	// We intersect the ray with each object contained in the group.
	bool intersected = false;
	for (int i = 0; i < size(); ++i) {
		Object3D* o = objects_[i].get();
		assert(o != nullptr);
		assert(h.t >= tmin);
		bool tmp = o->intersect(r, h, tmin);
		assert(h.t >= tmin);
		if (tmp)
			intersected = true;
	}
	assert(h.t >= tmin);
	return intersected;
}

bool Box::intersect(const Ray& r, Hit& h, float tmin) const {
// YOUR CODE HERE (EXTRA)
// Intersect the box with the ray!

  return false;
}

bool Plane::intersect( const Ray& r, Hit& h, float tmin ) const {
	// YOUR CODE HERE (R5)
	// Intersect the ray with the plane.
	// Pay attention to respecting tmin and h.t!
	// Equation for a plane:

	// ax + by + cz = d;

	// normal . p - d = 0
	//cout << normal().dot(Vec3f(0, offset(), 0)) - d << endl;

	// (plug in ray)
	// origin + direction * t = p(t)

	// origin . normal + t * direction . normal = d;
	
	if (FW::abs (normal().dot(r.direction)) == 0) {
		return false;
	}

	// t = (d - origin . normal) / (direction . normal);
	auto t = (offset() - r.origin.dot(normal())) / (r.direction.dot(normal()));

	if (h.t < t || t < tmin) {
		return false;
	}

	h.set(t, material_, normal());
	return true;
}

Transform::Transform(const Mat4f& m, Object3D* o) :
	matrix_(m),
	object_(o)
{
	assert(o != nullptr);
	inverse_ = matrix_.inverted();
	inverse_transpose_ = inverse_.transposed();
}

bool Transform::intersect(const Ray& r, Hit& h, float tmin) const {
	// YOUR CODE HERE (EXTRA)
	// Transform the ray to the coordinate system of the object inside,
	// intersect, then transform the normal back. If you don't renormalize
	// the ray direction, you can just keep the t value and do not need to
	// recompute it!
	// Remember how points, directions, and normals are transformed differently!

	return false; 
}

bool Sphere::intersect( const Ray& r, Hit& h, float tmin ) const {
	// Note that the sphere is not necessarily centered at the origin.
	
	Vec3f tmp = center_ - r.origin;
	Vec3f dir = r.direction;
	float A = dot(dir, dir);
	float B = - 2 * dot(dir, tmp);
	float C = dot(tmp, tmp) - sqr(radius_);
	float radical = B*B - 4*A*C;
	if (radical < 0)
		return false;

	radical = sqrtf(radical);
	float t_m = ( -B - radical ) / ( 2 * A );
	float t_p = ( -B + radical ) / ( 2 * A );
	Vec3f pt_m = r.pointAtParameter( t_m );
	Vec3f pt_p = r.pointAtParameter( t_p );

	assert(r.direction.length() > 0.9f);

	bool flag = t_m <= t_p;
	if (!flag) {
		::printf( "sphere ts: %f %f %f\n", tmin, t_m, t_p );
		return false;
	}
	assert( t_m <= t_p );

	// choose the closest hit in front of tmin
	float t = (t_m < tmin) ? t_p : t_m;

	if (h.t > t  && t > tmin) {
		Vec3f normal = r.pointAtParameter(t);
		normal -= center_;
		normal.normalize();
		h.set(t, this->material(), normal);
		return true;
	}
	return false;
} 

Triangle::Triangle(const Vec3f& a, const Vec3f& b, const Vec3f& c,
	Material *m, const Vec2f& ta, const Vec2f& tb, const Vec2f& tc, bool load_mesh) :
	Object3D(m)
{
	vertices_[0] = a;
	vertices_[1] = b;
	vertices_[2] = c;
	texcoords_[0] = ta;
	texcoords_[1] = tb;
	texcoords_[2] = tc;

	if (load_mesh) {
		preview_mesh.reset((FW::Mesh<FW::VertexPNT>*)FW::importMesh("preview_assets/tri.obj"));
		set_preview_materials();
	}
}

bool Triangle::intersect( const Ray& r, Hit& h, float tmin ) const {
	// YOUR CODE HERE (R6)
	// Intersect the triangle with the ray!
	// Again, pay attention to respecting tmin and h.t!

	// compute plane's normal
	auto v0 = vertices_[2];
	auto v1 = vertices_[1];
	auto v2 = vertices_[0];

	Vec3f v0v1 = v1 - v0;
	Vec3f v0v2 = v2 - v0;
	// no need to normalize
	Vec3f N = v0v1.cross(v0v2); // N 
	float denom = N.dot(N);

	// Step 1: finding P

	// check if ray and plane are parallel ?
	float NdotRayDirection = N.dot(r.direction);
	if (fabs(NdotRayDirection) < FLT_EPSILON) // almost 0 
		return false; // they are parallel so they don't intersect ! 

					  // compute d parameter using equation 2
	float d = N.dot(v0);

	// compute t (equation 3)
	auto t = (N.dot(r.origin) + d) / NdotRayDirection;
	// check if the triangle is in behind the ray
	if (t < tmin || t > h.t) return false; // the triangle is behind 

							 // compute the intersection point using equation 1
	Vec3f P = r.origin + t * r.direction;

	// Step 2: inside-outside test
	Vec3f C; // vector perpendicular to triangle's plane 

	// edge 0
	Vec3f edge0 = v1 - v0;
	Vec3f vp0 = P - v0;
	C = edge0.cross(vp0);
	if (N.dot(C) < 0) return false; // P is on the right side 

	// edge 1
	Vec3f edge1 = v2 - v1;
	Vec3f vp1 = P - v1;
	C = edge1.cross(vp1);
	float u, v;
	if ((u = N.dot(C)) < 0)  return false; // P is on the right side 

	// edge 2
	Vec3f edge2 = v0 - v2;
	Vec3f vp2 = P - v2;
	C = edge2.cross(vp2);
	if ((v = N.dot(C)) < 0) return false; // P is on the right side;

	u /= denom;
	v /= denom;

	//h.normal = N.normalized();
	h.normal = -r.direction.normalized();
	h.t = t;
	h.material = material_;


	return true; // this ray hits the triangle 
}

const Vec3f& Triangle::vertex(int i) const {
	assert(i >= 0 && i < 3);
	return vertices_[i];
}


