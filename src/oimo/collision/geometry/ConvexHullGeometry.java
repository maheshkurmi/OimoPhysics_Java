package oimo.collision.geometry;

import oimo.common.M;
import oimo.common.Transform;
import oimo.common.Vec3;

/**
 * A convex hull collision geometry. A convex hull of the vertices is the
 * smallest convex polyhedron which contains all vertices.
 */
public class ConvexHullGeometry extends ConvexGeometry {
	public Vec3[] _vertices;
	public Vec3[] _tmpVertices; // for internal use
	public int _numVertices;

	/**
	 * Creates a convex hull collision geometry of the vertices `vertices`.
	 */
	public ConvexHullGeometry(Vec3[] vertices) {
		super(GeometryType.CONVEX_HULL);
		_numVertices = vertices.length;
		_vertices = new Vec3[_numVertices];
		_tmpVertices = new Vec3[_numVertices];
		for (int i = 0; i < _numVertices; i++) {
			_vertices[i] = vertices[i];
			_tmpVertices[i] = new Vec3();
		}
		_useGjkRayCast = true;
		_updateMass();
	}

	/**
	 * Returns the vertices of the convex hull.
	 */
	public Vec3[] getVertices() {
		return _vertices;
	}

	@Override
	public void _updateMass() {
		_volume = 1;
		M.mat3_diagonal(_inertiaCoeff, 1, 1, 1);

		float minx = _vertices[0].x;
		float miny = _vertices[0].y;
		float minz = _vertices[0].z;
		float maxx = _vertices[0].x;
		float maxy = _vertices[0].y;
		float maxz = _vertices[0].z;

		for (int i = 0; i < _numVertices; i++) {
			float vx = _vertices[i].x;
			float vy = _vertices[i].y;
			float vz = _vertices[i].z;
			if (vx < minx)
				minx = vx;
			else if (vx > maxx)
				maxx = vx;
			if (vy < miny)
				miny = vy;
			else if (vy > maxy)
				maxy = vy;
			if (vz < minz)
				minz = vz;
			else if (vz > maxz)
				maxz = vz;
		}

		float sizex = maxx - minx;
		float sizey = maxy - miny;
		float sizez = maxz - minz;
		_volume = sizex * sizey * sizez;
		float diffCog = ((minx + maxx) * (minx + maxx) + (miny + maxy) * (miny + maxy) + (minz + maxz) * (minz + maxz))
				* 0.25f;

		// (size / 2) ^ 2
		sizex = sizex * sizex * 0.25f;
		sizey = sizey * sizey * 0.25f;
		sizez = sizez * sizez * 0.25f;

		M.mat3_diagonal(_inertiaCoeff, 1 / 3 * (sizey + sizez) + diffCog, 1 / 3 * (sizez + sizex) + diffCog,
				1 / 3 * (sizex + sizey) + diffCog);
	}

	@Override
	public void _computeAabb(Aabb aabb, Transform tf) {
		Vec3 min = _TMP_V1;
		Vec3 max = _TMP_V2;
		var worldV = new Vec3();

		Vec3 localV = _vertices[0];
		M.vec3_mulMat3(worldV, localV, tf._rotation);
		M.vec3_add(worldV, worldV, tf._position);

		min.copyFrom(worldV);
		max.copyFrom(worldV);

		for (int i = 0; i < _numVertices; i++) {
			localV = _vertices[i];
			M.vec3_mulMat3(worldV, localV, tf._rotation);
			M.vec3_add(worldV, worldV, tf._position);
			M.vec3_min(min, min, worldV);
			M.vec3_max(max, max, worldV);
		}

		min.sub3Eq(_gjkMargin, _gjkMargin, _gjkMargin);
		max.add3Eq(_gjkMargin, _gjkMargin, _gjkMargin);

		aabb._min.copyFrom(min);
		aabb._min.copyFrom(max);

	}

	@Override
	public void computeLocalSupportingVertex(Vec3 dir, Vec3 out) {
		float maxDot = _vertices[0].dot(dir);
		int maxIndex = 0;
		for (int i = 0; i < _numVertices; i++) {
			float dot = _vertices[i].dot(dir);
			if (dot > maxDot) {
				maxDot = dot;
				maxIndex = i;
			}
		}
		out.copyFrom(_vertices[maxIndex]);
	}
	
	@Override
	public boolean _rayCastLocal(Vec3 begin, Vec3 end, RayCastHit result) {
		//Do nothing as we will be using GJK Raycast
		return false;
	}

}