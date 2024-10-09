package oimo.dynamics.constraint.contact;
import oimo.common.M;
import oimo.common.MathUtil;
import oimo.common.Setting;
import oimo.common.Transform;
import oimo.common.Vec3;

/**
 * A contact manifold holds collision data of a pair of shapes.
 */
public class Manifold {
	public Vec3 _normal;
	public Vec3 _tangent;
	public Vec3 _binormal;
	public int _numPoints;
	public ManifoldPoint[] _points;

	public Manifold() {
		_normal=new Vec3();//M.vec3_zero(_normal);
		_tangent=new Vec3();//M.vec3_zero(_tangent);
		_binormal=new Vec3();//M.vec3_zero(_binormal);
		_numPoints = 0;
		_points = new ManifoldPoint[Setting.maxManifoldPoints];
		for (int i =0;i<Setting.maxManifoldPoints;i++) {
			_points[i] = new ManifoldPoint();
		}
	}

	// --- internal ---

	public void _clear() {
		for (int i =0;i<_numPoints;i++) {
			_points[i]._clear();
		}
		_numPoints = 0;
	}

	public void _buildBasis(Vec3 normal) {
		M.vec3_fromVec3(_normal, normal);
		double nx = normal.x;
		double ny = normal.y;
		double nz = normal.z;
		double nx2 = nx * nx;
		double ny2 = ny * ny;
		double nz2 = nz * nz;
		double tx;
		double ty;
		double tz;
		double bx;
		double by;
		double bz;
		if (nx2 < ny2) {
			if (nx2 < nz2) {
				// a < b, c
				double invL = 1 / MathUtil.sqrt(ny2 + nz2);
				tx = 0;
				ty = -nz * invL;
				tz = ny * invL;
				bx = ny * tz - nz * ty;
				by = -nx * tz;
				bz = nx * ty;
			} else {
				// c < a < b
				double invL = 1 / MathUtil.sqrt(nx2 + ny2);
				tx = -ny * invL;
				ty = nx * invL;
				tz = 0;
				bx = -nz * ty;
				by = nz * tx;
				bz = nx * ty - ny * tx;
			}
		} else {
			if (ny2 < nz2) {
				// b < a, c
				double invL = 1 / MathUtil.sqrt(nx2 + nz2);
				tx = nz * invL;
				ty = 0;
				tz = -nx * invL;
				bx = ny * tz;
				by = nz * tx - nx * tz;
				bz = -ny * tx;
			} else {
				// c < b < a
				double invL = 1 / MathUtil.sqrt(nx2 + ny2);
				tx = -ny * invL;
				ty = nx * invL;
				tz = 0;
				bx = -nz * ty;
				by = nz * tx;
				bz = nx * ty - ny * tx;
			}
		}
		
		
		
		M.vec3_set(_tangent, tx, ty, tz);
		M.vec3_set(_binormal, bx, by, bz);
	}

	public void _updateDepthsAndPositions(Transform tf1, Transform tf2) {
		for (int i=0;i<_numPoints;i++) {
			ManifoldPoint p = _points[i];
			M.vec3_mulMat3(p._relPos1, p._localPos1, tf1._rotation);
			M.vec3_mulMat3(p._relPos2, p._localPos2, tf2._rotation);
			M.vec3_add(p._pos1, p._relPos1, tf1._position);
			M.vec3_add(p._pos2, p._relPos2, tf2._position);

			double diffX=p._pos2.x- p._pos1.x;
			double diffY=p._pos2.y- p._pos1.y;
			double diffZ=p._pos2.z- p._pos1.z;
			p._depth = _normal.x*diffX+_normal.y*diffY+_normal.z*diffZ;
			//M.vec3_sub(diff, p._pos1, p._pos2);
			//var dotN:double = M.vec3_dot(diff, _normal);
			//p._depth = -dotN;
		}
	}

	// --- public ---

	/**
	 * Returns the normal vector of the contact manifold. The normal vector has unit
	 * length and is perpendicular to the contact plane.
	 */
	public Vec3 getNormal() {
		Vec3 v = new Vec3();
		M.vec3_toVec3(v, _normal);
		return v;
	}

	/**
	 * Sets `normal` to the normal vector of the contact manifold. The normal vector has
	 * unit length and is perpendicular to the contact plane.
	 *
	 * This does not create a new instance of `Vec3`.
	 */
	public void getNormalTo(Vec3 normal) {
		M.vec3_toVec3(normal, _normal);
	}

	/**
	 * Returns the tangent vector of the contact manifold. The tangent vector has unit
	 * length and is perpendicular to the normal vector.
	 */
	public Vec3 getTangent() {
		Vec3 v = new Vec3();
		M.vec3_toVec3(v, _tangent);
		return v;
	}

	/**
	 * Sets `tangent` to the tangent vector of the contact manifold. The tangent vector has
	 * unit length and is perpendicular to the normal vector.
	 *
	 * This does not create a new instance of `Vec3`.
	 */
	public void getTangentTo(Vec3 tangent) {
		M.vec3_toVec3(tangent, _tangent);
	}

	/**
	 * Returns the binormal vector of the contact manifold. The binormal vector has unit
	 * length and is perpendicular to both the normal and the tangent vector.
	 */
	public Vec3 getBinormal() {
		Vec3 v = new Vec3();
		M.vec3_toVec3(v, _binormal);
		return v;
	}

	/**
	 * Sets `binormal` to the binormal vector of the contact manifold. The binormal vector has
	 * unit length and is perpendicular to both the normal and the tangent vector.
	 *
	 * This does not create a new instance of `Vec3`.
	 */
	public void getBinormalTo(Vec3 binormal) {
		M.vec3_toVec3(binormal, _binormal);
	}

	/**
	 * Returns the manifold point vector of the contact manifold. Note that **only the first
	 * `Manifold.getNumPoints` elements of the vector are in use**, and the manifold points may
	 * be disabled (see `ManifoldPoint.isEnabled`).
	 */
	public ManifoldPoint[] getPoints() {
		return _points;
	}

	/**
	 * Returns the number of existing manifold points.
	 */
	public int getNumPoints() {
		return _numPoints;
	}

}
