package oimo.collision.geometry;
import oimo.common.MathUtil;
import oimo.common.Transform;
import oimo.common.Vec3;
import oimo.common.M;

/**
 * A cone collision geometry aligned with the y-axis.
 */
public class ConeGeometry extends ConvexGeometry {
	public double _radius;
	public double _halfHeight;

	double sinTheta;
	double cosTheta;

	/**
	 * Creates a cone collision geometry of radius `radius` and half-height `halfHeight`.
	 * Center located at symmetry axis and at midpoint of cone
	 */
	public ConeGeometry(double radius, double halfHeight) {
		super(GeometryType.CONE);
		_radius = radius;
		_halfHeight = halfHeight;
		sinTheta = radius / MathUtil.sqrt(radius * radius + 4 * halfHeight * halfHeight);
		cosTheta = 2 * halfHeight / MathUtil.sqrt(radius * radius + 4 * halfHeight * halfHeight);
		_updateMass();
		
	}

	/**
	 * Returns the radius of the cone.
	 */
	public double getRadius() {
		return _radius;
	}

	/**
	 * Returns the half-height of the cone.
	 */
	public double getHalfHeight() {
		return _halfHeight;
	}

	@Override 
	public void _updateMass() {
		double r2 = _radius * _radius;
		double h2 = _halfHeight * _halfHeight * 4;
		_volume = MathUtil.PI * r2 * _halfHeight * 2 / 3.0;
		//MOI about midpoint of cone
		M.mat3_diagonal(_inertiaCoeff,
			1 / 20.0 * (3 * r2 + 2 * h2),
			3 / 10.0 * r2,
			1 / 20.0 * (3 * r2 + 2 * h2)
		);
		//System.out.println(_inertiaCoeff);
	}

	@Override 
	public void _computeAabb(Aabb aabb, Transform tf) {
		Vec3 axis=new Vec3();
		Vec3 axis2=new Vec3();
		Vec3 eh=new Vec3();
		Vec3 er=new Vec3();
		M.mat3_getCol(axis, tf._rotation, 1);
	//	axis.normalize();
		M.vec3_compWiseMul(axis2, axis, axis);

		double axis2x = axis2.x;
		double axis2y = axis2.y;
		double axis2z = axis2.z;

		M.vec3_set(er, MathUtil.sqrt(1 - axis2x), MathUtil.sqrt(1 - axis2y), MathUtil.sqrt(1 - axis2z));
		M.vec3_scale(er, er, _radius);

		M.vec3_scale(eh, axis, _halfHeight);

		Vec3 rmin=new Vec3(); // -(signed projected axis) - (projected radius)
		Vec3 rmax=new Vec3(); // -(signed projected axis) + (projected radius)
		M.vec3_negate(rmin, eh);
		M.vec3_sub(rmin, rmin, er);
		M.vec3_negate(rmax, eh);
		M.vec3_add(rmax, rmax, er);

		Vec3 max=new Vec3(); 
		Vec3 min=new Vec3(); 
		M.vec3_max(max, rmin, rmax);
		M.vec3_max(max, max, eh);
		M.vec3_min(min, rmin, rmax);
		M.vec3_min(min, min, eh);

		M.vec3_add(aabb._min, tf._position, min);
		M.vec3_add(aabb._max, tf._position, max);
	
	}

	@Override 
	public void computeLocalSupportingVertex(Vec3 dir, Vec3 out) {
		double dx = dir.x;
		double dy = dir.y;
		double dz = dir.z;
		if (dy > 0 && dy * dy > sinTheta * sinTheta * (dx * dx + dy * dy + dz * dz)) {
			out.set(0, _halfHeight - _gjkMargin / sinTheta, 0);
			if (out.y < 0) out.y = 0;
			return;
		}
		double rx = dir.x;
		double rz = dir.z;
		double len = rx * rx + rz * rz;
		double height = 2 * _halfHeight;
		double coreRadius = (height - _gjkMargin) / height * _radius - _gjkMargin / cosTheta;
		if (coreRadius < 0) coreRadius = 0;
		double invLen = len > 0 ? coreRadius / MathUtil.sqrt(len) : 0;
		double coreHalfHeight = _halfHeight - _gjkMargin;
		if (coreHalfHeight < 0) coreHalfHeight = 0;
		out.x = rx * invLen;
		out.y = -coreHalfHeight;
		out.z = rz * invLen;
	}

	@Override 
	public boolean _rayCastLocal(Vec3 begin, Vec3 end,RayCastHit  result) {
		double p1x = begin.x;
		double p1y = begin.y;
		double p1z = begin.z;
		double p2x = end.x;
		double p2y = end.y;
		double p2z = end.z;
		double halfH = _halfHeight;
		double dx = p2x - p1x;
		double dy = p2y - p1y;
		double dz = p2z - p1z;

		// Y
		double tminy = 0;
		double tmaxy = 1;
		if (dy > -1e-6 && dy < 1e-6) {
			if (p1y <= -halfH || p1y >= halfH) {
				return false;
			}
		} else {
			double invDy = 1 / dy;
			double t1 = (-halfH - p1y) * invDy;
			double t2 = (halfH - p1y) * invDy;
			if (t1 > t2) {
				double tmp = t1;
				t1 = t2;
				t2 = tmp;
			}
			if (t1 > 0) tminy = t1;
			if (t2 < 1) tmaxy = t2;
		}
		if (tminy >= 1 || tmaxy <= 0) return false;

		// XZ
		double tminxz = 0;
		double tmaxxz = 0;

		p1y -= halfH; // translate so that the new origin be (0, -halfH, 0)

		double cos2 = cosTheta * cosTheta;
		double a = cos2 * (dx * dx + dy * dy + dz * dz) - dy * dy;
		double b = cos2 * (p1x * dx + p1y * dy + p1z * dz) - p1y * dy;
		double c = cos2 * (p1x * p1x + p1y * p1y + p1z * p1z) - p1y * p1y;
		double D = b * b - a * c;
		if (a != 0) {
			if (D < 0) return false;
			double sqrtD = MathUtil.sqrt(D);
			if (a < 0) {
				// ((-inf, t1) union (t2, +inf)) join (0, 1)
				if (dy > 0) {
					// (0, t1)
					tminxz = 0;
					tmaxxz = (-b + sqrtD) / a;
					if (tmaxxz <= 0) return false;
				} else {
					// (t2, 1)
					tminxz = (-b - sqrtD) / a;
					tmaxxz = 1;
					if (tminxz >= 1) return false;
				}
			} else {
				// (t1, t2) join (0, 1)
				tminxz = (-b - sqrtD) / a;
				tmaxxz = (-b + sqrtD) / a;
				if (tminxz >= 1 || tmaxxz <= 0) return false;
			}
		} else {
			double t = -c / (2 * b);
			if (b > 0) {
				// (0, t)
				tminxz = 0;
				tmaxxz = t;
				if (t <= 0) return false;
			} else {
				// (t, 1)
				tminxz = t;
				tmaxxz = 1;
				if (t >= 1) return false;
			}
		}

		p1y += halfH; // revert translation

		double min;
		if (tmaxxz <= tminy || tmaxy <= tminxz) return false;
		if (tminxz < tminy) {
			min = tminy;
			if (min == 0) return false; // the ray starts from inside
			result.normal.set(0, dy > 0 ? -1 : 1, 0);
		} else {
			min = tminxz;
			if (min == 0) return false; // the ray starts from inside
			result.normal.set(p1x + dx * min, 0, p1z + dz * min).normalize().scaleEq(cosTheta);
			result.normal.y += sinTheta;
		}

		result.position.set(p1x + min * dx, p1y + min * dy, p1z + min * dz);
		result.fraction = min;
		return true;
	}

}
