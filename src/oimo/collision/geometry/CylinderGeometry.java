package oimo.collision.geometry;
import oimo.common.M;
import oimo.common.MathUtil;
import oimo.common.Transform;
import oimo.common.Vec3;

/**
 * A cylinder collision geometry aligned with the y-axis.
 */
public class CylinderGeometry extends ConvexGeometry {
	public double _radius;
	public double _halfHeight;

	/**
	 * Creates a cylinder collision geometry of radius `radius` and half-height `halfHeight`.
	 */
	public CylinderGeometry(double radius, double halfHeight) {
		super(GeometryType.CYLINDER);
		_radius = radius;
		_halfHeight = halfHeight;
		_updateMass();
	}

	/**
	 * Returns the radius of the cylinder.
	 */
	public double getRadius() {
		return _radius;
	}

	/**
	 * Returns the half-height of the cylinder.
	 */
	public double getHalfHeight() {
		return _halfHeight;
	}

	@Override
	public void _updateMass() {
		double r2 = _radius * _radius;
		double h2 = _halfHeight * _halfHeight * 4;
		_volume = MathUtil.PI * r2 * _halfHeight * 2;
		M.mat3_diagonal(_inertiaCoeff,
			1 / 12.0 * (3 * r2 + h2),
			1 / 2.0 * r2,
			1 / 12.0 * (3 * r2 + h2)
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
		M.vec3_abs(axis, axis);
		M.vec3_compWiseMul(axis2, axis, axis);

		double axis2x = axis2.x;
		double axis2y = axis2.y;
		double axis2z = axis2.z;

		M.vec3_set(er, MathUtil.sqrt(1 - axis2x), MathUtil.sqrt(1 - axis2y), MathUtil.sqrt(1 - axis2z));
		M.vec3_scale(er, er, _radius);

		M.vec3_scale(eh, axis, _halfHeight);

		Vec3 max =new Vec3();
		M.vec3_add(max, er, eh);

		M.vec3_sub(aabb._min, tf._position, max);
		M.vec3_add(aabb._max, tf._position, max);
	}

	@Override
	public void computeLocalSupportingVertex(Vec3 dir, Vec3 out) {
		double rx = dir.x;
		double rz = dir.z;
		double len = rx * rx + rz * rz;
		double coreRadius = _radius - _gjkMargin;
		if (coreRadius < 0) coreRadius = 0;
		double invLen = len > 0 ? coreRadius / MathUtil.sqrt(len) : 0;
		double coreHeight = _halfHeight - _gjkMargin;
		if (coreHeight < 0) coreHeight = 0;
		out.x = rx * invLen;
		out.y = dir.y > 0 ? coreHeight : -coreHeight;
		out.z = rz * invLen;
	}

	@Override
	public boolean _rayCastLocal(Vec3 begin, Vec3 end,RayCastHit result) {
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
			//ray is perpendicular to axis
			if (p1y <= -halfH || p1y >= halfH) {
				//ray doesn't hit
				return false;
			}
		} else {
			//find t on ray where it meets infinite plane of each circular base
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
		double tminxz = 0;//min t where it meets curved surface
		double tmaxxz = 1;//max t where it meets curved surface
		
		//Assume point p1+t*(p2-p1) point lying on ray that also lies on curved surface of cylinder (assuming surface infinitely long)
		//solve for this point is distance from Y axis(cylinder axis) =radius
		double a = dx * dx + dz * dz;
		double b = p1x * dx + p1z * dz;
		double c = (p1x * p1x + p1z * p1z) - _radius * _radius;
		double D = b * b - a * c;
		//solve a*t^2+2*b*t+c=0
		if (D < 0) return false;

		if (a != 0) {
			//ray is inclined to axis therefore will meet infinite curved surface somewhere at solutions of above quadratic
			double sqrtD = MathUtil.sqrt(D);
			tminxz = (-b - sqrtD) / a;
			tmaxxz = (-b + sqrtD) / a;
			if (tminxz >= 1 || tmaxxz <= 0) return false;
		} else {
			//ray is parallel to y axis
			if (c >= 0) return false; //surely t will be <0
			tminxz = 0;
			tmaxxz = 1;
		}

		double min;
		if (tmaxxz <= tminy || tmaxy <= tminxz) return false;
		if (tminxz < tminy) { //ray meets flat surface(base) before the curved surface
			min = tminy;
			if (min == 0) return false; // the ray starts from inside
			result.normal.set(0, dy > 0 ? -1 : 1, 0);
		} else {
			min = tminxz;
			if (min == 0) return false; // the ray starts from inside
			result.normal.set(p1x + dx * min, 0, p1z + dz * min).normalize();
		}
		//update point on ray as p1+t*(p2-p1) where t is forst intersection position
		result.position.set(p1x + min * dx, p1y + min * dy, p1z + min * dz);
		result.fraction = min;
		return true;
	}

}
