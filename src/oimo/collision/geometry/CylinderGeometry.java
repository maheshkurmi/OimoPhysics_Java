package oimo.collision.geometry;
import oimo.common.MathUtil;
import oimo.common.Transform;
import oimo.common.Vec3;
import oimo.common.M;

/**
 * A cylinder collision geometry aligned with the y-axis.
 */
public class CylinderGeometry extends ConvexGeometry {
	public float _radius;
	public float _halfHeight;

	/**
	 * Creates a cylinder collision geometry of radius `radius` and half-height `halfHeight`.
	 */
	public CylinderGeometry(float radius, float halfHeight) {
		super(GeometryType.CYLINDER);
		_radius = radius;
		_halfHeight = halfHeight;
		_updateMass();
	}

	/**
	 * Returns the radius of the cylinder.
	 */
	public float getRadius() {
		return _radius;
	}

	/**
	 * Returns the half-height of the cylinder.
	 */
	public float getHalfHeight() {
		return _halfHeight;
	}

	@Override
	public void _updateMass() {
		float r2 = _radius * _radius;
		float h2 = _halfHeight * _halfHeight * 4;
		_volume = MathUtil.PI * r2 * _halfHeight * 2;
		M.mat3_diagonal(_inertiaCoeff,
			1 / 12 * (3 * r2 + h2),
			1 / 2 * r2,
			1 / 12 * (3 * r2 + h2)
		);
	}

	@Override
	public void _computeAabb(Aabb aabb, Transform tf) {
		Vec3 axis=new Vec3();
		Vec3 eh=_TMP_V1;
		Vec3 er=_TMP_V2;
		M.mat3_getCol(axis, tf._rotation, 1);
		M.vec3_abs(axis, axis);
		//er is 
		er.set(MathUtil.sqrt(1 - axis.x*axis.x), MathUtil.sqrt(1 - axis.y*axis.y), MathUtil.sqrt(1 - axis.z*axis.z));
		er.scaleEq(_radius);

		M.vec3_scale(eh, axis, _halfHeight);

		
		M.vec3_add(axis, er, eh);

		M.vec3_sub(aabb._min, tf._position, axis);
		M.vec3_add(aabb._max, tf._position, axis);
	}

	@Override
	public void computeLocalSupportingVertex(Vec3 dir, Vec3 out) {
		float rx = dir.x;
		float rz = dir.z;
		float len = rx * rx + rz * rz;
		float coreRadius = _radius - _gjkMargin;
		if (coreRadius < 0) coreRadius = 0;
		float invLen = len > 0 ? coreRadius / MathUtil.sqrt(len) : 0;
		float coreHeight = _halfHeight - _gjkMargin;
		if (coreHeight < 0) coreHeight = 0;
		out.x = rx * invLen;
		out.y = dir.y > 0 ? coreHeight : -coreHeight;
		out.z = rz * invLen;
	}

	@Override
	public boolean _rayCastLocal(Vec3 begin, Vec3 end,RayCastHit result) {
		float p1x = begin.x;
		float p1y = begin.y;
		float p1z = begin.z;
		float p2x = end.x;
		float p2y = end.y;
		float p2z = end.z;
		float halfH = _halfHeight;
		float dx = p2x - p1x;
		float dy = p2y - p1y;
		float dz = p2z - p1z;

		// Y
		float tminy = 0;
		float tmaxy = 1;
		if (dy > -1e-6 && dy < 1e-6) {
			//ray is perpendicular to axis
			if (p1y <= -halfH || p1y >= halfH) {
				//ray doesn't hit
				return false;
			}
		} else {
			//find t on ray where it meets infinite plane of each circular base
			float invDy = 1 / dy;
			float t1 = (-halfH - p1y) * invDy;
			float t2 = (halfH - p1y) * invDy;
			if (t1 > t2) {
				float tmp = t1;
				t1 = t2;
				t2 = tmp;
			}
			if (t1 > 0) tminy = t1;
			if (t2 < 1) tmaxy = t2;
		}
		if (tminy >= 1 || tmaxy <= 0) return false;

		// XZ
		float tminxz = 0;//min t where it meets curved surface
		float tmaxxz = 1;//max t where it meets curved surface
		
		//Assume point p1+t*(p2-p1) point lying on ray that also lies on curved surface of cylinder (assuming surface infinitely long)
		//solve for this point is distance from Y axis(cylinder axis) =radius
		float a = dx * dx + dz * dz;
		float b = p1x * dx + p1z * dz;
		float c = (p1x * p1x + p1z * p1z) - _radius * _radius;
		float D = b * b - a * c;
		//solve a*t^2+2*b*t+c=0
		if (D < 0) return false;

		if (a != 0) {
			//ray is inclined to axis therefore will meet infinite curved surface somewhere at solutions of above quadratic
			float sqrtD = MathUtil.sqrt(D);
			tminxz = (-b - sqrtD) / a;
			tmaxxz = (-b + sqrtD) / a;
			if (tminxz >= 1 || tmaxxz <= 0) return false;
		} else {
			//ray is parallel to y axis
			if (c >= 0) return false; //surely t will be <0
			tminxz = 0;
			tmaxxz = 1;
		}

		float min;
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
