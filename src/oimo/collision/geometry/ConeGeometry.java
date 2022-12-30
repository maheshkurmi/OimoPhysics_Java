package oimo.collision.geometry;
import oimo.common.MathUtil;
import oimo.common.Transform;
import oimo.common.Vec3;
import oimo.common.M;

/**
 * A cone collision geometry aligned with the y-axis.
 */
public class ConeGeometry extends ConvexGeometry {
	public float _radius;
	public float _halfHeight;

	float sinTheta;
	float cosTheta;

	private static Vec3 _TMP_V3=new Vec3();
	/**
	 * Creates a cone collision geometry of radius `radius` and half-height `halfHeight`.
	 */
	public ConeGeometry(float radius, float halfHeight) {
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
	public float getRadius() {
		return _radius;
	}

	/**
	 * Returns the half-height of the cone.
	 */
	public float getHalfHeight() {
		return _halfHeight;
	}

	@Override 
	public void _updateMass() {
		float r2 = _radius * _radius;
		float h2 = _halfHeight * _halfHeight * 4;
		_volume = MathUtil.PI * r2 * _halfHeight * 2 / 3;
		M.mat3_diagonal(_inertiaCoeff,
			1 / 20 * (3 * r2 + 2 * h2),
			3 / 10 * r2,
			1 / 20 * (3 * r2 + 2 * h2)
		);
	}

	@Override 
	public void _computeAabb(Aabb aabb, Transform tf) {
		Vec3 axis=_TMP_V1;
		Vec3 eh=_TMP_V2;
		Vec3 er=_TMP_V3;
		M.mat3_getCol(axis, tf._rotation, 1);
		//extract radius vector
		M.vec3_set(er, MathUtil.sqrt(1 - axis.x*axis.x), MathUtil.sqrt(1 - axis.y*axis.y), MathUtil.sqrt(1 - axis.z*axis.z));
		M.vec3_scale(er, er, _radius);
		M.vec3_scale(eh, axis, _halfHeight);

		Vec3 rmin=new Vec3(-eh.x-er.x,-eh.y-er.y,-eh.z-er.z); // -(signed projected axis) - (projected radius)
		Vec3 rmax=new Vec3(-eh.x+er.x,-eh.y+er.y,-eh.z+er.z); // -(signed projected axis) + (projected radius)
		
		Vec3 max=_TMP_V1;
		M.vec3_max(max, rmin, rmax);
		M.vec3_max(max, max, eh);
		M.vec3_add(aabb._max, tf._position, max);
		
		Vec3 min=_TMP_V1;
		M.vec3_min(min, rmin, rmax);
		M.vec3_min(min, min, eh);
		M.vec3_add(aabb._min, tf._position, min);
		
	
	}

	@Override 
	public void computeLocalSupportingVertex(Vec3 dir, Vec3 out) {
		float dx = dir.x;
		float dy = dir.y;
		float dz = dir.z;
		if (dy > 0 && dy * dy > sinTheta * sinTheta * (dx * dx + dy * dy + dz * dz)) {
			out.set(0, _halfHeight - _gjkMargin / sinTheta, 0);
			if (out.y < 0) out.y = 0;
			return;
		}
		float rx = dir.x;
		float rz = dir.z;
		float len = rx * rx + rz * rz;
		float height = 2 * _halfHeight;
		float coreRadius = (height - _gjkMargin) / height * _radius - _gjkMargin / cosTheta;
		if (coreRadius < 0) coreRadius = 0;
		float invLen = len > 0 ? coreRadius / MathUtil.sqrt(len) : 0;
		float coreHalfHeight = _halfHeight - _gjkMargin;
		if (coreHalfHeight < 0) coreHalfHeight = 0;
		out.x = rx * invLen;
		out.y = -coreHalfHeight;
		out.z = rz * invLen;
	}

	@Override 
	public boolean _rayCastLocal(Vec3 begin, Vec3 end,RayCastHit  result) {
		float p1x = begin.x;
		float p1y =	begin.y;
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
			if (p1y <= -halfH || p1y >= halfH) {
				return false;
			}
		} else {
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
		float tminxz = 0;
		float tmaxxz = 0;

		p1y -= halfH; // translate so that the new origin be (0, -halfH, 0)

		float cos2 = cosTheta * cosTheta;
		float a = cos2 * (dx * dx + dy * dy + dz * dz) - dy * dy;
		float b = cos2 * (p1x * dx + p1y * dy + p1z * dz) - p1y * dy;
		float c = cos2 * (p1x * p1x + p1y * p1y + p1z * p1z) - p1y * p1y;
		float D = b * b - a * c;
		if (a != 0) {
			if (D < 0) return false;
			float sqrtD = MathUtil.sqrt(D);
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
			float t = -c / (2 * b);
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

		float min;
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
