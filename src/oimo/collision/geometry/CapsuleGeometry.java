package oimo.collision.geometry;

import oimo.common.M;
import oimo.common.MathUtil;
import oimo.common.Transform;
import oimo.common.Vec3;

/**
 * A capsule collision geometry aligned with the y-axis.
 */
public class CapsuleGeometry extends ConvexGeometry {
	public double _radius;
	public double _halfHeight;

	/**
	 * Creates a capsule collision geometry of radius `radius` and half-height `halfHeight`.
	 */
	public CapsuleGeometry(double radius, double halfHeight) {
		super(GeometryType.CAPSULE);
		_radius = radius;
		_halfHeight = halfHeight;
		_gjkMargin = _radius;
		_updateMass();
	}

	/**
	 * Returns the radius of the capsule.
	 */
	public double getRadius() {
		return _radius;
	}

	/**
	 * Returns the half-height of the capsule.
	 */
	public double getHalfHeight() {
		return _halfHeight;
	}

	@Override 
	public void _updateMass() {
		double r2 = _radius * _radius;
		double hh2 = _halfHeight * _halfHeight;

		double cylinderVolume = MathUtil.TWO_PI * r2 * _halfHeight;
		double sphereVolume = MathUtil.PI * r2 * _radius * 4 / 3.0;
		_volume = cylinderVolume + sphereVolume;

		double invVolume = _volume == 0 ? 0 : 1 / _volume;

		double inertiaY = invVolume * (
			cylinderVolume * r2 * 0.5f +
			sphereVolume * r2 * 0.4f
		);

		double inertiaXZ = invVolume * (
			cylinderVolume * (r2 * 0.25f + hh2 / 3) +
			sphereVolume * (r2 * 0.4f + _halfHeight * _radius * 0.75f + hh2)
		);

		M.mat3_diagonal(_inertiaCoeff, inertiaXZ, inertiaY, inertiaXZ);
	}

	@Override 
	public void _computeAabb(Aabb aabb, Transform tf) {
		Vec3 radVec=new Vec3();
		radVec.set( _radius, _radius, _radius);
		Vec3 axis=new Vec3();
		M.mat3_getCol(axis, tf._rotation, 1);
		M.vec3_abs(axis, axis);
		M.vec3_scale(axis, axis, _halfHeight);

		M.vec3_add(radVec, radVec, axis);
		M.vec3_sub(aabb._min, tf._position, radVec);
		M.vec3_add(aabb._max, tf._position, radVec);
	}

	@Override 
	public void computeLocalSupportingVertex(Vec3 dir,Vec3 out) {
		if (dir.y > 0) {
			out.set(0, _halfHeight, 0);
		} else {
			out.set(0, -_halfHeight, 0);
		}
	}

	@Override 
	public boolean _rayCastLocal(Vec3 begin, Vec3 end, RayCastHit result) {
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

		// XZ
		double tminxz = 0;
		double tmaxxz = 1;
		double a = dx * dx + dz * dz;
		double b = p1x * dx + p1z * dz;
		double c = (p1x * p1x + p1z * p1z) - _radius * _radius;
		double D = b * b - a * c;
		if (D < 0) return false;
		if (a > 0) {
			var sqrtD = MathUtil.sqrt(D);
			tminxz = (-b - sqrtD) / a;
			tmaxxz = (-b + sqrtD) / a;
			if (tminxz >= 1 || tmaxxz <= 0) return false;
		} else {
			if (c >= 0) return false;
			tminxz = 0;
			tmaxxz = 1;
		}

		double crossY = p1y + dy * tminxz;
		double min;

		if (crossY > -halfH && crossY < halfH) {
			//hits with cylinder
			if (tminxz > 0) {
				// hit: side
				min = tminxz;
				result.normal.set(p1x + dx * min, 0, p1z + dz * min).normalize();
				result.position.set(p1x + min * dx, crossY, p1z + min * dz);
				result.fraction = min;
				return true;
			}
			return false;
		}

		//hits with sphere
		double sphereY = crossY < 0 ? -halfH : halfH;
		Vec3 spherePos=new Vec3();
		Vec3 sphereToBegin=new Vec3();
		spherePos.set(0, sphereY, 0);
		M.vec3_sub(sphereToBegin, begin, spherePos);

		// sphere test
		Vec3 d=new Vec3();
		M.vec3_sub(d, end, begin);

		a = M.vec3_dot(d, d);
		b = M.vec3_dot(sphereToBegin, d);
		c = M.vec3_dot(sphereToBegin, sphereToBegin) - _radius * _radius;

		D = b * b - a * c;
		if (D < 0) return false;

		double t = (-b - MathUtil.sqrt(D)) / a;
		if (t < 0 || t > 1) return false;

		Vec3 hitPos=sphereToBegin.addScaledEq(d, t);
		result.normal=hitPos.normalize();
		result.position.copyFrom(spherePos).addEq(hitPos);
		result.fraction = t;
//		var hitPos:IVec3;
//		var hitNormal:IVec3;
//		M.vec3_addRhsScaled(hitPos, sphereToBegin, d, t);
//		M.vec3_normalize(hitNormal, hitPos);
//
//		// sphere-oriented pos -> local pos
//		M.vec3_add(hitPos, hitPos, spherePos);
//
//		M.vec3_toVec3(hit.position, hitPos);
//		M.vec3_toVec3(hit.normal, hitNormal);
//		hit.fraction = t;

		return true;
	}

}
