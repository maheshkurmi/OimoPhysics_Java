package oimo.collision.geometry;

import oimo.common.M;
import oimo.common.MathUtil;
import oimo.common.Transform;
import oimo.common.Vec3;

/**
 * A sphere collision geometry.
 */
public class SphereGeometry extends ConvexGeometry {
	public float _radius;

	/**
	 * Creates a sphere collision geometry of radius `radius`.
	 */
	public SphereGeometry(float radius) {
		super(GeometryType.SPHERE);
		_radius = radius;
		_gjkMargin = _radius;
		_updateMass();
	}

	/**
	 * Returns the radius of the sphere.
	 */
	public float getRadius() {
		return _radius;
	}

	@Override 
	public void _updateMass() {
		_volume = 4 / 3 * MathUtil.PI * _radius * _radius * _radius;
		M.mat3_diagonal(_inertiaCoeff,
			2 / 5 * _radius * _radius,
			2 / 5 * _radius * _radius,
			2 / 5 * _radius * _radius
		);
	}

	@Override 
	public void _computeAabb(Aabb result,Transform tf) {
		var radVec =_TMP_V1.set(_radius, _radius, _radius);
		M.vec3_sub(result._min, tf._position, radVec);
		M.vec3_add(result._max, tf._position, radVec);
	}

	@Override 
	public void computeLocalSupportingVertex(Vec3 dir, Vec3 out) {
		out.zero();
	}

	@Override 
	public boolean _rayCastLocal(Vec3 begin ,Vec3 end, RayCastHit result) {
		
		Vec3 d=_TMP_V1;
		M.vec3_sub(d, end, begin);
		//Choose point on ray as begin+t*(end-begin) and solve for it to lie on sphere
		float a=d.dot(d);
		float b=begin.dot(d);
		float c=begin.dot(begin)-_radius * _radius;
		//solve quadratic int as at^2-2*b*t+c=0;
		float D=b * b - a * c;
		if(D<0)return false;
		//chhose lesser value of t (first hit point))
		float t = (-b - MathUtil.sqrt(D)) / a;
		if (t < 0 || t > 1) return false;
		M.vec3_addRhsScaled(result.position, begin, d, t);
		result.normal.copyFrom(result.position).normalize();
		result.fraction = t;

		return true;
	}

}
