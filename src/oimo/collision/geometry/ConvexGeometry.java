package oimo.collision.geometry;


import oimo.collision.narrowphase.detector.gjkepa.GjkEpa;
import oimo.common.Setting;
import oimo.common.Transform;
import oimo.common.Vec3;

/**
 * Abstract class of the convex collision geometries supported by GJK/EPA collision detection.
 */
public abstract class  ConvexGeometry extends Geometry {
	// TODO: divide margin into "inner" margin and "outer" margin
	public float _gjkMargin;
	// if true GJK ray cast is used rather than default 
	public boolean _useGjkRayCast;

	public ConvexGeometry(int type) {
		super(type);
		_gjkMargin = Setting.defaultGJKMargin;
		_useGjkRayCast = false;
	}

	/**
	 * Returns the GJK mergin around the "core" of the convex geometry.
	 */
	public float getGjkMergin() {
		return _gjkMargin;
	}

	/**
	 * Sets the GJK mergin around the "core" to `gjkMergin`.
	 */
	public void setGjkMergin(float gjkMergin) {
		if (gjkMergin < 0) gjkMergin = 0;
		_gjkMargin = gjkMergin;
	}

	/**
	 * Computes supporting vertex of the "core" of the geometry in local coordinates.
	 * Note that the direction vector `dir` might not be normalized. `out` is set to
	 * the computed supporting vertex.
	 * @param dir  Direction in shape's local space along which supporting vector is calculated
	 */
	public abstract void computeLocalSupportingVertex(Vec3 dir, Vec3 out) ;
	

	@Override
	public boolean rayCast(Vec3 begin, Vec3 end, Transform transform, RayCastHit hit) {
		if (_useGjkRayCast) {
			return GjkEpa.getInstance().rayCast(this, transform, begin, end, hit);
		} else {
			return super.rayCast(begin, end, transform, hit);
		}
	}
}