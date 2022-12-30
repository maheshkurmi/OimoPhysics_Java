package oimo.dynamics.constraint.contact;
import oimo.collision.narrowphase.DetectorResultPoint;
import oimo.common.M;
import oimo.common.Vec3;
import oimo.common.Transform;

/**
 * A manifold point is a contact point in a contact manifold. This holds detailed collision
 * data (position, overlap depth, impulse, etc...) for collision response.
 */
public class ManifoldPoint {
	// manifold point relative to rigid bodies. NOT SHAPES.
	public Vec3 _localPos1=new Vec3();
	public Vec3 _localPos2=new Vec3();

	// local position with rotation
	public Vec3 _relPos1=new Vec3();
	public Vec3 _relPos2=new Vec3();

	// world position
	public Vec3 _pos1=new Vec3();
	public Vec3 _pos2=new Vec3();
	public float _depth=0;

	public ContactImpulse _impulse= new ContactImpulse();

	public boolean _warmStarted=false;

	// manifold points can be disabled for some reasons (separated, etc...)
	public boolean _disabled=false;

	public int _id=-1;

	public ManifoldPoint() {
		
	}

	// --- internal ---

	public void _clear() {
		_localPos1.zero();
		_localPos2.zero();
		_relPos1.zero();
		_relPos2.zero();
		_pos1.zero();
		_pos2.zero();
		_depth = 0;
		_impulse.clear();
		_warmStarted = false;
		_disabled = false;
		_id = -1;
	}

	public void _initialize(DetectorResultPoint result, Transform tf1, Transform tf2) {
		// world position
		M.vec3_fromVec3(_pos1, result.position1);
		M.vec3_fromVec3(_pos2, result.position2);

		// local position with rotation
		M.vec3_sub(_relPos1, _pos1, tf1._position);
		M.vec3_sub(_relPos2, _pos2, tf2._position);

		// local position
		M.vec3_mulMat3Transposed(_localPos1, _relPos1, tf1._rotation);
		M.vec3_mulMat3Transposed(_localPos2, _relPos2, tf2._rotation);

		_depth = result.depth;

		_impulse.clear();

		_id = result.id;
		_warmStarted = false;
		_disabled = false;
	}

	public void _updateDepthAndPositions(DetectorResultPoint result, Transform tf1, Transform tf2) {
		// world position
		M.vec3_fromVec3(_pos1, result.position1);
		M.vec3_fromVec3(_pos2, result.position2);

		// local position with rotation
		M.vec3_sub(_relPos1, _pos1, tf1._position);
		M.vec3_sub(_relPos2, _pos2, tf2._position);

		// local position
		M.vec3_mulMat3Transposed(_localPos1, _relPos1, tf1._rotation);
		M.vec3_mulMat3Transposed(_localPos2, _relPos2, tf2._rotation);

		_depth = result.depth;
	}

	public void _copyFrom(ManifoldPoint cp) {
		M.vec3_assign(_localPos1, cp._localPos1);
		M.vec3_assign(_localPos2, cp._localPos2);
		M.vec3_assign(_relPos1, cp._relPos1);
		M.vec3_assign(_relPos2, cp._relPos2);
		M.vec3_assign(_pos1, cp._pos1);
		M.vec3_assign(_pos2, cp._pos2);
		_depth = cp._depth;
		_impulse.copyFrom(cp._impulse);
		_id = cp._id;
		_warmStarted = cp._warmStarted;
		_disabled = false;
	}

	// --- public ---

	/**
	 * Returns the first rigid body's manifold point in world coordinate.
	 */
	public Vec3  getPosition1() {
		Vec3 v = new Vec3();
		M.vec3_toVec3(v, _pos1);
		return v;
	}

	/**
	 * Sets `position` to the first rigid body's manifold point in world coordinate.
	 * This does not create a new instance of `Vec3`.
	 */
	public void getPosition1To(Vec3 position) {
		M.vec3_toVec3(position, _pos1);
	}

	/**
	 * Returns the second rigid body's manifold point in world coordinate.
	 */
	public Vec3 getPosition2() {
		Vec3 v = new Vec3();
		M.vec3_toVec3(v, _pos2);
		return v;
	}

	/**
	 * Sets `position` to the second rigid body's manifold point in world coordinate.
	 * This does not create a new instance of `Vec3`.
	 */
	public void getPosition2To(Vec3 position) {
		M.vec3_toVec3(position, _pos2);
	}

	/**
	 * Returns the amount of the overlap. If the manifold point is separate, a negative
	 * value is returned.
	 */
	public float getDepth() {
		return _depth;
	}

	/**
	 * Returns whether the manifold point has existed for more than two steps.
	 */
	public boolean isWarmStarted() {
		return _warmStarted;
	}

	/**
	 * Returns the normal impulse of the manifold point.
	 */
	public float getNormalImpulse() {
		return _impulse.impulseN;
	}

	/**
	 * Returns the tangent impulse of the manifold point.
	 */
	public float getTangentImpulse() {
		return _impulse.impulseT;
	}

	/**
	 * Returns the binormal impulse of the manifold point.
	 */
	public float getBinormalImpulse() {
		return _impulse.impulseB;
	}

	/**
	 * Returns whether the manifold point is enabled.
	 */
	public boolean isEnabled() {
		return !_disabled;
	}

}
