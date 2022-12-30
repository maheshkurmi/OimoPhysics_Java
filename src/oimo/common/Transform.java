package oimo.common;


/**
 * Transform class provides a set of translation and rotation.
 */
public class Transform {
	public Vec3 _position;
	public Mat3 _rotation;
	/**
	 * Creates a new identical transform.
	 */
	public Transform() {
		_position=new Vec3();
		_rotation=new Mat3();
	}

	/**
	 * Sets the transformation to identity and returns `this`.
	 */
	public Transform identity() {
		_position.zero();
		_rotation.identity();
		return this;
	}

	/**
	 * Returns the copy of position of this transformation.
	 */
	public Vec3 getPosition() {
		return _position.clone();
	}

	/**
	 * Sets `position` to the position of the transformation.
	 *
	 * This does not create a new instance of `Vec3`.
	 */
	public void getPositionTo(Vec3 position) {
		position.copyFrom(_position);
	}

	/**
	 * Sets the position of the transformation to `position` and returns `this`.
	 */
	public Transform setPosition(Vec3 position) {
		_position.copyFrom(position);
		return this;
	}

	/**
	 * Translates the position by `translation`.
	 */
	public Transform translate(Vec3 translation) {
		_position.addEq(translation);
		return this;
	}

	/**
	 * Returns the copy of rotation matrix of this transformation.
	 */
	public Mat3 getRotation() {
		return _rotation.clone();
	}

	/**
	 * Sets `out` to the rotation matrix.
	 *
	 * This does not create a new instance of `Mat3`.
	 */
	public void getRotationTo(Mat3 out) {
		out.copyFrom(_rotation);
	}

	/**
	 * Sets the rotation matrix to `rotation` and returns `this`.
	 */
	public Transform setRotation(Mat3 rotation) {
		_rotation.copyFrom(rotation);
		return this;
	}

	/**
	 * Sets the rotation by Euler angles `eulerAngles` in radians.
	 */
	public Transform setRotationXyz(Vec3 eulerAngles) {
		_rotation.fromEulerXyz(eulerAngles);
		return this;
	}

	/**
	 * Applies rotation by the rotation matrix `rotation`.
	 */
	public Transform rotate(Mat3 rotation) {
		//premultiply rotation matrix
		M.mat3_mul(_rotation, rotation, _rotation);
		return this;
	}

	/**
	 * Applies the rotation by Euler angles `eulerAngles` in radians.
	 */
	public Transform rotateXyz(Vec3 eulerAngles) {
		//		_rotation.m
		//		var xyz:IVec3;
		//		var rot:IMat3;
		//		M.vec3_fromVec3(xyz, eulerAngles);
		//		M.mat3_fromEulerXyz(rot, xyz);
		//		M.mat3_mul(_rotation, rot, _rotation);
		M.mat3_fromEulerXyz(_rotation, eulerAngles);
		return this;
	}

	/**
	 * Returns the rotation as a quaternion.
	 */
	public Quat getOrientation() {
		Quat q=new Quat();
		q.fromMat3(_rotation);
		//		
		//		var q:Quat = new Quat();
		//		var iq:IQuat;
				M.quat_fromMat3(q, _rotation);
		//		M.quat_toQuat(q, iq);
		return q;
	}

	/**
	 * Sets `orientation` to the quaternion representing the rotation.
	 *
	 * This does not create a new instance of `Quat`.
	 */
	public void getOrientationTo(Quat orientation) {
		//		var iq:IQuat;
		//		M.quat_fromMat3(iq, _rotation);
		//		M.quat_toQuat(orientation, iq);
		orientation.fromMat3(_rotation);
	}

	/**
	 * Sets the rotation from a quaternion `quaternion` and returns `this`.
	 */
	public Transform setOrientation(Quat quaternion) {
		//		var q:IQuat;
		//		M.quat_fromQuat(q, quaternion);
		//		M.mat3_fromQuat(_rotation, q);
		_rotation.fromQuat(quaternion);
		return this;
	}

	/**
	 * Returns a clone of the transformation.
	 */
	public Transform clone() {
		var tf = new Transform();
		tf._position.copyFrom(_position);
		tf._rotation.copyFrom(_rotation);
		return tf;
	}

	/**
	 * Sets the transformation to `transform` and returns `this`.
	 */
	public Transform copyFrom(Transform transform) {
		_position.copyFrom(transform._position);
		_rotation.copyFrom(transform._rotation);
		return this;
	}
	
	/**
	 * Applies this transform on the vector 'v' and returns transformed 'v' (mutates v)
	 * @param v vector to transform
	 * @return v after applying transform
	 */
	public Vec3 transformVec(Vec3 v) {
		//first rotate 
		M.vec3_mulMat3(v, v, _rotation);
		//now translate
		M.vec3_add(v, v, _position);
		// return transformed vecor
		return v;
	
	}

	/**
	 * Applies inverse of this transform on the vector 'v' and returns transformed 'v' (mutates v)
	 * @param v vector to transform
	 * @return v after applying inverse of this transform
	 */
	public Vec3 inverseTransform(Vec3 v) {
		//first apply inverse translation
		M.vec3_sub(v, v, _position);
		//now apply inverse rotation
		M.vec3_mulMat3Transposed(v, v, _rotation); //transpose is equal to inverse for orthogonal rotation matrix
		return v;
	}
}