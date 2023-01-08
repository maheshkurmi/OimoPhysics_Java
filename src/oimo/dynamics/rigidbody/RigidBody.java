package oimo.dynamics.rigidbody;

//import org.shikhar.simphy.gfx.canvas.scene3d.ModelNode;
//import org.shikhar.simphy.gfx.canvas.scene3d.SceneNode3D;
//import org.shikhar.simphy.gfx.math.Quaternion;

import oimo.collision.geometry.*;
import oimo.common.*;
import oimo.dynamics.*;
import oimo.dynamics.constraint.contact.*;
import oimo.dynamics.constraint.joint.*;
import oimo.common.M;

/**
 * A rigid body. To add a rigid body to a physics world, create a `RigidBody`
 * instance, create and add shapes via `RigidBody.addShape`, and add the rigid
 * body to the physics world through `World.addRigidBody`. Rigid bodies have
 * three motion types: dynamic, static, and kinematic. See `RigidBodyType` for
 * details of motion types.
 */
public class RigidBody {
	public RigidBody _next;
	public RigidBody _prev;

	public Shape _shapeList;
	public Shape _shapeListLast;
	public int _numShapes;

	public Vec3 _vel;
	public Vec3 _angVel;

	public Vec3 _pseudoVel;
	public Vec3 _angPseudoVel;

	public Transform _ptransform;
	public Transform _transform;

	public int _type;

	public double _sleepTime;
	public boolean _sleeping;
	public boolean _autoSleep;

	public double _mass;
	public double _invMass;
	public Mat3 _localInertia;
	public Vec3 _rotFactor;
	public Mat3 _invLocalInertia;
	public Mat3 _invLocalInertiaWithoutRotFactor;
	public Mat3 _invInertia;

	public double _linearDamping;
	public double _angularDamping;

	public Vec3 _force;
	public Vec3 _torque;

	public Vec3 _linearContactImpulse;
	public Vec3 _angularContactImpulse;

	public World _world;

	public ContactLink _contactLinkList;
	public ContactLink _contactLinkListLast;
	public int _numContactLinks;

	public JointLink _jointLinkList;
	public JointLink _jointLinkListLast;
	public int _numJointLinks;

	public boolean _addedToIsland;
	public double _gravityScale;

	/**
	 * Extra field that users can use for their own purposes.
	 */
	public Object userData;

	/**
	 * Creates a new rigid body by configuration `config`.
	 */
	public RigidBody(RigidBodyConfig config) {
		_next = null;
		_prev = null;

		_shapeList = null;
		_shapeListLast = null;
		_numShapes = 0;

		_contactLinkList = null;
		_contactLinkListLast = null;
		_numContactLinks = 0;

		_jointLinkList = null;
		_jointLinkListLast = null;
		_numJointLinks = 0;

		_vel=config.linearVelocity.clone();
		_angVel=config.angularVelocity.clone();
		
		_pseudoVel=new Vec3();
		_angPseudoVel=new Vec3();
		
		_ptransform = new Transform();
		_transform = new Transform();
		
		M.vec3_fromVec3(_ptransform._position, config.position);
		M.mat3_fromMat3(_ptransform._rotation, config.rotation);
		M.transform_assign(_transform, _ptransform);

		_type = config.type;

		_sleepTime = 0;
		_sleeping = false;
		_autoSleep = config.autoSleep;

		_mass = 0;
		_invMass = 0;
		_localInertia=new Mat3();
		_invLocalInertia=new Mat3();
		_invLocalInertiaWithoutRotFactor=new Mat3();
		_invInertia=new Mat3();
		
		M.mat3_zero(_localInertia);
		M.mat3_zero(_invLocalInertia);
		M.mat3_zero(_invLocalInertiaWithoutRotFactor);
		M.mat3_zero(_invInertia);

		_linearDamping = config.linearDamping;
		_angularDamping = config.angularDamping;

		_force=new Vec3();
		_torque=new Vec3();
	
		_linearContactImpulse=new Vec3();
		_angularContactImpulse=new Vec3();
	
		_rotFactor = new Vec3(1, 1, 1);

		_addedToIsland = false;
		_gravityScale = 1;

		_world = null;
	}

	// --- internal ---

	public void _integrate(double dt) {
		switch (_type) {
			case RigidBodyType._DYNAMIC:
			case RigidBodyType._KINEMATIC:
				Vec3 translation=new Vec3();
				Vec3 rotation=new Vec3();
				M.vec3_scale(translation, _vel, dt);
				M.vec3_scale(rotation, _angVel, dt);

				double translationLengthSq = M.vec3_dot(translation, translation);
				double rotationLengthSq = M.vec3_dot(rotation, rotation);

				if (translationLengthSq == 0 && rotationLengthSq == 0) {
					return; // no need of integration
				}

				// limit linear velocity
				if (translationLengthSq > Setting.maxTranslationPerStep * Setting.maxTranslationPerStep) {
					double l = Setting.maxTranslationPerStep / MathUtil.sqrt(translationLengthSq);
					M.vec3_scale(_vel, _vel, l);
					M.vec3_scale(translation, translation, l);
				}

				// limit angular velocity
				if (rotationLengthSq > Setting.maxRotationPerStep * Setting.maxRotationPerStep) {
					double l = Setting.maxRotationPerStep / MathUtil.sqrt(rotationLengthSq);
					M.vec3_scale(_angVel, _angVel, l);
					M.vec3_scale(rotation, rotation, l);
				}

				// update the transform
				_applyTranslation(translation);
				_applyRotation(rotation);
				
				break;
			case RigidBodyType._STATIC:
				M.vec3_zero(_vel);
				M.vec3_zero(_angVel);
				M.vec3_zero(_pseudoVel);
				M.vec3_zero(_angPseudoVel);
				break;
		}
	}

	public void _integratePseudoVelocity() {
		double pseudoVelLengthSq = M.vec3_dot(_pseudoVel, _pseudoVel);
		double angPseudoVelLengthSq = M.vec3_dot(_angPseudoVel, _angPseudoVel);
		if (pseudoVelLengthSq == 0 && angPseudoVelLengthSq == 0) {
			return; // no need of intgration
		}

		switch (_type) {
			case RigidBodyType._DYNAMIC:
			case RigidBodyType._KINEMATIC:
				Vec3 translation=_pseudoVel.clone();
				Vec3 rotation=_angPseudoVel.clone();
				
				// clear pseudo velocity
				M.vec3_zero(_pseudoVel);
				M.vec3_zero(_angPseudoVel);

				// update the transform
				_applyTranslation(translation);
				_applyRotation(rotation);
			   
				break;
			case RigidBodyType._STATIC:
				M.vec3_zero(_pseudoVel);
				M.vec3_zero(_angPseudoVel);
				break;
		}
	}

	 public boolean _isSleepy() {
		return _autoSleep
			&& M.vec3_dot(_vel, _vel) < Setting.sleepingVelocityThreshold * Setting.sleepingVelocityThreshold
			&& M.vec3_dot(_angVel, _angVel) < Setting.sleepingAngularVelocityThreshold * Setting.sleepingAngularVelocityThreshold;
	}

	 public boolean _isAlone() {
		return _numContactLinks == 0 && _numJointLinks == 0;
	}

	 public void _applyTranslation(Vec3 translation) {
		M.vec3_add(_transform._position, _transform._position, translation);
	}

	 public void _applyRotation(Vec3 rotation) {
		// compute derivative of the quaternion
		double theta = M.vec3_length(rotation);
		double halfTheta = theta * 0.5f;
		double rotationToSinAxisFactor; // sin(halfTheta) / theta;
		 double cosHalfTheta; // cos(halfTheta)
		if (halfTheta < 0.5) {
			// use Maclaurin expansion
			double ht2 = halfTheta * halfTheta;
			rotationToSinAxisFactor = (1 / 2f) * (1 - ht2 * (1 / 6f) + ht2 * ht2 * (1 / 120f));
			cosHalfTheta = 1 - ht2 * (1 / 2f) + ht2 * ht2 * (1 / 24f);
		} else {
			rotationToSinAxisFactor = MathUtil.sin(halfTheta) / theta;
			cosHalfTheta = MathUtil.cos(halfTheta);
		}
		Vec3 sinAxis=new Vec3();
		M.vec3_scale(sinAxis, rotation, rotationToSinAxisFactor);
		Quat dq=new Quat();
		M.quat_fromVec3AndFloat(dq, sinAxis, cosHalfTheta);
		// integrate quaternion
		Quat q=new Quat();
		M.quat_fromMat3(q, _transform._rotation);
		M.quat_mul(q, dq, q);
		q.normalize();
		//M.quat_normalize(q, q);

		// update rotation
		_transform._rotation.fromQuat(q);
		M.mat3_normalize(_transform._rotation);
		//M.mat3_fromQuat(_transform._rotation, q);

		// update inertia tensor
		updateInvInertia();
	}

	// call when added/removed/modified shapes
	 protected void _shapeModified() {
		updateMass();
		_syncShapes();
	}

	 public void _syncShapes() {
		Shape s = _shapeList;
		while(s!=null) {
			s._sync(_ptransform, _transform);
			s=s._next;
		}
		  this.updateMesh();
//		M.list_foreach(s, _next, {
//			M.call(s._sync(_ptransform, _transform));
//		});
	}

	 public void _applyLinearPositionImpulse(Vec3 imp) {
		Vec3 translation=new Vec3();
		M.vec3_scale(translation, imp, _invMass);
		_applyTranslation(translation);
	}

	 public void _applyAngularPositionImpulse(Vec3 imp) {
		Vec3 rotation=new Vec3();
		M.vec3_mulMat3(rotation, imp, _invInertia);
		_applyRotation(rotation);
	}

	// --- private ---

	public void updateMass() {
		Mat3 totalInertia=new Mat3();
		double totalMass;
		M.mat3_zero(totalInertia);
		totalMass = 0;

		Shape s = _shapeList;
		while(s!=null) {
			Geometry g = s._geom;
			g._updateMass();

			double mass = s._density * g._volume;
			Mat3 inertia=new Mat3();

			// I_transformed = (R * I_localCoeff * R^T) * mass
			M.mat3_transformInertia(inertia, g._inertiaCoeff, s._localTransform._rotation);
			inertia.scaleEq(mass);
			//M.mat3_scale(inertia, inertia, mass);

			// I_cog = |  y*y+z*z  -x*y      -x*z     |
			//         | -x*y       x*x+z*z  -y*z     | * mass
			//         | -x*z      -y*z       x*x+y*y |
			// I = I_transformed + I_cog
			Mat3 cogInertia=new Mat3();
			M.mat3_inertiaFromCOM(cogInertia, s._localTransform._position,mass);
			inertia.addEq(cogInertia);
			//M.mat3_addRhsScaled(inertia, inertia, cogInertia, mass);

			// add mass data
			totalMass += mass;
			totalInertia.addEq(inertia);
			s=s._next;
		};

		_mass = totalMass;
		_localInertia.copyFrom(totalInertia);
		completeMassData();
		// wake up the rigid body
		wakeUp();
	}

	// compute inverse mass and inertias from _mass and _localInertia
	public void completeMassData() {
		double det;
		det = _localInertia.determinant();//M.mat3_det(_localInertia);
		if (_mass > 0 && det > 0 && _type == RigidBodyType._DYNAMIC) {
			_invMass = 1 / _mass;
			_invLocalInertia.copyFrom(_localInertia).inverseEq();
			//M.mat3_inv(_invLocalInertia, _localInertia);
			_invLocalInertiaWithoutRotFactor.copyFrom(_invLocalInertia);
			//M.mat3_assign(_invLocalInertiaWithoutRotFactor, _invLocalInertia);
			
			M.mat3_scaleRows(_invLocalInertia, _invLocalInertiaWithoutRotFactor, _rotFactor.x, _rotFactor.y, _rotFactor.z);
		} else {
			// set mass and inertia zero
			_invMass = 0;
			M.mat3_zero(_invLocalInertia);
			M.mat3_zero(_invLocalInertiaWithoutRotFactor);

			// force static
			if (_type == RigidBodyType._DYNAMIC) {
				_type = RigidBodyType._STATIC;
			}
		}
		updateInvInertia();
	}

	public void updateInvInertia() {
		M.mat3_transformInertia(_invInertia, _invLocalInertia, _transform._rotation);
		M.mat3_scaleRows(_invInertia, _invInertia, _rotFactor.x, _rotFactor.y, _rotFactor.z);
	}

	// call when the transform is externally updated
	public void updateTransformExt() {
		M.transform_assign(_ptransform, _transform);
		M.mat3_normalize(_transform._rotation);
		_syncShapes();
		wakeUp();
	}

	// --- public ---

	/**
	 * Returns the copy of world position of the rigid body.
	 */
	public Vec3 getPosition() {
		return _transform._position.clone();
	}

	/**
	 * Sets `position` to the world position of the rigid body.
	 *
	 * This does not create a new instance of `Vec3`.
	 */
	public void getPositionTo(Vec3 position) {
		position.copyFrom(_transform._position);
		//M.vec3_toVec3(position, _transform._position);
	}

	/**
	 * Sets the world position of the rigid body to `position`.
	 */
	public void setPosition(Vec3 position) {
		M.vec3_fromVec3(_transform._position, position);
		updateTransformExt();
	}

	/**
	 * Translates the position of the rigid body by `translation`.
	 */
	public void translate(Vec3 translation) {
//		var diff:IVec3;
//		M.vec3_fromVec3(diff, translation);
//		M.vec3_add(_transform._position, _transform._position, diff);
		_transform._position.addEq(translation);
		updateTransformExt();
	}

	/**
	 * Returns the copy of rotation matrix of the rigid body.
	 */
	public Mat3 getRotation() {
		//var m:Mat3 = new Mat3();
		//M.mat3_toMat3(m, _transform._rotation);
		//return m;
		return _transform._rotation.clone();
	}

	/**
	 * Sets `rotation` to the rotation matrix of the rigid body.
	 *
	 * This does not create a new instance of `Mat3`.
	 */
	public void getRotationTo(Mat3 rotation) {
		M.mat3_toMat3(rotation, _transform._rotation);
	}

	/**
	 * Sets the rotation matrix of the rigid body to `rotation`.
	 */
	public void setRotation(Mat3 rotation) {
		M.mat3_fromMat3(_transform._rotation, rotation);
		updateInvInertia();
		updateTransformExt();
	}

	/**
	 * Sets the rotation of the rigid body by Euler angles `eulerAngles` in radians.
	 */
	public void setRotationXyz(Vec3 eulerAngles) {
		M.mat3_fromEulerXyz(_transform._rotation, eulerAngles);
		updateInvInertia();
		updateTransformExt();
	}

	/**
	 * Rotates the rigid body by the rotation matrix `rotation`.
	 */
	public void rotate(Mat3 rotation) {
		//var rot:IMat3;
		//M.mat3_fromMat3(rot, rotation);
		//M.mat3_mul(_transform._rotation, rot, _transform._rotation);

		M.mat3_mul(_transform._rotation, rotation, _transform._rotation);

		updateInvInertia();
		updateTransformExt();
	}

	/**
	 * Rotates the rigid body by Euler angles `eulerAngles` in radians.
	 */
	public void rotateXyz(Vec3 eulerAngles) {
		Mat3 rot =new Mat3();
		M.mat3_fromEulerXyz(rot, eulerAngles);
		M.mat3_mul(_transform._rotation, rot, _transform._rotation);

		updateInvInertia();
		updateTransformExt();
	}

	/**
	 * Returns the rotation of the rigid body as a quaternion.
	 */
	public Quat getOrientation() {
//		Quat q = new Quat();
//		var iq=new Quat();
//		M.quat_fromMat3(q, _transform._rotation);
//		M.quat_toQuat(q, iq);
		return _transform._rotation.toQuat();
	}

	/**
	 * Sets `orientation` to the rotation quaternion of the rigid body.
	 *
	 * This does not create a new instance of `Quat`.
	 */
	public void getOrientationTo(Quat orientation) {
//		var iq:IQuat;
//		M.quat_fromMat3(iq, _transform._rotation);
//		M.quat_toQuat(orientation, iq);
		orientation.fromMat3(_transform._rotation);
	}

	/**
	 * Sets the rotation of the rigid body from a quaternion `quaternion`.
	 */
	public void setOrientation(Quat quaternion) {
//		var q:IQuat;
//		M.quat_fromQuat(q, quaternion);
//		M.mat3_fromQuat(_transform._rotation, q);
		_transform._rotation.fromQuat(quaternion);
		updateInvInertia();
		updateTransformExt();
	}

	/**
	 * Returns the copy of transform of the rigid body.
	 */
	public Transform getTransform() {
		return _transform.clone();
	}

	/**
	 * Sets `transform` to the transform of the rigid body.
	 *
	 * This does not create a new instance of `Transform`.
	 */
	public void getTransformTo(Transform transform) {
		transform.copyFrom(_transform);
	}

	/**
	 * Sets the transform of the rigid body to `transform`.
	 *
	 * This does not keep any references to `transform`.
	 */
	public void setTransform(Transform transform) {
		//M.vec3_assign(_transform._position, transform._position);
		//M.mat3_assign(_transform._rotation, transform._rotation);
		_transform.copyFrom(transform);
		updateInvInertia();
		updateTransformExt();
	}

	/**
	 * Returns the mass of the rigid body.
	 *
	 * If the rigid body has infinite mass, `0` will be returned.
	 */
	public double getMass() {
		return _mass;
	}

	/**
	 * Returns the copy of moment of inertia tensor in local space.
	 */
	public Mat3 getLocalInertia() {
//		Mat3 m = new Mat3();
//		M.mat3_toMat3(m, _localInertia);
//		return m;
		return _localInertia.clone();
	}

	/**
	 * Sets `inertia` to the moment of inertia tensor in local space.
	 *
	 * This does not create a new instance of `Mat3`
	 */
	public void getLocalInertiaTo(Mat3 inertia) {
		M.mat3_toMat3(inertia, _localInertia);
	}

	/**
	 * Returns the mass data of the rigid body.
	 */
	public MassData getMassData() {
		MassData md = new MassData();
		md.mass = _mass;
		M.mat3_toMat3(md.localInertia, _localInertia);
		return md;
	}

	/**
	 * Sets `massData` to the mass data of the rigid body.
	 *
	 * This does not create a new instance of `MassData`.
	 */
	public void getMassDataTo(MassData massData) {
		massData.mass = _mass;
		M.mat3_toMat3(massData.localInertia, _localInertia);
	}

	/**
	 * Sets the mass and moment of inertia of the rigid body by the mass data `massData`.
	 * The properties set by this will be overwritten when
	 *
	 * - some shapes are added or removed
	 * - the type of the rigid body is changed
	 */
	public void setMassData(MassData massData) {
		_mass = massData.mass;
		M.mat3_fromMat3(_localInertia, massData.localInertia);
		completeMassData();
		wakeUp();
	}

	/**
	 * Returns the rotation factor of the rigid body.
	 */
	public Vec3 getRotationFactor() {
		return _rotFactor.clone();
	}

	/**
	 * Sets the rotation factor of the rigid body to `rotationFactor`.
	 *
	 * This changes moment of inertia internally, so that the change of
	 * angular velocity in **global space** along X, Y and Z axis will scale by `rotationFactor.x`,
	 * `rotationFactor.y` and `rotationFactor.z` times respectively.
	 */
	public void setRotationFactor(Vec3 rotationFactor) {
		_rotFactor.copyFrom(rotationFactor);

		updateInvInertia();
		wakeUp();
	}

	/**
	 * Returns the copy of linear velocity of the rigid body.
	 */
	public Vec3  getLinearVelocity() {
//		Vec3 v = new Vec3();
//		M.vec3_toVec3(v, _vel);
		return _vel.clone();
	}

	/**
	 * Sets `linearVelocity` to the linear velocity of the rigid body.
	 *
	 * This does not create a new intrance of `Vec3`.
	 */
	public void getLinearVelocityTo(Vec3 linearVelocity) {
		//M.vec3_toVec3(linearVelocity, _vel);
		linearVelocity.copyFrom(_vel);
	}

	/**
	 * Sets the linear velocity of the rigid body.
	 */
	public void setLinearVelocity(Vec3 linearVelocity) {
		if (_type == RigidBodyType._STATIC) {
			_vel.zero();
			//M.vec3_zero(_vel);
		} else {
			_vel.copyFrom(linearVelocity);
			//M.vec3_fromVec3(_vel, linearVelocity);
		}
		wakeUp();
	}

	/**
	 * Returns the angular velocity of the rigid body.
	 */
	public Vec3 getAngularVelocity() {
//		var v:Vec3 = new Vec3();
//		M.vec3_toVec3(v, _angVel);
//		return v;
		return _angVel.clone();
	}

	/**
	 * Sets `angularVelocity` to the angular velocity of the rigid body.
	 *
	 * This does not create a new intrance of `Vec3`.
	 */
	public void getAngularVelocityTo(Vec3 angularVelocity) {
		//M.vec3_toVec3(angularVelocity, _vel);
		angularVelocity.copyFrom(_angVel);
	}

	/**
	 * Sets the angular velocity of the rigid body.
	 */
	public void setAngularVelocity(Vec3 angularVelocity) {
		if (_type == RigidBodyType._STATIC) {
			M.vec3_zero(_angVel);
		} else {
			M.vec3_fromVec3(_angVel, angularVelocity);
		}
		wakeUp();
	}

	/**
	 * Adds `linearVelocityChange` to the linear velcity of the rigid body.
	 */
	public void addLinearVelocity(Vec3 linearVelocityChange) {
		if (_type != RigidBodyType._STATIC) {
//			var d;
//			M.vec3_fromVec3(d, linearVelocityChange);
//			M.vec3_add(_vel, _vel, d);
			_vel.add(linearVelocityChange);
		}
		wakeUp();
	}

	/**
	 * Adds `angularVelocityChange` to the angular velcity of the rigid body.
	 */
	public void addAngularVelocity(Vec3 angularVelocityChange) {
		if (_type != RigidBodyType._STATIC) {
//			var d:IVec3;
//			M.vec3_fromVec3(d, angularVelocityChange);
//			M.vec3_add(_angVel, _angVel, d);
			_angVel.add(angularVelocityChange);
		}
		wakeUp();
	}

	/**
	 * Applies the impulse `impulse` to the rigid body at `positionInWorld` in world position.
	 *
	 * This changes both the linear velocity and the angular velocity.
	 */
	public void applyImpulse(Vec3 impulse, Vec3 positionInWorld) {
		// linear
		Vec3 imp=new Vec3();
		M.vec3_fromVec3(imp, impulse);
		M.vec3_addRhsScaled(_vel, _vel, imp, _invMass);

		// angular
		Vec3 aimp=new Vec3();
		Vec3 pos=new Vec3();
		M.vec3_fromVec3(pos, positionInWorld);
		M.vec3_sub(pos, pos, _transform._position);
		M.vec3_cross(aimp, pos, imp);
		M.vec3_mulMat3(aimp, aimp, _invInertia);
		M.vec3_add(_angVel, _angVel, aimp);

		wakeUp();
	}

	/**
	 * Applies the linear impulse `impulse` to the rigid body.
	 *
	 * This does not change the angular velocity.
	 */
	public void applyLinearImpulse(Vec3 impulse) {
		//var imp:IVec3;
		//M.vec3_fromVec3(imp, impulse);
		M.vec3_addRhsScaled(_vel, _vel, impulse, _invMass);
		wakeUp();
	}

	/**
	 * Applies the angular impulse `impulse` to the rigid body.
	 *
	 * This does not change the linear velocity.
	 */
	public void  applyAngularImpulse(Vec3 impulse) {
		Vec3 imp=new Vec3();
		M.vec3_fromVec3(imp, impulse);
		M.vec3_mulMat3(imp, imp, _invInertia);
		M.vec3_add(_angVel, _angVel, imp);
		wakeUp();
	}

	/**
	 * Applies the force `force` to `positionInWorld` in world position.
	 */
	public void applyForce(Vec3 force, Vec3 positionInWorld) {
		// linear
//		var iforce:IVec3;
//		M.vec3_fromVec3(iforce, force);
//		M.vec3_add(_force, _force, iforce);
		_force.add(force);
		// angular
		Vec3 itorque=new Vec3();
		Vec3 pos=new Vec3();
		M.vec3_fromVec3(pos, positionInWorld);
		M.vec3_sub(pos, pos, _transform._position);
		M.vec3_cross(itorque, pos, force);
		M.vec3_add(_torque, _torque, itorque);

		wakeUp();
	}

	/**
	 * Applies the force `force` to the center of mass.
	 */
	public void applyForceToCenter(Vec3 force) {
		// linear
//		var iforce:IVec3;
//		M.vec3_fromVec3(iforce, force);
//		M.vec3_add(_force, _force, iforce);
		_force.add(force);
		wakeUp();
	}

	/**
	 * Applies the torque `torque`.
	 */
	public void applyTorque(Vec3 torque) {
		// angular
//		Vec3 itorque;
//		M.vec3_fromVec3(itorque, torque);
//		M.vec3_add(_torque, _torque, itorque);
		_torque.add(torque);
		wakeUp();
	}

	/**
	 * Returns the total linear impulse applied by contact constraints.
	 */
	public Vec3 getLinearContactImpulse() {
//		var res:Vec3 = new Vec3();
//		M.vec3_toVec3(res, _linearContactImpulse);
//		return res;
		return _linearContactImpulse.clone();
	}

	/**
	 * Sets `linearContactImpulse` to the total linear impulse applied by contact constraints.
	 *
	 * This does not create a new instance of `Vec3`.
	 */
	public void getLinearContactImpulseTo(Vec3 linearContactImpulse) {
		M.vec3_toVec3(linearContactImpulse, _linearContactImpulse);
	}

	/**
	 * Returns the total angular impulse applied by contact constraints.
	 */
	public Vec3 getAngularContactImpulse() {
//		var res:Vec3 = new Vec3();
//		M.vec3_toVec3(res, _angularContactImpulse);
//		return res;
		return _angularContactImpulse.clone();
	}

	/**
	 * Sets `angularContactImpulse` to the total angular impulse applied by contact constraints.
	 *
	 * This does not create a new instance of `Vec3`.
	 */
	public void getAngularContactImpulseTo(Vec3 angularContactImpulse) {
		M.vec3_toVec3(angularContactImpulse, _angularContactImpulse);
	}

	/**
	 * Returns the gravity scaling factor of the rigid body.
	 */
	public double getGravityScale() {
		return _gravityScale;
	}

	/**
	 * Sets the gravity scaling factor of the rigid body to `gravityScale`.
	 *
	 * If `0` is set, the rigid body will not be affected by gravity.
	 */
	public void setGravityScale(double gravityScale) {
		_gravityScale = gravityScale;
		wakeUp();
	}

	/**
	 * Returns the local coordinates of the point `worldPoint` in world coodinates.
	 */
	public Vec3 getLocalPoint(Vec3 worldPoint) {
		Vec3 v=new Vec3();
		M.vec3_fromVec3(v, worldPoint);
		M.vec3_sub(v, v, _transform._position);
		M.vec3_mulMat3Transposed(v, v, _transform._rotation);
		//var res:Vec3 = new Vec3();
		//M.vec3_toVec3(res, v);
		return v;
	}

	/**
	 * Sets `localPoint` to the local coordinates of the point `worldPoint` in world coodinates.
	 *
	 * This does not create a new instance of `Vec3`.
	 */
	public void getLocalPointTo(Vec3 worldPoint, Vec3 localPoint) {
		Vec3 v=localPoint;
		M.vec3_fromVec3(v, worldPoint);
		M.vec3_sub(v, v, _transform._position);
		M.vec3_mulMat3Transposed(v, v, _transform._rotation);
		//M.vec3_toVec3(localPoint, v);
	}

	/**
	 * Returns the local coordinates of the vector `worldVector` in world coodinates.
	 */
	public Vec3 getLocalVector(Vec3 worldVector) {
		Vec3 v=new Vec3();
		M.vec3_fromVec3(v, worldVector);
		M.vec3_mulMat3Transposed(v, v, _transform._rotation);
		//var res:Vec3 = new Vec3();
		//M.vec3_toVec3(res, v);
		return v;
	}

	/**
	 * Sets `localVector` to the local coordinates of the vector `worldVector` in world coodinates.
	 *
	 * This does not create a new instance of `Vec3`.
	 */
	public void getLocalVectorTo(Vec3 worldVector, Vec3 localVector) {
		Vec3 v=localVector;
		M.vec3_fromVec3(v, worldVector);
		M.vec3_mulMat3Transposed(v, v, _transform._rotation);
		//M.vec3_toVec3(localVector, v);
	}

	/**
	 * Returns the world coordinates of the point `localPoint` in local coodinates.
	 */
	public Vec3 getWorldPoint(Vec3 localPoint) {
		Vec3 v=new Vec3();
		M.vec3_fromVec3(v, localPoint);
		M.vec3_mulMat3(v, v, _transform._rotation);
		M.vec3_add(v, v, _transform._position);
		//var res:Vec3 = new Vec3();
		//M.vec3_toVec3(res, v);
		return v;
	}

	/**
	 * Sets `worldPoint` to the world coordinates of the point `localPoint` in local coodinates.
	 *
	 * This does not create a new instance of `Vec3`.
	 */
	public void getWorldPointTo(Vec3 localPoint, Vec3 worldPoint) {
		Vec3 v=worldPoint;
		M.vec3_fromVec3(v, localPoint);
		M.vec3_mulMat3(v, v, _transform._rotation);
		M.vec3_add(v, v, _transform._position);
		//M.vec3_toVec3(worldPoint, v);
	}

	/**
	 * Returns the world coordinates of the vector `localVector` in local coodinates.
	 */
	public Vec3 getWorldVector(Vec3 localVector) {
		Vec3 v=new Vec3();
		M.vec3_fromVec3(v, localVector);
		M.vec3_mulMat3(v, v, _transform._rotation);
		//var res:Vec3 = new Vec3();
	//	M.vec3_toVec3(res, v);
		return v;
	}

	/**
	 * Sets `worldVector` to the world coordinates of the vector `localVector` in local coordinates.
	 *
	 * This does not create a new instance of `Vec3`.
	 */
	public void getWorldVectorTo(Vec3 localVector,Vec3  worldVector) {
		Vec3 v=worldVector;
		M.vec3_fromVec3(v, localVector);
		M.vec3_mulMat3(v, v, _transform._rotation);
		//M.vec3_toVec3(worldVector, v);
	}

	/**
	 * Returns the number of the shapes added.
	 */
	public int getNumShapes() {
		return _numShapes;
	}

	/**
	 * Returns the list of the shapes of the rigid body.
	 */
	public Shape getShapeList() {
		return _shapeList;
	}

	/**
	 * Returns the number of the contact lists the rigid body is involved.
	 */
	public int getNumContactLinks() {
		return _numContactLinks;
	}

	/**
	 * Returns the list of the contact links the rigid body is involved.
	 */
	public ContactLink getContactLinkList() {
		return _contactLinkList;
	}

	/**
	 * Returns the number of the joint links the rigid body is attached.
	 */
	public int getNumJointLinks() {
		return _numJointLinks;
	}

	/**
	 * Returns the list of the joint links the rigid body is attached.
	 */
	public JointLink getJointLinkList() {
		return _jointLinkList;
	}

	/**
	 * Adds the shape to the rigid body.
	 */
	public void addShape(Shape shape) {
		// first, add the shape to the linked list so that it will be considered
		//M.list_push(_shapeList, _shapeListLast, _prev, _next, shape);
		if(this._shapeList == null) {
			this._shapeList = shape;
			this._shapeListLast = shape;
		} else {
			this._shapeListLast._next = shape;
			shape._prev = this._shapeListLast;
			this._shapeListLast = shape;
		}
		_numShapes++;
		shape._rigidBody = this;

		// then add the shape to the world
		if (_world != null) {
			_world._addShape(shape);
		}

		_shapeModified();
	}

	/**
	 * Removes the shape from the rigid body.
	 */
	public void removeShape(Shape shape) {
		// first remove the shape from the world
		if (_world != null) {
			_world._removeShape(shape);
		}
		
		// then, remove the shape from the linked list so that it will be ignored
		Shape prev = shape._prev;
		Shape next = shape._next;
		if(prev != null) {
			prev._next = next;
		}
		if(next != null) {
			next._prev = prev;
		}
		if(shape == this._shapeList) {
			this._shapeList = this._shapeList._next;
		}
		if(shape == this._shapeListLast) {
			this._shapeListLast = this._shapeListLast._prev;
		}
		shape._next = null;
		shape._prev = null;
		this._numShapes--;
		shape._rigidBody = null;

		_shapeModified();
	}

	/**
	 * Returns the rigid body's type of behaviour.
	 *
	 * See `RigidBodyType` class for details.
	 */
	public int getType() {
		return _type;
	}

	/**
	 * Sets the rigid body's type of behaviour.
	 *
	 * See `RigidBodyType` class for details.
	 */
	public void setType(int type) {
		_type = type;
		updateMass();
	}

	/**
	 * Sets the rigid body's sleep flag false.
	 *
	 * This also resets the sleeping timer of the rigid body.
	 */
	public void wakeUp() {
		_sleeping = false;
		_sleepTime = 0;
	}

	/**
	 * Sets the rigid body's sleep flag true.
	 *
	 * This also resets the sleeping timer of the rigid body.
	 */
	public void sleep() {
		_sleeping = true;
		_sleepTime = 0;
	}

	/**
	 * Returns whether the rigid body is sleeping.
	 */
	public boolean isSleeping() {
		return _sleeping;
	}

	/**
	 * Returns how long the rigid body is stopping moving. This returns `0` if the body
	 * has already slept.
	 */
	public double getSleepTime() {
		return _sleepTime;
	}

	/**
	 * Sets the rigid body's auto sleep flag.
	 *
	 * If auto sleep is enabled, the rigid body will automatically sleep when needed.
	 */
	public void setAutoSleep(boolean autoSleepEnabled) {
		_autoSleep = autoSleepEnabled;
		wakeUp();
	}

	/**
	 * Returns the linear damping.
	 */
	public double getLinearDamping() {
		return _linearDamping;
	}

	/**
	 * Sets the linear damping to `damping`.
	 */
	public void setLinearDamping(double damping) {
		_linearDamping = damping;
	}

	/**
	 * Returns the angular damping.
	 */
	public double getAngularDamping() {
		return _angularDamping;
	}

	/**
	 * Sets the angular damping to `damping`.
	 */
	public void setAngularDamping(double damping) {
		_angularDamping = damping;
	}

	public void updateMesh() {
		
	}
//	
//	SceneNode3D mesh;
//	/**
//	 * Returns the previous rigid body in the world.
//	 *
//	 * If the previous one does not exist, `null` will be returned.
//	 */
//	public RigidBody getPrev() {
//		return _prev;
//	}
//
//	/**
//	 * Returns the next rigid body in the world.
//	 *
//	 * If the next one does not exist, `null` will be returned.
//	 */
//	public RigidBody getNext() {
//		return _next;
//	}
//	
//	
//	  //---------------------------------------------
//    // AUTO UPDATE SceneNode Model MESH
//    //---------------------------------------------
//
//    public void connectMesh (SceneNode3D mesh ) {
//
//        this.mesh = mesh;
//        this.updateMesh();
//
//    }
//
//	public SceneNode3D getMesh() {
//		return mesh;
//	}
//	
//	
//	Quaternion q_=new Quaternion();
//   public void updateMesh(){
//
//      
//        if( this.getMesh() == null ) return;
//        
//        this.getMesh().setPosition((float)_transform._position.x,(float)_transform._position.y,(float)_transform._position.z );
//       
//        Vec3 v=_transform._rotation.toEulerXyz();
//        //q_.set(q.x,q.y,q.z,q.w);
//        this.getMesh().setRotation((float)v.x,(float)v.y,(float)v.z);
//    
//    }
   
}