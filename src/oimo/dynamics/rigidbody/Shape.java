package oimo.dynamics.rigidbody;
import oimo.collision.*;
import oimo.collision.broadphase.*;
import oimo.collision.geometry.*;
import oimo.common.Transform;
import oimo.common.Vec3;
import oimo.dynamics.callback.ContactCallback;
import oimo.common.M;

/**
 * A shape is a component of a rigid body. It attaches a collision geometry to the parent rigid body
 * with some physical properties such as coefficients of friction and restitution. The collision
 * geometry can locally be transformed relative to the parent rigid body's center of gravity.
 */
public class Shape {
	public int _id;

	public Shape _prev;
	public Shape _next;
	public RigidBody _rigidBody;
	public Geometry _geom;

	public Transform _localTransform;
	public Transform  _ptransform;
	public Transform _transform;

	public float _restitution;
	public float _friction;
	public float _density;

	public Aabb _aabb;

	public Proxy _proxy;

	public int _collisionGroup;
	public int _collisionMask;

	public ContactCallback _contactCallback;

	Vec3 displacement;

	/**
	 * Extra field that users can use for their own purposes.
	 */
	public Object userData;

	/**
	 * Creates a new shape by configuration `config`.
	 */
	public Shape(ShapeConfig config) {
		_id = -1;

		_localTransform = new Transform();
		_ptransform = new Transform();
		_transform = new Transform();

		M.vec3_fromVec3(_localTransform._position, config.position);
		M.mat3_fromMat3(_localTransform._rotation, config.rotation);
		M.transform_assign(_ptransform, _localTransform);
		M.transform_assign(_transform, _localTransform);
		_restitution = config.restitution;
		_friction = config.friction;
		_density = config.density;
		_geom = config.geometry;
		_collisionGroup = config.collisionGroup;
		_collisionMask = config.collisionMask;
		_contactCallback = config.contactCallback;

		_aabb = new Aabb();

		_proxy = null;
		displacement = new Vec3();
	}

	// --- internal ---

	 protected void _sync(Transform tf1, Transform tf2) {
		M.transform_mul(_ptransform, _localTransform, tf1);
		M.transform_mul(_transform, _localTransform, tf2);

		
		_geom._computeAabb(_aabb, _ptransform);
		Vec3 min=_aabb._min.clone();
		Vec3 max=_aabb._max.clone();
		_geom._computeAabb(_aabb, _transform);
		M.vec3_min(_aabb._min, min, _aabb._min);
		M.vec3_max(_aabb._max, max, _aabb._max);

		
		if (_proxy != null) {
			Vec3 d=new Vec3();
			M.vec3_sub(d, _transform._position, _ptransform._position);
			M.vec3_toVec3(displacement, d);
			_rigidBody._world._broadPhase.moveProxy(_proxy, _aabb, displacement);
		}
	}

	// --- public ---

	/**
	 * Returns the coefficient of friction.
	 */
	public float getFriction() {
		return _friction;
	}

	/**
	 * Sets the coefficient of friction to `friction`.
	 */
	public void setFriction(float friction) {
		_friction = friction;
	}

	/**
	 * Returns the coefficient of restitution.
	 */
	public float getRestitution() {
		return _restitution;
	}

	/**
	 * Sets the coefficient of restitution to `restitution`.
	 */
	public void setRestitution(float restitution) {
		_restitution = restitution;
	}

	/**
	 * Returns the density of the shape.
	 */
	public float getDensity() {
		return _density;
	}

	/**
	 * Sets the density of the shape to `density`.
	 *
	 * This affects the parent rigid body's mass data.
	 */
	public void setDensity(float density) {
		_density = density;
		if (_rigidBody != null) {
			_rigidBody._shapeModified();
		}
	}
	
	/**
	 * Returns the copy of transform of the shape relative to the parent rigid body's transform.
	 */
	public Transform getLocalTransform() {
		return _localTransform.clone();
	}

	/**
	 * Sets `transform` to the transform of the shape relative to the parent rigid body's
	 * transform.
	 *
	 * This does not create a new instance of `Transform`.
	 */
	public void getLocalTransformTo(Transform transform) {
		transform.copyFrom(_localTransform);
	}

	/**
	 * Returns the copy of world transform of the shape.
	 */
	public Transform getTransform() {
		return _transform.clone();
	}

	/**
	 * Sets `transform` to the world transform of the shape.
	 *
	 * This does not create a new instance of `Transform`.
	 */
	public void getTransformTo(Transform transform) {
		transform.copyFrom(_transform);
	}

	/**
	 * Sets the shape's transform to `transform` relative to the parent rigid body's transform.
	 *
	 * This affects the parent rigid body's mass data.
	 */
	public void setLocalTransform(Transform transform) {
		_localTransform.copyFrom(transform);
		if (_rigidBody != null) {
			_rigidBody._shapeModified();
		}
	}

	

	/**
	 * Returns the Acopy of ABB of the shape. The AABB may be incorrect if the shape doesn't have a
	 * parent rigid body.
	 */
	public Aabb  getAabb() {
		return _aabb.clone();
	}

	/**
	 * Sets `aabb` to the AABB of the shape. The AABB may be incorrect if the shape doesn't have a
	 * parent rigid body.
	 *
	 * This does not create a new instance of `AABB`.
	 */
	public void getAabbTo(Aabb aabb) {
		aabb.copyFrom(_aabb);
	}

	/**
	 * Returns the colision geometry of the shape.
	 */
	public Geometry getGeometry() {
		return _geom;
	}

	/**
	 * Returns the parent rigid body. This returns `null` if the shape doesn't have a parent
	 * rigid body.
	 */
	public RigidBody getRigidBody() {
		return _rigidBody;
	}

	/**
	 * Returns the collision group bits the shape belongs to.
	 */
	public int getCollisionGroup() {
		return _collisionGroup;
	}

	/**
	 * Sets the shape's collision group bits to `collisionGroup`.
	 */
	public void setCollisionGroup(int collisionGroup) {
		_collisionGroup = collisionGroup;
	}

	/**
	 * Returns the collision mask bits of the shape.
	 */
	public int getCollisionMask() {
		return _collisionMask;
	}

	/**
	 * Sets the shape's collision mask bits to `collisionMask`.
	 */
	public void setCollisionMask(int collisionMask) {
		_collisionMask = collisionMask;
	}

	/**
	 * Returns the contact callback of the shape.
	 */
	public ContactCallback getContactCallback() {
		return _contactCallback;
	}

	/**
	 * Sets the contact callback of the shape to `callback`.
	 */
	public void setContactCallback(ContactCallback callback) {
		_contactCallback = callback;
	}

	/**
	 * Returns the previous shape in the rigid body.
	 *
	 * If the previous one does not exist, `null` will be returned.
	 */
	public Shape getPrev() {
		return _prev;
	}

	/**
	 * Returns the next shape in the rigid body.
	 *
	 * If the next one does not exist, `null` will be returned.
	 */
	public Shape getNext() {
		return _next;
	}

}