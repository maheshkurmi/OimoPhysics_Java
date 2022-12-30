package oimo.dynamics.rigidbody;
import oimo.collision.geometry.Geometry;
import oimo.common.Mat3;
import oimo.common.Setting;
import oimo.common.Vec3;
import oimo.dynamics.callback.ContactCallback;

/**
 * A shape configuration is used for construction of shapes. An instance of
 * this class can safely be reused as a shape will not have any references
 * of a field of this class.
 */
class ShapeConfig {
	/**
	 * The shape's local position relative to the parent rigid body's origin.
	 */
	public Vec3 position;

	/**
	 * The shape's local rotation matrix relative to the parent rigid body's
	 * rotation.
	 */
	public Mat3 rotation;

	/**
	 * The coefficient of friction of the shape.
	 */
	public float friction;

	/**
	 * The coefficient of restitution of the shape.
	 */
	public float restitution;

	/**
	 * The density of the shape, usually in Kg/m^3.
	 */
	public float density;

	/**
	 * The collision geometry of the shape.
	 */
	public Geometry geometry;

	/**
	 * The collision group bits the shape belongs to. This is used for collision
	 * filtering.
	 *
	 * Two shapes `shape1` and `shape2` will collide only if both
	 * `shape1.collisionGroup & shape2.collisionMask` and
	 * `shape2.collisionGroup & shape1.collisionMask` are not zero.
	 */
	public int collisionGroup;

	/**
	 * The collision mask bits of the shape. This is used for collision
	 * filtering.
	 *
	 * Two shapes `shape1` and `shape2` will collide only if both
	 * `shape1.collisionGroup & shape2.collisionMask` and
	 * `shape2.collisionGroup & shape1.collisionMask` are not zero.
	 */
	public int collisionMask;

	/**
	 * The contact callback of the shape. The callback methods are called
	 * when contact events the shape is involved occurred.
	 */
	public ContactCallback contactCallback;

	/**
	 * Default constructor.
	 */
	public ShapeConfig() {
		position = new Vec3();
		rotation = new Mat3();
		friction = Setting.defaultFriction;
		restitution = Setting.defaultRestitution;
		density = Setting.defaultDensity;
		collisionGroup = Setting.defaultCollisionGroup;
		collisionMask = Setting.defaultCollisionMask;
		geometry = null;
		contactCallback = null;
	}
}