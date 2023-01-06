package oimo.dynamics.rigidbody;
import oimo.common.Mat3;
import oimo.common.Vec3;

/**
 * A rigid body configuration is used for constructions of rigid bodies. An instance of this
 * class can safely be reused, as a rigid body will not have any references to a field of
 * this class.
 */
public class RigidBodyConfig {
	/**
	 * The world position of the rigid body's center of gravity.
	 */
	public Vec3 position;

	/**
	 * The rotation matrix of the rigid body.
	 */
	public Mat3 rotation;

	/**
	 * The initial value of the rigid body's linear velocity.
	 */
	public Vec3 linearVelocity;

	/**
	 * The initial value of the rigid body's angular velocity.
	 */
	public Vec3 angularVelocity;

	/**
	 * The rigid body's motion type. See `RigidBodyType` for details.
	 */
	public int type;

	/**
	 * Whether to automatically sleep the rigid body when it stops moving
	 * for a certain period of time, namely `Setting.sleepingTimeThreshold`.
	 */
	public boolean autoSleep;

	/**
	 * The damping coefficient of the linear velocity. Set positive values to
	 * gradually reduce the linear velocity.
	 */
	public double linearDamping;

	/**
	 * The damping coefficient of the angular velocity. Set positive values to
	 * gradually reduce the angular velocity.
	 */
	public double angularDamping;

	/**
	 * Default constructor.
	 */
	public RigidBodyConfig() {
		position = new Vec3();
		rotation = new Mat3();
		linearVelocity = new Vec3();
		angularVelocity = new Vec3();
		type = RigidBodyType._DYNAMIC;
		autoSleep = true;
		linearDamping = 0;
		angularDamping = 0;
	}
}
