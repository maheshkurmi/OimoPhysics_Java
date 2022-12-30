package oimo.dynamics.constraint.joint;

import oimo.common.Vec3;
import oimo.dynamics.rigidbody.RigidBody;

/**
 * A universal joint config is used for constructions of universal joints.
 */
public class UniversalJointConfig extends JointConfig {
	/**
	 * The first body's local constraint axis.
	 */
	public Vec3 localAxis1;

	/**
	 * The second body's local constraint axis.
	 */
	public Vec3 localAxis2;

	/**
	 * The rotational spring and damper along the first body's constraint axis.
	 */
	public SpringDamper springDamper1;

	/**
	 * The rotational spring and damper along the second body's constraint axis.
	 */
	public SpringDamper springDamper2;

	/**
	 * The rotational limit and motor along the first body's constraint axis.
	 */
	public RotationalLimitMotor limitMotor1;

	/**
	 * The rotational limit and motor along the second body's constraint axis.
	 */
	public RotationalLimitMotor limitMotor2;

	/**
	 * Default constructor.
	 */
	public UniversalJointConfig() {
		super();
		localAxis1 = new Vec3(1, 0, 0);
		localAxis2 = new Vec3(1, 0, 0);
		springDamper1 = new SpringDamper();
		springDamper2 = new SpringDamper();
		limitMotor1 = new RotationalLimitMotor();
		limitMotor2 = new RotationalLimitMotor();
	}

	/**
	 * Sets rigid bodies, local anchors from the world anchor `worldAnchor`, local axes
	 * from the world axes `worldAxis1` and `worldAxis2`, and returns `this`.
	 */
	public UniversalJointConfig init(RigidBody rigidBody1, RigidBody rigidBody2, Vec3 worldAnchor, Vec3 worldAxis1, Vec3 worldAxis2) {
		_init(rigidBody1, rigidBody2, worldAnchor);
		rigidBody1.getLocalVectorTo(worldAxis1, localAxis1);
		rigidBody2.getLocalVectorTo(worldAxis2, localAxis2);
		return this;
	}

}
