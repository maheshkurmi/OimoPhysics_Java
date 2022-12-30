package oimo.dynamics.constraint.joint;

import oimo.common.MathUtil;
import oimo.common.Vec3;
import oimo.dynamics.rigidbody.RigidBody;

/**
 * A ragdoll joint config is used for constructions of ragdoll joints.
 */
public class RagdollJointConfig extends JointConfig {
	/**
	 * The first body's local twist axis.
	 */
	public Vec3 localTwistAxis1;

	/**
	 * The second body's local twist axis.
	 */
	public Vec3 localTwistAxis2;

	/**
	 * The first body's local swing axis.
	 *
	 * The second swing axis is also attached to the first body. It is perpendicular to the first swing
	 * axis, and is automatically computed when the joint is created.
	 */
	public Vec3 localSwingAxis1;

	/**
	 * The rotational spring and damper along the twist axis of the joint.
	 */
	public SpringDamper twistSpringDamper;

	/**
	 * The rotational limit and motor along the twist axis of the joint.
	 */
	public RotationalLimitMotor twistLimitMotor;

	/**
	 * The rotational spring and damper along the swing axis of the joint.
	 */
	public SpringDamper swingSpringDamper;

	/**
	 * The max angle of rotation along the first swing axis.
	 * This value must be positive.
	 */
	public float maxSwingAngle1;

	/**
	 * The max angle of rotation along the second swing axis.
	 * This value must be positive.
	 */
	public float maxSwingAngle2;

	/**
	 * Default constructor.
	 */
	public RagdollJointConfig() {
		super();
		localTwistAxis1 = new Vec3(1, 0, 0);
		localTwistAxis2 = new Vec3(1, 0, 0);
		localSwingAxis1 = new Vec3(0, 1, 0);
		twistSpringDamper = new SpringDamper();
		swingSpringDamper = new SpringDamper();
		twistLimitMotor = new RotationalLimitMotor();
		maxSwingAngle1 = MathUtil.PI;
		maxSwingAngle2 = MathUtil.PI;
	}

	/**
	 * Sets rigid bodies, local anchors from the world anchor `worldAnchor`, local twist axes
	 * from the world twist axis `worldTwistAxis`, local swing axis from the world swing axis
	 * `worldSwingAxis`, and returns `this`.
	 */
	public  RagdollJointConfig init(RigidBody rigidBody1, RigidBody rigidBody2,Vec3 worldAnchor,Vec3 worldTwistAxis, Vec3 worldSwingAxis) {
		_init(rigidBody1, rigidBody2, worldAnchor);
		rigidBody1.getLocalVectorTo(worldTwistAxis, localTwistAxis1);
		rigidBody2.getLocalVectorTo(worldTwistAxis, localTwistAxis2);
		rigidBody1.getLocalVectorTo(worldSwingAxis, localSwingAxis1);
		return this;
	}

}