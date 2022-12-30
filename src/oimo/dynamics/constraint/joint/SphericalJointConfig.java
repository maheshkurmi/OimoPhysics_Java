package oimo.dynamics.constraint.joint;

import oimo.common.Vec3;
import oimo.dynamics.rigidbody.RigidBody;

/**
 * A spherical joint config is used for constructions of spherical joints.
 */
public class SphericalJointConfig extends JointConfig {
	/**
	 * The spring and damper setting of the joint.
	 */
	public SpringDamper springDamper;

	/**
	 * Default constructor.
	 */
	public SphericalJointConfig() {
		super();
		springDamper = new SpringDamper();
	}

	/**
	 * Sets rigid bodies, local anchors from the world anchor `worldAnchor`, and returns `this`.
	 */
	public SphericalJointConfig init(RigidBody rigidBody1, RigidBody rigidBody2, Vec3 worldAnchor) {
		_init(rigidBody1, rigidBody2, worldAnchor);
		return this;
	}

}