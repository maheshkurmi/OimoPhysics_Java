package oimo.dynamics.constraint.joint;

import oimo.common.Setting;
import oimo.common.Vec3;
import oimo.dynamics.rigidbody.*;

/**
 * A joint configuration is used for constructions of various joints. An
 * instance of any kind of the joint configurations can safely be reused.
 */
public class JointConfig {
	/**
	 * The first rigid body attached to the joint.
	 */
	public RigidBody rigidBody1;

	/**
	 * The second rigid body attached to the joint.
	 */
	public RigidBody rigidBody2;

	/**
	 * The local position of the first rigid body's anchor point.
	 */
	public Vec3 localAnchor1;

	/**
	 * The local position of the second rigid body's anchor point.
	 */
	public Vec3 localAnchor2;

	/**
	 * Whether to allow the connected rigid bodies to collide each other.
	 */
	public boolean allowCollision;

	/**
	 * The type of the constraint solver for the joint.
	 *
	 * See `ConstraintSolverType` for details.
	 */
	public int solverType;

	/**
	 * The type of the position correction algorithm for the joint.
	 *
	 * See `PositionCorrectionAlgorithm` for details.
	 */
	public int positionCorrectionAlgorithm;

	/**
	 * The joint will be destroyed when magnitude of the constraint force exceeds
	 * the value.
	 *
	 * Set `0` for unbreakable joints.
	 */
	public double breakForce;

	/**
	 * The joint will be destroyed when magnitude of the constraint torque exceeds
	 * the value.
	 *
	 * Set `0` for unbreakable joints.
	 */
	public double breakTorque;

	public JointConfig() {
		rigidBody1 = null;
		rigidBody2 = null;
		localAnchor1 = new Vec3();
		localAnchor2 = new Vec3();
		allowCollision = false;
		solverType = Setting.defaultJointConstraintSolverType;
		positionCorrectionAlgorithm = Setting.defaultJointPositionCorrectionAlgorithm;
		breakForce = 0;
		breakTorque = 0;
	}

	// --- private ---

	public void _init(RigidBody rb1, RigidBody rb2, Vec3 worldAnchor) {
		rigidBody1 = rb1;
		rigidBody2 = rb2;
		rigidBody1.getLocalPointTo(worldAnchor, localAnchor1);
		rigidBody2.getLocalPointTo(worldAnchor, localAnchor2);
	}

}
