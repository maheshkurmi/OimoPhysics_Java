package oimo.dynamics.constraint.joint;

import oimo.dynamics.rigidbody.RigidBody;
import oimo.dynamics.rigidbody.*;

/**
 * A joint link is used to build a constraint graph for clustering rigid bodies.
 * In a constraint graph, rigid bodies are nodes and constraints are edges. See
 * also `ContactLink`.
 */
public class JointLink {
	public JointLink _prev;
	public JointLink _next;
	public Joint _joint;
	public RigidBody _other;

	public JointLink(Joint joint) {
		_joint = joint;
	}

	/**
	 * Returns the contact the rigid body is attached to.
	 */
	public Joint getContact() {
		return _joint;
	}

	/**
	 * Returns the other rigid body attached to the constraint. This provides a
	 * quick access from a rigid body to the other one attached to the constraint.
	 */
	public RigidBody getOther() {
		return _other;
	}

	/**
	 * Returns the previous joint link in the rigid body.
	 *
	 * If the previous one does not exist, `null` will be returned.
	 */
	public JointLink getPrev() {
		return _prev;
	}

	/**
	 * Returns the next joint link in the rigid body.
	 *
	 * If the previous one does not exist, `null` will be returned.
	 */
	public JointLink getNext() {
		return _next;
	}

}
