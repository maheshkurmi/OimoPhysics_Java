package oimo.dynamics;
import oimo.dynamics.rigidbody.*;

/**
 * A contact link is used to build a constraint graph for clustering rigid bodies.
 * In a constraint graph, rigid bodies are nodes and constraints are edges.
 * See also `JointLink`.
 */
public class ContactLink {
	public ContactLink _prev;
	public ContactLink _next;
	public Contact _contact;
	public RigidBody _other;

	public ContactLink() {
		_prev = null;
		_next = null;
		_contact = null;
		_other = null;
	}

	/**
	 * Returns the contact of the link.
	 */
	public Contact getContact() {
		return _contact;
	}

	/**
	 * Returns the other rigid body of the link. This provides a quick access from a
	 * rigid body to the other one of the contact.
	 */
	public RigidBody getOther() {
		return _other;
	}

	/**
	 * Returns the previous contact link in the rigid body.
	 *
	 * If the previous one does not exist, `null` will be returned.
	 */
	public ContactLink getPrev() {
		return _prev;
	}

	/**
	 * Returns the next contact link in the rigid body.
	 *
	 * If the next one does not exist, `null` will be returned.
	 */
	public ContactLink getNext() {
		return _next;
	}
}
