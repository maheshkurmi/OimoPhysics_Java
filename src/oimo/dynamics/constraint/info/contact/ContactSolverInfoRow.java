package oimo.dynamics.constraint.info.contact;

import oimo.dynamics.constraint.contact.ContactImpulse;
import oimo.dynamics.constraint.info.JacobianRow;

/**
 * Internal class.
 */
public class ContactSolverInfoRow {
	/** Used for both velocity and position solver. */
	public JacobianRow jacobianN;

	/** Used for velocity solver. */
	public JacobianRow jacobianT;

	/** Used for velocity solver. */
	public JacobianRow jacobianB;

	/** Used for both velocity and position solver. */
	public double rhs;

	/** Used for velocity solver. */
	public double cfm;

	/** Used for velocity solver. */
	public double friction;

	/** Used for both velocity and position solver. */
	public ContactImpulse impulse;

	public ContactSolverInfoRow() {
		jacobianN = new JacobianRow();
		jacobianT = new JacobianRow();
		jacobianB = new JacobianRow();
		rhs = 0;
		cfm = 0;
		friction = 0;
		impulse = null;
	}

	public void clear() {
		jacobianN.clear();
		jacobianT.clear();
		jacobianB.clear();
		rhs = 0;
		cfm = 0;
		friction = 0;
		impulse = null;
	}

}
