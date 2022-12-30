package oimo.dynamics.constraint.info.contact;
import oimo.common.Setting;
import oimo.dynamics.rigidbody.RigidBody;

/**
 * Internal class.
 */
public class ContactSolverInfo {
	public RigidBody b1;
	public RigidBody b2;

	public int numRows;
	public ContactSolverInfoRow[] rows;

	public ContactSolverInfo() {
		b1 = null;
		b2 = null;

		numRows = 0;
		rows = new ContactSolverInfoRow[Setting.maxManifoldPoints];
		for (int i=0;i<rows.length;i++) {
			rows[i] = new ContactSolverInfoRow();
		}
	}

}
