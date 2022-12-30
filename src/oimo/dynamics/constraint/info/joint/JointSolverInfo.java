package oimo.dynamics.constraint.info.joint;

import oimo.common.Setting;
import oimo.dynamics.constraint.joint.JointImpulse;
import oimo.dynamics.rigidbody.RigidBody;

/**
 * Internal class.
 */
public class JointSolverInfo {
	public RigidBody b1;
	public RigidBody b2;

	public int numRows;
	public JointSolverInfoRow[] rows;

	public JointSolverInfo() {
		b1 = null;
		b2 = null;

		numRows = 0;
		rows = new JointSolverInfoRow[Setting.maxJacobianRows];
		for (int i = 0; i < rows.length; i++) {
			rows[i] = new JointSolverInfoRow();
		}
	}

	public JointSolverInfoRow addRow(JointImpulse impulse) {
		JointSolverInfoRow row = rows[numRows++];
		row.clear();
		row.impulse = impulse;
		return row;
	}

}
