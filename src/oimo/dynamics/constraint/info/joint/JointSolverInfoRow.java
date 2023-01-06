package oimo.dynamics.constraint.info.joint;
import oimo.common.MathUtil;
import oimo.dynamics.constraint.info.JacobianRow;
import oimo.dynamics.constraint.joint.JointImpulse;

/**
 * Internal class.
 */
public class JointSolverInfoRow {
	/** Used for both velocity and position solver. */
	public JacobianRow jacobian;

	/** Used for both velocity and position solver. */
	public double rhs;

	/** Used for velocity solver. */
	public double cfm;

	/** Used for both velocity and position solver. */
	public double minImpulse;
	/** Used for both velocity and position solver. */
	public double maxImpulse;

	/** Used for velocity solver. */
	public double motorSpeed;
	/** Used for velocity solver. */
	public double motorMaxImpulse;

	/** Used for both velocity and position solver. */
	public JointImpulse impulse;

	public JointSolverInfoRow() {
		jacobian = new JacobianRow();
		rhs = 0;
		cfm = 0;
		minImpulse = 0;
		maxImpulse = 0;
		motorSpeed = 0;
		motorMaxImpulse = 0;
		impulse = null;
	}

	public void clear() {
		jacobian.clear();
		rhs = 0;
		cfm = 0;
		minImpulse = 0;
		maxImpulse = 0;
		motorSpeed = 0;
		motorMaxImpulse = 0;
		impulse = null;
	}

	 public void equalLimit(double rhs, double cfm) {
		this.rhs = rhs;
		this.cfm = cfm;
		minImpulse = MathUtil.NEGATIVE_INFINITY;
		maxImpulse = MathUtil.POSITIVE_INFINITY;
	}

	 public void motor(double speed,double maxImpulse) {
		motorSpeed = speed;
		motorMaxImpulse = maxImpulse;
	}
}
