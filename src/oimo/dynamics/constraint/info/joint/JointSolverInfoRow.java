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
	public float rhs;

	/** Used for velocity solver. */
	public float cfm;

	/** Used for both velocity and position solver. */
	public float minImpulse;
	/** Used for both velocity and position solver. */
	public float maxImpulse;

	/** Used for velocity solver. */
	public float motorSpeed;
	/** Used for velocity solver. */
	public float motorMaxImpulse;

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

	 public void equalLimit(float rhs, float cfm) {
		this.rhs = rhs;
		this.cfm = cfm;
		minImpulse = MathUtil.NEGATIVE_INFINITY;
		maxImpulse = MathUtil.POSITIVE_INFINITY;
	}

	 public void motor(float speed,float maxImpulse) {
		motorSpeed = speed;
		motorMaxImpulse = maxImpulse;
	}
}
