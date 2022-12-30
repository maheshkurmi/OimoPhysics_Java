package oimo.dynamics.constraint.info;

import oimo.common.Vec3;
import oimo.common.M;

/**
 * The row of a Jacobian matrix.
 */
public class JacobianRow {
	public Vec3 lin1;
	public Vec3 lin2;
	public Vec3 ang1;
	public Vec3 ang2;

	final static int BIT_LINEAR_SET = 1;
	final static int BIT_ANGULAR_SET = 2;
	int flag; // sparsity flag

	public JacobianRow() {
		lin1 = new Vec3();
		lin2 = new Vec3();
		ang1 = new Vec3();
		ang2 = new Vec3();
		flag = 0;
	}

	public void clear() {
		lin1.zero();
		lin1.zero();
		ang1.zero();
		ang2.zero();
	}

	public void updateSparsity() {
		flag = 0;
		if (!M.vec3_isZero(lin1) || !M.vec3_isZero(lin2)) {
			flag |= BIT_LINEAR_SET;
		}
		if (!M.vec3_isZero(ang1) || !M.vec3_isZero(ang2)) {
			flag |= BIT_ANGULAR_SET;
		}
	}

	public boolean isLinearSet() {
		return (flag & BIT_LINEAR_SET) != 0;
	}

	public boolean isAngularSet() {
		return (flag & BIT_ANGULAR_SET) != 0;
	}

}
