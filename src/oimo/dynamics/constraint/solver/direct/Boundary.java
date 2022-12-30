package oimo.dynamics.constraint.solver.direct;
import oimo.common.Setting;
import oimo.common.Vec3;
import oimo.dynamics.constraint.info.joint.JointSolverInfo;
import oimo.dynamics.constraint.info.joint.JointSolverInfoRow;

/**
 * Internal class
 */
public class Boundary {
	// number of impulses which are at lower or upper limits
	public int numBounded;
	// indices of impulses which are at lower or upper limits
	public int[] iBounded;
	// -1: at lower, 1: at upper
	public int[] signs;

	// number of impulses which are not at limits
	public int numUnbounded;
	// indices of impulses which are not at lower or upper limits
	public int[] iUnbounded;

	// used for impulse computation:
	//     impulse = massMatrix * b
	float[] b;

	// the id of mass matrix
	int matrixId;

	public Boundary(int maxRows) {
		iBounded = new int[maxRows];
		iUnbounded = new int[maxRows];
		signs =new int[maxRows];
		b = new float[maxRows];
		numBounded = 0;
		numUnbounded = 0;
		matrixId = 0;
	}

	public void init(BoundaryBuildInfo buildInfo) {
		// copy bounded part
		numBounded = buildInfo.numBounded;
		for (int i=0;i<numBounded;i++) {
			iBounded[i] = buildInfo.iBounded[i];
			signs[i] = buildInfo.signs[i];
		}

		// copy unbounded part and compute matrix id
		numUnbounded = buildInfo.numUnbounded;
		matrixId = 0;
		for (int i=0; i<numUnbounded;i++) {
			int idx = buildInfo.iUnbounded[i];
			iUnbounded[i] = idx;
			matrixId |= 1 << idx;
		}
	}

	public boolean computeImpulses(JointSolverInfo info, MassMatrix mass, float[] relVels, float[] impulses, float[] dImpulses, float impulseFactor,boolean noCheck) {
		// b = rhs - relV - cfm * totalImpulse
		for (int i=0;i<numUnbounded;i++) {
			int idx = iUnbounded[i];
			JointSolverInfoRow row = info.rows[idx];
			float relVel = relVels[idx];
			b[idx] = row.rhs * impulseFactor - relVel - row.cfm * impulses[idx];
		}

		// bounded part
		//	var invMassWithoutCfm:Vector<Vector<Float>> = mass._invMassWithoutCfm;
		float[][] invMassWithoutCfm = mass._invMassWithoutCfm;
		
		for (int i=0;i<numBounded;i++) {
			int idx = iBounded[i];
			int sign = signs[i];
			JointSolverInfoRow row = info.rows[idx];
			float oldImpulse = impulses[idx];
			float impulse = sign < 0 ? row.minImpulse : sign > 0 ? row.maxImpulse : 0;
			float dImpulse = impulse - oldImpulse;
			dImpulses[idx] = dImpulse;

			// update relative velocity for unbounded part
			if (dImpulse != 0) {
				for (int j=0;j<numUnbounded;j++) {
					int idx2 = iUnbounded[j];
					// delta of idx2'th relative velocity
					float dRelVel = invMassWithoutCfm[idx][idx2] * dImpulse;
					b[idx2] -= dRelVel;
				}
			}
		}

		float[][] massMatrix = mass.getSubmatrix(iUnbounded, numUnbounded);
		boolean ok = true;

		// unbounded part
		for (int i=0;i<numUnbounded;i++) {
			int idx = iUnbounded[i];
			JointSolverInfoRow row = info.rows[idx];
			float oldImpulse = impulses[idx];

			float impulse = oldImpulse;

			// compute unbounded impulse (massMatrix * b)
			for (int j=0;j<numUnbounded;j++) {
				int idx2 = iUnbounded[j];
				impulse += b[idx2] * massMatrix[i][j];
			}

			if (impulse < row.minImpulse - Setting.directMlcpSolverEps || impulse > row.maxImpulse + Setting.directMlcpSolverEps) {
				// we assumed that `impulse` holds `minImpulse <= impulse <= maxImpulse`, but actually
				// not. This boundary for the MLCP is not the answer of it.
				ok = false;
				break;
			}

			dImpulses[idx] = impulse - oldImpulse;
		}

		if (noCheck) return true;

		if (!ok) return false;

		// check if the impulses fulfill complementarity constraints
		for (int i=0;i<numBounded;i++) {
			int idx = iBounded[i];
			JointSolverInfoRow row = info.rows[idx];
			int sign = signs[i];
			float error = 0;

			float newImpulse = impulses[idx] + dImpulses[idx];
			float relVel = relVels[idx];

			// `relVel` is the relative velocity BEFORE we apply the delta impulses,
			// so we should update `relVel` so that it shows the relative velocity
			// AFTER we applied the delta impulse.
			for (int j=0;j<info.numRows;j++) {
				relVel += invMassWithoutCfm[idx][j] * dImpulses[j];
			}

			error = row.rhs * impulseFactor - relVel - row.cfm * newImpulse;
			// complementarity constraint: check if we added too large impulses
			//     sign < 0 => error <= 0
			//     sign > 0 => error >= 0
			if (sign < 0 && error > Setting.directMlcpSolverEps || sign > 0 && error < -Setting.directMlcpSolverEps) {
				ok = false; // the result is not feasible
				break;
			}
		}

		return ok;
	}

	/*
	public function dump():Void {
		trace("\nboundary info --------------------------------------");
		trace("bounded indices:");
		for (i in 0...numBounded) {
			var idx:Int = iBounded[i];
			trace("  " + idx + ", sign: " + signs[i]);
		}
		trace("unbounded indices:");
		for (i in 0...numUnbounded) {
			var idx:Int = iUnbounded[i];
			trace("  " + idx);
		}
		trace("mass matrix index: " + matrixId);
		trace("boundary info end ----------------------------------\n");
	}
	*/

}
