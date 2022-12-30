package oimo.dynamics.constraint.solver.direct;
import oimo.common.MathUtil;
import oimo.dynamics.constraint.info.joint.JointSolverInfo;
import oimo.dynamics.constraint.info.joint.JointSolverInfoRow;

/**
 * Internal class.
 */
public class BoundaryBuilder {
	public int numBoundaries;
	public Boundary[] boundaries;

	int maxRows;
	BoundaryBuildInfo bbInfo;

	public BoundaryBuilder(int maxRows) {
		// TODO: O(2^N) is inefficient?
		this.maxRows = maxRows;
		numBoundaries = 0;
		boundaries = new Boundary[1 << maxRows];

		bbInfo = new BoundaryBuildInfo(maxRows);
	}

	public void buildBoundariesRecursive(JointSolverInfo info, int i) {
		if (i == info.numRows) {
			addBoundary().init(bbInfo);
			return;
		}

		JointSolverInfoRow row = info.rows[i];
		boolean lowerLimitEnabled = row.minImpulse > MathUtil.NEGATIVE_INFINITY;
		boolean upperLimitEnabled = row.maxImpulse < MathUtil.POSITIVE_INFINITY;

		boolean disabled = row.minImpulse == 0 && row.maxImpulse == 0;

		if (disabled) {
			// try inactive case
			bbInfo.pushBounded(i, 0);
			buildBoundariesRecursive(info, i + 1);
			bbInfo.popBounded();
			return;
		}

		// try unbounded case
		bbInfo.pushUnbounded(i);
		buildBoundariesRecursive(info, i + 1);
		bbInfo.popUnbounded();

		if (lowerLimitEnabled) {
			// try lower bounded case
			bbInfo.pushBounded(i, -1);
			buildBoundariesRecursive(info, i + 1);
			bbInfo.popBounded();
		}
		if (upperLimitEnabled) {
			// try upper bounded case
			bbInfo.pushBounded(i, 1);
			buildBoundariesRecursive(info, i + 1);
			bbInfo.popBounded();
		}
	}

	public Boundary addBoundary() {
		if (boundaries[numBoundaries] == null) {
			boundaries[numBoundaries] = new Boundary(maxRows);
		}
		return boundaries[numBoundaries++];
	}

	public void buildBoundaries(JointSolverInfo info) {
		numBoundaries = 0;

		bbInfo.clear();
		buildBoundariesRecursive(info, 0);
	}

}
