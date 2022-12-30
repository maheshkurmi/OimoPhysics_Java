package oimo.dynamics.constraint.solver;

/**
 * The list of the constraint solvers.
 */

public class ConstraintSolverType {
	public static final int _ITERATIVE = 0;
	public static final int _DIRECT = 1;

	/**
	 * Iterative constraint solver. Fast and stable enough for common usages.
	 */
	public static final int ITERATIVE = _ITERATIVE;

	/**
	 * Direct constraint solver. Very stable but not suitable for a situation where fast
	 * computation is required.
	 */
	public static final int DIRECT = _DIRECT;
}
