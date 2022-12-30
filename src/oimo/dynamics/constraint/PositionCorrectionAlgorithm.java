package oimo.dynamics.constraint;

/**
 * The list of the algorithms for position corretion.
 */
public class PositionCorrectionAlgorithm {
	public static final int _BAUMGARTE = 0;
	public static final int _SPLIT_IMPULSE = 1;
	public static final int _NGS = 2;

	/**
	 * Baumgarte stabilizaiton. Fastest but introduces extra energy.
	 */
	public static  final int  BAUMGARTE = _BAUMGARTE;

	/**
	 * Use split impulse and pseudo velocity. Fast enough and does not introduce extra
	 * energy, but somewhat unstable, especially for joints.
	 */
	public static  final int  SPLIT_IMPULSE= _SPLIT_IMPULSE;

	/**
	 * Nonlinear Gauss-Seidel method. Slow but stable.
	 */
	public static final int  NGS = _NGS;
}
