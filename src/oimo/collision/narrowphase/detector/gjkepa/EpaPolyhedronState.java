package oimo.collision.narrowphase.detector.gjkepa;

/**
 * Internal class.
 */
public class EpaPolyhedronState {
	public static final int OK = 0;
	public static final int  INVALID_TRIANGLE = 1;
	public static final int  NO_ADJACENT_PAIR_INDEX = 2;
	public static final int  NO_ADJACENT_TRIANGLE = 3;
	public static final int  EDGE_LOOP_BROKEN = 4;
	public static final int  NO_OUTER_TRIANGLE = 5;
	public static final int  TRIANGLE_INVISIBLE = 6;
}
