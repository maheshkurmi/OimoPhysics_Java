package oimo.collision.narrowphase.detector.gjkepa;
/**
 * The list of the state of a result of `GjkEpa.computeClosestPoints`.
 */
public class GjkEpaResultState {
	public static final int _SUCCEEDED                          = 0x000;
	public static final int  _GJK_FAILED_TO_MAKE_TETRAHEDRON     = 0x001;
	public static final int  _GJK_DID_NOT_CONVERGE               = 0x002;
	public static final int  _EPA_FAILED_TO_INIT                 = 0x101;
	public static final int  _EPA_FAILED_TO_ADD_VERTEX           = 0x102;
	public static final int  _EPA_DID_NOT_CONVERGE               = 0x103;

	/**
	 * GJK/EPA computation is successfully finished.
	 */
	public static final int SUCCEEDED = _SUCCEEDED;

	/**
	 * Failed to construct a tetrahedron enclosing the origin in GJK computation.
	 */
	public static final int GJK_FAILED_TO_MAKE_TETRAHEDRON = _GJK_FAILED_TO_MAKE_TETRAHEDRON;

	/**
	 * GJK iterations did not converge in time.
	 */
	public static final int GJK_DID_NOT_CONVERGE = _GJK_DID_NOT_CONVERGE;

	/**
	 * Failed to construct initial polyhedron in EPA construction.
	 */
	public static final int EPA_FAILED_TO_INIT = _EPA_FAILED_TO_INIT;

	/**
	 * Failed to add a new vertex to the polyhedron in EPA computation.
	 */
	public static final int EPA_FAILED_TO_ADD_VERTEX= _EPA_FAILED_TO_ADD_VERTEX;

	/**
	 * EPA iterations did not converge in time.
	 */
	public static final int EPA_DID_NOT_CONVERGE = _EPA_DID_NOT_CONVERGE;
}