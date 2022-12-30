package oimo.collision.geometry;
/**
 * The list of collision geometry types.
 */
public class GeometryType {

	public static  int _CONVEX_MIN = 0;
	public static  int _CONVEX_MAX = 5;

	/**
	 * Represents a sphere collision geometry.
	 *
	 * See `SphereGeometry`.
	 */
	public static final int SPHERE = 0;

	/**
	 * Represents a box collision geometry.
	 *
	 * See `BoxGeometry`.
	 */
	public static final int BOX = 1;

	/**
	 * Represents a cylinder collision geometry.
	 *
	 * See `CylinderGeometry`.
	 */
	public static final int CYLINDER = 2;

	/**
	 * Represents a cone collision geometry.
	 *
	 * See `ConeGeometry`.
	 */
	public static final int CONE = 3;

	/**
	 * Represents a capsule collision geometry.
	 *
	 * See `CapsuleGeometry`.
	 */
	public static final int CAPSULE = 4;

	/**
	 * Represents a convex hull collision geometry.
	 *
	 * See `ConvexHullGeometry`.
	 */
	public static final int CONVEX_HULL = 5;
}