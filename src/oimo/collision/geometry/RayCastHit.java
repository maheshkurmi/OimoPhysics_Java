package oimo.collision.geometry;

import oimo.common.Vec3;

/**
 * A single ray cast hit data.
 */
public class RayCastHit {
	/**
	 * The position the ray hit at.
	 */
	public Vec3 position;

	/**
	 * The normal vector of the surface the ray hit.
	 */
	public Vec3 normal;

	/**
	 * The ratio of the position the ray hit from the start point to the end point.
	 */
	public float fraction;

	/**
	 * Default constructor.
	 */
	public RayCastHit() {
		position = new Vec3();
		normal = new Vec3();
		fraction = 0;
	}
}