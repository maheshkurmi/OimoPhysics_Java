package oimo.dynamics.callback;
import oimo.collision.geometry.RayCastHit;
import oimo.common.Vec3;
import oimo.dynamics.rigidbody.Shape;

/**
 * A ray cast callback implementation that keeps only the closest hit data.
 * This is reusable, but make sure to clear the old result by calling
 * `RayCastClosest.clear` if used once or more before.
 */
public  class RayCastClosest implements RayCastCallback {
	/**
	 * The shape the ray hit.
	 */
	public Shape shape;

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
	public double fraction;

	/**
	 * Whether the ray hit any shape in the world.
	 */
	public boolean hit;

	/**
	 * Default constructor.
	 */
	public RayCastClosest() {
		super();
		position = new Vec3();
		normal = new Vec3();
		clear();
	}

	/**
	 * Clears the result data.
	 */
	public void clear() {
		shape = null;
		fraction = 1;
		position.zero();
		normal.zero();
		hit = false;
	}

	@Override 
	public  void process(Shape shape, RayCastHit hit) {
		if (hit.fraction < fraction) {
			this.shape = shape;
			this.hit = true;
			fraction = hit.fraction;
			position.copyFrom(hit.position);
			normal.copyFrom(hit.normal);
		}
	}

}