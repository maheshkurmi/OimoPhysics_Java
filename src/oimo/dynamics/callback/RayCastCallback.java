package oimo.dynamics.callback;
import oimo.collision.geometry.RayCastHit;
import oimo.dynamics.rigidbody.Shape;

/**
 * A callback class for ray casts in a world.
 */
public interface RayCastCallback {


	/**
	 * This is called every time the world detects a shape `shape` that
	 * the ray intersects with the hit data `hit`.
	 */
	public abstract void process(Shape shape, RayCastHit hit);

}
