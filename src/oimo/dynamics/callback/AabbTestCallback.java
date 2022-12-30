package oimo.dynamics.callback;
import oimo.dynamics.rigidbody.Shape;

/**
 * A callback interface for aabb tests in a world.
 */
public interface AabbTestCallback {

	
	/**
	 * This is called every time the world detects a shape `shape` that
	 * the query aabb intersects.
	 */
	public abstract void process(Shape shape);

}