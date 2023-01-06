package demo.common;
import oimo.common.DebugDraw;
import oimo.common.Mat4;
import oimo.common.Vec3;

/**
 * DebugDraw with transformation matrices. Used to render a demo world.
 */
public interface IDemoGraphics {
	/**
	 * Begins rendering with the background `color`.
	 */
	public void begin(Vec3 color);

	/**
	 * Ends rendering.
	 */
	public void end();

	/**
	 * Returns the debug draw instance using.
	 */
	public DebugDraw getDebugDraw();

	/**
	 * Sets the view transformation matrix to `matrix`.
	 */
	public void setViewMatrix(Mat4 matrix);

	/**
	 * Sets the projection transformation matrix to `matrix`.
	 */
	public void setProjectionMatrix(Mat4 matrix);
}