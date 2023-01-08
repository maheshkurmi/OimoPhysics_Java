package demo.common;

import oimo.common.Mat4;
import oimo.common.MathUtil;
import oimo.common.Vec3;

public class Camera {
	/** Distance of near plane from camera */
	public double near = 1;
	/** Distance of fat plane from camera */
	public double far = 100;
	/** The vertical field-of-view angle in degrees. */
	public double verticalFOV = 90;
	/** aspect ratio width/height*/
	public double aspect=1;
	
	/** * location of camera(eye) in world space */
	public Vec3 eye = new Vec3(0, 0, 1);
	/** location of focus in world space	 */
	public Vec3 focus = new Vec3();
	/** direction of up vector in world space	 */
	public Vec3 up = new Vec3();

	
	/**
	 *  projection matrix (clipping) for the frustom, which converts view coordinated to clip coordinates
	 */
	protected Mat4 projectionMatrix = new Mat4();

	/**
	 * The camera view matrix, used to transform world coordinates to eye coordinates
	 */
	protected Mat4 viewMatrix = new Mat4();
	
	/** Constructs a new PerspectiveCamera, with default values  Which are..
	 * 
	 * <pre>
	 * Focus = [0,0,0] 
	 * Eye   = [0,0,1] 
	 * Up    = [0,1,0]	(choose rollAngle=0);
	 * frustum set to -1 to 1 in each x and y direction and near plane = 1 far plane=100
	 */
	public Camera() {
		this.eye.set(0.0f, 0.0f, 1.0f); // place camera at (0,0,4) for good perspective fell( as per mine)
		this.focus.set(0.0f, 0.0f, 0.0f); // set focus to origin in world space
		this.up.set(0.0f, 1.0f, 0.0f); // set up to +y axis of world space
		this.near =1;
		this.far=1;
		this.verticalFOV=90;
		updateMatrices();
	}
	
	/**
	 * Recalculates , Should be called after camera parameters are changed 
	 */
	public void updateMatrices() {
		viewMatrix.lookAt(this.eye.x, this.eye.y, this.eye.z, this.focus.x, this.focus.y, this.focus.z, this.up.x, this.up.y, this.up.z);
		projectionMatrix.perspective(MathUtil.TO_RADIANS*this.verticalFOV, this.aspect, this.near, this.far);
	}
	
}
