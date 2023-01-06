package oimo.dynamics.rigidbody;
import oimo.common.Mat3;

/**
 * This class holds mass and moment of inertia for a rigid body.
 */
public class MassData {
	/**
	 * Mass. `0` for a non-dynamic rigid body.
	 */
	public double mass;

	/**
	 * Inertia tensor in local space. Zero matrix for a non-dynamic rigid body.
	 */
	public Mat3 localInertia;

	/**
	 * Default constructor.
	 */
	public MassData() {
		mass = 0;
		localInertia = new Mat3();
	}
	

}
