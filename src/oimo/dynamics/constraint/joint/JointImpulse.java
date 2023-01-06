package oimo.dynamics.constraint.joint;

/**
 * Internal class.
 */
public class JointImpulse {
	// constraint impulse
	public double impulse;
	// motor impulse
	public double impulseM;
	// position impulse
	public double impulseP;

	public JointImpulse() {
		clear();
	}

	public void clear() {
		impulse = 0;
		impulseM = 0;
		impulseP = 0;
	}
}
