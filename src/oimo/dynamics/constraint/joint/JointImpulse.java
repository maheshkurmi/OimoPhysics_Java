package oimo.dynamics.constraint.joint;

/**
 * Internal class.
 */
public class JointImpulse {
	// constraint impulse
	public float impulse;
	// motor impulse
	public float impulseM;
	// position impulse
	public float impulseP;

	public JointImpulse() {
		clear();
	}

	public void clear() {
		impulse = 0;
		impulseM = 0;
		impulseP = 0;
	}
}
