package oimo.dynamics.constraint.joint;

/**
 * Spring and damper settings of a joint.
 */
public class SpringDamper {
	/**
	 * The frequency of the spring in Hz. Set `0.0` to disable the spring and make
	 * the constraint totally rigid.
	 */
	public double frequency;

	/**
	 * The damping ratio of the constraint. Set `1.0` to make the constraint
	 * critically dumped.
	 */
	public double dampingRatio;

	/**
	 * Whether to use symplectic Euler method instead of implicit Euler method, to
	 * numarically integrate the constraint. Note that symplectic Euler method
	 * conserves energy better than implicit Euler method does, but the constraint
	 * will be unstable under the high frequency.
	 */
	public boolean useSymplecticEuler;

	/**
	 * Default constructor.
	 */
	public SpringDamper() {
		frequency = 0;
		dampingRatio = 0;
		useSymplecticEuler = false;
	}

	/**
	 * Sets spring and damper parameters at once and returns `this`.
	 * `this.frequency` is set to `frequency`, and `this.dampingRatio` is set to
	 * `dampingRatio`.
	 */
	public SpringDamper setSpring(double frequency, double dampingRatio) {
		this.frequency = frequency;
		this.dampingRatio = dampingRatio;
		return this;
	}

	public SpringDamper setSymplecticEuler(boolean useSymplecticEuler) {
		this.useSymplecticEuler = useSymplecticEuler;
		return this;
	}

	/**
	 * Returns a clone of the object.
	 */
	public SpringDamper clone() {
		SpringDamper sd = new SpringDamper();
		sd.frequency = frequency;
		sd.dampingRatio = dampingRatio;
		sd.useSymplecticEuler = useSymplecticEuler;
		return sd;
	}

}
