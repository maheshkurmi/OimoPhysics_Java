package oimo.dynamics.constraint.joint;

/**
 * Translational limits and motor settings of a joint.
 */
public  class TranslationalLimitMotor {
	/**
	 * The lower bound of the limit in usually meters.
	 *
	 * The limit is disabled if `lowerLimit > upperLimit`.
	 */
	public double lowerLimit;

	/**
	 * The upper bound of the limit in usually meters.
	 *
	 * The limit is disabled if `lowerLimit > upperLimit`.
	 */
	public double upperLimit ;

	/**
	 * The target speed of the motor in usually meters per second.
	 */
	public double motorSpeed ;

	/**
	 * The maximum force of the motor in usually newtons.
	 *
	 * The motor is disabled if `motorForce <= 0`.
	 */
	public double motorForce ;

	/**
	 * Default constructor.
	 */
	public TranslationalLimitMotor() {
		lowerLimit = 1;
		upperLimit = 0;
		motorForce = 0;
	}

	/**
	 * Sets limit properties at once and returns `this`.
	 * `this.lowerLimit` is set to `lower`, and `this.upperLimit` is set to `upper`.
	 */
	public TranslationalLimitMotor setLimits(double lower , double upper ){
		lowerLimit = lower;
		upperLimit = upper;
		return this;
	}

	/**
	 * Sets motor properties at once and returns `this`.
	 * `this.motorSpeed` is set to `speed`, and `this.motorForce` is set to `force`.
	 */
	public TranslationalLimitMotor setMotor(double speed , double force ) {
		motorSpeed = speed;
		motorForce = force;
		return this;
	}

	/**
	 * Returns a clone of the object.
	 */
	public TranslationalLimitMotor clone() {
		TranslationalLimitMotor lm = new TranslationalLimitMotor();
		lm.lowerLimit = lowerLimit;
		lm.upperLimit = upperLimit;
		lm.motorSpeed = motorSpeed;
		lm.motorForce = motorForce;
		return lm;
	}
}
