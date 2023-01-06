package oimo.dynamics.constraint.joint;

/**
 * Rotational limits and motor settings of a joint.
 */
public class RotationalLimitMotor {
	/**
	 * The lower bound of the limit in radians.
	 *
	 * The limit is disabled if `lowerLimit > upperLimit`.
	 */
	public double lowerLimit;

	/**
	 * The upper bound of the limit in radians.
	 *
	 * The limit is disabled if `lowerLimit > upperLimit`.
	 */
	public double upperLimit;

	/**
	 * The target speed of the motor in usually radians per second.
	 */
	public double motorSpeed;

	/**
	 * The maximum torque of the motor in usually newton meters.
	 *
	 * The motor is disabled if `motorTorque <= 0`.
	 */
	public double motorTorque;

	/**
	 * Default constructor.
	 */
	public RotationalLimitMotor() {
		lowerLimit = 1;
		upperLimit = 0;
		motorTorque = 0;
	}

	/**
	 * Sets limit properties at once and returns `this`.
	 * `this.lowerLimit` is set to `lower`, and `this.upperLimit` is set to `upper`.
	 */
	public RotationalLimitMotor setLimits(double lower, double upper) {
		lowerLimit = lower;
		upperLimit = upper;
		return this;
	}

	/**
	 * Sets motor properties at once and returns `this`.
	 * `this.motorSpeed` is set to `speed`, and `this.motorTorque` is set to `torque`.
	 */
	public RotationalLimitMotor setMotor(double speed,double torque) {
		motorSpeed = speed;
		motorTorque = torque;
		return this;
	}

	/**
	 * Returns a clone of the object.
	 */
	public RotationalLimitMotor clone() {
		RotationalLimitMotor lm = new RotationalLimitMotor();
		lm.lowerLimit = lowerLimit;
		lm.upperLimit = upperLimit;
		lm.motorSpeed = motorSpeed;
		lm.motorTorque = motorTorque;
		return lm;
	}

}
