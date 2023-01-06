package oimo.dynamics;

/**
 * Information of time-step sizes of the simulation.
 */
public class TimeStep {
	/**
	 * The time step of simulation.
	 */
	public double dt;

	/**
	 * The inverse time step of simulation, equivalent to simulation FPS.
	 */
	public double invDt;

	/**
	 * The ratio of time steps. Defined by current time step divided by previous
	 * time step.
	 */
	public double dtRatio;

	public TimeStep() {
		dt = 0;
		invDt = 0;
		dtRatio = 1;
	}

}
