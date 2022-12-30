package oimo.dynamics;

/**
 * Information of time-step sizes of the simulation.
 */
public class TimeStep {
	/**
	 * The time step of simulation.
	 */
	public float dt;

	/**
	 * The inverse time step of simulation, equivalent to simulation FPS.
	 */
	public float invDt;

	/**
	 * The ratio of time steps. Defined by current time step divided by previous
	 * time step.
	 */
	public float dtRatio;

	public TimeStep() {
		dt = 0;
		invDt = 0;
		dtRatio = 1;
	}

}
