package oimo.dynamics.constraint;
import oimo.dynamics.TimeStep;
import oimo.dynamics.rigidbody.RigidBody;
import oimo.dynamics.*;
import oimo.dynamics.rigidbody.*;

/**
 * The base class of all constarint solvers.
 */
public abstract class ConstraintSolver {
	public RigidBody _b1;
	public RigidBody _b2;
	public boolean _addedToIsland;

	public ConstraintSolver() {
		_b1 = null;
		_b2 = null;
		_addedToIsland = false;
	}

	/**
	 * Prepares for velocity iteration. Time step information `timeStep` is given for
	 * computing time-depending data.
	 */
	public abstract void preSolveVelocity( TimeStep timeStep);

	/**
	 * Applies initial impulses.
	 */
	public abstract void  warmStart(TimeStep timeStep) ;

	/**
	 * Performs single velocity iteration.
	 */
	public abstract void  solveVelocity();
	/**
	 * Performs post-processes of velocity part. Time step information `timeStep` is given
	 * for computing time-depending data.
	 */
	public abstract void  postSolveVelocity(TimeStep timeStep);

	/**
	 * Prepares for position iteration (split impulse or nonlinear Gauss-Seidel). Time step
	 * information `timeStep` is given for computing time-depending data.
	 *
	 * This may not be called depending on position correction algorithm.
	 */
	public abstract void  preSolvePosition(TimeStep timeStep);

	/**
	 * Performs single position iteration (split impulse)
	 */
	public abstract void  solvePositionSplitImpulse();

	/**
	 * Performs single position iteration (nonlinear Gauss-Seidel)
	 */
	public abstract void  solvePositionNgs(TimeStep timeStep);

	/**
	 * Performs post-processes.
	 */
	public abstract void  postSolve();
}
