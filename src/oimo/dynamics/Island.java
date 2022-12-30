package oimo.dynamics;
import oimo.common.Setting;
import oimo.common.Vec3;
import oimo.dynamics.constraint.ConstraintSolver;
import oimo.dynamics.constraint.PositionCorrectionAlgorithm;
import oimo.dynamics.rigidbody.*;
import oimo.common.M;

/**
 * Simulation island.
 */
public class Island {
	Vec3 gravity;

	int numRigidBodies;
	RigidBody[] rigidBodies;

	// all the constraint solvers
	int numSolvers;
	ConstraintSolver[] solvers;//:Vector<ConstraintSolver>;

	// the constraint solvers use split impulse for position part
	int numSolversSi;
	ConstraintSolver[] solversSi;//:Vector<ConstraintSolver>;

	// the constraint solvers use nonlinear Gauss-Seidel for position part
	int numSolversNgs;
	ConstraintSolver[] solversNgs;///:Vector<ConstraintSolver>;

	public Island() {
		rigidBodies = new RigidBody[Setting.islandInitialRigidBodyArraySize];
		solvers = new ConstraintSolver[Setting.islandInitialConstraintArraySize];
		solversSi = new ConstraintSolver[Setting.islandInitialConstraintArraySize];
		solversNgs = new ConstraintSolver[Setting.islandInitialConstraintArraySize];
		numRigidBodies = 0;
		numSolvers = 0;
		numSolversSi = 0;
		numSolversNgs = 0;
	}

	// --- private ---

	private float fastInvExp(float x) {
		float x2 = x * x;
		return 1 / (1 + x + x2 * (1 / 2 + x * (1 / 6) + x2 * (1 / 24)));
	}

	private void  addConstraintSolverSI(ConstraintSolver solver) {
		if (numSolversSi == solversSi.length) {
			int newLength = ( this.numSolversSi << 1 );
			ConstraintSolver[] newArray=new ConstraintSolver[newLength];
			for(int i=0;i<this.numSolversSi;i++) {
				newArray[i]=solversSi[i];
				solversSi[i]=null;
			}
			this.solversSi=newArray;
			//M.array_expand(solversSi, numSolversSi);
		}
		solversSi[numSolversSi++] = solver;
	}
	

	private void addConstraintSolverNgs(ConstraintSolver solver) {
		if (numSolversNgs == solversNgs.length) {
			int newLength = ( this.numSolversNgs << 1 );
			ConstraintSolver[] newArray=new ConstraintSolver[newLength];
			for(int i=0;i<this.numSolversNgs;i++) {
				newArray[i]=solversNgs[i];
				solversNgs[i]=null;
			}
			this.solversNgs=newArray;
			//M.array_expand(solversNgs, numSolversNgs);
		}
		solversNgs[numSolversNgs++] = solver;
	}

	// --- internal ---

	public void _clear() {
		while(this.numRigidBodies > 0) this.rigidBodies[--this.numRigidBodies] = null;
		while(this.numSolvers > 0) this.solvers[--this.numSolvers] = null;
		while(this.numSolversSi > 0) this.solversSi[--this.numSolversSi] = null;
		while(this.numSolversNgs > 0) this.solversNgs[--this.numSolversNgs] = null;
//		M.array_free(rigidBodies, numRigidBodies);
//		M.array_free(solvers, numSolvers);
//		M.array_free(solversSi, numSolversSi);
//		M.array_free(solversNgs, numSolversNgs);
	}

    public void _setGravity(Vec3 gravity) {
		M.vec3_fromVec3(this.gravity, gravity);
	}

	public void _addRigidBody(RigidBody rigidBody) {
		if (numRigidBodies == rigidBodies.length) {
			int newLength = ( this.numRigidBodies << 1 );
			RigidBody[] newArray=new RigidBody[newLength];
			for(int i=0;i<this.numRigidBodies;i++) {
				newArray[i]=rigidBodies[i];
				rigidBodies[i]=null;
			}
			this.rigidBodies=newArray;
			//M.array_expand(rigidBodies, numRigidBodies);
		}
		rigidBody._addedToIsland = true;
		rigidBodies[numRigidBodies++] = rigidBody;
	}

	public void _addConstraintSolver(ConstraintSolver solver, int positionCorrection) {
		if (numSolvers == solvers.length) {
			int newLength = ( this.numSolvers << 1 );
			ConstraintSolver[] newArray=new ConstraintSolver[newLength];
			for(int i=0;i<this.numSolvers;i++) {
				newArray[i]=solvers[i];
				solvers[i]=null;
			}
			this.solvers=newArray;
			//M.array_expand(solvers, numSolvers);
		}
		solver._addedToIsland = true;
		solvers[numSolvers++] = solver;

		if (positionCorrection == PositionCorrectionAlgorithm.SPLIT_IMPULSE) {
			addConstraintSolverSI(solver);
		}
		if (positionCorrection == PositionCorrectionAlgorithm.NGS) {
			addConstraintSolverNgs(solver);
		}
	}

	// steps the single rigid body
	public void _stepSingleRigidBody(TimeStep timeStep, RigidBody rb) {
		float dt = timeStep.dt;

		// store previous transform
		M.transform_assign(rb._ptransform, rb._transform);

		// clear linear/angular contact impulse
		M.vec3_zero(rb._linearContactImpulse);
		M.vec3_zero(rb._angularContactImpulse);

		// update sleep time
		if (rb._isSleepy()) {
			rb._sleepTime += dt;
			if (rb._sleepTime > Setting.sleepingTimeThreshold) {
				rb.sleep();
			}
		} else {
			rb._sleepTime = 0;
		}

		if (!rb._sleeping) {
			// the rigid body is awake
			if (rb._type == RigidBodyType._DYNAMIC) {
				// damping
				float linScale = fastInvExp(dt * rb._linearDamping);
				float angScale = fastInvExp(dt * rb._angularDamping);

				// compute accelerations
				Vec3 linAcc=new Vec3();
				Vec3 angAcc=new Vec3();
				M.vec3_scale(linAcc, gravity, rb._gravityScale);
				M.vec3_addRhsScaled(linAcc, linAcc, rb._force, rb._invMass);
				M.vec3_mulMat3(angAcc, rb._torque, rb._invInertia);

				// update velocity
				M.vec3_addRhsScaled(rb._vel, rb._vel, linAcc, dt);
				M.vec3_scale(rb._vel, rb._vel, linScale);
				M.vec3_addRhsScaled(rb._angVel, rb._angVel, angAcc, dt);
				M.vec3_scale(rb._angVel, rb._angVel, angScale);
			}
			rb._integrate(dt);
			rb._syncShapes();
		}
	}

	// steps the island with multiple bodies and constraints
	public void _step(TimeStep timeStep, int numVelocityIterations, int numPositionIterations) {
		float dt = timeStep.dt;

		boolean sleepIsland = true;

		// sleep check and apply gravity
		for (int i=0;i<numRigidBodies;i++) {
			RigidBody rb = rigidBodies[i];

			// store previous transform
			M.transform_assign(rb._ptransform, rb._transform);

			// clear linear/angular contact impulse
			M.vec3_zero(rb._linearContactImpulse);
			M.vec3_zero(rb._angularContactImpulse);

			// don't let the rigid body sleep
			rb._sleeping = false;

			// update sleep time
			if (rb._isSleepy()) {
				rb._sleepTime += dt;
			} else {
				rb._sleepTime = 0;
			}

			// check if the rigid body is awaken
			if (rb._sleepTime < Setting.sleepingTimeThreshold) {
				// awaken the whole island
				sleepIsland = false;
			}

			// apply forces
			if (rb._type == RigidBodyType._DYNAMIC) {
				// damping
				float linScale = fastInvExp(dt * rb._linearDamping);
				float angScale = fastInvExp(dt * rb._angularDamping);

				// compute accelerations
				Vec3 linAcc=new Vec3();
				Vec3 angAcc=new Vec3();
				M.vec3_scale(linAcc, gravity, rb._gravityScale);
				M.vec3_addRhsScaled(linAcc, linAcc, rb._force, rb._invMass);
				M.vec3_mulMat3(angAcc, rb._torque, rb._invInertia);

				// update velocity
				M.vec3_addRhsScaled(rb._vel, rb._vel, linAcc, dt);
				M.vec3_scale(rb._vel, rb._vel, linScale);
				M.vec3_addRhsScaled(rb._angVel, rb._angVel, angAcc, dt);
				M.vec3_scale(rb._angVel, rb._angVel, angScale);
			}
		}

		if (sleepIsland) {
			// sleep the whole island
			for (int i=0;i<numRigidBodies;i++) {
				RigidBody rb = rigidBodies[i];
				rb.sleep();
			}
			return;
		}


		// -------------- test --------------

		/*
		// randomize constraint order

		for (i in 1...numSolvers) {
			var j = Std.int(Math.random() * (i + 1));
			var tmp = solvers[i];
			solvers[i] = solvers[j];
			solvers[j] = tmp;
		}

		for (i in 1...numSolversSi) {
			var j = Std.int(Math.random() * (i + 1));
			var tmp = solversSi[i];
			solversSi[i] = solversSi[j];
			solversSi[j] = tmp;
		}

		for (i in 1...numSolversNgs) {
			var j = Std.int(Math.random() * (i + 1));
			var tmp = solversNgs[i];
			solversNgs[i] = solversNgs[j];
			solversNgs[j] = tmp;
		}
		*/

		// -------------- test --------------

		// solve velocity
		for (int i=0;i<numSolvers;i++) {
			ConstraintSolver s = solvers[i];
			s.preSolveVelocity(timeStep);
		}
		for (int i=0;i< numSolvers;i++) {
			ConstraintSolver s = solvers[i];
			s.warmStart(timeStep);
		}
		for (int t=0;t< numVelocityIterations;t++) {
			for (int i=0;i<numSolvers;i++) {
				ConstraintSolver s = solvers[i];
				s.solveVelocity();
			}
		}

		// post-solve (velocity)
		for (int i=0;i<numSolvers;i++) {
			ConstraintSolver s = solvers[i];
			s.postSolveVelocity(timeStep);
		}

		// integrate
		for (int i=0;i<numRigidBodies;i++) {
			RigidBody rb = rigidBodies[i];
			rb._integrate(dt);
		}

		// solve split impulse
		for (int i =0;i<numSolversSi ;i++) {
			ConstraintSolver s = solversSi[i];
			s.preSolvePosition(timeStep);
		}
		for (int t=0;t< numPositionIterations;t++) {
			for (int i=0;i<numSolversSi;i++) {
				ConstraintSolver s = solversSi[i];
				s.solvePositionSplitImpulse();
			}
		}

		// solve integrate pseudo velocity
		for (int i=0;i<numRigidBodies;i++) {
			RigidBody rb = rigidBodies[i];
			rb._integratePseudoVelocity();
		}

		// solve nonlinear Gauss-Seidel
		for (int i=0;i<numSolversNgs;i++) {
			ConstraintSolver s = solversNgs[i];
			s.preSolvePosition(timeStep);
		}
		for (int t=0;t<numPositionIterations;t++) {
			for (int i=0;i< numSolversNgs;i++) {
				ConstraintSolver s = solversNgs[i];
				s.solvePositionNgs(timeStep);
			}
		}

		// post-solve (some constraints may be removed)
		for (int i=0;i<numSolvers;i++) {
			ConstraintSolver s = solvers[i];
			s.postSolve();
		}

		// synchronize shapes
		for (int i=0;i<numRigidBodies;i++) {
			RigidBody rb = rigidBodies[i];
			rb._syncShapes();
		}
	}

}
