package oimo.dynamics.constraint.solver.direct;

import oimo.common.Setting;
import oimo.common.Vec3;
import oimo.dynamics.TimeStep;
import oimo.dynamics.constraint.ConstraintSolver;
import oimo.dynamics.constraint.info.JacobianRow;
import oimo.dynamics.constraint.info.joint.JointSolverInfo;
import oimo.dynamics.constraint.info.joint.JointSolverInfoRow;
import oimo.dynamics.constraint.joint.Joint;
import oimo.dynamics.constraint.joint.JointImpulse;
import oimo.dynamics.constraint.solver.common.JointSolverMassDataRow;
import oimo.common.M;

/**
 * The direct solver of a mixed linear complementality problem (MLCP) for joint
 * constraints.
 */
public class DirectJointConstraintSolver extends ConstraintSolver {
	JointSolverInfo info;
	JointSolverMassDataRow[] massData;

	double[] relVels;
	double[] impulses;
	double[] dImpulses;
	double[] dTotalImpulses;

	Joint joint;

	MassMatrix massMatrix;

	BoundaryBuilder boundaryBuilder;

	BoundarySelector velBoundarySelector;
	BoundarySelector posBoundarySelector;

	public DirectJointConstraintSolver(Joint joint) {
		super();
		this.joint = joint;
		info = new JointSolverInfo();

		int maxRows = Setting.maxJacobianRows;

		massMatrix = new MassMatrix(maxRows);
		boundaryBuilder = new BoundaryBuilder(maxRows);
		massData = new JointSolverMassDataRow[maxRows];
		for (int i = 0; i < massData.length; i++) {
			massData[i] = new JointSolverMassDataRow();
		}

		int numMaxBoundaries = boundaryBuilder.boundaries.length;
		velBoundarySelector = new BoundarySelector(numMaxBoundaries);
		posBoundarySelector = new BoundarySelector(numMaxBoundaries);

		relVels = new double[maxRows];
		impulses = new double[maxRows];
		dImpulses = new double[maxRows];
		dTotalImpulses = new double[maxRows];

		for (int i = 0; i < maxRows; i++) {
			relVels[i] = 0;
			impulses[i] = 0;
			dImpulses[i] = 0;
			dTotalImpulses[i] = 0;
		}
	}

	public void applyImpulses(double[] impulses) {
		boolean linearSet = false;
		boolean angularSet = false;
		Vec3 lv1 = new Vec3();
		Vec3 lv2 = new Vec3();
		Vec3 av1 = new Vec3();
		Vec3 av2 = new Vec3();
		M.vec3_assign(lv1, _b1._vel);
		M.vec3_assign(lv2, _b2._vel);
		M.vec3_assign(av1, _b1._angVel);
		M.vec3_assign(av2, _b2._angVel);

		for (int i = 0; i < info.numRows; i++) {
			JointSolverInfoRow row = info.rows[i];
			JacobianRow j = row.jacobian;
			JointSolverMassDataRow md = massData[i];
			double imp = impulses[i];
			if (j.isLinearSet()) {
				M.vec3_addRhsScaled(lv1, lv1, md.invMLin1, imp);
				M.vec3_addRhsScaled(lv2, lv2, md.invMLin2, -imp);
				linearSet = true;
			}
			if (j.isAngularSet()) {
				M.vec3_addRhsScaled(av1, av1, md.invMAng1, imp);
				M.vec3_addRhsScaled(av2, av2, md.invMAng2, -imp);
				angularSet = true;
			}
		}
		if (linearSet) {
			M.vec3_assign(_b1._vel, lv1);
			M.vec3_assign(_b2._vel, lv2);
		}
		if (angularSet) {
			M.vec3_assign(_b1._angVel, av1);
			M.vec3_assign(_b2._angVel, av2);
		}
	}

	public void applySplitImpulses(double[] impulses) {
		boolean linearSet = false;
		boolean angularSet = false;
		Vec3 lv1 = new Vec3();
		Vec3 lv2 = new Vec3();
		Vec3 av1 = new Vec3();
		Vec3 av2 = new Vec3();
		M.vec3_assign(lv1, _b1._pseudoVel);
		M.vec3_assign(lv2, _b2._pseudoVel);
		M.vec3_assign(av1, _b1._angPseudoVel);
		M.vec3_assign(av2, _b2._angPseudoVel);

		for (int i = 0; i < info.numRows; i++) {
			JointSolverInfoRow row = info.rows[i];
			JacobianRow j = row.jacobian;
			JointSolverMassDataRow md = massData[i];
			double imp = impulses[i];
			if (j.isLinearSet()) {
				M.vec3_addRhsScaled(lv1, lv1, md.invMLin1, imp);
				M.vec3_addRhsScaled(lv2, lv2, md.invMLin2, -imp);
				linearSet = true;
			}
			if (j.isAngularSet()) {
				M.vec3_addRhsScaled(av1, av1, md.invMAng1, imp);
				M.vec3_addRhsScaled(av2, av2, md.invMAng2, -imp);
				angularSet = true;
			}
		}
		if (linearSet) {
			M.vec3_assign(_b1._pseudoVel, lv1);
			M.vec3_assign(_b2._pseudoVel, lv2);
		}
		if (angularSet) {
			M.vec3_assign(_b1._angPseudoVel, av1);
			M.vec3_assign(_b2._angPseudoVel, av2);
		}
	}

	public void applyPositionImpulses(double[] impulses) {
		boolean linearSet = false;
		boolean angularSet = false;
		Vec3 lv1 = new Vec3();
		Vec3 lv2 = new Vec3();
		Vec3 av1 = new Vec3();
		Vec3 av2 = new Vec3();

		for (int i = 0; i < info.numRows; i++) {
			JointSolverInfoRow row = info.rows[i];
			JacobianRow j = row.jacobian;
			JointSolverMassDataRow md = massData[i];
			double imp = impulses[i];
			if (j.isLinearSet()) {
				M.vec3_addRhsScaled(lv1, lv1, md.invMLin1, imp);
				M.vec3_addRhsScaled(lv2, lv2, md.invMLin2, -imp);
				linearSet = true;
			}
			if (j.isAngularSet()) {
				M.vec3_addRhsScaled(av1, av1, md.invMAng1, imp);
				M.vec3_addRhsScaled(av2, av2, md.invMAng2, -imp);
				angularSet = true;
			}
		}
		if (linearSet) {
			_b1._applyTranslation(lv1);
			_b2._applyTranslation(lv2);
		}
		if (angularSet) {
			_b1._applyRotation(av1);
			_b2._applyRotation(av2);
		}
	}

	public void preSolveVelocity(TimeStep timeStep) {
		joint._syncAnchors();
		joint._getVelocitySolverInfo(timeStep, info);

		_b1 = info.b1;
		_b2 = info.b2;

		// compute inverse mass matrix
		massMatrix.computeInvMass(info, massData);

		// build boundaries
		boundaryBuilder.buildBoundaries(info);

		// update the size of the boundary selector
		velBoundarySelector.setSize(boundaryBuilder.numBoundaries);
	}

	public void warmStart(TimeStep timeStep) {
		double factor = joint._getWarmStartingFactor();

		// adjust impulse for variable time step
		factor *= timeStep.dtRatio;

		// warm start disabled
		if (factor <= 0) {
			for (int i = 0; i < info.numRows; i++) {
				JointSolverInfoRow row = info.rows[i];
				row.impulse.clear();
			}
			return;
		}

		for (int i = 0; i < info.numRows; i++) {
			JointSolverInfoRow row = info.rows[i];
			JointImpulse imp = row.impulse;

			// update limit impulse
			double impulse = imp.impulse * factor;
			if (impulse < row.minImpulse)
				impulse = row.minImpulse;
			else if (impulse > row.maxImpulse)
				impulse = row.maxImpulse;
			imp.impulse = impulse;

			if (row.motorMaxImpulse > 0) {
				double impulseM = imp.impulseM * factor;
				double max = row.motorMaxImpulse;
				if (impulseM < -max)
					impulseM = -max;
				else if (impulseM > max)
					impulseM = max;
				imp.impulseM = impulseM;
			} else {
				imp.impulseM = 0;
			}

			dImpulses[i] = imp.impulse + imp.impulseM;
		}
		// apply initial impulse
		applyImpulses(dImpulses);
	}

	@Override
	public void solveVelocity() {
		int numRows = info.numRows;
		Vec3 lv1 = new Vec3();
		Vec3 lv2 = new Vec3();
		Vec3 av1 = new Vec3();
		Vec3 av2 = new Vec3();
		M.vec3_assign(lv1, _b1._vel);
		M.vec3_assign(lv2, _b2._vel);
		M.vec3_assign(av1, _b1._angVel);
		M.vec3_assign(av2, _b2._angVel);

		for (int i = 0; i < numRows; i++) {
			JointSolverInfoRow row = info.rows[i];
			JointImpulse imp = row.impulse;
			JacobianRow j = row.jacobian;

			// compute relative velocity
			double relVel = 0;
			relVel += M.vec3_dot(lv1, j.lin1);
			relVel -= M.vec3_dot(lv2, j.lin2);
			relVel += M.vec3_dot(av1, j.ang1);
			relVel -= M.vec3_dot(av2, j.ang2);
			relVels[i] = relVel;

			// get impulse
			impulses[i] = imp.impulse;

			// clear total impulse
			dTotalImpulses[i] = 0;
		}

		// solve motors first
		double[][] invMass = massMatrix._invMassWithoutCfm;
		for (int i = 0; i < numRows; i++) {
			JointSolverInfoRow row = info.rows[i];
			JointImpulse imp = row.impulse;
			JointSolverMassDataRow md = massData[i];
			if (row.motorMaxImpulse > 0) {
				// relative velocity : body1 - body2
				// motor speed : body2 - body1
				// =>
				// target relative velocity : -[motor speed]
				double oldImpulseM = imp.impulseM;
				double impulseM = oldImpulseM + md.massWithoutCfm * (-row.motorSpeed - relVels[i]);

				// clamp motor impulse
				double maxImpulseM = row.motorMaxImpulse;
				if (impulseM < -maxImpulseM)
					impulseM = -maxImpulseM;
				else if (impulseM > maxImpulseM)
					impulseM = maxImpulseM;
				imp.impulseM = impulseM;

				// compute delta motor impulse
				double dImpulseM = impulseM - oldImpulseM;
				dTotalImpulses[i] = dImpulseM;

				// update relative velocity (apply the delta impulse virtually)
				for (int j = 0; j < numRows; j++) {
					relVels[j] += dImpulseM * invMass[i][j];
				}
			}
		}

		// try all the boundaries
		boolean solved = false;
		for (int i = 0; i < boundaryBuilder.numBoundaries; i++) {
			// select a boundary
			int idx = velBoundarySelector.getIndex(i);
			Boundary b = boundaryBuilder.boundaries[idx];

			// try the case
			if (b.computeImpulses(info, massMatrix, relVels, impulses, dImpulses, 1, false)) {
				// the solution found
				for (int j = 0; j < numRows; j++) {
					JointSolverInfoRow row = info.rows[j];
					JointImpulse imp = row.impulse;
					double dimp = dImpulses[j];

					// accumulate the delta impulses
					imp.impulse += dimp;
					dTotalImpulses[j] += dimp;
				}

				// apply motor + limit impulses
				applyImpulses(dTotalImpulses);

				// make the priority of the boundary higher for the next iteration
				velBoundarySelector.select(idx);
				solved = true;
				break;
			}
		}

		if (!solved) {
			System.out.println("could not find solution. (velocity)");
			return;
		}
	}

	@Override
	public void postSolveVelocity(TimeStep timeStep) {
		// compute total linear and angular impulse
		Vec3 lin = new Vec3();
		Vec3 ang = new Vec3();

		for (int i = 0; i < info.numRows; i++) {
			JointSolverInfoRow row = info.rows[i];
			JointImpulse imp = row.impulse;
			JacobianRow j = row.jacobian;
			if (j.isLinearSet()) {
				// assume that this row is linear
				M.vec3_addRhsScaled(lin, lin, j.lin1, imp.impulse);
			} else if (j.isAngularSet()) {
				// assume that this row is angular
				M.vec3_addRhsScaled(ang, ang, j.ang1, imp.impulse);
			}
		}

		M.vec3_scale(joint._appliedForce, lin, timeStep.invDt);
		M.vec3_scale(joint._appliedTorque, ang, timeStep.invDt);
	}

	public void updatePositionData() {
		joint._syncAnchors();
		joint._getPositionSolverInfo(info);

		_b1 = info.b1;
		_b2 = info.b2;

		// compute inverse mass matrix
		massMatrix.computeInvMass(info, massData);

		// build boundaries
		boundaryBuilder.buildBoundaries(info);

		// update the size of the boundary selector
		posBoundarySelector.setSize(boundaryBuilder.numBoundaries);
	}

	@Override
	public void preSolvePosition(TimeStep timeStep) {
		updatePositionData();

		// clear position impulses
		for (int i = 0; i < info.numRows; i++) {
			info.rows[i].impulse.impulseP = 0;
		}
	}

	@Override
	public void solvePositionSplitImpulse() {
		int numRows = info.numRows;
		Vec3 lv1 = new Vec3();
		Vec3 lv2 = new Vec3();
		Vec3 av1 = new Vec3();
		Vec3 av2 = new Vec3();
		M.vec3_assign(lv1, _b1._pseudoVel);
		M.vec3_assign(lv2, _b2._pseudoVel);
		M.vec3_assign(av1, _b1._angPseudoVel);
		M.vec3_assign(av2, _b2._angPseudoVel);

		for (int i = 0; i < numRows; i++) {
			JointSolverInfoRow row = info.rows[i];
			JointImpulse imp = row.impulse;
			JacobianRow j = row.jacobian;

			// compute relative velocity
			double relVel = 0;
			relVel += M.vec3_dot(lv1, j.lin1);
			relVel -= M.vec3_dot(lv2, j.lin2);
			relVel += M.vec3_dot(av1, j.ang1);
			relVel -= M.vec3_dot(av2, j.ang2);
			relVels[i] = relVel;

			// trace('pre relVels[$i] = ${relVels[i]}');

			// get impulse
			impulses[i] = imp.impulseP;
		}

		// try all the boundaries
		boolean solved = false;
		for (int i = 0; i < boundaryBuilder.numBoundaries; i++) {
			// select a boundary
			int idx = posBoundarySelector.getIndex(i);
			Boundary b = boundaryBuilder.boundaries[idx];

			// try the case
			if (b.computeImpulses(info, massMatrix, relVels, impulses, dImpulses, Setting.positionSplitImpulseBaumgarte,
					false)) {
				// the solution found
				for (int j = 0; j < numRows; j++) {
					JointSolverInfoRow row = info.rows[j];
					JointImpulse imp = row.impulse;
					double dimp = dImpulses[j];

					// accumulate the delta impulses
					imp.impulseP += dimp;
				}

				// apply delta impulses
				applySplitImpulses(dImpulses);

				// make the priority of the boundary higher for the next iteration
				posBoundarySelector.select(idx);
				solved = true;
				break;
			}
		}

		if (!solved) {
			M.trace("could not find solution. (split impulse)");
			return;
		}
	}

	@Override
	public void solvePositionNgs(TimeStep timeStep) {
		updatePositionData();

		int numRows = info.numRows;

		for (int i = 0; i < numRows; i++) {
			JointSolverInfoRow row = info.rows[i];
			JointImpulse imp = row.impulse;
			JacobianRow j = row.jacobian;

			// set relative velocity zero for NGS
			relVels[i] = 0;

			// get impulse
			impulses[i] = imp.impulseP;
		}

		// try all the boundaries
		boolean solved = false;
		for (int i = 0; i < boundaryBuilder.numBoundaries; i++) {
			// select a boundary
			int idx = posBoundarySelector.getIndex(i);
			Boundary b = boundaryBuilder.boundaries[idx];

			// try the case
			if (b.computeImpulses(info, massMatrix, relVels, impulses, dImpulses, Setting.positionNgsBaumgarte,
					false)) {
				// the solution found
				for (int j = 0; j < numRows; j++) {
					JointSolverInfoRow row = info.rows[j];
					JointImpulse imp = row.impulse;
					double dimp = dImpulses[j];

					// accumulate the delta impulses
					imp.impulseP += dimp;
				}

				// apply delta impulses
				applyPositionImpulses(dImpulses);

				// make the priority of the boundary higher for the next iteration
				posBoundarySelector.select(idx);
				solved = true;
				break;
			}
		}

		if (!solved) {
			M.trace("could not find solution. (NGS)");
			return;
		}
	}

	@Override
	public void postSolve() {
		joint._syncAnchors();
		joint._checkDestruction();
	}

}
