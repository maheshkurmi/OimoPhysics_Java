package oimo.dynamics.constraint.solver.pgs;
import oimo.common.M;
import oimo.common.Mat3;
import oimo.common.Setting;
import oimo.common.Vec3;
import oimo.dynamics.TimeStep;
import oimo.dynamics.constraint.*;
import oimo.dynamics.constraint.info.JacobianRow;
import oimo.dynamics.constraint.info.joint.*;
import oimo.dynamics.constraint.joint.*;
import oimo.dynamics.constraint.solver.common.JointSolverMassDataRow;

/**
 * A joint constraint solver using projected Gauss-Seidel (sequential impulse).
 */
public class PgsJointConstraintSolver extends ConstraintSolver {
	Joint joint;
	JointSolverInfo info;
	JointSolverMassDataRow[] massData;

	public PgsJointConstraintSolver(Joint joint) {
		super();
		this.joint = joint;

		info = new JointSolverInfo();

		massData = new JointSolverMassDataRow[Setting.maxJacobianRows];
		for (int i=0;i<massData.length;i++) {
			massData[i] = new JointSolverMassDataRow();
		}
	}

	@Override 
	public void preSolveVelocity(TimeStep timeStep) {
		joint._syncAnchors();
		joint._getVelocitySolverInfo(timeStep, info);

		_b1 = info.b1;
		_b2 = info.b2;

		double invM1 = _b1._invMass;
		double invM2 = _b2._invMass;

		Mat3 invI1=_b1._invInertia;
		Mat3 invI2=_b2._invInertia;
		//M.mat3_assign(invI1, _b1._invInertia);
		//M.mat3_assign(invI2, _b2._invInertia);

		// compute mass data
		for (int i=0;i<info.numRows;i++) {
			JointSolverInfoRow row = info.rows[i];
			JointSolverMassDataRow md = massData[i];
			JacobianRow j = row.jacobian;

			j.updateSparsity();

			if (j.isLinearSet()) {
				M.vec3_scale(md.invMLin1, j.lin1, invM1);
				M.vec3_scale(md.invMLin2, j.lin2, invM2);
			} else {
				M.vec3_zero(md.invMLin1);
				M.vec3_zero(md.invMLin2);
			}

			if (j.isAngularSet()) {
				M.vec3_mulMat3(md.invMAng1, j.ang1, invI1);
				M.vec3_mulMat3(md.invMAng2, j.ang2, invI2);
			} else {
				M.vec3_zero(md.invMAng1);
				M.vec3_zero(md.invMAng2);
			}

			md.massWithoutCfm =
				M.vec3_dot(md.invMLin1, j.lin1) +
				M.vec3_dot(md.invMLin2, j.lin2) +
				M.vec3_dot(md.invMAng1, j.ang1) +
				M.vec3_dot(md.invMAng2, j.ang2)
			;
			md.mass = md.massWithoutCfm + row.cfm;

			if (md.massWithoutCfm != 0) md.massWithoutCfm = 1 / md.massWithoutCfm;
			if (md.mass != 0) md.mass = 1 / md.mass;
		}
	}

	@Override 
	public void warmStart(TimeStep timeStep) {

		double factor= joint._getWarmStartingFactor();

		// adjust impulse for variable time step
		factor *= timeStep.dtRatio;

		// warm start disabled
		if (factor <= 0) {
			for (int i=0;i< info.numRows;i++) {
				JointSolverInfoRow row = info.rows[i];
				row.impulse.clear();
			}
			return;
		}

		Vec3 lv1 = new Vec3();
		Vec3 lv2 = new Vec3();
		Vec3 av1 = new Vec3();
		Vec3 av2 = new Vec3();
		M.vec3_assign(lv1, _b1._vel);
		M.vec3_assign(lv2, _b2._vel);
		M.vec3_assign(av1, _b1._angVel);
		M.vec3_assign(av2, _b2._angVel);

		for (int i=0;i<info.numRows;i++) {
			JointSolverInfoRow row = info.rows[i];
			JointSolverMassDataRow md = massData[i];
			JointImpulse imp = row.impulse;
			JacobianRow j = row.jacobian;

			imp.impulse *= factor;
			imp.impulseM *= factor;

			double impulse = imp.impulse + imp.impulseM;

			// apply initial impulse
			M.vec3_addRhsScaled(lv1, lv1, md.invMLin1, impulse);
			M.vec3_addRhsScaled(lv2, lv2, md.invMLin2, -impulse);
			M.vec3_addRhsScaled(av1, av1, md.invMAng1, impulse);
			M.vec3_addRhsScaled(av2, av2, md.invMAng2, -impulse);
		}

		M.vec3_assign(_b1._vel, lv1);
		M.vec3_assign(_b2._vel, lv2);
		M.vec3_assign(_b1._angVel, av1);
		M.vec3_assign(_b2._angVel, av2);
	}

	@Override 
	public void solveVelocity() {
		Vec3 lv1 = new Vec3();
		Vec3 lv2 = new Vec3();
		Vec3 av1 = new Vec3();
		Vec3 av2 = new Vec3();
		M.vec3_assign(lv1, _b1._vel);
		M.vec3_assign(lv2, _b2._vel);
		M.vec3_assign(av1, _b1._angVel);
		M.vec3_assign(av2, _b2._angVel);

		// solve motor
		for (int i=0;i<info.numRows;i++) {
			JointSolverInfoRow row = info.rows[i];
			JointSolverMassDataRow md = massData[i];
			JointImpulse imp = row.impulse;
			JacobianRow j = row.jacobian;

			if (row.motorMaxImpulse == 0) continue;

			// measure relative velocity
			double rv = 0;
			rv += M.vec3_dot(lv1, j.lin1);
			rv -= M.vec3_dot(lv2, j.lin2);
			rv += M.vec3_dot(av1, j.ang1);
			rv -= M.vec3_dot(av2, j.ang2);

			double impulseM = (-row.motorSpeed - rv) * md.massWithoutCfm;

			// clamp impulse
			double oldImpulseM = imp.impulseM;
			imp.impulseM += impulseM;
			if (imp.impulseM < -row.motorMaxImpulse) {
				imp.impulseM = -row.motorMaxImpulse;
			} else if (imp.impulseM > row.motorMaxImpulse) {
				imp.impulseM = row.motorMaxImpulse;
			}
			impulseM = imp.impulseM - oldImpulseM;

			// apply delta impulse
			if (j.isLinearSet()) {
				M.vec3_addRhsScaled(lv1, lv1, md.invMLin1, impulseM);
				M.vec3_addRhsScaled(lv2, lv2, md.invMLin2, -impulseM);
			}
			if (j.isAngularSet()) {
				M.vec3_addRhsScaled(av1, av1, md.invMAng1, impulseM);
				M.vec3_addRhsScaled(av2, av2, md.invMAng2, -impulseM);
			}
		}

		// solve normal
		for (int i=0;i< info.numRows;i++) {
			JointSolverInfoRow row = info.rows[i];
			JointSolverMassDataRow md = massData[i];
			JointImpulse imp = row.impulse;
			JacobianRow j = row.jacobian;

			// measure relative velocity
			double rv = 0;
			rv += M.vec3_dot(lv1, j.lin1);
			rv -= M.vec3_dot(lv2, j.lin2);
			rv += M.vec3_dot(av1, j.ang1);
			rv -= M.vec3_dot(av2, j.ang2);

			double impulse = (row.rhs - rv - imp.impulse * row.cfm) * md.mass;

			// clamp impulse
			double oldImpulse = imp.impulse;
			imp.impulse += impulse;
			if (imp.impulse < row.minImpulse) {
				imp.impulse = row.minImpulse;
			} else if (imp.impulse > row.maxImpulse) {
				imp.impulse = row.maxImpulse;
			}
			impulse = imp.impulse - oldImpulse;

			// apply delta impulse
			if (j.isLinearSet()) {
				M.vec3_addRhsScaled(lv1, lv1, md.invMLin1, impulse);
				M.vec3_addRhsScaled(lv2, lv2, md.invMLin2, -impulse);
			}
			if (j.isAngularSet()) {
				M.vec3_addRhsScaled(av1, av1, md.invMAng1, impulse);
				M.vec3_addRhsScaled(av2, av2, md.invMAng2, -impulse);
			}
		}

		M.vec3_assign(_b1._vel, lv1);
		M.vec3_assign(_b2._vel, lv2);
		M.vec3_assign(_b1._angVel, av1);
		M.vec3_assign(_b2._angVel, av2);
	}

	@Override 
	public void postSolveVelocity(TimeStep timeStep) {
		// compute total linear and angular impulse
		Vec3 lin =new Vec3();
		Vec3 ang=new Vec3();

		for (int i=0;i<info.numRows;i++) {
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

		double invM1 = _b1._invMass;
		double invM2 = _b2._invMass;

		Mat3 invI1=_b1._invInertia;
		Mat3 invI2=_b2._invInertia;
		M.mat3_assign(invI1, _b1._invInertia);
		M.mat3_assign(invI2, _b2._invInertia);

		// compute mass data
		for (int i=0;i<info.numRows;i++) {
			JointSolverInfoRow row = info.rows[i];
			JointSolverMassDataRow md = massData[i];
			JointImpulse imp = row.impulse;
			JacobianRow j = row.jacobian;

			M.vec3_scale(md.invMLin1, j.lin1, invM1);
			M.vec3_scale(md.invMLin2, j.lin2, invM2);
			M.vec3_mulMat3(md.invMAng1, j.ang1, invI1);
			M.vec3_mulMat3(md.invMAng2, j.ang2, invI2);

			md.mass =
				M.vec3_dot(md.invMLin1, j.lin1) +
				M.vec3_dot(md.invMLin2, j.lin2) +
				M.vec3_dot(md.invMAng1, j.ang1) +
				M.vec3_dot(md.invMAng2, j.ang2)
			;

			if (md.mass != 0) md.mass = 1 / md.mass;
		}
	}

	@Override 
	public void preSolvePosition(TimeStep timeStep) {
		updatePositionData();

		// clear position impulses
		for (int i=0;i<info.numRows;i++) {
			info.rows[i].impulse.impulseP = 0;
		}
	}

	@Override 
	public void solvePositionSplitImpulse() {
		Vec3 lv1 = new Vec3();
		Vec3 lv2 = new Vec3();
		Vec3 av1 = new Vec3();
		Vec3 av2 = new Vec3();
		M.vec3_assign(lv1, _b1._pseudoVel);
		M.vec3_assign(lv2, _b2._pseudoVel);
		M.vec3_assign(av1, _b1._angPseudoVel);
		M.vec3_assign(av2, _b2._angPseudoVel);

		for (int i=0;i<info.numRows;i++) {
			JointSolverInfoRow row = info.rows[i];
			JointSolverMassDataRow md = massData[i];
			JointImpulse imp = row.impulse;
			JacobianRow j = row.jacobian;

			// measure relative velocity
			double rv = 0;
			rv += M.vec3_dot(lv1, j.lin1);
			rv -= M.vec3_dot(lv2, j.lin2);
			rv += M.vec3_dot(av1, j.ang1);
			rv -= M.vec3_dot(av2, j.ang2);

			double impulseP = (row.rhs * Setting.positionSplitImpulseBaumgarte - rv) * md.mass;

			// clamp impulse
			double oldImpulseP = imp.impulseP;
			imp.impulseP += impulseP;
			if (imp.impulseP < row.minImpulse) {
				imp.impulseP = row.minImpulse;
			} else if (imp.impulseP > row.maxImpulse) {
				imp.impulseP = row.maxImpulse;
			}

			impulseP = imp.impulseP - oldImpulseP;

			// apply delta impulse
			M.vec3_addRhsScaled(lv1, lv1, md.invMLin1, impulseP);
			M.vec3_addRhsScaled(lv2, lv2, md.invMLin2, -impulseP);
			M.vec3_addRhsScaled(av1, av1, md.invMAng1, impulseP);
			M.vec3_addRhsScaled(av2, av2, md.invMAng2, -impulseP);
		}

		M.vec3_assign(_b1._pseudoVel, lv1);
		M.vec3_assign(_b2._pseudoVel, lv2);
		M.vec3_assign(_b1._angPseudoVel, av1);
		M.vec3_assign(_b2._angPseudoVel, av2);
	}

	@Override 
	public void solvePositionNgs(TimeStep timeStep) {
		updatePositionData();

		Vec3 lv1 = new Vec3();
		Vec3 lv2 = new Vec3();
		Vec3 av1 = new Vec3();
		Vec3 av2 = new Vec3();

		for (int i=0;i<info.numRows;i++) {
			JointSolverInfoRow row = info.rows[i];
			JointSolverMassDataRow md = massData[i];
			JointImpulse imp = row.impulse;
			JacobianRow j = row.jacobian;

			// measure relative velocity
			double rv = 0;
			rv += M.vec3_dot(lv1, j.lin1);
			rv -= M.vec3_dot(lv2, j.lin2);
			rv += M.vec3_dot(av1, j.ang1);
			rv -= M.vec3_dot(av2, j.ang2);

			double impulseP = (row.rhs * Setting.positionNgsBaumgarte - rv) * md.mass;

			// clamp impulse
			double oldImpulseP = imp.impulseP;
			imp.impulseP += impulseP;
			if (imp.impulseP < row.minImpulse) {
				imp.impulseP = row.minImpulse;
			} else if (imp.impulseP > row.maxImpulse) {
				imp.impulseP = row.maxImpulse;
			}

			impulseP = imp.impulseP - oldImpulseP;

			// apply delta impulse
			M.vec3_addRhsScaled(lv1, lv1, md.invMLin1, impulseP);
			M.vec3_addRhsScaled(lv2, lv2, md.invMLin2, -impulseP);
			M.vec3_addRhsScaled(av1, av1, md.invMAng1, impulseP);
			M.vec3_addRhsScaled(av2, av2, md.invMAng2, -impulseP);
		}

		_b1._applyTranslation(lv1);
		_b2._applyTranslation(lv2);
		_b1._applyRotation(av1);
		_b2._applyRotation(av2);
	}

	@Override 
	public void postSolve() {
		joint._syncAnchors();
		joint._checkDestruction();
	}

}
