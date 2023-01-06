package oimo.dynamics.constraint.solver.pgs;
import oimo.dynamics.constraint.contact.ContactConstraint;
import oimo.dynamics.constraint.contact.ContactImpulse;
import oimo.dynamics.constraint.info.contact.ContactSolverInfo;
import oimo.dynamics.constraint.info.contact.ContactSolverInfoRow;
import oimo.common.M;
import oimo.common.Mat3;
import oimo.common.MathUtil;
import oimo.common.Setting;
import oimo.common.Vec3;
import oimo.dynamics.TimeStep;
import oimo.dynamics.constraint.ConstraintSolver;
import oimo.dynamics.constraint.contact.*;
import oimo.dynamics.constraint.info.JacobianRow;
import oimo.dynamics.constraint.info.contact.*;
import oimo.dynamics.constraint.solver.common.ContactSolverMassDataRow;

/**
 * A contact constraint solver using projected Gauss-Seidel (sequential impulse).
 */
public class PgsContactConstraintSolver extends ConstraintSolver {
	ContactConstraint constraint;

	ContactSolverInfo info;

	ContactSolverMassDataRow[] massData;

	public PgsContactConstraintSolver(ContactConstraint constraint) {
		super();
		this.constraint = constraint;

		info = new ContactSolverInfo();

		massData = new ContactSolverMassDataRow[Setting.maxManifoldPoints];

		for (int i=0;i<massData.length;i++) {
			massData[i] = new ContactSolverMassDataRow();
		}
	}

	@Override 
	public void preSolveVelocity(TimeStep timeStep) {
		constraint._getVelocitySolverInfo(timeStep, info);

		_b1 = info.b1;
		_b2 = info.b2;

		double invM1 = _b1._invMass;
		double invM2 = _b2._invMass;

		var invI1= _b1._invInertia;
		var invI2= _b2._invInertia;
		//M.mat3_assign(invI1, _b1._invInertia);
		//M.mat3_assign(invI2, _b2._invInertia);

		// compute mass data
		for (int i=0; i<info.numRows;i++) {
			ContactSolverInfoRow row = info.rows[i];
			ContactSolverMassDataRow md = massData[i];
			JacobianRow j;

			// normal mass
			j = row.jacobianN;
			M.vec3_scale(md.invMLinN1, j.lin1, invM1);
			M.vec3_scale(md.invMLinN2, j.lin2, invM2);
			M.vec3_mulMat3(md.invMAngN1, j.ang1, invI1);
			M.vec3_mulMat3(md.invMAngN2, j.ang2, invI2);

			md.massN = invM1 + invM2 + M.vec3_dot(md.invMAngN1, j.ang1) + M.vec3_dot(md.invMAngN2, j.ang2);
			if (md.massN != 0) md.massN = 1 / md.massN;

			// tangent/binormal mass
			JacobianRow jt = row.jacobianT;
			JacobianRow jb = row.jacobianB;
			M.vec3_scale(md.invMLinT1, jt.lin1, invM1);
			M.vec3_scale(md.invMLinT2, jt.lin2, invM2);
			M.vec3_scale(md.invMLinB1, jb.lin1, invM1);
			M.vec3_scale(md.invMLinB2, jb.lin2, invM2);
			M.vec3_mulMat3(md.invMAngT1, jt.ang1, invI1);
			M.vec3_mulMat3(md.invMAngT2, jt.ang2, invI2);
			M.vec3_mulMat3(md.invMAngB1, jb.ang1, invI1);
			M.vec3_mulMat3(md.invMAngB2, jb.ang2, invI2);

			// compute effective mass matrix for friction
			double invMassTB00 = invM1 + invM2 + M.vec3_dot(md.invMAngT1, jt.ang1) + M.vec3_dot(md.invMAngT2, jt.ang2);
			double invMassTB01 = M.vec3_dot(md.invMAngT1, jb.ang1) + M.vec3_dot(md.invMAngT2, jb.ang2);
			double invMassTB10 = invMassTB01;
			double invMassTB11 = invM1 + invM2 + M.vec3_dot(md.invMAngB1, jb.ang1) + M.vec3_dot(md.invMAngB2, jb.ang2);

			double invDet = invMassTB00 * invMassTB11 - invMassTB01 * invMassTB10;
			if (invDet != 0) invDet = 1 / invDet;

			md.massTB00 = invMassTB11 * invDet;
			md.massTB01 = -invMassTB01 * invDet;
			md.massTB10 = -invMassTB10 * invDet;
			md.massTB11 = invMassTB00 * invDet;
		}
	}

	@Override 
	public void warmStart(TimeStep timeStep) {
		Vec3 lv1=new Vec3();
		Vec3 lv2=new Vec3();
		Vec3 av1=new Vec3();
		Vec3 av2=new Vec3();
		M.vec3_assign(lv1, _b1._vel);
		M.vec3_assign(lv2, _b2._vel);
		M.vec3_assign(av1, _b1._angVel);
		M.vec3_assign(av2, _b2._angVel);

		for (int i=0;i<info.numRows;i++) {
			ContactSolverInfoRow row = info.rows[i];
			ContactImpulse imp = row.impulse;
			ContactSolverMassDataRow md = massData[i];
			JacobianRow jt = row.jacobianT;
			JacobianRow jb = row.jacobianB;

			double impulseN = imp.impulseN;
			double impulseT = M.vec3_dot(imp.impulseL, jt.lin1);
			double impulseB = M.vec3_dot(imp.impulseL, jb.lin1);
			imp.impulseT = impulseT;
			imp.impulseB = impulseB;

			// adjust impulse for variable time step
			imp.impulseN *= timeStep.dtRatio;
			imp.impulseT *= timeStep.dtRatio;
			imp.impulseB *= timeStep.dtRatio;

			M.vec3_addRhsScaled(lv1, lv1, md.invMLinN1, impulseN);
			M.vec3_addRhsScaled(lv1, lv1, md.invMLinT1, impulseT);
			M.vec3_addRhsScaled(lv1, lv1, md.invMLinB1, impulseB);
			M.vec3_addRhsScaled(lv2, lv2, md.invMLinN2, -impulseN);
			M.vec3_addRhsScaled(lv2, lv2, md.invMLinT2, -impulseT);
			M.vec3_addRhsScaled(lv2, lv2, md.invMLinB2, -impulseB);
			M.vec3_addRhsScaled(av1, av1, md.invMAngN1, impulseN);
			M.vec3_addRhsScaled(av1, av1, md.invMAngT1, impulseT);
			M.vec3_addRhsScaled(av1, av1, md.invMAngB1, impulseB);
			M.vec3_addRhsScaled(av2, av2, md.invMAngN2, -impulseN);
			M.vec3_addRhsScaled(av2, av2, md.invMAngT2, -impulseT);
			M.vec3_addRhsScaled(av2, av2, md.invMAngB2, -impulseB);
		}

		M.vec3_assign(_b1._vel, lv1);
		M.vec3_assign(_b2._vel, lv2);
		M.vec3_assign(_b1._angVel, av1);
		M.vec3_assign(_b2._angVel, av2);
	}

	@Override 
	public void solveVelocity() {
		Vec3 lv1=new Vec3();
		Vec3 lv2=new Vec3();
		Vec3 av1=new Vec3();
		Vec3 av2=new Vec3();
		M.vec3_assign(lv1, _b1._vel);
		M.vec3_assign(lv2, _b2._vel);
		M.vec3_assign(av1, _b1._angVel);
		M.vec3_assign(av2, _b2._angVel);

		// solve friction
		for (int i=0;i<info.numRows;i++) {
			ContactSolverInfoRow row = info.rows[i];
			ContactSolverMassDataRow md = massData[i];
			ContactImpulse imp = row.impulse;
			JacobianRow j;

			// measure relative velocity
			double rvt = 0;
			j = row.jacobianT;
			rvt += M.vec3_dot(lv1, j.lin1);
			rvt -= M.vec3_dot(lv2, j.lin2);
			rvt += M.vec3_dot(av1, j.ang1);
			rvt -= M.vec3_dot(av2, j.ang2);

			double rvb = 0;
			j = row.jacobianB;
			rvb += M.vec3_dot(lv1, j.lin1);
			rvb -= M.vec3_dot(lv2, j.lin2);
			rvb += M.vec3_dot(av1, j.ang1);
			rvb -= M.vec3_dot(av2, j.ang2);

			double impulseT = -(rvt * md.massTB00 + rvb * md.massTB01);
			double impulseB = -(rvt * md.massTB10 + rvb * md.massTB11);

			double oldImpulseT = imp.impulseT;
			double oldImpulseB = imp.impulseB;
			imp.impulseT += impulseT;
			imp.impulseB += impulseB;

			// cone friction
			double maxImpulse = row.friction * imp.impulseN;
			if (maxImpulse == 0) {
				imp.impulseT = 0;
				imp.impulseB = 0;
			} else {
				double impulseLengthSq = imp.impulseT * imp.impulseT + imp.impulseB * imp.impulseB;
				if (impulseLengthSq > maxImpulse * maxImpulse) {
					double invL = maxImpulse / MathUtil.sqrt(impulseLengthSq);
					imp.impulseT *= invL;
					imp.impulseB *= invL;
				}
			}

			impulseT = imp.impulseT - oldImpulseT;
			impulseB = imp.impulseB - oldImpulseB;

			// apply delta impulse
			M.vec3_addRhsScaled(lv1, lv1, md.invMLinT1, impulseT);
			M.vec3_addRhsScaled(lv1, lv1, md.invMLinB1, impulseB);
			M.vec3_addRhsScaled(lv2, lv2, md.invMLinT2, -impulseT);
			M.vec3_addRhsScaled(lv2, lv2, md.invMLinB2, -impulseB);
			M.vec3_addRhsScaled(av1, av1, md.invMAngT1, impulseT);
			M.vec3_addRhsScaled(av1, av1, md.invMAngB1, impulseB);
			M.vec3_addRhsScaled(av2, av2, md.invMAngT2, -impulseT);
			M.vec3_addRhsScaled(av2, av2, md.invMAngB2, -impulseB);
		}

		// solve normal
		for (int i=0;i<info.numRows;i++) {
			ContactSolverInfoRow row = info.rows[i];
			ContactSolverMassDataRow md = massData[i];
			ContactImpulse imp = row.impulse;
			JacobianRow j;

			// measure relative velocity
			double rvn = 0;
			j = row.jacobianN;
			rvn += M.vec3_dot(lv1, j.lin1);
			rvn -= M.vec3_dot(lv2, j.lin2);
			rvn += M.vec3_dot(av1, j.ang1);
			rvn -= M.vec3_dot(av2, j.ang2);

			double impulseN = (row.rhs - rvn) * md.massN;
			//System.out.println(impulseN+""+j.lin1+j.lin2+rvn);
			// clamp impulse
			double oldImpulseN = imp.impulseN;
			imp.impulseN += impulseN;
			if (imp.impulseN < 0) imp.impulseN = 0;
			impulseN = imp.impulseN - oldImpulseN;

			// apply delta impulse
			M.vec3_addRhsScaled(lv1, lv1, md.invMLinN1, impulseN);
			M.vec3_addRhsScaled(lv2, lv2, md.invMLinN2, -impulseN);
			M.vec3_addRhsScaled(av1, av1, md.invMAngN1, impulseN);
			M.vec3_addRhsScaled(av2, av2, md.invMAngN2, -impulseN);
		}

		M.vec3_assign(_b1._vel, lv1);
		M.vec3_assign(_b2._vel, lv2);
		M.vec3_assign(_b1._angVel, av1);
		M.vec3_assign(_b2._angVel, av2);
	}

	public void updatePositionData() {
		constraint._syncManifold();
		constraint._getPositionSolverInfo(info);

		double invM1 = _b1._invMass;
		double invM2 = _b2._invMass;

		Mat3 invI1=_b1._invInertia;
		Mat3 invI2 =_b2._invInertia;
		//M.mat3_assign(invI1, _b1._invInertia);
		//M.mat3_assign(invI2, _b2._invInertia);

		// compute mass data
		for (int i=0;i<info.numRows;i++) {
			ContactSolverInfoRow row = info.rows[i];
			ContactSolverMassDataRow md = massData[i];
			JacobianRow j = row.jacobianN;

			M.vec3_scale(md.invMLinN1, j.lin1, invM1);
			M.vec3_scale(md.invMLinN2, j.lin2, invM2);
			M.vec3_mulMat3(md.invMAngN1, j.ang1, invI1);
			M.vec3_mulMat3(md.invMAngN2, j.ang2, invI2);

			md.massN = invM1 + invM2 + M.vec3_dot(md.invMAngN1, j.ang1) + M.vec3_dot(md.invMAngN2, j.ang2);
			if (md.massN != 0) md.massN = 1 / md.massN;
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
		Vec3 lv1=new Vec3();
		Vec3 lv2=new Vec3();
		Vec3 av1=new Vec3();
		Vec3 av2=new Vec3();
		M.vec3_assign(lv1, _b1._pseudoVel);
		M.vec3_assign(lv2, _b2._pseudoVel);
		M.vec3_assign(av1, _b1._angPseudoVel);
		M.vec3_assign(av2, _b2._angPseudoVel);

		// solve normal
		for (int i=0;i<info.numRows;i++) {
			ContactSolverInfoRow row = info.rows[i];
			ContactSolverMassDataRow md = massData[i];
			ContactImpulse imp = row.impulse;
			JacobianRow j = row.jacobianN;

			// measure relative velocity
			double rvn = 0;
			rvn += M.vec3_dot(lv1, j.lin1);
			rvn -= M.vec3_dot(lv2, j.lin2);
			rvn += M.vec3_dot(av1, j.ang1);
			rvn -= M.vec3_dot(av2, j.ang2);

			double impulseP = (row.rhs - rvn) * md.massN * Setting.positionSplitImpulseBaumgarte;

			// clamp impulse
			double oldImpulseP = imp.impulseP;
			imp.impulseP += impulseP;
			if (imp.impulseP < 0) imp.impulseP = 0;
			impulseP = imp.impulseP - oldImpulseP;

			// apply delta impulse
			M.vec3_addRhsScaled(lv1, lv1, md.invMLinN1, impulseP);
			M.vec3_addRhsScaled(lv2, lv2, md.invMLinN2, -impulseP);
			M.vec3_addRhsScaled(av1, av1, md.invMAngN1, impulseP);
			M.vec3_addRhsScaled(av2, av2, md.invMAngN2, -impulseP);
		}

		M.vec3_assign(_b1._pseudoVel, lv1);
		M.vec3_assign(_b2._pseudoVel, lv2);
		M.vec3_assign(_b1._angPseudoVel, av1);
		M.vec3_assign(_b2._angPseudoVel, av2);
	}

	@Override 
	public void solvePositionNgs(TimeStep timeStep) {
		updatePositionData();

		Vec3 lv1=new Vec3();
		Vec3 lv2=new Vec3();
		Vec3 av1=new Vec3();
		Vec3 av2=new Vec3();

		for (int i=0;i<info.numRows;i++) {
			ContactSolverInfoRow row = info.rows[i];
			ContactSolverMassDataRow md = massData[i];
			ContactImpulse imp = row.impulse;
			JacobianRow j = row.jacobianN;

			// estimate translation along the normal
			double rvn = 0;
			rvn += M.vec3_dot(lv1, j.lin1);
			rvn -= M.vec3_dot(lv2, j.lin2);
			rvn += M.vec3_dot(av1, j.ang1);
			rvn -= M.vec3_dot(av2, j.ang2);

			double impulseP = (row.rhs - rvn) * md.massN * Setting.positionNgsBaumgarte;

			// clamp impulse
			double oldImpulseP = imp.impulseP;
			imp.impulseP += impulseP;
			if (imp.impulseP < 0) imp.impulseP = 0;
			impulseP = imp.impulseP - oldImpulseP;

			// apply delta impulse
			M.vec3_addRhsScaled(lv1, lv1, md.invMLinN1, impulseP);
			M.vec3_addRhsScaled(lv2, lv2, md.invMLinN2, -impulseP);
			M.vec3_addRhsScaled(av1, av1, md.invMAngN1, impulseP);
			M.vec3_addRhsScaled(av2, av2, md.invMAngN2, -impulseP);
		}

		_b1._applyTranslation(lv1);
		_b2._applyTranslation(lv2);
		_b1._applyRotation(av1);
		_b2._applyRotation(av2);
		
		//		M.call(_b1._applyTranslation(lv1));
		//		M.call(_b2._applyTranslation(lv2));
		//		M.call(_b1._applyRotation(av1));
		//		M.call(_b2._applyRotation(av2));
	}

	@Override 
	public void postSolve() {
		// contact impulses
		Vec3 lin1=new Vec3();
		// lin2 == lin1
		Vec3 ang1=new Vec3();
		Vec3 ang2=new Vec3();

		for (int i=0;i<info.numRows;i++) {
			ContactSolverInfoRow row = info.rows[i];
			ContactImpulse imp = row.impulse;
			JacobianRow jn = row.jacobianN;
			JacobianRow jt = row.jacobianT;
			JacobianRow jb = row.jacobianB;
			double impN = imp.impulseN;
			double impT = imp.impulseT;
			double impB = imp.impulseB;
			Vec3 impulseL = new Vec3();

			// store lateral impulse
			M.vec3_zero(impulseL);
			M.vec3_addRhsScaled(impulseL, impulseL, jt.lin1, impT);
			M.vec3_addRhsScaled(impulseL, impulseL, jb.lin1, impB);
			M.vec3_assign(imp.impulseL, impulseL);

			// accumulate contact impulses
			M.vec3_addRhsScaled(lin1, lin1, jn.lin1, impN);
			M.vec3_addRhsScaled(ang1, ang1, jn.ang1, impN);
			M.vec3_addRhsScaled(ang2, ang2, jn.ang2, impN);
			M.vec3_addRhsScaled(lin1, lin1, jt.lin1, impT);
			M.vec3_addRhsScaled(ang1, ang1, jt.ang1, impT);
			M.vec3_addRhsScaled(ang2, ang2, jt.ang2, impT);
			M.vec3_addRhsScaled(lin1, lin1, jb.lin1, impB);
			M.vec3_addRhsScaled(ang1, ang1, jb.ang1, impB);
			M.vec3_addRhsScaled(ang2, ang2, jb.ang2, impB);
		}

		M.vec3_add(_b1._linearContactImpulse, _b1._linearContactImpulse, lin1);
		M.vec3_add(_b1._angularContactImpulse, _b1._angularContactImpulse, ang1);
		M.vec3_sub(_b2._linearContactImpulse, _b2._linearContactImpulse, lin1);
		M.vec3_sub(_b2._angularContactImpulse, _b2._angularContactImpulse, ang2);

		constraint._syncManifold();
	}

	@Override
	public void postSolveVelocity(TimeStep timeStep) {
		// TODO Auto-generated method stub
		
	}

}
