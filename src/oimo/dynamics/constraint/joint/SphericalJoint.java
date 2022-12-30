package oimo.dynamics.constraint.joint;

import oimo.common.M;
import oimo.common.Mat3;
import oimo.common.MathUtil;
import oimo.common.Setting;
import oimo.common.Vec3;
import oimo.dynamics.TimeStep;
import oimo.dynamics.constraint.info.JacobianRow;
import oimo.dynamics.constraint.info.joint.JointSolverInfo;
import oimo.dynamics.constraint.info.joint.JointSolverInfoRow;

/**
 * A spherical joint (a.k.a. ball and socket joint) constrains two rigid bodies to share
 * their anchor points. This joint provides three degrees of freedom. You can enable a
 * spring and damper effect of the constraint.
 */
public class SphericalJoint extends Joint {
	public SpringDamper _sd;

	/**
	 * Creates a new spherical joint by configuration `config`.
	 */
	public SphericalJoint(SphericalJointConfig config) {
		super(config, JointType._SPHERICAL);

		_sd = config.springDamper.clone();
	}

	// --- private ---

	protected void  getInfo(JointSolverInfo info, TimeStep timeStep, boolean isPositionPart) {
		if (_sd.frequency > 0 && isPositionPart) return;

		// compute positional error
		Vec3 error=new Vec3();;
		M.vec3_sub(error, _anchor2, _anchor1);

		// compute CFM and ERP
		float cfm;
		float erp;
		if (_sd.frequency > 0) {
			//JointMacro.computeSoftConstraintParameters(_sd.frequency, _sd.dampingRatio, timeStep.dt, _sd.useSymplecticEuler, cfm, erp);
			float omega = MathUtil.TWO_PI * this._sd.frequency;
			float zeta = this._sd.dampingRatio;
			if(zeta < Setting.minSpringDamperDampingRatio) {
				zeta = Setting.minSpringDamperDampingRatio;
			}
			float h = timeStep.dt;
			float c = 2 * zeta * omega;
			float k = omega * omega;
			if(this._sd.useSymplecticEuler) {
				cfm = 1 / (h * c);
				erp = k / c;
			} else {
				cfm = 1 / (h * (h * k + c));
				erp = k / (h * k + c);
			}
			cfm *= this._b1._invMass + this._b2._invMass;
		} else {
			cfm = 0;
			erp = getErp(timeStep, isPositionPart);
		}

		// compute rhs
		float linRhsX =error.x*erp;
		float linRhsY = error.y*erp;
		float linRhsZ = error.z*erp;

		Mat3 crossR1=new Mat3();
		Mat3 crossR2=new Mat3();
		M.vec3_toCrossMatrix(crossR1, _relativeAnchor1);
		M.vec3_toCrossMatrix(crossR2, _relativeAnchor2);
		crossR1.scaleEq(-1);
		crossR2.scaleEq(-1);
		
		JointSolverInfoRow row;
		JacobianRow j;

		// linear X
		row = info.addRow(_impulses[0]);
		row.equalLimit(linRhsX, cfm);

		j = row.jacobian;
		M.vec3_set(j.lin1, 1, 0, 0);
		M.vec3_set(j.lin2, 1, 0, 0);
		M.mat3_getRow(j.ang1, crossR1, 0);
		M.mat3_getRow(j.ang2, crossR2, 0);

		// linear Y
		row = info.addRow(_impulses[1]);
		row.equalLimit(linRhsY, cfm);

		j = row.jacobian;
		M.vec3_set(j.lin1, 0, 1, 0);
		M.vec3_set(j.lin2, 0, 1, 0);
		M.mat3_getRow(j.ang1, crossR1, 1);
		M.mat3_getRow(j.ang2, crossR2, 1);

		// linear Z
		row = info.addRow(_impulses[2]);
		row.equalLimit(linRhsZ, cfm);

		j = row.jacobian;
		M.vec3_set(j.lin1, 0, 0, 1);
		M.vec3_set(j.lin2, 0, 0, 1);
		M.mat3_getRow(j.ang1, crossR1, 2);
		M.mat3_getRow(j.ang2, crossR2, 2);
	}

	// --- internal ---

	@Override 
	public void _getVelocitySolverInfo(TimeStep timeStep, JointSolverInfo info) {
		super._getVelocitySolverInfo(timeStep, info);
		getInfo(info, timeStep, false);
	}

	@Override 
	public void _getPositionSolverInfo(JointSolverInfo info) {
		super._getPositionSolverInfo(info);
		getInfo(info, null, true);
	}

	// --- public ---

	/**
	 * Returns the spring and damper settings.
	 */
	public SpringDamper getSpringDamper() {
		return _sd;
	}
}