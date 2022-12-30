package oimo.dynamics.constraint.joint;

import oimo.common.M;
import oimo.common.Mat3;
import oimo.common.MathUtil;
import oimo.common.Vec3;
import oimo.dynamics.TimeStep;
import oimo.dynamics.constraint.info.JacobianRow;
import oimo.dynamics.constraint.info.joint.JointSolverInfo;
import oimo.dynamics.constraint.info.joint.JointSolverInfoRow;

/**
 * A revolute joint (a.k.a. hinge joint) constrains two rigid bodies to share
 * their anchor points and constraint axes, and restricts relative rotation onto
 * the constraint axis. This joint provides one degree of freedom. You can enable
 * lower and upper limits, a motor, a spring and damper effect of the rotational
 * part of the constraint.
 */
public class RevoluteJoint extends Joint {
	public SpringDamper _sd;
	public RotationalLimitMotor _lm;

	public BasisTracker _basis;

	float angle;
	float angularErrorY;
	float angularErrorZ;
	Vec3 linearError;

	/**
	 * Creates a new revolute joint by configuration `config`.
	 */
	public RevoluteJoint(RevoluteJointConfig config) {
		super(config, JointType._REVOLUTE);

		_localBasisX1=config.localAxis1.clone();
		_localBasisX2=config.localAxis2.clone();

		buildLocalBasesFromX();

		angle = 0;
		angularErrorY = 0;
		angularErrorZ = 0;

		_basis = new BasisTracker(this);

		_sd = config.springDamper.clone();
		_lm = config.limitMotor.clone();
	}

	// --- private ---

	protected void getInfo(JointSolverInfo info, TimeStep timeStep, boolean isPositionPart) {
		// compute ERP
		float erp = getErp(timeStep, isPositionPart);

		// compute rhs
		float linRhsX = linearError.x*erp;//M.vec3_get(linearRhs, 0);
		float linRhsY = linearError.y*erp;//M.vec3_get(linearRhs, 1);
		float linRhsZ = linearError.z*erp;//M.vec3_get(linearRhs, 2);
		float angRhsY = angularErrorY * erp;
		float angRhsZ = angularErrorZ * erp;

		Mat3 crossR1=new Mat3();
		Mat3 crossR2=new Mat3();
		M.vec3_toCrossMatrix(crossR1, _relativeAnchor1);
		M.vec3_toCrossMatrix(crossR2, _relativeAnchor2);
		crossR1.scaleEq(-1);
		crossR2.scaleEq(-1);
		
		JointSolverInfoRow row;
		JacobianRow j;
		float motorMass = this.computeEffectiveInertiaMoment(_basis.x);

		// linear X
		row = info.addRow(_impulses[0]);
		row.equalLimit(linRhsX, 0);

		j = row.jacobian;
		M.vec3_set(j.lin1, 1, 0, 0);
		M.vec3_set(j.lin2, 1, 0, 0);
		M.mat3_getRow(j.ang1, crossR1, 0);
		M.mat3_getRow(j.ang2, crossR2, 0);

		// linear Y
		row = info.addRow(_impulses[1]);
		row.equalLimit(linRhsY, 0);

		j = row.jacobian;
		M.vec3_set(j.lin1, 0, 1, 0);
		M.vec3_set(j.lin2, 0, 1, 0);
		M.mat3_getRow(j.ang1, crossR1, 1);
		M.mat3_getRow(j.ang2, crossR2, 1);

		// linear Z
		row = info.addRow(_impulses[2]);
		row.equalLimit(linRhsZ, 0);

		j = row.jacobian;
		M.vec3_set(j.lin1, 0, 0, 1);
		M.vec3_set(j.lin2, 0, 0, 1);
		M.mat3_getRow(j.ang1, crossR1, 2);
		M.mat3_getRow(j.ang2, crossR2, 2);

		// angular X
		if (_sd.frequency <= 0 || !isPositionPart) {
			row = info.addRow(_impulses[3]);
			setSolverInfoRowAngular(row, angle, _lm, motorMass, _sd, timeStep, isPositionPart);

			j = row.jacobian;
			M.vec3_assign(j.ang1, _basis.x);
			M.vec3_assign(j.ang2, _basis.x);
		}

		// angular Y
		row = info.addRow(_impulses[4]);
		row.equalLimit(angRhsY, 0);

		j = row.jacobian;
		M.vec3_assign(j.ang1, _basis.y);
		M.vec3_assign(j.ang2, _basis.y);

		// angular Z
		row = info.addRow(_impulses[5]);
		row.equalLimit(angRhsZ, 0);

		j = row.jacobian;
		M.vec3_assign(j.ang1, _basis.z);
		M.vec3_assign(j.ang2, _basis.z);
	}

	private void computeErrors() {
		

		// compute angular error along Y and Z
		Vec3 angError =_basisX1.cross(_basisX2);
		//M.vec3_cross(angError, _basisX1, _basisX2);
		float cos = M.vec3_dot(_basisX1, _basisX2);
		float theta = MathUtil.safeAcos(cos);
		M.vec3_normalize(angError, angError);
		M.vec3_scale(angError, angError, theta);
		angularErrorY = M.vec3_dot(angError, _basis.y);
		angularErrorZ = M.vec3_dot(angError, _basis.z);

		// measure the rotation angle along X
		Vec3 perpCross=_basisY1.cross(_basisY2);
		//M.vec3_cross(perpCross, _basisY1, _basisY2);
		cos = M.vec3_dot(_basisY1, _basisY2);
		angle = MathUtil.safeAcos(cos);
		if (M.vec3_dot(perpCross, _basis.x) < 0) {
			angle = -angle;
		}

		// compute linear error
		M.vec3_sub(linearError, _anchor2, _anchor1);
	}

	// --- internal ---

	@Override 
	public void _syncAnchors() {
		super._syncAnchors();
		_basis.trackByX();

		// compute positional errors
		computeErrors();
	}

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
	 * Returns the first rigid body's constraint axis in world coordinates.
	 */
	public Vec3 getAxis1() {
		Vec3 v = new Vec3();
		M.vec3_toVec3(v, _basisX1);
		return v;
	}

	/**
	 * Returns the second rigid body's constraint axis in world coordinates.
	 */
	public Vec3 getAxis2() {
		Vec3 v = new Vec3();
		M.vec3_toVec3(v, _basisX2);
		return v;
	}

	/**
	 * Sets `axis` to the first rigid body's constraint axis in world coordinates.
	 *
	 * This does not create a new instance of `Vec3`.
	 */
	public void getAxis1To(Vec3 axis) {
		M.vec3_toVec3(axis, _basisX1);
	}

	/**
	 * Sets `axis` to the second rigid body's constraint axis in world coordinates.
	 *
	 * This does not create a new instance of `Vec3`.
	 */
	public void getAxis2To(Vec3 axis) {
		M.vec3_toVec3(axis, _basisX2);
	}

	/**
	 * Returns the first rigid body's constraint axis relative to the rigid body's transform.
	 */
	public Vec3 getLocalAxis1() {
		Vec3 v = new Vec3();
		M.vec3_toVec3(v, _localBasisX1);
		return v;
	}

	/**
	 * Returns the second rigid body's constraint axis relative to the rigid body's transform.
	 */
	public Vec3 getLocalAxis2() {
		Vec3 v = new Vec3();
		M.vec3_toVec3(v, _localBasisX2);
		return v;
	}

	/**
	 * Sets `axis` to the first rigid body's constraint axis relative to the rigid body's transform.
	 *
	 * This does not create a new instance of `Vec3`.
	 */
	public void getLocalAxis1To(Vec3 axis) {
		M.vec3_toVec3(axis, _localBasisX1);
	}

	/**
	 * Sets `axis` to the second rigid body's constraint axis relative to the rigid body's transform.
	 *
	 * This does not create a new instance of `Vec3`.
	 */
	public void getLocalAxis2To(Vec3 axis) {
		M.vec3_toVec3(axis, _localBasisX2);
	}

	/**
	 * Returns the rotational spring and damper settings.
	 */
	public SpringDamper getSpringDamper() {
		return _sd;
	}

	/**
	 * Returns the rotational limits and motor settings.
	 */
	public RotationalLimitMotor getLimitMotor() {
		return _lm;
	}

	/**
	 * Returns the rotation angle in radians.
	 */
	public float getAngle() {
		return angle;
	}

}
