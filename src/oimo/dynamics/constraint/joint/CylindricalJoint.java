package oimo.dynamics.constraint.joint;

import oimo.common.M;
import oimo.common.MathUtil;
import oimo.common.Vec3;
import oimo.dynamics.TimeStep;
import oimo.dynamics.constraint.info.JacobianRow;
import oimo.dynamics.constraint.info.joint.JointSolverInfo;
import oimo.dynamics.constraint.info.joint.JointSolverInfoRow;

/**
 * A cylindrical joint constrains two rigid bodies to share their constraint
 * axes, and restricts relative translation and rotation onto the constraint
 * axis. This joint provides two degrees of freedom. You can enable lower and
 * upper limits, motors, spring and damper effects of both translation and
 * rotation part of the constraint.
 */
public class CylindricalJoint extends Joint {
	public SpringDamper _translSd;
	public TranslationalLimitMotor _translLm;
	public SpringDamper _rotSd;
	public RotationalLimitMotor _rotLm;

	public BasisTracker _basis;

	double angle=0;
	double angularErrorY=0;
	double angularErrorZ=0;
	double translation=0;
	double linearErrorY=0;
	double linearErrorZ=0;

	/**
	 * Creates a new cylindrical joint by configuration `config`.
	 */
	public CylindricalJoint(CylindricalJointConfig config) {
		super(config, JointType._CYLINDRICAL);

		_localBasisX1=config.localAxis1.clone();
		_localBasisX2=config.localAxis2.clone();

		buildLocalBasesFromX();

		angle = 0;
		angularErrorY = 0;
		angularErrorZ = 0;
		translation = 0;
		linearErrorY = 0;
		linearErrorZ = 0;

		_basis = new BasisTracker(this);

		_translSd = config.translationalSpringDamper.clone();
		_translLm = config.translationalLimitMotor.clone();
		_rotSd = config.rotationalSpringDamper.clone();
		_rotLm = config.rotationalLimitMotor.clone();
	}

	// --- private ---

	protected void getInfo(JointSolverInfo info, TimeStep timeStep, boolean isPositionPart) {
		// compute ERP
		double erp = getErp(timeStep, isPositionPart);

		// compute rhs
		double linRhsY = linearErrorY * erp;
		double linRhsZ = linearErrorZ * erp;
		double angRhsY = angularErrorY * erp;
		double angRhsZ = angularErrorZ * erp;

		JointSolverInfoRow row;
		JacobianRow j;
		double translationalMotorMass = 1 / (_b1._invMass + _b2._invMass);
		double rotationalMotorMass = this.computeEffectiveInertiaMoment(_basis.x);

		// linear X
		if (_translSd.frequency <= 0 || !isPositionPart) {
			row = info.addRow(_impulses[0]);
			setSolverInfoRowLinear(row, translation, _translLm, translationalMotorMass, _translSd, timeStep, isPositionPart);

			j = row.jacobian;
			M.vec3_assign(j.lin1, _basis.x);
			M.vec3_assign(j.lin2, _basis.x);
			M.vec3_cross(j.ang1, _relativeAnchor1, _basis.x);
			M.vec3_cross(j.ang2, _relativeAnchor2, _basis.x);
		}

		// linear Y
		row = info.addRow(_impulses[1]);
		row.equalLimit(linRhsY, 0);

		j = row.jacobian;
		M.vec3_assign(j.lin1, _basis.y);
		M.vec3_assign(j.lin2, _basis.y);
		M.vec3_cross(j.ang1, _relativeAnchor1, _basis.y);
		M.vec3_cross(j.ang2, _relativeAnchor2, _basis.y);

		// linear Z
		row = info.addRow(_impulses[2]);
		row.equalLimit(linRhsZ, 0);

		j = row.jacobian;
		M.vec3_assign(j.lin1, _basis.z);
		M.vec3_assign(j.lin2, _basis.z);
		M.vec3_cross(j.ang1, _relativeAnchor1, _basis.z);
		M.vec3_cross(j.ang2, _relativeAnchor2, _basis.z);

		// angular X
		if (_rotSd.frequency <= 0 || !isPositionPart) {
			row = info.addRow(_impulses[3]);
			setSolverInfoRowAngular(row, angle, _rotLm, rotationalMotorMass, _rotSd, timeStep, isPositionPart);

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
		double cos;

		// compute angular error along Y and Z
		Vec3 TMP=new Vec3();
		Vec3 angError=TMP;
		M.vec3_cross(angError, _basisX1, _basisX2);
		cos = M.vec3_dot(_basisX1, _basisX2);
		double theta = MathUtil.safeAcos(cos);
		M.vec3_normalize(angError, angError);
		M.vec3_scale(angError, angError, theta);
		angularErrorY = M.vec3_dot(angError, _basis.y);
		angularErrorZ = M.vec3_dot(angError, _basis.z);

		// measure the rotation angle along X
		Vec3 perpCross=TMP;
		M.vec3_cross(perpCross, _basisY1, _basisY2);
		cos = M.vec3_dot(_basisY1, _basisY2);
		angle = MathUtil.safeAcos(cos);
		if (M.vec3_dot(perpCross, _basis.x) < 0) {
			angle = -angle;
		}

		// compute linear error
		Vec3 anchorDiff=TMP;
		M.vec3_sub(anchorDiff, _anchor2, _anchor1);
		translation = M.vec3_dot(anchorDiff, _basis.x);
		linearErrorY = M.vec3_dot(anchorDiff, _basis.y);
		linearErrorZ = M.vec3_dot(anchorDiff, _basis.z);
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
	public Vec3  getLocalAxis2() {
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
	 * Returns the translational spring and damper settings.
	 */
	public SpringDamper getTranslationalSpringDamper() {
		return _translSd;
	}

	/**
	 * Returns the rotational spring and damper settings.
	 */
	public SpringDamper getRotationalSpringDamper() {
		return _rotSd;
	}

	/**
	 * Returns the translational limits and motor settings.
	 */
	public TranslationalLimitMotor getTranslationalLimitMotor() {
		return _translLm;
	}

	/**
	 * Returns the rotational limits and motor settings.
	 */
	public RotationalLimitMotor getRotationalLimitMotor() {
		return _rotLm;
	}

	/**
	 * Returns the rotation angle in radians.
	 */
	public double getAngle() {
		return angle;
	}

	/**
	 * Returns the translation of the joint.
	 */
	public double getTranslation() {
		return translation;
	}

}