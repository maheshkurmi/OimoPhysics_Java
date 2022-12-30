package oimo.dynamics.constraint.joint;

import oimo.common.M;
import oimo.common.Mat3;
import oimo.common.MathUtil;
import oimo.common.Quat;
import oimo.common.Vec3;
import oimo.dynamics.TimeStep;
import oimo.dynamics.constraint.info.JacobianRow;
import oimo.dynamics.constraint.info.joint.JointSolverInfo;
import oimo.dynamics.constraint.info.joint.JointSolverInfoRow;

/**
 * A prismatic joint (a.k.a. slider joint) constrains two rigid bodies to
 * share their anchor points and constraint axes, and restricts relative
 * translation onto the constraint axis. This joint provides one degree of
 * freedom. You can enable lower and upper limits, a motor, a spring and
 * damper effect of the translational part of the constraint.
 */
public class PrismaticJoint extends Joint {
	public SpringDamper _sd;
	public TranslationalLimitMotor _lm;

	public BasisTracker _basis;

	float translation;
	float linearErrorY;
	float linearErrorZ;
	Vec3 angularError;

	/**
	 * Creates a new prismatic joint by configuration `config`.
	 */
	public PrismaticJoint(PrismaticJointConfig config) {
		super(config, JointType.PRISMATIC);

		_localBasisX1=config.localAxis1.clone();
		_localBasisX2=config.localAxis2.clone();

		buildLocalBasesFromX();

		_basis = new BasisTracker(this);

		translation = 0;
		linearErrorY = 0;
		linearErrorZ = 0;
		angularError=new Vec3();

		_sd = config.springDamper.clone();
		_lm = config.limitMotor.clone();
	}

	// --- priate ---


	protected void getInfo(JointSolverInfo info, TimeStep timeStep, boolean isPositionPart) {
		// compute ERP
		float erp = getErp(timeStep, isPositionPart);

		// compute rhs
		float linRhsY = linearErrorY * erp;
		float linRhsZ = linearErrorZ * erp;
		float angRhsX = angularError.x * erp;
		float angRhsY = angularError.y * erp;
		float angRhsZ = angularError.z * erp;

		JointSolverInfoRow row;
		JacobianRow j;
		float motorMass = 1 / (_b1._invMass + _b2._invMass);

		// linear X
		if (_sd.frequency <= 0 || !isPositionPart) {
			row = info.addRow(_impulses[0]);
			setSolverInfoRowLinear(row, translation, _lm, motorMass, _sd, timeStep, isPositionPart);

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
		row = info.addRow(_impulses[3]);
		row.equalLimit(angRhsX, 0);

		j = row.jacobian;
		M.vec3_set(j.ang1, 1, 0, 0);
		M.vec3_set(j.ang2, 1, 0, 0);

		// angular Y
		row = info.addRow(_impulses[4]);
		row.equalLimit(angRhsY, 0);

		j = row.jacobian;
		M.vec3_set(j.ang1, 0, 1, 0);
		M.vec3_set(j.ang2, 0, 1, 0);

		// angular Z
		row = info.addRow(_impulses[5]);
		row.equalLimit(angRhsZ, 0);

		j = row.jacobian;
		M.vec3_set(j.ang1, 0, 0, 1);
		M.vec3_set(j.ang2, 0, 0, 1);
	}

	private void computeErrors() {
		// compute angular error
		Mat3 rot1=new Mat3();
		Mat3 rot2=new Mat3();
		M.mat3_fromCols(rot1, _basisX1, _basisY1, _basisZ1);
		M.mat3_fromCols(rot2, _basisX2, _basisY2, _basisZ2);
		Mat3 relRot=rot2.mulTransposeEq(rot1);
		//M.mat3_mulRhsTransposed(relRot, rot2, rot1);
		Quat relQ=new Quat();
		M.quat_fromMat3(relQ, relRot);

		float cosHalfTheta = relQ.w;//.quat_getReal(relQ);
		float theta = MathUtil.safeAcos(cosHalfTheta) * 2;
		// [rotation vector] = [rotation axis] * [rotation angle]
		
		M.vec3_fromQuat(angularError, relQ);
		M.vec3_normalize(angularError, angularError);
		M.vec3_scale(angularError, angularError, theta);

		// compute linear error
		Vec3 anchorDiff=new Vec3();
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
	public Vec3  getAxis1() {
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
	 * Returns the translational spring and damper settings.
	 */
	public SpringDamper getSpringDamper() {
		return _sd;
	}

	/**
	 * Returns the translational limits and motor settings.
	 */
	public TranslationalLimitMotor getLimitMotor() {
		return _lm;
	}

	/**
	 * Returns the translation of the joint.
	 */
	public float getTranslation() {
		return translation;
	}

}