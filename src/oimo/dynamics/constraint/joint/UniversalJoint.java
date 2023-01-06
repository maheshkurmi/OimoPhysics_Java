package oimo.dynamics.constraint.joint;

import oimo.common.M;
import oimo.common.Mat3;
import oimo.common.Vec3;
import oimo.dynamics.TimeStep;
import oimo.dynamics.constraint.info.JacobianRow;
import oimo.dynamics.constraint.info.joint.JointSolverInfo;
import oimo.dynamics.constraint.info.joint.JointSolverInfoRow;

/**
 * A universal joint constrains two rigid bodies' constraint axes to be perpendicular
 * to each other. Rigid bodies can rotate along their constraint axes, but cannot along
 * the direction perpendicular to two constraint axes. This joint provides two degrees
 * of freedom. You can enable lower and upper limits, motors, spring and damper effects
 * of the two rotational constraints.
 */
public class UniversalJoint extends Joint {
	public SpringDamper _sd1;
	public SpringDamper _sd2;
	public RotationalLimitMotor _lm1;
	public RotationalLimitMotor _lm2;

	public Vec3 _axisX;
	public Vec3 _axisY;
	public Vec3 _axisZ;

	public double _angleX;
	public double _angleY;
	public double _angleZ;

	boolean xSingular;
	boolean ySingular;
	boolean zSingular;

	Vec3 linearError;

	/**
	 * Creates a new universal joint by configuration `config`.
	 */
	public UniversalJoint(UniversalJointConfig config) {
		super(config, JointType.UNIVERSAL);
		_localBasisX1=config.localAxis1.clone();
		_localBasisX2=config.localAxis2.clone();
		
		buildLocalBasesFromX1Z2();

		_angleX = 0;
		_angleY = 0;
		_angleZ = 0;

		_axisX=new Vec3();
		_axisY=new Vec3();
		_axisZ=new Vec3();
		
		xSingular = false;
		ySingular = false;
		zSingular = false;

		linearError=new Vec3();
		
		_sd1 = config.springDamper1.clone();
		_sd2 = config.springDamper2.clone();
		_lm1 = config.limitMotor1.clone();
		_lm2 = config.limitMotor2.clone();
	}

	private void updateConstraintAxes() {
		Mat3 rot1=new Mat3();
		Mat3 rot2=new Mat3();
		M.mat3_fromCols(rot1, _basisX1, _basisY1, _basisZ1);
		M.mat3_fromCols(rot2, _basisX2, _basisY2, _basisZ2);

		//     local --(rot1)--> body1
		//     local --(rot2)--> body2
		//     body1 --(relRot)--> body2
		// and
		//     body1 -------------(relRot)------------> body2
		//     body1 --(inv(rot1))--> local --(rot2)--> body2
		//
		// so relative rotation matrix is
		//     inv(rot1) * rot2
		// and NOT
		//     rot2 * inv(rot1)
		Mat3 relRot =new Mat3();///rot1.transposeEq().mulEq(rot2);
		M.mat3_mulLhsTransposed(relRot, rot1, rot2);

		Vec3 angleAxisX=_basisX1;
		Vec3 angleAxisZ=_basisZ2;
		Vec3 angleAxisY=angleAxisZ.cross(angleAxisX);// right-handed coordinate system
		
	
		// constraint axes are not equal to rotation axes of Euler angles, because rotation axes
		// of Euler angles are not orthogonal. if we want to constrain relative angular velocity
		// w2-w1 along X-axis of Euler angles, w2-w1 should fulfill
		//   w2-w1 = alpha * angleAxisY + beta * angleAxisZ
		// so
		//   (w2-w1) dot (angleAxisY cross angleAxisZ) = 0
		//
		// be careful about the fact that this does NOT mean
		//   (w2-w1) dot angleAxisX = 0
		//
		// note that we can directory use Y-axis of Euler angles to constrain relative velocity
		// along the axis, as `angleAxisY` is parallel to `angleAxisX cross angleAxisZ`.
		M.vec3_cross(_axisX, angleAxisY, angleAxisZ);
		M.vec3_assign(_axisY, angleAxisY);
		M.vec3_cross(_axisZ, angleAxisX, angleAxisY);

		M.vec3_normalize(_axisX, _axisX);
		M.vec3_normalize(_axisY, _axisY);
		M.vec3_normalize(_axisZ, _axisZ);

		xSingular = M.vec3_dot(_axisX, _axisX) == 0;
		ySingular = M.vec3_dot(_axisY, _axisY) == 0;
		zSingular = M.vec3_dot(_axisZ, _axisZ) == 0;
	}

	protected void getInfo(JointSolverInfo info,TimeStep timeStep, boolean isPositionPart) {
		// compute ERP
		double erp = getErp(timeStep, isPositionPart);

		// compute rhs
		double linRhsX = linearError.x*erp;
		double linRhsY = linearError.y*erp;
		double linRhsZ = linearError.z*erp;
		double angRhsY = _angleY * erp;

		Mat3 crossR1=new Mat3();
		Mat3 crossR2=new Mat3();
		M.vec3_toCrossMatrix(crossR1, _relativeAnchor1);
		M.vec3_toCrossMatrix(crossR2, _relativeAnchor2);
		crossR1.scaleEq(-1);
		crossR2.scaleEq(-1);
		
		JointSolverInfoRow row;
		JacobianRow j;
		double motorMassX = this.computeEffectiveInertiaMoment(_axisX);
		double motorMassZ =this.computeEffectiveInertiaMoment(_axisZ);

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
		if (!xSingular && (_sd1.frequency <= 0 || !isPositionPart)) {
			row = info.addRow(_impulses[3]);
			setSolverInfoRowAngular(row, _angleX, _lm1, motorMassX, _sd1, timeStep, isPositionPart);

			j = row.jacobian;
			M.vec3_assign(j.ang1, _axisX);
			M.vec3_assign(j.ang2, _axisX);
		}

		// angular Y
		if (!ySingular) {
			row = info.addRow(_impulses[4]);
			row.equalLimit(angRhsY, 0);

			j = row.jacobian;
			M.vec3_assign(j.ang1, _axisY);
			M.vec3_assign(j.ang2, _axisY);
		}

		// angular Z
		if (!zSingular && (_sd2.frequency <= 0 || !isPositionPart)) {
			row = info.addRow(_impulses[5]);
			setSolverInfoRowAngular(row, _angleZ, _lm2, motorMassZ, _sd2, timeStep, isPositionPart);

			j = row.jacobian;
			M.vec3_assign(j.ang1, _axisZ);
			M.vec3_assign(j.ang2, _axisZ);
		}
	}

	private void computeErrors() {
		Mat3 rot1 =new Mat3();
		Mat3 rot2 =new Mat3();
		M.mat3_fromCols(rot1, _basisX1, _basisY1, _basisZ1);
		M.mat3_fromCols(rot2, _basisX2, _basisY2, _basisZ2);

		//     local --(rot1)--> body1
		//     local --(rot2)--> body2
		//     body1 --(relRot)--> body2
		// and
		//     body1 -------------(relRot)------------> body2
		//     body1 --(inv(rot1))--> local --(rot2)--> body2
		//
		// so relative rotation matrix is
		//     inv(rot1) * rot2
		// but NOT
		//     rot2 * inv(rot1)
		Mat3 relRot=new Mat3();//rot1.transposeEq().mulEq(rot2);
		M.mat3_mulLhsTransposed(relRot, rot1, rot2);

		Vec3 angles=new Vec3();
		M.mat3_toEulerXyz(angles, relRot);
		_angleX = angles.x;
		_angleY = angles.y;
		_angleZ = angles.z;

		// compute linear error
		M.vec3_sub(linearError, _anchor2, _anchor1);
	}

	// --- internal ---

	@Override 
	public void _syncAnchors() {
		super._syncAnchors();
		updateConstraintAxes();

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
		M.vec3_toVec3(v, _basisZ2);
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
		M.vec3_toVec3(axis, _basisZ2);
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
		M.vec3_toVec3(v, _localBasisZ2);
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
		M.vec3_toVec3(axis, _localBasisZ2);
	}

	/**
	 * Returns the rotational spring and damper settings along the first body's constraint axis.
	 */
	public SpringDamper getSpringDamper1() {
		return _sd1;
	}

	/**
	 * Returns the rotational spring and damper settings along the second body's constraint axis.
	 */
	public SpringDamper getSpringDamper2() {
		return _sd2;
	}

	/**
	 * Returns the rotational limits and motor settings along the first body's constraint axis.
	 */
	public RotationalLimitMotor getLimitMotor1() {
		return _lm1;
	}

	/**
	 * Returns the rotational limits and motor settings along the second body's constraint axis.
	 */
	public RotationalLimitMotor getLimitMotor2() {
		return _lm2;
	}

	/**
	 * Returns the rotation angle along the first body's constraint axis.
	 */
	public double getAngle1() {
		return _angleX;
	}

	/**
	 * Returns the rotation angle along the second body's constraint axis.
	 */
	public double getAngle2() {
		return _angleZ;
	}

}
