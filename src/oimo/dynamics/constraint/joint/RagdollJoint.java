package oimo.dynamics.constraint.joint;

import oimo.common.M;
import oimo.common.Mat3;
import oimo.common.MathUtil;
import oimo.common.Quat;
import oimo.common.Setting;
import oimo.common.Transform;
import oimo.common.Vec3;
import oimo.dynamics.TimeStep;
import oimo.dynamics.constraint.info.JacobianRow;
import oimo.dynamics.constraint.info.joint.JointSolverInfo;
import oimo.dynamics.constraint.info.joint.JointSolverInfoRow;

/**
 * A ragdoll joint is designed to simulate ragdoll's limbs. It constrains
 * swing and twist angles between two rigid bodies. The two rigid bodies
 * have constraint axes, and the swing angle is defined by the angle of
 * two constraint axes, while the twist angle is defined by the rotation
 * angle along the two axes. In addition to lower and upper limits of the
 * twist angle, You can set an "elliptic cone limit" of the swing angle
 * by specifying two swing axes (though one of them is automatically
 * computed) and corresponding maximum swing angles. You can also enable a
 * motor of the twist part of the constraint, spring and damper effect of
 * the both swing and twist part of the constraint.
 */
public class RagdollJoint extends Joint {
	public SpringDamper _twistSd;
	public SpringDamper _swingSd;
	public RotationalLimitMotor _twistLm;
	public float _maxSwingAngle1;
	public float _maxSwingAngle2;

	Vec3 swingAxis;
	Vec3 twistAxis;

	Vec3 linearError;
	float swingError;
	RotationalLimitMotor dummySwingLm;

	public float _swingAngle;
	public float _twistAngle;

	/**
	 * Creates a new ragdoll joint by configuration `config`.
	 */
	public RagdollJoint(RagdollJointConfig config) {
		super(config, JointType.RAGDOLL);

		_localBasisX1=config.localTwistAxis1.clone();
		_localBasisX2=config.localTwistAxis2.clone();
		_localBasisY1=config.localSwingAxis1.clone();
		
	
		buildLocalBasesFromXY1X2();

		_twistSd = config.twistSpringDamper.clone();
		_twistLm = config.twistLimitMotor.clone();
		_swingSd = config.swingSpringDamper.clone();
		_maxSwingAngle1 = config.maxSwingAngle1;
		_maxSwingAngle2 = config.maxSwingAngle2;

		if (_maxSwingAngle1 < Setting.minRagdollMaxSwingAngle) {
			_maxSwingAngle1 = Setting.minRagdollMaxSwingAngle;
		}
		if (_maxSwingAngle2 < Setting.minRagdollMaxSwingAngle) {
			_maxSwingAngle2 = Setting.minRagdollMaxSwingAngle;
		}

		dummySwingLm = new RotationalLimitMotor();
		dummySwingLm.lowerLimit = -1;
		dummySwingLm.upperLimit = 0;

		_swingAngle = 0;
		_twistAngle = 0;
		swingError = 0;
		swingAxis=new Vec3();
		twistAxis=new Vec3();
	}

	// --- private ---

	protected void getInfo(JointSolverInfo info, TimeStep timeStep, boolean isPositionPart) {
		// compute ERP
		float erp = getErp(timeStep, isPositionPart);

		// compute rhs
		float linearRhsX = this.linearError.x * erp;
		float linearRhsY = this.linearError.y * erp;
		float linearRhsZ = this.linearError.z * erp;
		
		Mat3 crossR1=new Mat3();;
		Mat3 crossR2=new Mat3();
		M.vec3_toCrossMatrix(crossR1, _relativeAnchor1);
		M.vec3_toCrossMatrix(crossR2, _relativeAnchor2);
		crossR1.scaleEq(-1);
		crossR2.scaleEq(-1);
		//M.mat3_negate(crossR1, crossR1);
		//M.mat3_negate(crossR2, crossR2);

		JointSolverInfoRow row;
		JacobianRow j;
		float swingMass =this.computeEffectiveInertiaMoment(swingAxis);
		float twistMass = this.computeEffectiveInertiaMoment(_basisX2);

		// linear X
		row = info.addRow(_impulses[0]);
		row.equalLimit(linearRhsX, 0);

		j = row.jacobian;
		M.vec3_set(j.lin1, 1, 0, 0);
		M.vec3_set(j.lin2, 1, 0, 0);
		M.mat3_getRow(j.ang1, crossR1, 0);
		M.mat3_getRow(j.ang2, crossR2, 0);

		// linear Y
		row = info.addRow(_impulses[1]);
		row.equalLimit(linearRhsY, 0);

		j = row.jacobian;
		M.vec3_set(j.lin1, 0, 1, 0);
		M.vec3_set(j.lin2, 0, 1, 0);
		M.mat3_getRow(j.ang1, crossR1, 1);
		M.mat3_getRow(j.ang2, crossR2, 1);

		// linear Z
		row = info.addRow(_impulses[2]);
		row.equalLimit(linearRhsZ, 0);

		j = row.jacobian;
		M.vec3_set(j.lin1, 0, 0, 1);
		M.vec3_set(j.lin2, 0, 0, 1);
		M.mat3_getRow(j.ang1, crossR1, 2);
		M.mat3_getRow(j.ang2, crossR2, 2);

		// swing
		if (swingError > 0 && (_swingSd.frequency <= 0 || !isPositionPart)) {
			row = info.addRow(_impulses[3]);
			setSolverInfoRowAngular(row, swingError, dummySwingLm, swingMass, _swingSd, timeStep, isPositionPart);

			j = row.jacobian;
			M.vec3_assign(j.ang1, swingAxis);
			M.vec3_assign(j.ang2, swingAxis);
		}

		// twist
		if (_twistSd.frequency <= 0 || !isPositionPart) {
			row = info.addRow(_impulses[4]);
			setSolverInfoRowAngular(row, _twistAngle, _twistLm, twistMass, _twistSd, timeStep, isPositionPart);

			j = row.jacobian;

			M.vec3_assign(j.ang1, twistAxis);
			M.vec3_assign(j.ang2, twistAxis);
		}
	}

	private void computeErrors() {
		Transform tf1 = _b1._transform;
		Transform tf2 = _b2._transform;

		// twist axes
		Vec3 axis1=_basisX1;
		Vec3 axis2=_basisX2;
		//M.vec3_assign(axis1, _basisX1);
		//M.vec3_assign(axis2, _basisX2);

		// build basis matrices
		Mat3 basis1Mat=new Mat3();
		Mat3 basis2Mat=new Mat3();
		M.mat3_fromCols(basis1Mat, _basisX1, _basisY1, _basisZ1);
		M.mat3_fromCols(basis2Mat, _basisX2, _basisY2, _basisZ2);

		// compute the swing matrix
		Quat swingQ=new Quat();
		Mat3 swingM=new Mat3();
		Vec3 swingV=new Vec3();
		M.quat_arc(swingQ, axis1, axis2);
		M.mat3_fromQuat(swingM, swingQ);

		// get swing angle
		_swingAngle = MathUtil.safeAcos(M.quat_getReal(swingQ)) * 2;
		M.vec3_fromQuat(swingV, swingQ);
		//M.vec3_assign(swingV, swingQ);

		// swing basisY2 into rb1's coordinates
		Vec3 basisY2In1=new Vec3();
		M.vec3_mulMat3Transposed(basisY2In1, _basisY2, swingM);

		// compute twist angle
		float yCoord;
		float zCoord;
		yCoord = M.vec3_dot(_basisY1, basisY2In1);
		zCoord = M.vec3_dot(_basisZ1, basisY2In1);
		_twistAngle = MathUtil.atan2(zCoord, yCoord);

		// compute twist axis: middle vector between basis1X and basis2X
		M.vec3_add(twistAxis, _basisX1, _basisX2);
		M.vec3_normalize(twistAxis, twistAxis);

		// scale the swing vector so that its length shows the swing rotation angle
		float invLen = M.vec3_length(swingV);
		if (invLen > 0) invLen = 1 / invLen;
		M.vec3_scale(swingV, swingV, invLen * _swingAngle);

		// take the swing axis to the first body's constraint coordinate system
		M.vec3_mulMat3Transposed(swingV, swingV, basis1Mat);

		// project the swing rotation angles onto XY plane
		float x = swingV.y;///.vec3_get(swingV, 1);
		float y = swingV.z;//.vec3_get(swingV, 2);

		// constraint ellipse: x^2/a^2 + y^2/b^2 <= 1
		float a = _maxSwingAngle1;
		float b = _maxSwingAngle2;
		float invA2 = 1 / (a * a);
		float invB2 = 1 / (b * b);

		float w = x * x * invA2 + y * y * invB2;
		if (w == 0) {
			M.vec3_set(swingAxis, 0, 0, 0);
			swingError = 0;
		} else {
			float t = MathUtil.sqrt(1 / w);
			float x0 = x * t;
			float y0 = y * t;
			float nx = x0 * invA2;
			float ny = y0 * invB2;
			invLen = 1 / MathUtil.sqrt(nx * nx + ny * ny);
			nx *= invLen;
			ny *= invLen;
			float depth = (x - x0) * nx + (y - y0) * ny;
			if (depth > 0) {
				swingError = depth;

				// normal vector in constraint ellipse space
				M.vec3_set(swingAxis, 0, nx, ny);
				// take it to the first body's space
				M.vec3_mulMat3(swingAxis, swingAxis, basis1Mat);
				// then swing it
				M.vec3_mulMat3(swingAxis, swingAxis, swingM);
			} else {
				swingError = 0;
			}
		}

		// compute linear error
		M.vec3_sub(linearError, _anchor2, _anchor1);
	}

	// --- internal ---

	@Override 
	public void _syncAnchors() {
		super._syncAnchors();

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
	 * Returns the rotational spring and damper settings along the twist axis.
	 */
	public SpringDamper getTwistSpringDamper() {
		return _twistSd;
	}

	/**
	 * Returns the rotational limits and motor settings along the twist axis.
	 */
	public RotationalLimitMotor getTwistLimitMotor() {
		return _twistLm;
	}

	/**
	 * Returns the rotational spring and damper settings along the swing axis.
	 */
	public SpringDamper getSwingSpringDamper() {
		return _swingSd;
	}

	/**
	 * Returns the swing axis in world coordinates.
	 */
	public Vec3 getSwingAxis() {
		Vec3 v = new Vec3();
		M.vec3_toVec3(v, swingAxis);
		return v;
	}

	/**
	 * Sets `axis` to the swing axis in world coordinates.
	 *
	 * This does not create a new instance of `Vec3`.
	 */
	public void getSwingAxisTo(Vec3 axis) {
		M.vec3_toVec3(axis, swingAxis);
	}

	/**
	 * Returns the swing angle in radians.
	 */
	public float getSwingAngle() {
		return _swingAngle;
	}

	/**
	 * Returns the twist angle in radians.
	 */
	public float getTwistAngle() {
		return _twistAngle;
	}

}
