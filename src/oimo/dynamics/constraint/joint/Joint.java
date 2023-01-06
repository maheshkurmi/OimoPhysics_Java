package oimo.dynamics.constraint.joint;
import oimo.common.*;
import oimo.dynamics.*;
import oimo.dynamics.constraint.*;
import oimo.dynamics.constraint.info.joint.*;
import oimo.dynamics.constraint.solver.*;
import oimo.dynamics.constraint.solver.direct.*;
import oimo.dynamics.constraint.solver.pgs.*;
import oimo.dynamics.rigidbody.*;
import oimo.common.Mat3;
import oimo.common.Quat;
import oimo.common.Vec3;
import oimo.common.M;

/**
 * The base class of joints. Joints are used to connect two rigid bodies
 * in various ways. See `JointType` for all types of joints.
 */
public class Joint {
	public RigidBody _b1;
	public RigidBody _b2;

	public JointLink _link1;
	public JointLink _link2;
	public int _positionCorrectionAlgorithm;

	public boolean _allowCollision;

	public Joint _prev;
	public Joint _next;
	public World _world;

	public Vec3 _localAnchor1;
	public Vec3 _localAnchor2;
	public Vec3 _relativeAnchor1;
	public Vec3 _relativeAnchor2;
	public Vec3 _anchor1;
	public Vec3 _anchor2;

	public Vec3 _localBasisX1;
	public Vec3 _localBasisY1;
	public Vec3 _localBasisZ1;
	public Vec3 _localBasisX2;
	public Vec3 _localBasisY2;
	public Vec3 _localBasisZ2;

	public Vec3 _basisX1;
	public Vec3 _basisY1;
	public Vec3 _basisZ1;
	public Vec3 _basisX2;
	public Vec3 _basisY2;
	public Vec3 _basisZ2;

	public JointImpulse[] _impulses;

	// computed in constraint solver
	public Vec3 _appliedForce;
	public Vec3 _appliedTorque;

	public double _breakForce;
	public double _breakTorque;

	public int _type;

	public ConstraintSolver _solver;

	/**
	 * Extra field that users can use for their own purposes.
	 */
	public Object userData;

	public Joint(JointConfig config, int type) {
		_link1 = new JointLink(this);
		_link2 = new JointLink(this);
		_positionCorrectionAlgorithm = Setting.defaultJointPositionCorrectionAlgorithm;
		_type = type;
		_world = null;
		_b1 = config.rigidBody1;
		_b2 = config.rigidBody2;
		_allowCollision = config.allowCollision;
		_breakForce = config.breakForce;
		_breakTorque = config.breakTorque;

		switch (config.solverType) {
		case ConstraintSolverType._DIRECT:
			_solver = new DirectJointConstraintSolver(this);
			break;
		case ConstraintSolverType._ITERATIVE:
			_solver = new PgsJointConstraintSolver(this);
			break;
		}

		_localAnchor1=config.localAnchor1.clone();
		_localAnchor2=config.localAnchor2.clone();

		_relativeAnchor1 =new Vec3();
		_relativeAnchor2 =new Vec3();
		_anchor1 =new Vec3();
		_anchor2 =new Vec3();
		
		_localBasisX1 =new Vec3();
		_localBasisY1 =new Vec3();
		_localBasisZ1 =new Vec3();
		_localBasisX2 =new Vec3();
		_localBasisY2 =new Vec3();
		_localBasisZ2 =new Vec3();
		
	

		_basisX1 =new Vec3();
		_basisY1 =new Vec3();
		_basisZ1 =new Vec3();
		_basisX2 =new Vec3();
		_basisY2 =new Vec3();
		_basisZ2 =new Vec3();
		
		_appliedForce=new Vec3();
		_appliedTorque=new Vec3();

		
		_impulses = new JointImpulse[Setting.maxJacobianRows];
		for (int i=0;i<Setting.maxJacobianRows;i++) {
			_impulses[i] = new JointImpulse();
		}
	}

	// --- private ---

	protected void buildLocalBasesFromX() {
		// validate X
		if (M.vec3_dot(_localBasisX1, _localBasisX1) == 0) {
			M.vec3_set(_localBasisX1, 1, 0, 0);
		} else {
			_localBasisX1.normalize();
			//M.vec3_normalize(_localBasisX1, _localBasisX1);
		}
		if (M.vec3_dot(_localBasisX2, _localBasisX2) == 0) {
			M.vec3_set(_localBasisX2, 1, 0, 0);
		} else {
			_localBasisX2.normalize();
			//M.vec3_normalize(_localBasisX2, _localBasisX2);
		}

		Quat slerpQ=new Quat();
		Mat3 slerpM=new Mat3();
		M.quat_arc(slerpQ, _localBasisX1, _localBasisX2);
		M.mat3_fromQuat(slerpM, slerpQ);

		M.vec3_perp(_localBasisY1, _localBasisX1);
		M.vec3_cross(_localBasisZ1, _localBasisX1, _localBasisY1);

		M.vec3_mulMat3(_localBasisX2, _localBasisX1, slerpM);
		M.vec3_mulMat3(_localBasisY2, _localBasisY1, slerpM);
		M.vec3_mulMat3(_localBasisZ2, _localBasisZ1, slerpM);
	}

	protected void buildLocalBasesFromXY() {
		// validate X
		if (M.vec3_dot(_localBasisX1, _localBasisX1) == 0) {
			M.vec3_set(_localBasisX1, 1, 0, 0);
		} else {
			M.vec3_normalize(_localBasisX1, _localBasisX1);
		}

		if (M.vec3_dot(_localBasisX2, _localBasisX2) == 0) {
			M.vec3_set(_localBasisX2, 1, 0, 0);
		} else {
			M.vec3_normalize(_localBasisX2, _localBasisX2);
		}

		// build Z and recompute Y
		M.vec3_cross(_localBasisZ1, _localBasisX1, _localBasisY1);
		M.vec3_cross(_localBasisZ2, _localBasisX2, _localBasisY2);

		if (M.vec3_dot(_localBasisZ1, _localBasisZ1) == 0) {
			// invalid Y, set Y perpendicular to X
			M.vec3_perp(_localBasisY1, _localBasisX1);
			// Z = cross(X, Y)
			M.vec3_cross(_localBasisZ1, _localBasisX1, _localBasisY1);
		} else {
			// normalize Z
			M.vec3_normalize(_localBasisZ1, _localBasisZ1);
			// Y = cross(Z, X)
			M.vec3_cross(_localBasisY1, _localBasisZ1, _localBasisX1);
		}

		if (M.vec3_dot(_localBasisZ2, _localBasisZ2) == 0) {
			// invalid Y, set Y perpendicular to X
			M.vec3_perp(_localBasisY2, _localBasisX2);
			// Z = cross(X, Y)
			M.vec3_cross(_localBasisZ2, _localBasisX2, _localBasisY2);
		} else {
			// normalize Z
			M.vec3_normalize(_localBasisZ2, _localBasisZ2);
			// Y = cross(Z, X)
			M.vec3_cross(_localBasisY2, _localBasisZ2, _localBasisX2);
		}
	}

	protected void buildLocalBasesFromX1Z2() {
		// validate X1 and Z2
		if (M.vec3_dot(_localBasisX1, _localBasisX1) == 0) {
			M.vec3_set(_localBasisX1, 1, 0, 0);
		} else {
			_localBasisX1.normalize();
			//M.vec3_normalize(_localBasisX1, _localBasisX1);
		}

		if (M.vec3_dot(_localBasisZ2, _localBasisZ2) == 0) {
			M.vec3_set(_localBasisZ2, 0, 0, 1);
		} else {
			_localBasisZ2.normalize();
			//M.vec3_normalize(_localBasisZ2, _localBasisZ2);
		}

		Transform tf1 = _b1._transform;
		Transform tf2 = _b2._transform;

		// compute world bases
		Vec3 worldX1=new Vec3();
		Vec3 worldZ1=new Vec3();
		Vec3 worldY=new Vec3();
		Vec3 worldX2=new Vec3();
		Vec3 worldZ2=new Vec3();
		M.vec3_mulMat3(worldX1, _localBasisX1, tf1._rotation);
		M.vec3_mulMat3(worldZ2, _localBasisZ2, tf2._rotation);
		M.vec3_cross(worldY, worldZ2, worldX1);

		if (M.vec3_dot(worldY, worldY) == 0) {
			M.vec3_perp(worldY, worldX1);
		}

		M.vec3_cross(worldZ1, worldX1, worldY);
		M.vec3_cross(worldX2, worldY, worldZ2);

		// return to local
		M.vec3_mulMat3Transposed(_localBasisX1, worldX1, tf1._rotation);
		M.vec3_mulMat3Transposed(_localBasisY1, worldY, tf1._rotation);
		M.vec3_mulMat3Transposed(_localBasisZ1, worldZ1, tf1._rotation);
		M.vec3_mulMat3Transposed(_localBasisX2, worldX2, tf2._rotation);
		M.vec3_mulMat3Transposed(_localBasisY2, worldY, tf2._rotation);
		M.vec3_mulMat3Transposed(_localBasisZ2, worldZ2, tf2._rotation);
	}

	protected void buildLocalBasesFromXY1X2() {
		// validate X1
		if (M.vec3_dot(_localBasisX1, _localBasisX1) == 0) {
			M.vec3_set(_localBasisX1, 1, 0, 0);
		} else {
			_localBasisX1.normalize();
			//M.vec3_normalize(_localBasisX1, _localBasisX1);
		}

		// build Z1 and recompute Y1
		M.vec3_cross(_localBasisZ1, _localBasisX1, _localBasisY1);

		if (M.vec3_dot(_localBasisZ1, _localBasisZ1) == 0) {
			// invalid Y1, set Y1 perpendicular to X1
			M.vec3_perp(_localBasisY1, _localBasisX1);
			// Z1 = cross(X1, Y1)
			M.vec3_cross(_localBasisZ1, _localBasisX1, _localBasisY1);
		} else {
			// normalize Z1
			_localBasisZ1.normalize();
			//M.vec3_normalize(_localBasisZ1, _localBasisZ1);
			// Y1 = cross(Z1, X1)
			M.vec3_cross(_localBasisY1, _localBasisZ1, _localBasisX1);
		}

		// rotate (X1, Y1, Z1) to (X2, Y2, Z2) by arc(X1, X2)
		Quat slerpQ=new Quat();
		Mat3 slerpM=new Mat3();
		M.quat_arc(slerpQ, _localBasisX1, _localBasisX2);
		M.mat3_fromQuat(slerpM, slerpQ);

		M.vec3_mulMat3(_localBasisX2, _localBasisX1, slerpM);
		M.vec3_mulMat3(_localBasisY2, _localBasisY1, slerpM);
		M.vec3_mulMat3(_localBasisZ2, _localBasisZ1, slerpM);
	}

	protected void setSolverInfoRowLinear(JointSolverInfoRow row, double diff, TranslationalLimitMotor lm, double mass, SpringDamper sd, TimeStep timeStep, boolean isPositionPart) {
		double cfmFactor;
		double erp;
		double slop = Setting.linearSlop;

		if (isPositionPart) {
			cfmFactor = 0;
			erp = 1;
		} else {
			if (sd.frequency > 0) {
				// the constraint is softened
				slop = 0;
				// computes CFM_factor and ERP
				// note:
				//     CFM = CFM_factor / mass
				//     deltaImpulse = (1 / (1 / mass + CFM)) * (posError * ERP - velocity - totalImpulse * CFM)
				//                  = mass / (1 + CFM_factor) * (posError * ERP - velocity) - totalImpulse * CFM_factor / (1 + CFM_factor)

				double omega = MathUtil.TWO_PI * sd.frequency;
				double zeta = sd.dampingRatio;
				if(zeta < oimo.common.Setting.minSpringDamperDampingRatio) {
					zeta = oimo.common.Setting.minSpringDamperDampingRatio;
				}
				double h = timeStep.dt;
				double c = 2 * zeta * omega;
				double k = omega * omega;
				if(sd.useSymplecticEuler) {
					cfmFactor = 1 / (h * c);
					erp = k / c;
				} else {
					cfmFactor = 1 / (h * (h * k + c));
					erp = k / (h * k + c);
				}			
			} else {
				// the constraint is rigid
				cfmFactor = 0;
				erp = getErp(timeStep, false);
			}

			// set motor parameters if enabled
			if (lm.motorForce > 0) {
				row.motor(lm.motorSpeed, lm.motorForce * timeStep.dt);
			} else {
				row.motor(0, 0);
			}
		}

		double lower = lm.lowerLimit;
		double upper = lm.upperLimit;

		double minImp;
		double maxImp;
		double error;
		if (lower > upper) {
			// inactive
			minImp = 0;
			maxImp = 0;
			error = 0;
		} else if (lower == upper) {
			// locked
			minImp = MathUtil.NEGATIVE_INFINITY;
			maxImp = MathUtil.POSITIVE_INFINITY;
			error = diff - lower;
		} else if (diff < lower) {
			// at lower limit
			minImp = MathUtil.NEGATIVE_INFINITY;
			maxImp = 0;
			error = diff - lower + slop;
			if (error > 0) {
				error = 0;
			}
		} else if (diff > upper) {
			// at upper limit
			minImp = 0;
			maxImp = MathUtil.POSITIVE_INFINITY;
			error = diff - upper - slop;
			if (error < 0) {
				error = 0;
			}
		} else {
			// inactive
			minImp = 0;
			maxImp = 0;
			error = 0;
		}

		// inverse motor mass
		double invMass = mass == 0 ? 0 : 1 / mass;

		row.minImpulse = minImp;
		row.maxImpulse = maxImp;
		row.cfm = cfmFactor * invMass;
		row.rhs = error * erp;
	}

	protected void setSolverInfoRowAngular(JointSolverInfoRow row,double diff, RotationalLimitMotor lm,double mass, SpringDamper sd, TimeStep timeStep, boolean isPositionPart) {
		double cfmFactor;
		double erp;
		double slop = Setting.angularSlop;

		if (isPositionPart) {
			cfmFactor = 0;
			erp = 1;
		} else {
			if (sd.frequency > 0) {
				// the constraint is softened
				slop = 0;
				// computes CFM_factor and ERP
				// note:
				//     CFM = CFM_factor / mass
				//     deltaImpulse = (1 / (1 / mass + CFM)) * (posError * ERP - velocity - totalImpulse * CFM)
				//                  = mass / (1 + CFM_factor) * (posError * ERP - velocity) - totalImpulse * CFM_factor / (1 + CFM_factor)

				double omega = MathUtil.TWO_PI * sd.frequency;
				double zeta = sd.dampingRatio;
				if(zeta < oimo.common.Setting.minSpringDamperDampingRatio) {
					zeta = oimo.common.Setting.minSpringDamperDampingRatio;
				}
				double h = timeStep.dt;
				double c = 2 * zeta * omega;
				double k = omega * omega;
				if(sd.useSymplecticEuler) {
					cfmFactor = 1 / (h * c);
					erp = k / c;
				} else {
					cfmFactor = 1 / (h * (h * k + c));
					erp = k / (h * k + c);
				}
			} else {
				// the constraint is rigid
				cfmFactor = 0;
				erp = getErp(timeStep, false);
			}

			// set motor parameters if enabled
			if (lm.motorTorque > 0) {
				row.motor(lm.motorSpeed, lm.motorTorque * timeStep.dt);
			} else {
				row.motor(0, 0);
			}
		}

		double lower = lm.lowerLimit;
		double upper = lm.upperLimit;

		// adjust theta (in [-pi, pi] => in [mid - pi, mid + pi])
		double mid = (lower + upper) * 0.5f;
		diff -= mid;
		diff = ((diff + MathUtil.PI) % MathUtil.TWO_PI + MathUtil.TWO_PI) % MathUtil.TWO_PI - MathUtil.PI;
		diff += mid;

		double minImp;
		double maxImp;
		double error;
		if (lower > upper) {
			// inactive
			minImp = 0;
			maxImp = 0;
			error = 0;
		} else if (lower == upper) {
			// locked
			minImp = MathUtil.NEGATIVE_INFINITY;
			maxImp = MathUtil.POSITIVE_INFINITY;
			error = diff - lower;
		} else if (diff < lower) {
			// at lower limit
			minImp = MathUtil.NEGATIVE_INFINITY;
			maxImp = 0;
			error = diff - lower + slop;
			if (error > 0) {
				error = 0;
			}
		} else if (diff > upper) {
			// at upper limit
			minImp = 0;
			maxImp = MathUtil.POSITIVE_INFINITY;
			error = diff - upper - slop;
			if (error < 0) {
				error = 0;
			}
		} else {
			// inactive
			minImp = 0;
			maxImp = 0;
			error = 0;
		}

		// inverse motor mass
		double invMass = mass == 0 ? 0 : 1 / mass;

		row.minImpulse = minImp;
		row.maxImpulse = maxImp;
		row.cfm = cfmFactor * invMass;
		row.rhs = error * erp;
	}

	/**
	 * Returns error reduction parameter
	 * @param timeStep
	 * @param isPositionPart
	 * @return
	 */
	protected double getErp(TimeStep timeStep,boolean isPositionPart) {
		if (isPositionPart) {
			return 1;
		} else {
			if (_positionCorrectionAlgorithm == PositionCorrectionAlgorithm.BAUMGARTE) {
				return timeStep.invDt * Setting.velocityBaumgarte;
			} else {
				return 0;
			}
		}
	}

	protected double computeEffectiveInertiaMoment(Vec3 axis) {
		Vec3 ia1=new Vec3();
		Vec3 ia2=new Vec3();
		M.vec3_mulMat3(ia1, axis, _b1._invInertia);
		M.vec3_mulMat3(ia2, axis, _b2._invInertia);
		double invI1 = M.vec3_dot(ia1, axis);
		double invI2 = M.vec3_dot(ia2, axis);
		if (invI1 > 0) {
			double rsq = M.vec3_dot(_relativeAnchor1, _relativeAnchor1);
			double dot = M.vec3_dot(axis, _relativeAnchor1);
			double projsq = rsq - dot * dot;
			if (projsq > 0) {
				if (_b1._invMass > 0) {
					invI1 = 1 / (1 / invI1 + _b1._mass * projsq);
				} else {
					invI1 = 0;
				}
			}
		}
		if (invI2 > 0) {
			double rsq = M.vec3_dot(_relativeAnchor2, _relativeAnchor2);
			double dot = M.vec3_dot(axis, _relativeAnchor2);
			double projsq = rsq - dot * dot;
			if (projsq > 0) {
				if (_b2._invMass > 0) {
					invI2 = 1 / (1 / invI2 + _b2._mass * projsq);
				} else {
					invI2 = 0;
				}
			}
		}
		return invI1 + invI2 == 0 ? 0 : 1 / (invI1 + invI2);
	}

	protected double computeEffectiveInertiaMoment2(Vec3 axis1, Vec3 axis2) {
		Vec3 ia1=new Vec3();
		Vec3 ia2=new Vec3();
		M.vec3_mulMat3(ia1, axis1, _b1._invInertia);
		M.vec3_mulMat3(ia2, axis2, _b2._invInertia);
		double invI1 = M.vec3_dot(ia1, axis1);
		double invI2 = M.vec3_dot(ia2, axis2);
		if (invI1 > 0) {
			double rsq = M.vec3_dot(_relativeAnchor1, _relativeAnchor1);
			double dot = M.vec3_dot(axis1, _relativeAnchor1);
			double projsq = rsq * rsq - dot * dot;
			if (projsq > 0) {
				if (_b1._invMass > 0) {
					invI1 = 1 / (1 / invI1 + _b1._mass * projsq);
				} else {
					invI1 = 0;
				}
			}
		}
		if (invI2 > 0) {
			double rsq = M.vec3_dot(_relativeAnchor2, _relativeAnchor2);
			double dot = M.vec3_dot(axis2, _relativeAnchor2);
			double projsq = rsq * rsq - dot * dot;
			if (projsq > 0) {
				if (_b2._invMass > 0) {
					invI2 = 1 / (1 / invI2 + _b2._mass * projsq);
				} else {
					invI2 = 0;
				}
			}
		}
		return invI1 + invI2 == 0 ? 0 : 1 / (invI1 + invI2);
	}

	// --- internal ---

	 public double _getWarmStartingFactor() {
		switch (_positionCorrectionAlgorithm) {
		case PositionCorrectionAlgorithm.BAUMGARTE:
			return Setting.jointWarmStartingFactorForBaungarte;
		default:
			return Setting.jointWarmStartingFactor;
		}
	}

	// !! don't forget to call this from constraint solver !!
	public void _syncAnchors() {
		Transform tf1;
		Transform tf2;
		tf1 = _b1._transform;
		tf2 = _b2._transform;

		// anchors
		M.vec3_mulMat3(_relativeAnchor1, _localAnchor1, tf1._rotation);
		M.vec3_mulMat3(_relativeAnchor2, _localAnchor2, tf2._rotation);
		M.vec3_add(_anchor1, _relativeAnchor1, tf1._position);
		M.vec3_add(_anchor2, _relativeAnchor2, tf2._position);

		// bases
		M.vec3_mulMat3(_basisX1, _localBasisX1, tf1._rotation);
		M.vec3_mulMat3(_basisY1, _localBasisY1, tf1._rotation);
		M.vec3_mulMat3(_basisZ1, _localBasisZ1, tf1._rotation);
		M.vec3_mulMat3(_basisX2, _localBasisX2, tf2._rotation);
		M.vec3_mulMat3(_basisY2, _localBasisY2, tf2._rotation);
		M.vec3_mulMat3(_basisZ2, _localBasisZ2, tf2._rotation);
	}

	public void _getVelocitySolverInfo(TimeStep timeStep, JointSolverInfo info) {
		info.b1 = _b1;
		info.b2 = _b2;
		info.numRows = 0;
	}

	public void _getPositionSolverInfo(JointSolverInfo info) {
		info.b1 = _b1;
		info.b2 = _b2;
		info.numRows = 0;
	}

	public void _checkDestruction() {
		double forceSq = M.vec3_dot(_appliedForce, _appliedForce);
		double torqueSq = M.vec3_dot(_appliedTorque, _appliedTorque);

		if (_breakForce > 0 && forceSq > _breakForce * _breakForce) {
			_world.removeJoint(this);
			return;
		}
		if (_breakTorque > 0 && torqueSq > _breakTorque * _breakTorque) {
			_world.removeJoint(this);
			return;
		}
	}

   public void _attachLinks() {
		_link1._other = _b2;
		_link2._other = _b1;
		//M.list_push(_b1._jointLinkList, _b1._jointLinkListLast, _prev, _next, _link1);
		
		if(_b1._jointLinkList==null) {
			_b1._jointLinkList=_link1;
			_b1._jointLinkListLast=_link1;
		}else {
			_b1._jointLinkListLast._next=_link1;
			_link1._prev=_b1._jointLinkListLast;
			_b1._jointLinkListLast=_link1;
		}
		
		//M.list_push(_b2._jointLinkList, _b2._jointLinkListLast, _prev, _next, _link2);
		if(_b2._jointLinkList==null) {
			_b2._jointLinkList=_link2;
			_b2._jointLinkListLast=_link2;
		}else {
			_b2._jointLinkListLast._next=_link2;
			_link2._prev=_b2._jointLinkListLast;
			_b2._jointLinkListLast=_link2;
		}
		_b1._numJointLinks++;
		_b2._numJointLinks++;
		_b1.wakeUp();
		_b2.wakeUp();
	}

   public void  _detachLinks() {
		//M.list_remove(_b1._jointLinkList, _b1._jointLinkListLast, _prev, _next, _link1);
	   JointLink prev1 = _link1._prev;
	   JointLink next1 = _link1._next;
	   if(prev1 != null) {
			prev1._next = next1;
		}
		if(next1 != null) {
			next1._prev = prev1;
		}
		if(_link1 == _b1._jointLinkList) {
			_b1._jointLinkList = _b1._jointLinkList._next;
		}
		if(_link1 == _b1._jointLinkListLast) {
			_b1._jointLinkListLast = _b1._jointLinkListLast._prev;
		}
		_link1._next = null;
		_link1._prev = null;
		
		//M.list_remove(_b2._jointLinkList, _b2._jointLinkListLast, _prev, _next, _link2);
		JointLink prev2 = _link2._prev;
		JointLink next2 = _link2._next;
		if(prev2 != null) {
			prev2._next = next2;
		}
		if(next2 != null) {
			next2._prev = prev2;
		}
		if(_link2 == _b2._jointLinkList) {
			_b2._jointLinkList = _b2._jointLinkList._next;
		}
		if(_link2 == _b2._jointLinkListLast) {
			_b2._jointLinkListLast = _b2._jointLinkListLast._prev;
		}
		_link2._next = null;
		_link2._prev = null;
		
		
		_link1._other = null;
		_link2._other = null;
		_b1._numJointLinks--;
		_b2._numJointLinks--;
		_b1.wakeUp();
		_b2.wakeUp();
	}

	// --- public ---

	/**
	 * Returns the first rigid body.
	 */
	public RigidBody getRigidBody1() {
		return _b1;
	}

	/**
	 * Returns the second rigid body.
	 */
	public RigidBody getRigidBody2() {
		return _b2;
	}

	/**
	 * Returns the type of the joint.
	 *
	 * See `JointType` for details.
	 */
	public int getType() {
		return _type;
	}

	/**
	 * Returns the copy of first rigid body's anchor point in world coordinates.
	 */
	public Vec3 getAnchor1() {
		Vec3 v = new Vec3();
		M.vec3_toVec3(v, _anchor1);
		return v;
	}

	/**
	 * Returns the copy of second rigid body's anchor point in world coordinates.
	 */
	public Vec3 getAnchor2() {
		Vec3 v = new Vec3();
		M.vec3_toVec3(v, _anchor2);
		return v;
	}

	/**
	 * Sets `anchor` to the first rigid body's anchor point in world coordinates.
	 *
	 * This does not create a new instance of `Vec3`.
	 */
	public void getAnchor1To(Vec3 anchor) {
		M.vec3_toVec3(anchor, _anchor1);
	}

	/**
	 * Sets `anchor` to the second rigid body's anchor point in world coordinates.
	 *
	 * This does not create a new instance of `Vec3`.
	 */
	public void getAnchor2To(Vec3 anchor) {
		M.vec3_toVec3(anchor, _anchor2);
	}

	/**
	 * Returns the first rigid body's local anchor point in world coordinates.
	 */
	public Vec3 getLocalAnchor1() {
		Vec3 v = new Vec3();
		M.vec3_toVec3(v, _localAnchor1);
		return v;
	}

	/**
	 * Returns the second rigid body's local anchor point in world coordinates.
	 */
	public Vec3 getLocalAnchor2() {
		Vec3 v = new Vec3();
		M.vec3_toVec3(v, _localAnchor2);
		return v;
	}

	/**
	 * Sets `localAnchor` to the first rigid body's anchor point in local coordinates.
	 *
	 * This does not create a new instance of `Vec3`.
	 */
	public void getLocalAnchor1To(Vec3 localAnchor) {
		M.vec3_toVec3(localAnchor, _localAnchor1);
	}

	/**
	 * Sets `localAnchor` to the second rigid body's anchor point in local coordinates.
	 *
	 * This does not create a new instance of `Vec3`.
	 */
	public void getLocalAnchor2To(Vec3 localAnchor) {
		M.vec3_toVec3(localAnchor, _localAnchor2);
	}

	/**
	 * Returns the basis of the joint for the first rigid body in world coordinates.
	 */
	public Mat3 getBasis1() {
		Mat3 m = new Mat3();
		m.fromCols(_basisX1, _basisY1, _basisZ1);
		//M.mat3_fromCols(m, _basisX1, _basisY1, _basisZ1);
		return m;
	}

	/**
	 * Returns the basis of the joint for the second rigid body in world coordinates.
	 */
	public Mat3 getBasis2() {
		Mat3 m = new Mat3();
		m.fromCols(_basisX2, _basisY2, _basisZ2);
//		var b:IMat3;
//		M.mat3_fromCols(b, _basisX2, _basisY2, _basisZ2);
//		M.mat3_toMat3(m, b);
		return m;
	}

	/**
	 * Sets `basis` to the basis of the joint for the first rigid body in world coordinates.
	 *
	 * This does not create a new instance of `Mat3`.
	 */
	public void getBasis1To(Mat3 basis) {
//		var b:IMat3;
//		M.mat3_fromCols(b, _basisX1, _basisY1, _basisZ1);
//		M.mat3_toMat3(basis, b);
		basis.fromCols(_basisX1, _basisY1, _basisZ1);
	}

	/**
	 * Sets `basis` to the basis of the joint for the second rigid body in world coordinates.
	 *
	 * This does not create a new instance of `Mat3`.
	 */
	public void getBasis2To(Mat3 basis) {
//		var b:IMat3;
//		M.mat3_fromCols(b, _basisX2, _basisY2, _basisZ2);
//		M.mat3_toMat3(basis, b);
		basis.fromCols(_basisX2, _basisY2, _basisZ2);
	}

	/**
	 * Returns whether to allow the connected rigid bodies to collide each other.
	 */
	public boolean getAllowCollision() {
		return _allowCollision;
	}

	/**
	 * Sets whether to allow the connected rigid bodies to collide each other.
	 */
	public void setAllowCollision(boolean allowCollision) {
		_allowCollision = allowCollision;
	}

	/**
	 * Returns the magnitude of the constraint force at which the joint will be destroyed.
	 *
	 * Returns `0` if the joint is unbreakable.
	 */
	public double getBreakForce() {
		return _breakForce;
	}

	/**
	 * Sets the magnitude of the constraint force at which the joint will be destroyed.
	 *
	 * Set `0` for unbreakable joints.
	 */
	public void setBreakForce(double breakForce) {
		_breakForce = breakForce;
	}

	/**
	 * Returns the magnitude of the constraint torque at which the joint will be destroyed.
	 *
	 * Returns `0` if the joint is unbreakable.
	 */
	public double getBreakTorque() {
		return _breakTorque;
	}

	/**
	 * Sets the magnitude of the constraint force at which the joint will be destroyed.
	 *
	 * Set `0` for unbreakable joints.
	 */
	public void setBreakTorque(double breakTorque) {
		_breakTorque = breakTorque;
	}

	/**
	 * Returns the type of the position correction algorithm for the joint.
	 *
	 * See `PositionCorrectionAlgorithm` for details.
	 */
	public int getPositionCorrectionAlgorithm() {
		return _positionCorrectionAlgorithm;
	}

	/**
	 * Sets the type of the position correction algorithm to `positionCorrectionAlgorithm` for the joint.
	 *
	 * See `PositionCorrectionAlgorithm` for details.
	 */
	public void setPositionCorrectionAlgorithm(int positionCorrectionAlgorithm) {
		switch (positionCorrectionAlgorithm) {
		case PositionCorrectionAlgorithm._BAUMGARTE:
		case PositionCorrectionAlgorithm._SPLIT_IMPULSE:
		case 	PositionCorrectionAlgorithm._NGS:
			break;
		default:
			 M.error("invalid position correction algorithm id: " + positionCorrectionAlgorithm);
			 return;
		}
		_positionCorrectionAlgorithm = positionCorrectionAlgorithm;
	}

	/**
	 * Returns the force applied to the first rigid body at the last time step.
	 */
	public Vec3 getAppliedForce() {
		Vec3 v = new Vec3();
		M.vec3_toVec3(v, _appliedForce);
		return v;
	}

	/**
	 * Sets `appliedForce` to the force applied to the first rigid body at the last time step.
	 *
	 * This does not create a new instance of `Vec3`.
	 */
	public void getAppliedForceTo(Vec3 appliedForce) {
		M.vec3_toVec3(appliedForce, _appliedForce);
	}

	/**
	 * Returns the torque applied to the first rigid body at the last time step.
	 */
	public Vec3 getAppliedTorque() {
		Vec3 v = new Vec3();
		M.vec3_toVec3(v, _appliedTorque);
		return v;
	}

	/**
	 * Sets `appliedTorque` to the torque applied to the first rigid body at the last time step.
	 *
	 * This does not create a new instance of `Vec3`.
	 */
	public void getAppliedTorqueTo(Vec3 appliedTorque) {
		M.vec3_toVec3(appliedTorque, _appliedTorque);
	}

	/**
	 * Returns the previous joint in the world.
	 *
	 * If the previous one does not exist, `null` will be returned.
	 */
	public Joint getPrev() {
		return _prev;
	}

	/**
	 * Returns the next joint in the world.
	 *
	 * If the next one does not exist, `null` will be returned.
	 */
	public Joint getNext() {
		return _next;
	}
	
	
	
}
