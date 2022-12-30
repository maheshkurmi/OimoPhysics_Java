package oimo.dynamics.constraint.contact;
import oimo.common.*;
import oimo.dynamics.TimeStep;
import oimo.dynamics.constraint.*;
import oimo.dynamics.constraint.info.*;
import oimo.dynamics.constraint.info.contact.*;
import oimo.dynamics.constraint.solver.pgs.*;
import oimo.dynamics.rigidbody.*;

/**
 * A contact constraint provides collision information for a contact constraint solver.
 * This holds a contact manifold, which has some contact points, contact normals, and
 * contact impulses. See `Manifold` for more information.
 */
public class ContactConstraint {
	public int _positionCorrectionAlgorithm;

	public Manifold _manifold;

	public Shape _s1;
	public Shape _s2;
	public Transform _tf1;
	public Transform _tf2;
	public float _invM1;
	public float _invM2;
	public float _friction;
	public float _restitution;

	public Mat3 _invI1;
	public Mat3 _invI2;

	public RigidBody _b1;
	public RigidBody _b2;

	public ConstraintSolver _solver;

	public ContactConstraint(Manifold manifold) {
		_solver = new PgsContactConstraintSolver(this);
		_manifold = manifold;
	}

	// --- internal ---

	public ContactConstraint(Shape s1, Shape s2) {
		_s1 = s1;
		_s2 = s2;
		_b1 = _s1._rigidBody;
		_b2 = _s2._rigidBody;
		_tf1 = _b1._transform;
		_tf2 = _b2._transform;
	}
	
	
	public void _attach(Shape s1,Shape s2) {
		_s1 = s1;
		_s2 = s2;
		_b1 = _s1._rigidBody;
		_b2 = _s2._rigidBody;
		_tf1 = _b1._transform;
		_tf2 = _b2._transform;
	}

	public void _detach() {
		_s1 = null;
		_s2 = null;
		_b1 = null;
		_b2 = null;
		_tf1 = null;
		_tf2 = null;
	}

	public void _getVelocitySolverInfo(TimeStep timeStep, ContactSolverInfo info) {
		info.b1 = _b1;
		info.b2 = _b2;

		Vec3 normal=new Vec3();
		Vec3 tangent=new Vec3();
		Vec3 binormal=new Vec3();
		M.vec3_assign(normal, _manifold._normal);
		M.vec3_assign(tangent, _manifold._tangent);
		M.vec3_assign(binormal, _manifold._binormal);

		float friction = MathUtil.sqrt(_s1._friction * _s2._friction);
		float restitution = MathUtil.sqrt(_s1._restitution * _s2._restitution);

		int num = _manifold._numPoints;
		info.numRows = 0;

		//Vec3 posDiff;
		//M.vec3_sub(posDiff, _tf1._position, _tf2._position);

		for (int i=0;i<num;i++) {
			ManifoldPoint p = _manifold._points[i];

			if (p._depth < 0) {
				p._disabled = true;

				// clear accumulated impulses
				p._impulse.clear();

				// skip separated points
				continue;
			} else {
				p._disabled = false;
			}

			ContactSolverInfoRow row = info.rows[info.numRows++];

			row.friction = friction;
			row.cfm = 0; // TODO: implement APIs for CFM setting?

			// set Jacobian
			JacobianRow j;

			j = row.jacobianN;
			M.vec3_assign(j.lin1, normal);
			M.vec3_assign(j.lin2, normal);
			M.vec3_cross(j.ang1, p._relPos1, normal);
			M.vec3_cross(j.ang2, p._relPos2, normal);

			j = row.jacobianT;
			M.vec3_assign(j.lin1, tangent);
			M.vec3_assign(j.lin2, tangent);
			M.vec3_cross(j.ang1, p._relPos1, tangent);
			M.vec3_cross(j.ang2, p._relPos2, tangent);

			j = row.jacobianB;
			M.vec3_assign(j.lin1, binormal);
			M.vec3_assign(j.lin2, binormal);
			M.vec3_cross(j.ang1, p._relPos1, binormal);
			M.vec3_cross(j.ang2, p._relPos2, binormal);

			// compute relative velocity
			j = row.jacobianN;
			float rvn =
				(M.vec3_dot(j.lin1, _b1._vel) + M.vec3_dot(j.ang1, _b1._angVel)) -
				(M.vec3_dot(j.lin2, _b2._vel) + M.vec3_dot(j.ang2, _b2._angVel))
			;

			// disable bounce for warm-started contacts
			if (rvn < -Setting.contactEnableBounceThreshold && !p._warmStarted) {
				row.rhs = -rvn * restitution;
			} else {
				row.rhs = 0;
			}

			// set minimum RHS for baumgarte position correction
			if (_positionCorrectionAlgorithm == PositionCorrectionAlgorithm.BAUMGARTE) {
				if (p._depth > Setting.linearSlop) {
					float minRhs = (p._depth - Setting.linearSlop) * Setting.velocityBaumgarte * timeStep.invDt;
					if (row.rhs < minRhs) row.rhs = minRhs;
				}
			}

			// reset impulses if warm starting is disabled
			if (!p._warmStarted) {
				p._impulse.clear();
			}

			row.impulse = p._impulse;
		}
	}

	public void _getPositionSolverInfo(ContactSolverInfo info) {
		info.b1 = _b1;
		info.b2 = _b2;

		Vec3 normal=_manifold._normal;

		int num = _manifold._numPoints;
		info.numRows = 0;

		for (int i=0;i<num;i++) {
			ManifoldPoint p = _manifold._points[i];

			if (p._disabled) {
				continue; // skip disabled points
			}

			ContactSolverInfoRow row = info.rows[info.numRows++];

			// set normal Jacobian
			JacobianRow j = row.jacobianN;
			M.vec3_assign(j.lin1, normal);
			M.vec3_assign(j.lin2, normal);
			M.vec3_cross(j.ang1, p._relPos1, normal);
			M.vec3_cross(j.ang2, p._relPos2, normal);

			row.rhs = p._depth - Setting.linearSlop;
			if (row.rhs < 0) {
				row.rhs = 0;
			}

			row.impulse = p._impulse;
		}
	}

	// !! don't forget to call this from constraint solvers !!
	public void _syncManifold() {
		_manifold._updateDepthsAndPositions(_tf1, _tf2);
	}

	// --- public ---

	/**
	 * Returns the first shape of the contact.
	 */
	public Shape getShape1() {
		return _s1;
	}

	/**
	 * Returns the second shape of the contact.
	 */
	public Shape getShape2() {
		return _s2;
	}

	/**
	 * Returns the contact manifold.
	 */
	public Manifold getManifold() {
		return _manifold;
	}

	/**
	 * Returns whether the two rigid bodies are touching.
	 */
	public boolean isTouching() {
		for (int i=0;i<_manifold._numPoints;i++) {
			if (_manifold._points[i]._depth >= 0) return true;
		}
		return false;
	}

}
