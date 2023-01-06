package oimo.dynamics.constraint.joint;
import oimo.common.M;
import oimo.common.Vec3;
import oimo.common.Quat;
import oimo.common.Mat3;
/**
 * Internal class
 */
public class BasisTracker {
	Joint joint;

	public Vec3 x;
	public Vec3 y;
	public Vec3 z;

	public BasisTracker(Joint joint) {
		this.joint = joint;
		x=new Vec3();
		y=new Vec3();
		z=new Vec3();
	}

	public void trackByX() {
		trackByAxis(0);
	}

	public void trackByY() {
		trackByAxis(1);
	}

	 public void trackByZ() {
		trackByAxis(2);
	}

	public void trackByAxis(int axis) {
		double invM1 = joint._b1._invMass;
		double invM2 = joint._b2._invMass;
		Quat q=new Quat();
		Quat idQ=new Quat();
		Quat slerpQ=new Quat();
		Mat3 slerpM=new Mat3();

		Vec3 newX=new Vec3();
		Vec3 newY=new Vec3();
		Vec3 newZ=new Vec3();

		Vec3 prevX;
		Vec3 prevY;

		if (axis == 0) {
			// compute X'
			
			M.quat_arc(q, joint._basisX1, joint._basisX2);
			M.quat_id(idQ);
			M.quat_slerp(slerpQ, idQ, q, invM1 / (invM1 + invM2));
			M.mat3_fromQuat(slerpM, slerpQ);
			M.vec3_mulMat3(newX, joint._basisX1, slerpM);

			// set X, Y
			prevX=x;
			prevY=y;
			//M.vec3_assign(prevX, x);
			//M.vec3_assign(prevY, y);
		} else if (axis == 1) {
			// compute X'
			M.quat_arc(q, joint._basisY1, joint._basisY2);
			M.quat_id(idQ);
			M.quat_slerp(slerpQ, idQ, q, invM1 / (invM1 + invM2));
			M.mat3_fromQuat(slerpM, slerpQ);
			M.vec3_mulMat3(newX, joint._basisY1, slerpM);

			// set X, Y
			prevX=y;
			prevY=z;
			//M.vec3_assign(prevX, y);
			//M.vec3_assign(prevY, z);
		} else {
			// compute X'
			M.quat_arc(q, joint._basisZ1, joint._basisZ2);
			M.quat_id(idQ);
			M.quat_slerp(slerpQ, idQ, q, invM1 / (invM1 + invM2));
			M.mat3_fromQuat(slerpM, slerpQ);
			M.vec3_mulMat3(newX, joint._basisZ1, slerpM);

			// set X, Y
			prevX=z;
			prevY=x;
			//M.vec3_assign(prevX, z);
			//M.vec3_assign(prevY, x);
		}

		// we compute Y' and Z' from X, Y, and X' (new basis: <X', Y', Z'>, previous basis: <X, Y, Z>)
		// in following algorithm:
		//   slerp = X -> X'
		//   Y'' = slerp(Y)
		//   Z'' = X' cross Y''
		//   if |Z''| > eps then
		//     Z' = normalize(Z'')
		//   else
		//     Z' = perp(X')         # give up rotating previous basis
		//   Y' = Z' cross X'

		// slerp = X -> X'
		M.quat_arc(slerpQ, prevX, newX);
		M.mat3_fromQuat(slerpM, slerpQ);

		// Y'' = slerp(Y)
		M.vec3_mulMat3(newY, prevY, slerpM);

		// Z'' = X cross Y''
		M.vec3_cross(newZ, newX, newY);

		if (M.vec3_dot(newZ, newZ) > 1e-6) {
			// Z' = normalize(Z'')
			//M.vec3_normalize(newZ, newZ);
			newZ.normalize();
		} else {
			// failed to rotate previous basis, build a new right-handed orthonormal system
			M.vec3_perp(newZ, newX);
		}

		// Y' = Z' cross X'
		M.vec3_cross(newY, newZ, newX);

		if (axis == 0) {
			M.vec3_assign(x, newX);
			M.vec3_assign(y, newY);
			M.vec3_assign(z, newZ);
		} else if (axis == 1) {
			M.vec3_assign(x, newZ);
			M.vec3_assign(y, newX);
			M.vec3_assign(z, newY);
		} else {
			M.vec3_assign(x, newY);
			M.vec3_assign(y, newZ);
			M.vec3_assign(z, newX);
		}

	}

}
