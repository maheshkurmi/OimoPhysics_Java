package oimo.collision.narrowphase.detector;

import oimo.collision.geometry.*;
import oimo.collision.narrowphase.*;
import oimo.common.M;
import oimo.common.MathUtil;
import oimo.common.Transform;
import oimo.common.Vec3;

/**
 * Capsule vs Capsule detector.
 */
public class CapsuleCapsuleDetector extends Detector {
	/**
	 * Default constructor.
	 */
	public CapsuleCapsuleDetector() {
		super(false);
	}

	protected void detectImpl(DetectorResult result, Geometry geom1, Geometry geom2, Transform tf1, Transform tf2,CachedDetectorData cachedData) {
		CapsuleGeometry c1 =  (CapsuleGeometry) geom1;
		CapsuleGeometry c2 =  (CapsuleGeometry) geom2;

		result.incremental = false;

		// Y axes
		Vec3 axis1=new Vec3();
		Vec3 axis2=new Vec3();
		M.mat3_getCol(axis1, tf1._rotation, 1);
		M.mat3_getCol(axis2, tf2._rotation, 1);

		double hh1 = c1._halfHeight;
		double hh2 = c2._halfHeight;
		double r1 = c1._radius;
		double r2 = c2._radius;

		// line segments (p1, q1), (p2, q2)
		Vec3 p1=new Vec3();
		Vec3 q1=new Vec3();
		Vec3 p2=new Vec3();
		Vec3 q2=new Vec3();
		M.vec3_addRhsScaled(p1, tf1._position, axis1, -hh1);
		M.vec3_addRhsScaled(q1, tf1._position, axis1, hh1);
		M.vec3_addRhsScaled(p2, tf2._position, axis2, -hh2);
		M.vec3_addRhsScaled(q2, tf2._position, axis2, hh2);

		// p1 - p2
		Vec3 p12=new Vec3();
		M.vec3_sub(p12, p1, p2);

		// q - p
		Vec3 d1=new Vec3();
		Vec3 d2=new Vec3();
		M.vec3_sub(d1, q1, p1);
		M.vec3_sub(d2, q2, p2);

		double p21d1 = -M.vec3_dot(p12, d1);
		double p12d2 = M.vec3_dot(p12, d2);

		double d11 = hh1 * hh1 * 4;
		double d12 = M.vec3_dot(d1, d2);
		double d22 = hh2 * hh2 * 4;

		// closest points: p1 + t1 * d1, p2 + t2 * d2
		double t1;
		double t2;

		if (d11 == 0 && d22 == 0) {
			// point vs point
			t1 = 0;
			t2 = 0;
		} else if (d11 == 0) {
			// point vs segment
			t1 = 0;

			// t2 = t1 * d12 + p12d2; <- t1 = 0
			t2 = p12d2;
			if (t2 < 0) t2 = 0;
			else if (t2 > d22) t2 = 1;
			else t2 /= d22;
		} else if (d22 == 0) {
			// segment vs point
			t2 = 0;

			// t1 = t2 * d12 + p21d1; <- t2 = 0
			t1 = p21d1;
			if (t1 < 0) t1 = 0;
			else if (t1 > d11) t1 = 1;
			else t1 /= d11;
		} else {
			double det = d11 * d22 - d12 * d12;

			if (det == 0) {
				// d1 is parallel to d2. use 0 for t1
				t1 = 0;
			} else {
				t1 = d12 * p12d2 + d22 * p21d1;
				if (t1 < 0) t1 = 0;
				else if (t1 > det) t1 = 1;
				else t1 /= det;
			}

			t2 = t1 * d12 + p12d2;
			if (t2 < 0) {
				// clamp t2 and recompute t1
				t2 = 0;

				// t1 = t2 * d12 + p21d1; <- t2 = 0
				t1 = p21d1;
				if (t1 < 0) t1 = 0;
				else if (t1 > d11) t1 = 1;
				else t1 /= d11;
			} else if (t2 > d22) {
				// clamp t2 and recompute t1
				t2 = 1;

				// t1 = t2 * d12 + p21d1; <- t2 = 1
				t1 = d12 + p21d1;
				if (t1 < 0) t1 = 0;
				else if (t1 > d11) t1 = 1;
				else t1 /= d11;
			} else {
				t2 /= d22;
			}
		}

		Vec3 cp1=new Vec3();
		Vec3 cp2=new Vec3();
		M.vec3_addRhsScaled(cp1, p1, d1, t1);
		M.vec3_addRhsScaled(cp2, p2, d2, t2);

		// perform sphere vs sphere collision
		Vec3 d=new Vec3();
		M.vec3_sub(d, cp1, cp2);
		double len2 = M.vec3_dot(d, d);
		if (len2 >= (r1 + r2) * (r1 + r2)) return;
		double len = MathUtil.sqrt(len2);
		Vec3 n=new Vec3();
		if (len > 0) {
			M.vec3_scale(n, d, 1 / len);
		} else {
			M.vec3_set(n, 1, 0, 0);
		}
		this.setNormal(result, n);
		Vec3 pos1=new Vec3();
		Vec3 pos2=new Vec3();
		M.vec3_addRhsScaled(pos1, cp1, n, -r1);
		M.vec3_addRhsScaled(pos2, cp2, n, r2);
		this.addPoint(result, pos1, pos2, r1 + r2 - len, 0);	
			
	}
	
	
	
	
	protected void detectImplOld(DetectorResult result, Geometry geom1, Geometry geom2, Transform tf1, Transform tf2,
			CachedDetectorData cachedData) {
		CapsuleGeometry c1 = (CapsuleGeometry) geom1;
		CapsuleGeometry c2 = (CapsuleGeometry) geom2;

		result.incremental = false;

		// Y axes
		double axis1X = tf1._rotation.e01;
		double axis1Y = tf1._rotation.e11;
		double axis1Z = tf1._rotation.e21;
		double axis2X = tf2._rotation.e01;
		double axis2Y = tf2._rotation.e11;
		double axis2Z = tf2._rotation.e21;

		double hh1 = c1._halfHeight;
		double hh2 = c2._halfHeight;
		double r1 = c1._radius;
		double r2 = c2._radius;

		// line segments (p1, q1), (p2, q2)
		double p1X = tf1._position.x + axis1X * -hh1;
		double p1Y = tf1._position.y + axis1Y * -hh1;
		double p1Z = tf1._position.y + axis1Z * -hh1;
		double q1X = tf1._position.x + axis1X * hh1;
		double q1Y = tf1._position.y + axis1Y * hh1;
		double q1Z = tf1._position.z + axis1Z * hh1;
		double p2X = tf2._position.x + axis2X * -hh2;
		double p2Y = tf2._position.y + axis2Y * -hh2;
		double p2Z = tf2._position.z + axis2Z * -hh2;
		double q2X = tf2._position.x + axis2X * hh2;
		double q2Y = tf2._position.y + axis2Y * hh2;
		double q2Z = tf2._position.y + axis2Z * hh2;

		// p1 - p2
		double p12X = p1X - p2X;
		double p12Y = p1Y - p2Y;
		double p12Z = p1Z - p2Z;

		// d1=q1-p1, d2=q2-p2
		double d1X = q1X - p1X;
		double d1Y = q1Y - p1Y;
		double d1Z = q1Z - p1Z;
		double d2X = q2X - p2X;
		double d2Y = q2Y - p2Y;
		double d2Z = q2Z - p2Z;

		double p21d1 = -(p12X * d1X + p12Y * d1Y + p12Z * d1Z); // vec3_dot(p12, d1);
		double p12d2 = p12X * d2X + p12Y * d2Y + p12Z * d2Z; // vec3_dot(p12, d2);
		double d11 = hh1 * hh1 * 4;
		double d12 = d1X * d2X + d1Y * d2Y + d1Z * d2Z;
		double d22 = hh2 * hh2 * 4;

		// closest points: p1 + t1 * d1, p2 + t2 * d2
		double t1;
		double t2;

		if (d11 == 0 && d22 == 0) {
			// point vs point
			t1 = 0;
			t2 = 0;
		} else if (d11 == 0) {
			// point vs segment
			t1 = 0;
			// t2 = t1 * d12 + p12d2; <- t1 = 0
			t2 = p12d2;
			if (t2 < 0)
				t2 = 0;
			else if (t2 > d22)
				t2 = 1;
			else
				t2 /= d22;
		} else if (d22 == 0) {
			// segment vs point
			t2 = 0;

			// t1 = t2 * d12 + p21d1; <- t2 = 0
			t1 = p21d1;
			if (t1 < 0)
				t1 = 0;
			else if (t1 > d11)
				t1 = 1;
			else
				t1 /= d11;
		} else {
			double det = d11 * d22 - d12 * d12;

			if (det == 0) {
				// d1 is parallel to d2. use 0 for t1
				t1 = 0;
			} else {
				t1 = d12 * p12d2 + d22 * p21d1;
				if (t1 < 0)
					t1 = 0;
				else if (t1 > det)
					t1 = 1;
				else
					t1 /= det;
			}

			t2 = t1 * d12 + p12d2;
			if (t2 < 0) {
				// clamp t2 and recompute t1
				t2 = 0;

				// t1 = t2 * d12 + p21d1; <- t2 = 0
				t1 = p21d1;
				if (t1 < 0)
					t1 = 0;
				else if (t1 > d11)
					t1 = 1;
				else
					t1 /= d11;
			} else if (t2 > d22) {
				// clamp t2 and recompute t1
				t2 = 1;

				// t1 = t2 * d12 + p21d1; <- t2 = 1
				t1 = d12 + p21d1;
				if (t1 < 0)
					t1 = 0;
				else if (t1 > d11)
					t1 = 1;
				else
					t1 /= d11;
			} else {
				t2 /= d22;
			}
		}

		double cp1X = p1X + d1X * t1;
		double cp1Y = p1Y + d1Y * t1;
		double cp1Z = p1Z + d1Z * t1;
		double cp2X = p2X + d2X * t2;
		double cp2Y = p2Y + d2Y * t2;
		double cp2Z = p2Z + d2Z * t2;

		// perform sphere vs sphere collision
		double dX = cp1X - cp2X;
		double dY = cp1Y - cp2Y;
		double dZ = cp1Z - cp2Z;
		double len2 = dX * dX + dY * dY + dZ * dZ;
		if (len2 >= (r1 + r2) * (r1 + r2)) {
			return;
		}
		double len = MathUtil.sqrt(len2);
		double nX;
		double nY;
		double nZ;
		if (len > 0) {
			nX = dX * (1 / len);
			nY = dY * (1 / len);
			nZ = dZ * (1 / len);
		} else {
			nX = 1;
			nY = 0;
			nZ = 0;
		}
		this.setNormal(result, nX, nY, nZ);

		double pos1X = cp1X + nX * -r1;
		double pos1Y = cp1Y + nY * -r1;
		double pos1Z = cp1Z + nZ * -r1;
		double pos2X = cp2X + nX * r2;
		double pos2Y = cp2Y + nY * r2;
		double pos2Z = cp2Z + nZ * r2;
		this.addPoint(result, pos1X, pos1Y, pos1Z, pos2X, pos2Y, pos2Z, r1 + r2 - len, 0);
	}
}