package oimo.collision.narrowphase.detector;

import oimo.collision.geometry.*;
import oimo.collision.narrowphase.*;
import oimo.common.MathUtil;
import oimo.common.Transform;

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

	@Override
	public void detectImpl(DetectorResult result, Geometry geom1, Geometry geom2, Transform tf1, Transform tf2,
			CachedDetectorData cachedData) {
		CapsuleGeometry c1 = (CapsuleGeometry) geom1;
		CapsuleGeometry c2 = (CapsuleGeometry) geom2;

		result.incremental = false;

		// Y axes
		float axis1X = tf1._rotation.e01;
		float axis1Y = tf1._rotation.e11;
		float axis1Z = tf1._rotation.e21;
		float axis2X = tf2._rotation.e01;
		float axis2Y = tf2._rotation.e11;
		float axis2Z = tf2._rotation.e21;

		float hh1 = c1._halfHeight;
		float hh2 = c2._halfHeight;
		float r1 = c1._radius;
		float r2 = c2._radius;

		// line segments (p1, q1), (p2, q2)
		float p1X = tf1._position.x + axis1X * -hh1;
		float p1Y = tf1._position.y + axis1Y * -hh1;
		float p1Z = tf1._position.y + axis1Z * -hh1;
		float q1X = tf1._position.x + axis1X * hh1;
		float q1Y = tf1._position.y + axis1Y * hh1;
		float q1Z = tf1._position.z + axis1Z * hh1;
		float p2X = tf2._position.x + axis2X * -hh2;
		float p2Y = tf2._position.y + axis2Y * -hh2;
		float p2Z = tf2._position.z + axis2Z * -hh2;
		float q2X = tf2._position.x + axis2X * hh2;
		float q2Y = tf2._position.y + axis2Y * hh2;
		float q2Z = tf2._position.y + axis2Z * hh2;

		// p1 - p2
		float p12X = p1X - p2X;
		float p12Y = p1Y - p2Y;
		float p12Z = p1Z - p2Z;

		// d1=q1-p1, d2=q2-p2
		float d1X = q1X - p1X;
		float d1Y = q1Y - p1Y;
		float d1Z = q1Z - p1Z;
		float d2X = q2X - p2X;
		float d2Y = q2Y - p2Y;
		float d2Z = q2Z - p2Z;

		float p21d1 = -(p12X * d1X + p12Y * d1Y + p12Z * d1Z); // vec3_dot(p12, d1);
		float p12d2 = p12X * d2X + p12Y * d2Y + p12Z * d2Z; // vec3_dot(p12, d2);
		float d11 = hh1 * hh1 * 4;
		float d12 = d1X * d2X + d1Y * d2Y + d1Z * d2Z;
		float d22 = hh2 * hh2 * 4;

		// closest points: p1 + t1 * d1, p2 + t2 * d2
		float t1;
		float t2;

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
			float det = d11 * d22 - d12 * d12;

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

		float cp1X = p1X + d1X * t1;
		float cp1Y = p1Y + d1Y * t1;
		float cp1Z = p1Z + d1Z * t1;
		float cp2X = p2X + d2X * t2;
		float cp2Y = p2Y + d2Y * t2;
		float cp2Z = p2Z + d2Z * t2;

		// perform sphere vs sphere collision
		float dX = cp1X - cp2X;
		float dY = cp1Y - cp2Y;
		float dZ = cp1Z - cp2Z;
		float len2 = dX * dX + dY * dY + dZ * dZ;
		if (len2 >= (r1 + r2) * (r1 + r2)) {
			return;
		}
		float len = MathUtil.sqrt(len2);
		float nX;
		float nY;
		float nZ;
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

		float pos1X = cp1X + nX * -r1;
		float pos1Y = cp1Y + nY * -r1;
		float pos1Z = cp1Z + nZ * -r1;
		float pos2X = cp2X + nX * r2;
		float pos2Y = cp2Y + nY * r2;
		float pos2Z = cp2Z + nZ * r2;
		this.addPoint(result, pos1X, pos1Y, pos1Z, pos2X, pos2Y, pos2Z, r1 + r2 - len, 0);
	}
}