package oimo.collision.narrowphase.detector;
import oimo.collision.geometry.*;
import oimo.collision.narrowphase.*;
import oimo.common.MathUtil;
import oimo.common.Transform;

/**
 * Sphere vs Capsule detector.
 */
public class SphereCapsuleDetector extends Detector {
	/**
	 * If `swapped` is `true`, the collision detector expects `CapsuleGeometry` and `SphereGeometry` for the
	 * first and second argument of `SphereCapsuleDetector.detect`. If `swapped` is `false`, the collision detector expects
	 * `SphereGeometry` and `CapsuleGeometry` instead.
	 */
	public SphereCapsuleDetector(boolean swapped) {
		super(swapped);
	}

	@Override 
	public void detectImpl(DetectorResult result, Geometry geom1, Geometry geom2, Transform tf1, Transform tf2, CachedDetectorData cachedData) {
		SphereGeometry s1 =  (SphereGeometry) geom1;
		CapsuleGeometry c2 =  (CapsuleGeometry) geom2;

		result.incremental = false;

		float hh2 = c2._halfHeight;
		float r1 = s1._radius;
		float r2 = c2._radius;

		//extract y axis from cylinder's rotation matrix
		float axis2X = tf2._rotation.e01;
		float axis2Y = tf2._rotation.e11;
		float axis2Z = tf2._rotation.e21;
		
	
		// closest point 1
		float cp1X = tf1._position.x;
		float cp1Y = tf1._position.y;
		float cp1Z = tf1._position.z;
		
		// find closest point on segment 2

		// line segment (p2, q2)
		float p2X = tf2._position.x + axis2X * -hh2;
		float p2Y = tf2._position.y + axis2Y * -hh2;
		float p2Z = tf2._position.z + axis2Z * -hh2;
		float q2X = tf2._position.z + axis2X * hh2;
		float q2Y = tf2._position.y + axis2Y * hh2;
		float q2Z = tf2._position.z + axis2Z * hh2;
		
		// p1-p2
		float p12X = cp1X - p2X;
		float p12Y = cp1Y - p2Y;
		float p12Z = cp1Z - p2Z;
		
		// q - p
		float d2X = q2X - p2X;
		float d2Y = q2Y - p2Y;
		float d2Z = q2Z - p2Z;
		
	
		float d22 = hh2 * hh2 * 4;
		float t = p12X * d2X + p12Y * d2Y + p12Z * d2Z;
		if(t < 0) {
			t = 0;
		} else if(t > d22) {
			t = 1;
		} else {
			t /= d22;
		}
		
		float cp2X = p2X + d2X * t;
		float cp2Y = p2Y + d2Y * t;
		float cp2Z = p2Z + d2Z * t;

		// perform sphere vs sphere collision
		float dX = cp1X - cp2X;
		float dY = cp1Y - cp2Y;
		float dZ = cp1Z - cp2Z;
		
		float len2 = dX * dX + dY * dY + dZ * dZ;
		if(len2 >= (r1 + r2) * (r1 + r2)) {
			return;
		}
		float len = MathUtil.sqrt(len2);
		float nX;
		float nY;
		float nZ;
		if(len > 0) {
			nX = dX * (1 / len);
			nY = dY * (1 / len);
			nZ = dZ * (1 / len);
		} else {
			nX = 1;
			nY = 0;
			nZ = 0;
		}
		
		this.setNormal(result,nX,nY,nZ);
		float pos1X = cp1X + nX * -r1;
		float pos1Y = cp1Y + nY * -r1;
		float pos1Z = cp1Z + nZ * -r1;
		float pos2X = cp2X + nX * r2;
		float pos2Y = cp2Y + nY * r2;
		float pos2Z = cp2Z + nZ * r2;
		this.addPoint(result,pos1X,pos1Y,pos1Z,pos2X,pos2Y,pos2Z,r1 + r2 - len,0);
	}
}