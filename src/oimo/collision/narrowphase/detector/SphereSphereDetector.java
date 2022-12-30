package oimo.collision.narrowphase.detector;
import oimo.collision.geometry.*;
import oimo.collision.narrowphase.*;
import oimo.common.MathUtil;
import oimo.common.Transform;

/**
 * Sphere vs Sphere detector.
 */
public class SphereSphereDetector extends Detector {
	/**
	 * Default constructor.
	 */
	public SphereSphereDetector() {
		super(false);
	}

	@Override
	public void detectImpl(DetectorResult result, Geometry geom1, Geometry geom2, Transform tf1, Transform tf2, CachedDetectorData cachedData) {
		SphereGeometry s1 =  (SphereGeometry) geom1;
		SphereGeometry s2 =  (SphereGeometry) geom2;

		result.incremental = false;
		float dX=tf1._position.x- tf2._position.x;
		float dY=tf1._position.y- tf2._position.y;
		float dZ=tf1._position.z- tf2._position.z;
		float r1 = s1._radius;
		float r2 = s2._radius;
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
		float pos1X = tf1._position.x + nX * -r1;
		float pos1Y = tf1._position.y + nY * -r1;
		float pos1Z = tf1._position.z + nZ * -r1;
		float pos2X = tf2._position.x + nX * r2;
		float pos2Y = tf2._position.y + nY * r2;
		float pos2Z = tf2._position.z + nZ * r2;
		this.addPoint(result,pos1X,pos1Y,pos1Z,pos2X,pos2Y,pos2Z,r1 + r2 - len,0);
	
	}
}
