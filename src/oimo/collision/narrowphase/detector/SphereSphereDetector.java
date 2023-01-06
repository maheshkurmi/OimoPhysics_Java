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
	protected void detectImpl(DetectorResult result, Geometry geom1, Geometry geom2, Transform tf1, Transform tf2, CachedDetectorData cachedData) {
		SphereGeometry s1 =  (SphereGeometry) geom1;
		SphereGeometry s2 =  (SphereGeometry) geom2;

		result.incremental = false;
		double dX=tf1._position.x- tf2._position.x;
		double dY=tf1._position.y- tf2._position.y;
		double dZ=tf1._position.z- tf2._position.z;
		double r1 = s1._radius;
		double r2 = s2._radius;
		double len2 = dX * dX + dY * dY + dZ * dZ;
		if(len2 >= (r1 + r2) * (r1 + r2)) {
			return;
		}
		double len = MathUtil.sqrt(len2);
		double nX;
		double nY;
		double nZ;
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
		double pos1X = tf1._position.x + nX * -r1;
		double pos1Y = tf1._position.y + nY * -r1;
		double pos1Z = tf1._position.z + nZ * -r1;
		double pos2X = tf2._position.x + nX * r2;
		double pos2Y = tf2._position.y + nY * r2;
		double pos2Z = tf2._position.z + nZ * r2;
		this.addPoint(result,pos1X,pos1Y,pos1Z,pos2X,pos2Y,pos2Z,r1 + r2 - len,0);
	
	}
}
