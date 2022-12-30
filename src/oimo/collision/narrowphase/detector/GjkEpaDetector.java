package oimo.collision.narrowphase.detector;
import oimo.collision.geometry.*;
import oimo.collision.narrowphase.*;
import oimo.collision.narrowphase.detector.gjkepa.*;
import oimo.common.*;

/**
 * General convex collision detector using GJK/EPA
 */
public class GjkEpaDetector extends Detector {
	/**
	 * Default constructor.
	 */
	public GjkEpaDetector() {
		super(false);
	}

	@Override 
	public void detectImpl(DetectorResult result, Geometry geom1, Geometry geom2, Transform tf1, Transform tf2, CachedDetectorData cachedData) {
		GjkEpa gjkEpa = GjkEpa.getInstance();
		ConvexGeometry g1 =  (ConvexGeometry) geom1;
		ConvexGeometry g2 =  (ConvexGeometry) geom2;
		int status = gjkEpa.computeClosestPoints(g1, g2, tf1, tf2, Setting.enableGJKCaching ? cachedData : null);
		result.incremental = true;
		
		if (status != GjkEpaResultState.SUCCEEDED) {
			System.err.println("GJK/EPA failed: status=" + status);
			return;
		}

	
		if (gjkEpa.distance > g1._gjkMargin + g1._gjkMargin) { // geometries are separating
			return;
		}
		
		Vec3 v = gjkEpa.closestPoint1;
		float pos1X = v.x;
		float pos1Y = v.y;
		float pos1Z = v.z;
		Vec3 v1 = gjkEpa.closestPoint2;
		float pos2X = v1.x;
		float pos2Y = v1.y;
		float pos2Z = v1.z;
		
		float normalX;
		float normalY;
		float normalZ;
		normalX = pos1X - pos2X;
		normalY = pos1Y - pos2Y;
		normalZ = pos1Z - pos2Z;
		if(normalX * normalX + normalY * normalY + normalZ * normalZ == 0) {
			return;
		}
		if(gjkEpa.distance < 0) {
			normalX = -normalX;
			normalY = -normalY;
			normalZ = -normalZ;
		}
		float l = normalX * normalX + normalY * normalY + normalZ * normalZ;
		if(l > 0) {
			l = 1 / MathUtil.sqrt(l);
		}
		normalX *= l;
		normalY *= l;
		normalZ *= l;
		this.setNormal(result,normalX,normalY,normalZ);
		pos1X += normalX * -g1._gjkMargin;
		pos1Y += normalY * -g1._gjkMargin;
		pos1Z += normalZ * -g1._gjkMargin;
		pos2X += normalX * g2._gjkMargin;
		pos2Y += normalY * g2._gjkMargin;
		pos2Z += normalZ * g2._gjkMargin;
		this.addPoint(result,pos1X,pos1Y,pos1Z,pos2X,pos2Y,pos2Z,g1._gjkMargin + g2._gjkMargin - gjkEpa.distance,0);

	}

}
