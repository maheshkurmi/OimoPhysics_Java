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
	protected void detectImpl(DetectorResult result, Geometry geom1, Geometry geom2, Transform tf1, Transform tf2, CachedDetectorData cachedData) {
		GjkEpa gjkEpa = GjkEpa.getInstance();
		ConvexGeometry g1 =  (ConvexGeometry) geom1;
		ConvexGeometry g2 =  (ConvexGeometry) geom2;
		int status = gjkEpa.computeClosestPoints(g1, g2, tf1, tf2, Setting.enableGJKCaching ? cachedData : null);
		result.incremental = true;
		
		if (status != GjkEpaResultState.SUCCEEDED) {
			System.err.println("GJK/EPA failed: status=" + status);
			return;
		}
	
		if (gjkEpa.distance > g1._gjkMargin + g2._gjkMargin) { // geometries are separating
			return;
		}
		
	
		Vec3 pos1=new Vec3();
		Vec3 pos2=new Vec3();
		M.vec3_fromVec3(pos1, gjkEpa.closestPoint1);
		M.vec3_fromVec3(pos2, gjkEpa.closestPoint2);
		
		Vec3 normal=new Vec3();
		M.vec3_sub(normal, pos1, pos2);
		
		if (M.vec3_dot(normal, normal) == 0) {
			return; // core geometries are just touching
		}
		if (gjkEpa.distance < 0) {
			M.vec3_negate(normal, normal);
		}
		M.vec3_normalize(normal, normal);
		this.setNormal(result, normal);

		// move the closest points to the surface of the geometries
		M.vec3_addRhsScaled(pos1, pos1, normal, -g1._gjkMargin);
		M.vec3_addRhsScaled(pos2, pos2, normal, g2._gjkMargin);
		this.addPoint(result, pos1, pos2, g1._gjkMargin + g2._gjkMargin - gjkEpa.distance, 0);

	}

}
