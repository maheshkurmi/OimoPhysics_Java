package oimo.collision.narrowphase.detector;
import oimo.collision.geometry.*;
import oimo.collision.narrowphase.*;
import oimo.common.Transform;
import oimo.common.Vec3;

/**
 * Interface of a collision detector for narrow-phase collision detection.
 */
public abstract class Detector {
	boolean swapped;

	public Detector(boolean swapped) {
		this.swapped = swapped;
	}

	// --- private ---

	public void setNormal(DetectorResult result, Vec3 n) {
		setNormal(result, n.x,n.y,n.z);
	}
	
	public void setNormal(DetectorResult result, float nx, float ny, float nz) {
		result.normal.set(nx,ny,nz);
		if (swapped) {
			result.normal.negateEq();
		}
	}

	public void addPoint(DetectorResult result,Vec3 pos1, Vec3 pos2, float depth, int id) {
		addPoint(result,pos1.x,pos1.y,pos1.z,pos2.x,pos2.y,pos2.z,depth,id);
	}
	
	public void addPoint(DetectorResult result,float p1x,float p1y,float p1z, float p2x,float p2y,float p2z, float depth, int id) {
		DetectorResultPoint p = result.points[result.numPoints++];
		p.depth = depth;
		p.id = id;
		if (swapped) {
			p.position1.set(p2x,p2y,p2z);
			p.position2.set(p1x,p1y,p1z);
		} else {
			p.position1.set(p1x,p1y,p1z);
			p.position2.set(p2x,p2y,p2z);
		}
	}

	public abstract void detectImpl(DetectorResult result, Geometry geom1, Geometry geom2, Transform tf1, Transform tf2, CachedDetectorData cachedData);

	// --- public ---

	/**
	 * Computes the contact manifold of two collision geometries `geom1` and `geom2` with the transforms
	 * `transform1` and `transform2`, and stores it to `result`. `cachedData` is used to improve performance
	 * of collision detection in some detectors.
	 */
	public void detect(DetectorResult result, Geometry geom1, Geometry geom2, Transform transform1, Transform transform2, CachedDetectorData cachedData) {
		result.clear();
		if (swapped) {
			detectImpl(result, geom2, geom1, transform2, transform1, cachedData);
		} else {
			detectImpl(result, geom1, geom2, transform1, transform2, cachedData);
		}
	}
}
