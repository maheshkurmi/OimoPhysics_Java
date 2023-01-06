package oimo.collision.narrowphase.detector;
import oimo.collision.geometry.*;
import oimo.collision.narrowphase.*;
import oimo.common.Transform;
import oimo.common.Vec3;

/**
 * Interface of a collision detector for narrow-phase collision detection.
 */
public abstract class Detector {
	//internal flag to chek order of shape type being collided so that Box-Sphere and Sphere-box can be handled by single detector
	boolean swapped;

	public Detector(boolean swapped) {
		this.swapped = swapped;
	}

	// --- private ---

	protected void setNormal(DetectorResult result, Vec3 n) {
		setNormal(result, n.x,n.y,n.z);
	}
	
	protected void setNormal(DetectorResult result, double nx, double ny, double nz) {
		result.normal.set(nx,ny,nz);
		if (swapped) {
			result.normal.negateEq();
		}
	}

	protected void addPoint(DetectorResult result,Vec3 pos1, Vec3 pos2, double depth, int id) {
		addPoint(result,pos1.x,pos1.y,pos1.z,pos2.x,pos2.y,pos2.z,depth,id);
	}
	
	protected void addPoint(DetectorResult result,double p1x,double p1y,double p1z, double p2x,double p2y,double p2z, double depth, int id) {
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

	protected abstract void detectImpl(DetectorResult result, Geometry geom1, Geometry geom2, Transform tf1, Transform tf2, CachedDetectorData cachedData);

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
