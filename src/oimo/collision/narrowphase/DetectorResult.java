package oimo.collision.narrowphase;
import oimo.common.Setting;
import oimo.common.Vec3;

/**
 * The result of narrow-phase collision detection. This is used for generating contact
 * points of a contact constraint at once or incrementally.
 */
public class DetectorResult {
	/**
	 * The number of the result points.
	 */
	public int numPoints;

	/**
	 * The result points. Note that **only the first `DetectorResult.numPoints` points are
	 * computed by the collision detector**.
	 */
	public DetectorResultPoint[] points;

	/**
	 * The normal vector of the contact plane.
	 */
	public Vec3 normal;

	/**
	 * Whether the result points are to be used for incremental menifold update.
	 */
	public boolean incremental; // for GJK/EPA detector

	/**
	 * Default constructor.
	 */
	public DetectorResult() {
		numPoints = 0;
		normal = new Vec3();
		points = new DetectorResultPoint[Setting.maxManifoldPoints];
		incremental = false;

		for (int i=0;i<Setting.maxManifoldPoints;i++) {
			points[i] = new DetectorResultPoint();
		}
	}

	// --- public ---

	/**
	 * Returns the maximum depth of the result points. Returns `0.0` if no result
	 * points are available.
	 */
	public float getMaxDepth() {
		float max = 0;
		for (int i =0; i<numPoints;i++) {
			if (points[i].depth > max) {
				max = points[i].depth;
			}
		}
		return max;
	}

	/**
	 * Cleans up the result data.
	 */
	public void  clear() {
		numPoints = 0;
		for (DetectorResultPoint p: points) {
			p.position1.zero();
			p.position2.zero();
			p.depth = 0;
			p.id = 0;
		}
		normal.zero();
	}
}
