package oimo.collision.narrowphase.detector.gjkepa;
import oimo.common.Vec3;

/**
 * Internal class.
 */
public class GjkCache {
	public Vec3 prevClosestDir;

	public GjkCache() {
		prevClosestDir = new Vec3();
	}

	public void clear() {
		prevClosestDir.zero();
	}

}