package oimo.collision.narrowphase.detector;

import oimo.collision.narrowphase.detector.gjkepa.GjkCache;

/**
 * This is used for caching narrow-phase data of a pair of collision geometries.
 */
public class CachedDetectorData {
	public GjkCache _gjkCache;

	public CachedDetectorData() {
	}

	public void _clear() {
		if (_gjkCache != null) _gjkCache.clear();
	}
}