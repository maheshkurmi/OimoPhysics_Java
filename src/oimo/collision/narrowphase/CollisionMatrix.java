package oimo.collision.narrowphase;

import oimo.collision.geometry.GeometryType;
import oimo.collision.narrowphase.detector.*;

/**
 * CollisionMatrix provides corresponding collision detector for a pair of
 * two geometries of given types.
 */
public class CollisionMatrix {
	public Detector[][] detectors;;

	public CollisionMatrix() {
		detectors=new Detector[8][];
		for (int i=0;i<6;i++) {
			detectors[i] = new Detector[8];
		}

		GjkEpaDetector gjkEpaDetector = new GjkEpaDetector();

		int sp = GeometryType.SPHERE;
		int bo = GeometryType.BOX;
		int cy = GeometryType.CYLINDER;
		int co = GeometryType.CONE;
		int ca = GeometryType.CAPSULE;
		int ch = GeometryType.CONVEX_HULL;

		detectors[sp][sp] = new SphereSphereDetector();
		detectors[sp][bo] = new SphereBoxDetector(false);
		detectors[sp][cy] = gjkEpaDetector;
		detectors[sp][co] = gjkEpaDetector;
		detectors[sp][ca] = new SphereCapsuleDetector(false);
		detectors[sp][ch] = gjkEpaDetector;

		detectors[bo][sp] = new SphereBoxDetector(true);
		detectors[bo][bo] = new BoxBoxDetector();
		detectors[bo][cy] = gjkEpaDetector;
		detectors[bo][co] = gjkEpaDetector;
		detectors[bo][ca] = gjkEpaDetector;
		detectors[bo][ch] = gjkEpaDetector;

		detectors[cy][sp] = gjkEpaDetector;
		detectors[cy][bo] = gjkEpaDetector;
		detectors[cy][cy] = gjkEpaDetector;
		detectors[cy][co] = gjkEpaDetector;
		detectors[cy][ca] = gjkEpaDetector;
		detectors[cy][ch] = gjkEpaDetector;

		detectors[co][sp] = gjkEpaDetector;
		detectors[co][bo] = gjkEpaDetector;
		detectors[co][cy] = gjkEpaDetector;
		detectors[co][co] = gjkEpaDetector;
		detectors[co][ca] = gjkEpaDetector;
		detectors[co][ch] = gjkEpaDetector;

		detectors[ca][sp] = new SphereCapsuleDetector(true);
		detectors[ca][bo] = gjkEpaDetector;
		detectors[ca][cy] = gjkEpaDetector;
		detectors[ca][co] = gjkEpaDetector;
		detectors[ca][ca] = new CapsuleCapsuleDetector();
		detectors[ca][ch] = gjkEpaDetector;

		detectors[ch][sp] = gjkEpaDetector;
		detectors[ch][bo] = gjkEpaDetector;
		detectors[ch][cy] = gjkEpaDetector;
		detectors[ch][co] = gjkEpaDetector;
		detectors[ch][ca] = gjkEpaDetector;
		detectors[ch][ch] = gjkEpaDetector;
	}

	// --- public ---

	/**
	 * Returns an appropriate collision detector of two geometries of types `geomType1` and `geomType2`.
	 *
	 * This method is **not symmetric**, so `getDetector(a, b)` may not be equal to `getDetector(b, a)`.
	 */
	public Detector getDetector(int geomType1, int geomType2) {
		return detectors[geomType1][geomType2];
	}
}