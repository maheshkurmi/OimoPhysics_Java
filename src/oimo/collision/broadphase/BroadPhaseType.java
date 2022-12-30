package oimo.collision.broadphase;

public class BroadPhaseType {
	/**
	 * The brute force algorithm searches all the possible pairs of the proxies every time.
	 * This is **very slow** and so users should not choose this algorithm without exceptional reasons.
	 */
	public static final int BRUTE_FORCE =1;

	/**
	 * The BVH algorithm uses bounding volume hierarchy for detecting overlapping pairs of proxies efficiently.
	 */
	public static final int BVH=2;
}