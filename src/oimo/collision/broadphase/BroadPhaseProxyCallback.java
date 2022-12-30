package oimo.collision.broadphase;

/**
 * A callback class for queries in a broad phase.
 */
public abstract class BroadPhaseProxyCallback {
	/**
	 * Default constructor.
	 */
	public  BroadPhaseProxyCallback() {
	}

	/**
	 * This is called every time a broad phase algorithm reports a proxy `proxy`.
	 */
	public abstract void process(Proxy proxy) ;
}