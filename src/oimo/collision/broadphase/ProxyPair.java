package oimo.collision.broadphase;
/**
 * A pair between two proxies. Broad-phase collision algorithms collect pairs of proxies
 * as linked list of ProxyPair.
 */
public class ProxyPair {
	public ProxyPair _next;

	public Proxy _p1;
	public Proxy _p2;

	public ProxyPair() {
		_p1 = null;
		_p2 = null;
	}

	// --- public ---

	/**
	 * Returns the first proxy of the pair.
	 */
	public Proxy getProxy1() {
		return _p1;
	}

	/**
	 * Returns the second proxy of the pair.
	 */
	public Proxy getProxy2() {
		return _p2;
	}

	/**
	 * Returns the next pair.
	 */
	public ProxyPair getNext() {
		return _next;
	}

}
