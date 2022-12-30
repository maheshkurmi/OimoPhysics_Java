package oimo.collision.broadphase;
import oimo.collision.geometry.Aabb;
import oimo.common.Vec3;

/**
 * A proxy is an object that can be added to a broad-phase collision detection algorithm.
 * Users of the collision part of the library can move an axis-aligned bounding box of
 * a proxy through `BroadPhase` class.
 */
public class Proxy {
	public Proxy _prev;
	public Proxy _next;

	// fattened aabb
	public Vec3 _aabbMin;
	public Vec3 _aabbMax;

	public int _id;

	/**
	 * Extra field that users can use for their own purposes. **Do not modify this property if
	 * you use the physics part of the library**, as the physics part of the library uses this property
	 * for connecting proxies and shapes of rigid bodies.
	 */
	public Object userData;

	public Proxy(Object userData, int id) {
		this.userData = userData;
		_id = id;
		_prev = null;
		_next = null;
		_aabbMin.zero();
		_aabbMax.zero();
	}

	// --- internal ---

	 public void _setAabb(Aabb aabb) {
		_aabbMin.copyFrom(aabb._min);
		_aabbMax.copyFrom(aabb._max);
	}

	// --- public ---

	/**
	 * Returns the unique id of the proxy.
	 */
	public int getId() {
		return _id;
	}

	/**
	 * Returns the fat AABB of the proxy.
	 */
	public Aabb getFatAabb() {
		Aabb aabb = new Aabb();
		return aabb.set(_aabbMin, _aabbMax);
	}

	/**
	 * Sets `aabb` to the fat AABB of the proxy.
	 *
	 * This does not create a new instance of `Aabb`.
	 */
	public void getFatAabbTo(Aabb aabb) {
		aabb.set(_aabbMin, _aabbMax);
	}
}

