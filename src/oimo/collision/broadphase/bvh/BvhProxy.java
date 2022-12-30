package oimo.collision.broadphase.bvh;

import oimo.collision.broadphase.Proxy;

/**
 * Internal class.
 *
 * BVH Proxy
 */
public class BvhProxy extends Proxy {
	public BvhNode _leaf;
	public boolean _moved;

	public BvhProxy(Object userData, int id) {
		super(userData, id);
		_leaf = null;
		_moved = false;
	}

}