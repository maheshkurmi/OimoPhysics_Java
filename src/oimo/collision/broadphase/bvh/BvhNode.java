package oimo.collision.broadphase.bvh;

import oimo.common.M;
import oimo.common.Vec3;

/**
 * Internal class.
 *
 * BVH Node
 */
public class BvhNode {
	// for object pool
	public BvhNode _next;

	// for BvhTree.leafList
	public BvhNode _prevLeaf;
	public BvhNode _nextLeaf;

	public BvhNode[] _children;
	public int _childIndex; // must be 0 or 1 regardless of having parent
	public BvhNode _parent;
	public int _height;
	public BvhProxy _proxy;

	// node's aabb. if the node is a leaf, the aabb is equal to the proxy's one.
	public Vec3 _aabbMin;
	public Vec3 _aabbMax;

	// used by other classes
	public Vec3 _tmp=new Vec3();

	public BvhNode() {
		_next = null;
		_prevLeaf = null;
		_nextLeaf = null;
		_children =new BvhNode[2];
		_childIndex = 0;
		_parent = null;
		_height = 0;
		_proxy = null;
		_aabbMin=new Vec3();
		_aabbMax=new Vec3();
	}

	// --- internal ---

	public void _setChild(int index, BvhNode child) {
		_children[index] = child;
		child._parent = this;
		child._childIndex = index;
	}

	 public void _removeReferences() {
		_next = null;
		_childIndex = 0;
		_children[0] = null;
		_children[1] = null;
		_childIndex = 0;
		_parent = null;
		_height = 0;
		_proxy = null;
	}

	 public void _computeAabb() {
		BvhNode c1 = _children[0];
		BvhNode c2 = _children[1];
		M.vec3_min(_aabbMin, c1._aabbMin, c2._aabbMin);
		M.vec3_max(_aabbMax, c1._aabbMax, c2._aabbMax);
	}

	 public void _computeHeight() {
		int h1 = _children[0]._height;
		int h2 = _children[1]._height;
		_height = (h1 > h2 ? h1 : h2) + 1;
	}

	 public float _perimeter() {
		float x = _aabbMax.x-_aabbMin.x;// M.vec3_get(size, 0);
		float y = _aabbMax.y-_aabbMin.y;//.vec3_get(size, 1);
		float z = _aabbMax.z-_aabbMin.z;// = M.vec3_get(size, 2);
		return x * (y + z) + y * z;
	}

}