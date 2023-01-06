package oimo.collision.broadphase.bvh;

import oimo.common.M;
import oimo.dynamics.rigidbody.Shape;

/**
 * Internal class.
 *
 * BVH Tree
 */
public class BvhTree {
	public BvhNode _root;
	public int _numLeaves;
	public BvhStrategy _strategy;

	BvhNode _nodePool;

	BvhNode leafList;
	BvhNode leafListLast;

	BvhNode[] tmp;

	public BvhTree() {
		_root = null;
		_numLeaves = 0;
		_strategy = new BvhStrategy();
		_nodePool = null;
		leafList = null;
		leafListLast = null;
		tmp =new BvhNode[1024];
	}

	// --- internal ---

	public void _print(BvhNode root, String indent) {
		if (root == null) return;
		if(indent == null) {
			indent = "";
		}
		if (root._height == 0) {
			System.out.println(indent + root._proxy._id);
		} else {
			_print(root._children[0], indent + "  ");
			double tmp =0;
			double sizeX = root._aabbMax.x - root._aabbMin.x ;
			double sizeY = root._aabbMax.y - root._aabbMin.y;
			double sizeZ = root._aabbMax.z - root._aabbMin.z;
			double y = sizeY;
			double z = sizeZ;
			if(sizeX * (y + z) + y * z > 0) {
				sizeX = root._aabbMax.x - root._aabbMin.x ;
				sizeY = root._aabbMax.y - root._aabbMin.y;
				sizeZ = root._aabbMax.z - root._aabbMin.z;
				 y = sizeY;
				 z = sizeZ;
				tmp = ((int)((sizeX * (y + z) + y * z) * 1000 + 0.5f) | 0) / 1000;
			} else {
				 sizeX = root._aabbMax.x  - root._aabbMin.x ;
				 sizeY = root._aabbMax.y - root._aabbMin.y;
				 sizeZ = root._aabbMax.z - root._aabbMin.z;
				 y = sizeY;
				 z = sizeZ;
				tmp = ((int)((sizeX * (y + z) + y * z) * 1000 - 0.5f) | 0) / 1000;
			}
			
			System.out.println(indent + "#" + root._height + ", " + tmp);
			_print(root._children[1], indent + "  ");
		}
	}

	/**
	 * Inserts the proxy.
	 * This creates a leaf connected to the proxy and inserts it to the tree and `leafList`.
	 */
	 public void _insertProxy(BvhProxy proxy) {
		 BvhNode leaf = pick();
		leaf._proxy = proxy;
		proxy._leaf = leaf;

		M.vec3_assign(leaf._aabbMin, proxy._aabbMin);
		M.vec3_assign(leaf._aabbMax, proxy._aabbMax);

		_numLeaves++;
		
		if(leafList == null) {
			leafList = leaf;
			leafListLast = leaf;
		} else {
			leafListLast._nextLeaf = leaf;
			leaf._prevLeaf = leafListLast;
			leafListLast = leaf;
		}
		//M.list_push(leafList, leafListLast, _prevLeaf, _nextLeaf, leaf);

		
		insertLeaf(leaf);
	}

	/**
	 * Deletes the proxy.
	 * This also deletes the leaf connected to the proxy from the tree and `leafList`.
	 */
	public void _deleteProxy(BvhProxy proxy) {
		BvhNode leaf = proxy._leaf;

		_numLeaves--;
		BvhNode prev1 = leaf._prevLeaf;
		BvhNode next1 = leaf._nextLeaf;
		if(prev1 != null) {
			prev1._nextLeaf = next1;
		}
		if(next1 != null) {
			next1._prevLeaf = prev1;
		}
		if(leaf == this.leafList) {
			this.leafList = this.leafList._nextLeaf;
		}
		if(leaf == this.leafListLast) {
			this.leafListLast = this.leafListLast._prevLeaf;
		}
		leaf._nextLeaf = null;
		leaf._prevLeaf = null;
		//M.list_remove(leafList, leafListLast, _prevLeaf, _nextLeaf, leaf);

		deleteLeaf(leaf);

		proxy._leaf = null;
		pool(leaf);
	}

	/**
	 * Clears whole the tree.
	 * All leaves are disposed and deleted from `leafList`.
	 */
	 public void _clear() {
		if (_root == null) return;
		deleteRecursive(_root);
		_root = null;
		_numLeaves = 0;
	}

	 public void _optimize(int count) {
		if (_root == null) return;
		for (int i=0;i<count;i++) {
			BvhNode leaf = _root;
			while (leaf._height > 0) {
				int h1 = leaf._children[0]._height;
				int h2 = leaf._children[1]._height;
				// TODO: better strategy
				leaf = leaf._children[Math.random() > (h1 / (h1 + h2)) ? 1 : 0];
			}
			deleteLeaf(leaf);
			insertLeaf(leaf);
		}
	}

	 protected void _buildTopDown() {
		if (_root == null) return;
		decompose();

		while (tmp.length < _numLeaves) {
			int newLength=tmp.length<<1;
			BvhNode[] newArray =new BvhNode[newLength];
			for(int i=0;i<tmp.length;i++) {
				newArray[i]=tmp[i];
				tmp[i]=null;
			}
			tmp=newArray;
			//M.array_expand(tmp, tmp.length);
		}

		// collect leaves
		int idx = 0;
		BvhNode leaf = leafList;
		
		while(leaf != null) {
			tmp[idx] = leaf;
			idx++;
			leaf = leaf._nextLeaf;;
		}
		
		//		M.list_foreach(leaf, _nextLeaf, {
		//			tmp[idx] = leaf;
		//			idx++;
		//		});
		_root = buildTopDownRecursive(tmp, 0, _numLeaves);
	}
	

	public int _getBalance() {
		return getBalanceRecursive(this._root);
	}

	// --- private ---

	/**
	 * Makes the tree empty, but leaf nodes are not disposed and are reusable.
	 * The tree must be reconstructed using `leafList` after the call of this method.
	 */
	public void decompose() {
		if (_root == null) return;
		decomposeRecursive(_root);
		_root = null;
	}

	public void deleteRecursive(BvhNode root) {
		if(root._height == 0) {
			BvhNode prev = root._prevLeaf;
			BvhNode next = root._nextLeaf;
			if(prev != null) {
				prev._nextLeaf = next;
			}
			if(next != null) {
				next._prevLeaf = prev;
			}
			if(root == this.leafList) {
				this.leafList = this.leafList._nextLeaf;
			}
			if(root == this.leafListLast) {
				this.leafListLast = this.leafListLast._prevLeaf;
			}
			root._nextLeaf = null;
			root._prevLeaf = null;
			root._proxy._leaf = null;
			root._next = null;
			root._childIndex = 0;
			root._children[0] = null;
			root._children[1] = null;
			root._childIndex = 0;
			root._parent = null;
			root._height = 0;
			root._proxy = null;
			root._next = this._nodePool;
			this._nodePool = root;
			return;
		}
		this.deleteRecursive(root._children[0]);
		this.deleteRecursive(root._children[1]);
		root._next = null;
		root._childIndex = 0;
		root._children[0] = null;
		root._children[1] = null;
		root._childIndex = 0;
		root._parent = null;
		root._height = 0;
		root._proxy = null;
		root._next = this._nodePool;
		this._nodePool = root;
	}

	public void decomposeRecursive(BvhNode root) {
		if(root._height == 0) {
			root._childIndex = 0;
			root._parent = null;
			return;
		}
		this.decomposeRecursive(root._children[0]);
		this.decomposeRecursive(root._children[1]);
		root._next = null;
		root._childIndex = 0;
		root._children[0] = null;
		root._children[1] = null;
		root._childIndex = 0;
		root._parent = null;
		root._height = 0;
		root._proxy = null;
		root._next = this._nodePool;
		this._nodePool = root;
	}

	public BvhNode buildTopDownRecursive(BvhNode[] leaves, int from,int until) {
		if(until - from == 1) {
			BvhNode leaf = leaves[from];
			BvhProxy proxy = leaf._proxy;
			leaf._aabbMin.x = proxy._aabbMin.x;
			leaf._aabbMin.y = proxy._aabbMin.y;
			leaf._aabbMin.z = proxy._aabbMin.z;
			leaf._aabbMax.x = proxy._aabbMax.x;
			leaf._aabbMax.y = proxy._aabbMax.y;
			leaf._aabbMax.z = proxy._aabbMax.z;
			return leaf;
		}
		int splitAt = this._strategy._splitLeaves(leaves,from,until);
		BvhNode child1 = this.buildTopDownRecursive(leaves,from,splitAt);
		BvhNode child2 = this.buildTopDownRecursive(leaves,splitAt,until);
		BvhNode first = this._nodePool;
		if(first != null) {
			this._nodePool = first._next;
			first._next = null;
		} else {
			first = new oimo.collision.broadphase.bvh.BvhNode();
		}
		BvhNode parent = first;
		parent._children[0] = child1;
		child1._parent = parent;
		child1._childIndex = 0;
		parent._children[1] = child2;
		child2._parent = parent;
		child2._childIndex = 1;
		BvhNode c1 = parent._children[0];
		BvhNode c2 = parent._children[1];
		parent._aabbMin.x = c1._aabbMin.x < c2._aabbMin.x ? c1._aabbMin.x : c2._aabbMin.x;
		parent._aabbMin.y = c1._aabbMin.y < c2._aabbMin.y ? c1._aabbMin.y : c2._aabbMin.y;
		parent._aabbMin.z = c1._aabbMin.z < c2._aabbMin.z ? c1._aabbMin.z : c2._aabbMin.z;
		parent._aabbMax.x = c1._aabbMax.x > c2._aabbMax.x ? c1._aabbMax.x : c2._aabbMax.x;
		parent._aabbMax.y = c1._aabbMax.y > c2._aabbMax.y ? c1._aabbMax.y : c2._aabbMax.y;
		parent._aabbMax.z = c1._aabbMax.z > c2._aabbMax.z ? c1._aabbMax.z : c2._aabbMax.z;
		int h1 = parent._children[0]._height;
		int h2 = parent._children[1]._height;
		parent._height = (h1 > h2 ? h1 : h2) + 1;
		return parent;
		
//		var num:Int = until - from;
//		M.assert(num > 0);
//		if (num == 1) {
//			var leaf:BvhNode = leaves[from];
//			var proxy:BvhProxy = leaf._proxy;
//			M.vec3_assign(leaf._aabbMin, proxy._aabbMin);
//			M.vec3_assign(leaf._aabbMax, proxy._aabbMax);
//			return leaf;
//		}
//		// sort and split
//		var splitAt:Int = _strategy._splitLeaves(leaves, from, until);
//		var child1:BvhNode = buildTopDownRecursive(leaves, from, splitAt);
//		var child2:BvhNode = buildTopDownRecursive(leaves, splitAt, until);
//		var parent:BvhNode = pick();
//		parent._setChild(0, child1);
//		parent._setChild(1, child2);
//		parent._computeAabb();
//		parent._computeHeight();
//		return parent;
	}

	public int getBalanceRecursive(BvhNode root) {
		if(root == null || root._height == 0) {
			return 0;
		}
		int balance = root._children[0]._height - root._children[1]._height;
		if(balance < 0) {
			balance = -balance;
		}
		return balance + this.getBalanceRecursive(root._children[0]) + this.getBalanceRecursive(root._children[1]);

//		if (root == null || root._height == 0) return 0;
//		var balance:Int = root._children[0]._height - root._children[1]._height;
//		if (balance < 0) balance = -balance;
//		return balance + getBalanceRecursive(root._children[0]) + getBalanceRecursive(root._children[1]);
	}

	void  insertLeaf(BvhNode leaf) {
		//assertBeLeaf(leaf);
		if (_root == null) { // the tree is empty
			_root = leaf;
			return;
		}
		// search for the position to insert
		BvhNode sibling = _root;

		while (sibling._height > 0) {
			int nextStep = _strategy._decideInsertion(sibling, leaf);

			if (nextStep == -1) {
				// insert to current position
				break;
			} else {
				sibling = sibling._children[nextStep];
			}
		}

		BvhNode parent = sibling._parent;

		// new common parent with the sibling
		BvhNode node = pick();

		if (parent == null) {
			// replace the root node
			_root = node;
		} else {
			// connect to the old parent
			parent._setChild(sibling._childIndex, node);
		}
		node._setChild(sibling._childIndex, sibling);
		node._setChild(sibling._childIndex ^ 1, leaf);

		// fix data
		while (node != null) {
			if (_strategy._balancingEnabled) {
				node = balance(node);
			}
			node._computeHeight();
			node._computeAabb();
			node = node._parent;
		}
	}

	void deleteLeaf(BvhNode leaf) {
		//assertBeLeaf(leaf);
		if (_root == leaf) { // the tree has only the leaf
			_root = null;
			return;
		}
		BvhNode parent = leaf._parent;
		BvhNode sibling = parent._children[leaf._childIndex ^ 1];
		BvhNode grandParent = parent._parent;
		if (grandParent == null) {
			sibling._parent = null;
			sibling._childIndex = 0;
			_root = sibling;
			pool(parent);
			return;
		}
		sibling._parent = grandParent;
		grandParent._setChild(parent._childIndex, sibling);
		pool(parent);

		// fix data
		BvhNode node = grandParent;
		while (node != null) {
			if (_strategy._balancingEnabled) {
				node = balance(node);
			}
			node._computeHeight();
			node._computeAabb();
			node = node._parent;
		}
	}

	/**
	 * Balances and returns the node at the same position of `node`.
	 */
	BvhNode balance(BvhNode node) {
		int nh = node._height;
		if (nh < 2) {
			return node;
		}
		BvhNode p = node._parent;
		BvhNode l = node._children[0];
		BvhNode r = node._children[1];
		int lh = l._height;
		int rh = r._height;
		int balance = lh - rh;
		int nodeIndex = node._childIndex;

		//          [ N ]
		//         /     \
		//    [ L ]       [ R ]
		//     / \         / \
		// [L-L] [L-R] [R-L] [R-R]

		// is the tree balanced?
		if (balance > 1) {
			BvhNode ll = l._children[0];
			BvhNode lr = l._children[1];
			int llh = ll._height;
			int lrh = lr._height;

			// is L-L higher than L-R?
			if (llh > lrh) {
				// set N to L-R
				l._setChild(1, node);

				//          [ L ]
				//         /     \
				//    [L-L]       [ N ]
				//     / \         / \
				// [...] [...] [ L ] [ R ]

				// set L-R
				node._setChild(0, lr);

				//          [ L ]
				//         /     \
				//    [L-L]       [ N ]
				//     / \         / \
				// [...] [...] [L-R] [ R ]

				// fix bounds and heights
				l._computeAabb();
				l._computeHeight();
				node._computeAabb();
				node._computeHeight();
			} else {
				// set N to L-L
				l._setChild(0, node);

				//          [ L ]
				//         /     \
				//    [ N ]       [L-R]
				//     / \         / \
				// [ L ] [ R ] [...] [...]

				// set L-L
				node._setChild(0, ll);

				//          [ L ]
				//         /     \
				//    [ N ]       [L-R]
				//     / \         / \
				// [L-L] [ R ] [...] [...]

				// fix bounds and heights
				l._computeAabb();
				l._computeHeight();
				node._computeAabb();
				node._computeHeight();
			}
			// set new parent of L
			if (p != null) {
				p._setChild(nodeIndex, l);
			} else {
				_root = l;
				l._parent = null;
			}
			return l;
		}
		if (balance < -1) {
			BvhNode rl = r._children[0];
			BvhNode rr = r._children[1];
			int rlh = rl._height;
			int rrh = rr._height;

			// is R-L higher than R-R?
			if (rlh > rrh) {
				// set N to R-R
				r._setChild(1, node);

				//          [ R ]
				//         /     \
				//    [R-L]       [ N ]
				//     / \         / \
				// [...] [...] [ L ] [ R ]

				// set R-R
				node._setChild(1, rr);

				//          [ R ]
				//         /     \
				//    [R-L]       [ N ]
				//     / \         / \
				// [...] [...] [ L ] [R-R]

				// fix bounds and heights
				r._computeAabb();
				r._computeHeight();
				node._computeAabb();
				node._computeHeight();
			} else {
				// set N to R-L
				r._setChild(0, node);

				//          [ R ]
				//         /     \
				//    [ N ]       [R-R]
				//     / \         / \
				// [ L ] [ R ] [...] [...]

				// set R-L
				node._setChild(1, rl);

				//          [ R ]
				//         /     \
				//    [ N ]       [R-R]
				//     / \         / \
				// [ L ] [R-L] [...] [...]

				// fix bounds and heights
				r._computeAabb();
				r._computeHeight();
				node._computeAabb();
				node._computeHeight();
			}
			// set new parent of R
			if (p != null) {
				p._setChild(nodeIndex, r);
			} else {
				_root = r;
				r._parent = null;
			}
			return r;
		}
		return node;
	}

//	extern inline function assertBeLeaf(leaf:BvhNode):Void {
//		M.assert(leaf._proxy != null && leaf._proxy._leaf == leaf && leaf._children[0] == null && leaf._children[1] == null && leaf._height == 0);
//	}

	private void pool(BvhNode node) {
		//M.assert(node._proxy == null || node._proxy._leaf == null);
		node._removeReferences();
		//node._removeReferences();
		node._next = this._nodePool;
		this._nodePool = node;
		//M.singleList_pool(_nodePool, _next, node);
	}

	private BvhNode  pick() {
		BvhNode first = _nodePool;
		if(first != null) {
			_nodePool = first._next;
			first._next = null;
		} else {
			first = new BvhNode();
		}
		return first;
		
		//return M.singleList_pick(_nodePool, _next, new BvhNode());
	}

}