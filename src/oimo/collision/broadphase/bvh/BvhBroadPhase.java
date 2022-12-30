package oimo.collision.broadphase.bvh;

import oimo.collision.broadphase.BroadPhase;
import oimo.collision.broadphase.BroadPhaseProxyCallback;
import oimo.collision.broadphase.BroadPhaseType;
import oimo.collision.broadphase.Proxy;
import oimo.collision.geometry.Aabb;
import oimo.collision.geometry.ConvexGeometry;
import oimo.collision.narrowphase.detector.gjkepa.GjkEpa;
import oimo.common.M;
import oimo.common.Setting;
import oimo.common.Transform;
import oimo.common.Vec3;

/**
 * The broad-phase collision detection algorithm based on bounding volume hierarchy (BVH).
 * Average time complexity is O(NlogN) or lower.
 */
public class BvhBroadPhase extends BroadPhase {
	public BvhTree _tree;

	BvhProxy[] movedProxies;
	int numMovedProxies;

	public BvhBroadPhase() {
		super(BroadPhaseType.BVH);
		_incremental = true;
		_tree = new BvhTree();
		movedProxies = new BvhProxy[1024];
		numMovedProxies = 0;
	}

	// --- private ---

	private void addToMovedProxy(BvhProxy bvhProxy) {
		// add to the buffer
		if (bvhProxy._moved) return;
		bvhProxy._moved = true;

		// expand the buffer
		if (movedProxies.length == numMovedProxies) {
			BvhProxy[] newArray = new BvhProxy[this.numMovedProxies << 1];
			for(int i=0;i<this.numMovedProxies;i++) {
				newArray[i] = this.movedProxies[i];
				this.movedProxies[i] = null;
			}
			this.movedProxies = newArray;
			//M.array_expand(movedProxies, numMovedProxies);
		}

		movedProxies[numMovedProxies++] = bvhProxy;
	}

	private void updateProxy(BvhProxy p, Aabb aabb, Vec3 displacement) {
		// set tight AABB
		p._setAabb(aabb);

		// fatten the AABB
		float padding = Setting.bvhProxyPadding;
		p._aabbMin.sub3Eq(padding, padding, padding);
		p._aabbMax.add3Eq(padding, padding, padding);
		
	
		if (displacement != null) {
			// predict movement
			Vec3 d=displacement;
			//Vec3 zero:IVec3;
			Vec3 addToMin=new Vec3();
			Vec3 addToMax=new Vec3();
			M.vec3_zero(zero);
			M.vec3_fromVec3(d, displacement);
			M.vec3_min(addToMin, zero, d);
			M.vec3_max(addToMax, zero, d);
			M.vec3_add(p._aabbMin, p._aabbMin, addToMin);
			M.vec3_add(p._aabbMax, p._aabbMax, addToMax);
		}
	}

	void collide(BvhNode n1, BvhNode n2) {
		_testCount++;
		boolean l1 = n1._height == 0;
		boolean l2 = n2._height == 0;
		if (n1 == n2) {
			if (l1) return;
			collide(n1._children[0], n2);
			collide(n1._children[1], n2);
			return;
		}
		if (!M.aabb_overlap(n1._aabbMin, n1._aabbMax, n2._aabbMin, n2._aabbMax)) {
			return;
		}
		if (l1 && l2) {
			pickAndPushProxyPair(n1._proxy, n2._proxy);
			return;
		}
		if (l2 || n1._height > n2._height) {
			// descend node 1
			collide(n1._children[0], n2);
			collide(n1._children[1], n2);
		} else {
			// descend node 2
			collide(n2._children[0], n1);
			collide(n2._children[1], n1);
		}
	}

	void rayCastRecursive(BvhNode node, Vec3 _p1,Vec3 _p2, BroadPhaseProxyCallback callback) {
		// TODO: use stack?
		float x1 = _p1.x;
		float y1 = _p1.y;
		float z1 = _p1.z;
		float x2 = _p2.x;
		float y2 = _p2.y;
		float z2 = _p2.z;
		float pminx = node._aabbMin.x;
		float pminy = node._aabbMin.y;
		float pminz = node._aabbMin.z;
		float pmaxx = node._aabbMax.x;
		float pmaxy = node._aabbMax.y;
		float pmaxz = node._aabbMax.z;
		boolean tmp;
		if(pminx > (x1 > x2 ? x1 : x2) || pmaxx < (x1 < x2 ? x1 : x2) || pminy > (y1 > y2 ? y1 : y2) || pmaxy < (y1 < y2 ? y1 : y2) || pminz > (z1 > z2 ? z1 : z2) || pmaxz < (z1 < z2 ? z1 : z2)) {
			tmp = false;
		} else {
			float dx = x2 - x1;
			float dy = y2 - y1;
			float dz = z2 - z1;
			float adx = dx < 0 ? -dx : dx;
			float ady = dy < 0 ? -dy : dy;
			float adz = dz < 0 ? -dz : dz;
			float pextx = (pmaxx - pminx) * 0.5f;
			float pexty = (pmaxy - pminy) * 0.5f;
			float pextz = (pmaxz - pminz) * 0.5f;
			float cpx = x1 - (pmaxx + pminx) * 0.5f;
			float cpy = y1 - (pmaxy + pminy) * 0.5f;
			float cpz = z1 - (pmaxz + pminz) * 0.5f;
			boolean tmp1;
			boolean tmp2;
			float x = cpy * dz - cpz * dy;
			if(!((x < 0 ? -x : x) - (pexty * adz + pextz * ady) > 0)) {
				x = cpz * dx - cpx * dz;
				tmp2 = (x < 0 ? -x : x) - (pextz * adx + pextx * adz) > 0;
			} else {
				tmp2 = true;
			}
			if(!tmp2) {
				x = cpx * dy - cpy * dx;
				tmp1 = (x < 0 ? -x : x) - (pextx * ady + pexty * adx) > 0;
			} else {
				tmp1 = true;
			}
			tmp = tmp1 ? false : true;
		}
		if(!tmp) {
			return;
		}
		if(node._height == 0) {
			callback.process(node._proxy);
			return;
		}
		this.rayCastRecursive(node._children[0],_p1,_p2,callback);
		this.rayCastRecursive(node._children[1],_p1,_p2,callback);
	}

	void convexCastRecursive(BvhNode node, ConvexGeometry convex,Transform begin,Vec3 translation, BroadPhaseProxyCallback callback) {
		// TODO: use stack?
		Vec3 v = this._aabb.min;
		Vec3 v1 = this._aabb.max;
		this._convexSweep.init(convex,begin,translation);
		GjkEpa gjkEpa = oimo.collision.narrowphase.detector.gjkepa.GjkEpa.instance;
		if(!(gjkEpa.computeClosestPointsImpl(this._convexSweep,this._aabb,begin,this.identity,null,false) == 0 && gjkEpa.distance <= 0)) {
			return;
		}
		if(node._height == 0) {
			callback.process(node._proxy);
			return;
		}
		this.convexCastRecursive(node._children[0],convex,begin,translation,callback);
		this.convexCastRecursive(node._children[1],convex,begin,translation,callback);
	}

	void aabbTestRecursive(BvhNode node, Aabb aabb,BroadPhaseProxyCallback callback) {
		if (!M.aabb_overlap(node._aabbMin, node._aabbMax, aabb._min, aabb._max)) {
			return;
		}

		if (node._height == 0) { // leaf
			callback.process(node._proxy);
			return;
		}

		aabbTestRecursive(node._children[0], aabb, callback);
		aabbTestRecursive(node._children[1], aabb, callback);
	}

	// --- public ---

	@Override 
	public Proxy createProxy(Object userData,Aabb aabb) {
		BvhProxy p = new BvhProxy(userData, _idCount++);
		addProxy(p);

		updateProxy(p, aabb, null);
		_tree._insertProxy(p);
		addToMovedProxy(p);

		return p;
	}

	@Override 
	public void destroyProxy(Proxy proxy) {
		removeProxy(proxy);

		BvhProxy bvhProxy =  (BvhProxy) proxy;
		_tree._deleteProxy(bvhProxy);
		bvhProxy.userData = null;
		bvhProxy._next = null;
		bvhProxy._prev = null;

		if (bvhProxy._moved) {
			bvhProxy._moved = false;
		}
	}

	@Override 
	public void moveProxy(Proxy proxy,Aabb aabb,Vec3 displacement) {
		BvhProxy p =  (BvhProxy) proxy;
		
		if (M.aabb_contains(p._aabbMin, p._aabbMax, aabb._min, aabb._max)) {
			// need not move proxy
			return;
		}

		updateProxy(p, aabb, displacement);
		addToMovedProxy(p);
	}

	@Override 
	public void collectPairs() {
		poolProxyPairs();
		_testCount = 0;
		if (_numProxies < 2) return;

//		boolean topDown = false;
//
//		if (topDown) {
//			while (numMovedProxies > 0) {
//				movedProxies[--numMovedProxies] = null;
//			}
//			_tree._buildTopDown();
//			collide(_tree._root, _tree._root);
//			return;
//		}

		boolean incrementalCollision = numMovedProxies / _numProxies < Setting.bvhIncrementalCollisionThreshold;

		// incremental modification
		for (int i=0;i<numMovedProxies;i++) {
			BvhProxy p = movedProxies[i];
			if (p._moved) {
				_tree._deleteProxy(p);
				_tree._insertProxy(p);
				if (incrementalCollision) {
					collide(_tree._root, p._leaf);
				}
				p._moved = false;
			}
			movedProxies[i] = null;
		}
		if (!incrementalCollision) {
			collide(_tree._root, _tree._root);
		}

		numMovedProxies = 0;
	}

	@Override 
	public void rayCast(Vec3 begin, Vec3 end, BroadPhaseProxyCallback callback) {
		if (_tree._root == null) return; // no AABBs in the broadphase
//		var p1:IVec3;
//		var p2:IVec3;
//		M.vec3_fromVec3(p1, begin);
//		M.vec3_fromVec3(p2, end);
		this.rayCastRecursive(_tree._root, begin, end, callback);
	}

	@Override 
	public void convexCast(ConvexGeometry convex,Transform begin, Vec3 translation,BroadPhaseProxyCallback callback) {
		if (_tree._root == null) return; // no AABBs in the broadphase

		convexCastRecursive(_tree._root, convex, begin, translation, callback);
	}

	
	@Override 
	public void aabbTest(Aabb aabb, BroadPhaseProxyCallback callback) {
		if (_tree._root == null) return; // no AABBs in the broadphase

		aabbTestRecursive(_tree._root, aabb, callback);
	}

	/**
	 * Returns the balance of the bounding volume tree.
	 */
	public int getTreeBalance() {
		return _tree._getBalance();
	}
}