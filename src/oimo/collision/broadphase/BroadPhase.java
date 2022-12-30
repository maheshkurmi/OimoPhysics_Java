package oimo.collision.broadphase;

import oimo.collision.geometry.Aabb;
import oimo.collision.geometry.ConvexGeometry;
import oimo.collision.geometry.RayCastHit;
import oimo.collision.narrowphase.detector.gjkepa.GjkEpa;
import oimo.collision.narrowphase.detector.gjkepa.GjkEpaResultState;
import oimo.common.M;
import oimo.common.MathUtil;
import oimo.common.Transform;
import oimo.common.Vec3;

/**
 * The abstract class of a broad-phase collision detection algorithm.
 */
public abstract class BroadPhase {
	public int _type;
	public int _numProxies;
	public Proxy _proxyList;
	public Proxy _proxyListLast;

	public ProxyPair _proxyPairList;
	public boolean _incremental;

	public int _testCount;

	protected ProxyPair _proxyPairPool;
	protected int _idCount;

	protected ConvexSweepGeometry _convexSweep;
	protected AabbGeometry _aabb;

	protected Transform identity;
	protected Vec3 zero;
	protected RayCastHit rayCastHit;

	public BroadPhase(int type) {
		_type = type;

		_numProxies = 0;
		_proxyList = null;
		_proxyListLast = null;

		_proxyPairList = null;
		_incremental = false;

		_testCount = 0;

		_proxyPairPool = null;
		_idCount = 0;

		_convexSweep = new ConvexSweepGeometry();
		_aabb = new AabbGeometry();

		identity = new Transform();
		zero = new Vec3();
		rayCastHit = new RayCastHit();
	}

	// --- public ---

	/**
	 * Returns a new proxy connected with the user data `userData` containing the
	 * axis-aligned bounding box `aabb`, and adds the proxy into the collision
	 * space.
	 */
	public abstract Proxy createProxy(Object userData, Aabb aabb);

	/**
	 * Removes the proxy `proxy` from the collision space.
	 */
	public abstract void destroyProxy(Proxy proxy);

	
	/**
	 * Returns whether the pair of `proxy1` and `proxy2` is overlapping. As proxies
	 * can be larger than the containing AABBs, two proxies may overlap even though
	 * their inner AABBs are separate.
	 */
	public boolean isOverlapping(Proxy proxy1, Proxy proxy2) {
		return M.aabb_overlap(proxy1._aabbMin, proxy1._aabbMax, proxy2._aabbMin, proxy2._aabbMax);
	}

	/**
	 * Collects overlapping pairs of the proxies and put them into a linked list.
	 * The linked list can be get through `BroadPhase.getProxyPairList` method.
	 *
	 * Note that in order to collect pairs, the broad-phase algorithm requires to be
	 * informed of movements of proxies through `BroadPhase.moveProxy` method.
	 */
	public abstract void collectPairs();

	/**
	 * Returns the linked list of collected pairs of proxies.
	 */
	public ProxyPair getProxyPairList() {
		return _proxyPairList;
	}

	/**
	 * Returns whether to collect only pairs created in the last step. If this
	 * returns true, the pairs that are not collected might still be overlapping.
	 * Otherwise, such pairs are guaranteed to be separated.
	 */
	public boolean isIncremental() {
		return _incremental;
	}

	/**
	 * Returns the number of broad-phase AABB tests.
	 */
	public int getTestCount() {
		return _testCount;
	}

	/**
	 * Performs a ray casting. `callback.process` is called for all proxies the line
	 * segment from `begin` to `end` intersects.
	 */
	public abstract void rayCast(Vec3 begin, Vec3 end, BroadPhaseProxyCallback callback);

	/**
	 * Performs a convex casting. `callback.process` is called for all shapes the
	 * convex geometry `convex` hits. The convex geometry translates by
	 * `translation` starting from the beginning transform `begin`.
	 */
	public abstract void convexCast(ConvexGeometry convex, Transform begin, Vec3 translation,
			BroadPhaseProxyCallback callback);

	/**
	 * Performs an AABB query. `callback.process` is called for all proxies that
	 * their AABB and `aabb` intersect.
	 */
	public abstract void aabbTest(Aabb aabb, BroadPhaseProxyCallback callback);

	// --- private ---

	public void pickAndPushProxyPair(Proxy p1, Proxy p2) {		
		ProxyPair first = this._proxyPairPool;
		if ( first != null ) {
			this._proxyPairPool = first._next;
			first._next = null;
		}else{
			first = new oimo.collision.broadphase.ProxyPair();
		}
		
		ProxyPair pp = first;
		if ( this._proxyPairList == null ){
			this._proxyPairList = pp;
		}else{
			pp._next = this._proxyPairList;
			this._proxyPairList = pp;
		}
		
		pp._p1 = p1;
		pp._p2 = p2;
	}

	public void poolProxyPairs() {
		ProxyPair p = _proxyPairList;
		if (p != null) {
			do {
				p._p1 = null;
				p._p2 = null;
				p = p._next;
			} while (p != null);
			_proxyPairList._next = _proxyPairPool;
			_proxyPairPool = _proxyPairList;
			_proxyPairList = null;
		}
	}

	public void addProxy(Proxy proxy) {
		_numProxies++;
		if ((this._proxyList == null)) {
			this._proxyList = proxy;
			this._proxyListLast = proxy;
		} else {
			this._proxyListLast._next = proxy;
			proxy._prev = this._proxyListLast;
			this._proxyListLast = proxy;
		}

	}

	public void removeProxy(Proxy proxy) {
		_numProxies--;
		Proxy prev = proxy._prev;
		Proxy next = proxy._next;
		if (prev != null)
			prev._next = next;
		if (next != null)
			next._prev = prev;

		if (proxy == this._proxyList)
			this._proxyList = this._proxyList._next;

		if (proxy == this._proxyListLast)
			this._proxyListLast = this._proxyListLast._prev;

		proxy._next = null;
		proxy._prev = null;
		proxy.userData = null;
	}

	/**
	 * Moves the proxy `proxy` to the axis-aligned bounding box `aabb`.
	 * `displacement` is the difference between current and previous center of the
	 * AABB. This is used for predicting movement of the proxy.
	 */
	public  void moveProxy(Proxy proxy, Aabb aabb, Vec3 displacement) {
		proxy._setAabb(aabb);
	};

	
	public boolean aabbSegmentTest(Vec3 aabbMin, Vec3 aabbMax, Vec3 begin, Vec3 end) {
		float x1 = begin.x;// .vec3_get(begin, 0);
		float y1 = begin.y;
		float z1 = begin.z;
		float x2 = end.x;
		float y2 = end.y;
		float z2 = end.z;
		float sminx = MathUtil.min(x1, x2);
		float sminy = MathUtil.min(y1, y2);
		float sminz = MathUtil.min(z1, z2);
		float smaxx = MathUtil.max(x1, x2);
		float smaxy = MathUtil.max(y1, y2);
		float smaxz = MathUtil.max(z1, z2);
		float pminx = aabbMin.x;// .vec3_get(aabbMin, 0);
		float pminy = aabbMin.y;// M.vec3_get(aabbMin, 1);
		float pminz = aabbMin.z;// M.vec3_get(aabbMin, 2);
		float pmaxx = aabbMax.x;// .vec3_get(aabbMax, 0);
		float pmaxy = aabbMax.y;// .vec3_get(aabbMax, 1);
		float pmaxz = aabbMax.z;// .vec3_get(aabbMax, 2);

		if (
		// axis1: (1, 0, 0)
		// axis2: (0, 1, 0)
		// axis3: (0, 0, 1)
		pminx > smaxx || pmaxx < sminx || pminy > smaxy || pmaxy < sminy || pminz > smaxz || pmaxz < sminz) {
			return false;
		}

		float dx = x2 - x1;
		float dy = y2 - y1;
		float dz = z2 - z1;
		float adx = MathUtil.abs(dx);
		float ady = MathUtil.abs(dy);
		float adz = MathUtil.abs(dz);
		float pextx = (pmaxx - pminx) * 0.5f;
		float pexty = (pmaxy - pminy) * 0.5f;
		float pextz = (pmaxz - pminz) * 0.5f;
		float pcntx = (pmaxx + pminx) * 0.5f;
		float pcnty = (pmaxy + pminy) * 0.5f;
		float pcntz = (pmaxz + pminz) * 0.5f;
		float cpx = x1 - pcntx;
		float cpy = y1 - pcnty;
		float cpz = z1 - pcntz;

		if (
		// axis4: (dx, dy, dz) x (1, 0, 0) = (0, dz, -dy)
		// axis5: (dx, dy, dz) x (0, 1, 0) = (-dz, 0, dx)
		// axis6: (dx, dy, dz) x (0, 0, 1) = (dy, -dx, 0)
		MathUtil.abs(cpy * dz - cpz * dy) - (pexty * adz + pextz * ady) > 0
				|| MathUtil.abs(cpz * dx - cpx * dz) - (pextz * adx + pextx * adz) > 0
				|| MathUtil.abs(cpx * dy - cpy * dx) - (pextx * ady + pexty * adx) > 0) {
			return false;
		}

		return true;
	}

	public boolean aabbConvexSweepTest(Vec3 aabbMin, Vec3 aabbMax, ConvexGeometry convex, Transform begin,
			Vec3 translation) {
		M.vec3_toVec3(_aabb.min, aabbMin);
		M.vec3_toVec3(_aabb.max, aabbMax);
		_convexSweep.init(convex, begin, translation);
		GjkEpa gjkEpa = GjkEpa.getInstance();
		if (gjkEpa.computeDistance(_convexSweep, _aabb, begin, identity, null) == GjkEpaResultState._SUCCEEDED) {
			return gjkEpa.distance <= 0;
		}
		return false;
	}

	public class ConvexSweepGeometry extends ConvexGeometry {
		ConvexGeometry c;
		Vec3 localTranslation;

		public ConvexSweepGeometry() {
			super(-1);
		}

		public void init(ConvexGeometry c, Transform transform, Vec3 translation) {
			this.c = c;
			Vec3 tr = new Vec3().copyFrom(translation);

			Vec3 localTr = new Vec3();
			M.vec3_mulMat3Transposed(localTr, tr, transform._rotation);

			localTranslation = new Vec3();
			M.vec3_toVec3(localTranslation, localTr);

			_gjkMargin = c._gjkMargin;
		}

		@Override
		public void computeLocalSupportingVertex(Vec3 dir, Vec3 out) {
			c.computeLocalSupportingVertex(dir, out);
			if (dir.dot(localTranslation) > 0) {
				out.addEq(localTranslation);
			}
		}

		@Override
		public void _updateMass() {
			// TODO Auto-generated method stub

		}

		@Override
		public void _computeAabb(Aabb result, Transform tf) {
			// TODO Auto-generated method stub

		}

		@Override
		public boolean _rayCastLocal(Vec3 begin, Vec3 end, RayCastHit result) {
			// TODO Auto-generated method stub
			return false;
		}
	}

	public class AabbGeometry extends ConvexGeometry {
		public Vec3 min;
		public Vec3 max;

		public AabbGeometry() {
			super(-1);
			min = new Vec3();
			max = new Vec3();
		}

		@Override
		public void computeLocalSupportingVertex(Vec3 dir, Vec3 out) {
			out.x = dir.x > 0 ? max.x : min.x;
			out.y = dir.y > 0 ? max.y : min.y;
			out.z = dir.z > 0 ? max.z : min.z;
		}

		@Override
		public void _updateMass() {
			// TODO Auto-generated method stub

		}

		@Override
		public void _computeAabb(Aabb result, Transform tf) {
			// TODO Auto-generated method stub

		}

		@Override
		public boolean _rayCastLocal(Vec3 begin, Vec3 end, RayCastHit result) {
			// TODO Auto-generated method stub
			return false;
		}
	}

}
