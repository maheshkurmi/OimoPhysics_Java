package oimo.collision.broadphase.bruteforce;
import oimo.collision.broadphase.*;
import oimo.collision.geometry.Aabb;
import oimo.collision.geometry.ConvexGeometry;
import oimo.common.M;
import oimo.common.Transform;
import oimo.common.Vec3;

/**
 * Brute force implementation of broad-phase collision detection. Time complexity is O(n^2).
 */
public class BruteForceBroadPhase extends BroadPhase {

	public BruteForceBroadPhase() {
		super(BroadPhaseType.BRUTE_FORCE);
		_incremental = false;
	}


	// --- public ---
	@Override 
	public Proxy createProxy(Object userData, Aabb aabb) {
		Proxy proxy = new Proxy(userData, _idCount++);
		addProxy(proxy);
		proxy._setAabb(aabb);
		return proxy;
	}

	@Override 
	public void destroyProxy(Proxy proxy) {
		removeProxy(proxy);
		proxy.userData = null;
	}

	@Override 
	public void moveProxy(Proxy proxy, Aabb aabb, Vec3 dislacement) {
		proxy._setAabb(aabb);
	}

	@Override 
	public void collectPairs() {
		poolProxyPairs();
		_testCount = 0;
//		Proxy p1 = _proxyList;
//		M.list_foreach(p1, _next, {
//			var p2:Proxy = p1._next;
//			M.list_foreach(p2, _next, {
//				_testCount++;
//				if (overlap(p1, p2)) {
//					pickAndPushProxyPair(p1, p2);
//				}
//			});
//		});
//		
		Proxy p1 = this._proxyList;
		while ( p1 != null ){
			Proxy p2 = p1._next;
			while ( p2 != null ) {
				this._testCount++;
				if (isOverlapping(p1, p2)) {
					pickAndPushProxyPair(p1, p2);
				}
				p2 = p2._next;
			}
			p1 = p1._next;
		}
	}

	@Override 
	public void rayCast(Vec3 begin, Vec3 end, BroadPhaseProxyCallback callback) {
		Vec3 min = new Vec3();
		Vec3 max = new Vec3();
		M.vec3_min(min, begin, end);
		M.vec3_max(max, begin, end);

		Proxy p= _proxyList;
		while ( p != null ){
			if(aabbSegmentTest(p._aabbMin, p._aabbMax, begin, end)) {
				callback.process(p);
			}
			p = p._next;
		}
//		var p1:IVec3;
//		var p2:IVec3;
//		var dir:IVec3;
//		M.vec3_fromVec3(p1, begin);
//		M.vec3_fromVec3(p2, end);
//		M.vec3_sub(dir, p2, p1);
//
//		var min:IVec3;
//		var max:IVec3;
//		M.vec3_min(min, p1, p2);
//		M.vec3_max(max, p1, p2);
//
//		var p:Proxy = _proxyList;
//		M.list_foreach(p, _next, {
//			if (M.call(aabbSegmentTest(p._aabbMin, p._aabbMax, p1, p2))) {
//				callback.process(p);
//			}
//		});
	}

	@Override 
	public void convexCast(ConvexGeometry convex, Transform begin, Vec3 translation, BroadPhaseProxyCallback callback) {
		Proxy p = _proxyList;
		while ( p != null ){
			if(aabbConvexSweepTest(p._aabbMin, p._aabbMax, convex,begin, translation)) {
				callback.process(p);
			}
			p = p._next;
		}
//		Proxy p = _proxyList;
//		M.list_foreach(p, _next, {
//			if (M.aabb_overlap(aabb._min, aabb._max, p._aabbMin, p._aabbMax)) {
//				callback.process(p);
//			}
//		});
	}

	
	@Override 
	public void aabbTest(Aabb aabb, BroadPhaseProxyCallback callback) {
		Proxy p  = _proxyList;
		while ( p != null ){
			if(M.aabb_overlap(aabb._min, aabb._max, p._aabbMin, p._aabbMax)) {
				callback.process(p);
			}
			p = p._next;
		}
//		var p:Proxy = _proxyList;
//		M.list_foreach(p, _next, {
//			if (M.aabb_overlap(aabb._min, aabb._max, p._aabbMin, p._aabbMax)) {
//				callback.process(p);
//			}
//		});

	}

}
