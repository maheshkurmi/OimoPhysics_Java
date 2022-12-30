package oimo.dynamics.constraint.contact;
import oimo.collision.narrowphase.*;
import oimo.common.Setting;
import oimo.common.Transform;
import oimo.common.Vec3;
import oimo.dynamics.rigidbody.*;
import oimo.common.M;

/**
 * Internal class
 */
public class ManifoldUpdater {
	Manifold _manifold;

	int numOldPoints;
	ManifoldPoint[] oldPoints;

	public ManifoldUpdater(Manifold manifold) {
		_manifold = manifold;

		numOldPoints = 0;
		oldPoints = new ManifoldPoint[Setting.maxManifoldPoints];
		for (int i=0;i<Setting.maxManifoldPoints;i++) {
			oldPoints[i] = new ManifoldPoint();
		}
	}

	// --- private ---

	private void removeOutdatedPoints() {
		int num = _manifold._numPoints;
		int index = num;
		while (--index >= 0) {
			ManifoldPoint p = _manifold._points[index];

			Vec3 diff=new Vec3();
			M.vec3_sub(diff, p._pos1, p._pos2);
			float dotN = M.vec3_dot(_manifold._normal, diff);

			if (dotN > Setting.contactPersistenceThreshold) {
				removeManifoldPoint(index);
				continue;
			}

			// compute projection of diff
			M.vec3_addRhsScaled(diff, diff, _manifold._normal, -dotN);
			if (M.vec3_dot(diff, diff) > Setting.contactPersistenceThreshold * Setting.contactPersistenceThreshold) {
				// the amount of horizontal sliding exceeds threshold
				removeManifoldPoint(index);
				continue;
			}
		}
	}

	private void removeManifoldPoint(int index) {
		int lastIndex = --_manifold._numPoints;
		if (index != lastIndex) {
			ManifoldPoint tmp = _manifold._points[index];
			_manifold._points[index] = _manifold._points[lastIndex];
			_manifold._points[lastIndex] = tmp;
		}
		_manifold._points[lastIndex]._clear();
	}

	public void addManifoldPoint(DetectorResultPoint point, Transform tf1, Transform tf2) {
		// check if the number of points will exceed the limit
		int num = _manifold._numPoints;
		if (num == Setting.maxManifoldPoints) {
			int targetIndex = computeTargetIndex(point, tf1, tf2);
			_manifold._points[targetIndex]._initialize(point, tf1, tf2);
			return;
		}

		// just add the point
		_manifold._points[num]._initialize(point, tf1, tf2);
		_manifold._numPoints++;
	}

	public int computeTargetIndex(DetectorResultPoint newPoint, Transform tf1, Transform tf2) {
		ManifoldPoint p1 = _manifold._points[0];
		ManifoldPoint p2 = _manifold._points[1];
		ManifoldPoint p3 = _manifold._points[2];
		ManifoldPoint p4 = _manifold._points[3];
		float maxDepth = p1._depth;
		int maxDepthIndex = 0;
		if (p2._depth > maxDepth) {
			maxDepth = p2._depth;
			maxDepthIndex = 1;
		}
		if (p3._depth > maxDepth) {
			maxDepth = p3._depth;
			maxDepthIndex = 2;
		}
		if (p4._depth > maxDepth) {
			maxDepth = p4._depth;
			maxDepthIndex = 3;
		}

		Vec3 rp1=new Vec3();
		M.vec3_fromVec3(rp1, newPoint.position1);
		M.vec3_sub(rp1, rp1, tf1._position);

		float a1 = quadAreaFast(p2._relPos1, p3._relPos1, p4._relPos1, rp1);
		float a2 = quadAreaFast(p1._relPos1, p3._relPos1, p4._relPos1, rp1);
		float a3 = quadAreaFast(p1._relPos1, p2._relPos1, p4._relPos1, rp1);
		float a4 = quadAreaFast(p1._relPos1, p2._relPos1, p3._relPos1, rp1);
		float max = a1;
		int target = 0;
		if (a2 > max && maxDepthIndex != 1 || maxDepthIndex == 0) {
			max = a2;
			target = 1;
		}
		if (a3 > max && maxDepthIndex != 2) {
			max = a3;
			target = 2;
		}
		if (a4 > max && maxDepthIndex != 3) {
			max = a4;
			target = 3;
		}
		return target;
	}

	private float quadAreaFast(Vec3 p1, Vec3 p2, Vec3 p3, Vec3 p4) {
		// possible diagonals (12-34, 13-24, 14-23)
		Vec3 v12=new Vec3();
		Vec3 v34=new Vec3();
		Vec3 v13=new Vec3();
		Vec3 v24=new Vec3();
		Vec3 v14=new Vec3();
		Vec3 v23=new Vec3();
		M.vec3_sub(v12, p2, p1);
		M.vec3_sub(v34, p4, p3);
		M.vec3_sub(v13, p3, p1);
		M.vec3_sub(v24, p4, p2);
		M.vec3_sub(v14, p4, p1);
		M.vec3_sub(v23, p3, p2);
		Vec3 cross1=new Vec3();
		Vec3 cross2=new Vec3();
		Vec3 cross3=new Vec3();
		M.vec3_cross(cross1, v12, v34);
		M.vec3_cross(cross2, v13, v24);
		M.vec3_cross(cross3, v14, v23);
		float a1 = M.vec3_dot(cross1, cross1);
		float a2 = M.vec3_dot(cross2, cross2);
		float a3 = M.vec3_dot(cross3, cross3);
		if (a1 > a2) {
			if (a1 > a3) {
				return a1;
			} else {
				return a3;
			}
		} else {
			if (a2 > a3) {
				return a2;
			} else {
				return a3;
			}
		}
	}

	protected void computeRelativePositions(Transform tf1, Transform tf2) {
		int num = _manifold._numPoints;
		for (int i=0;i<num;i++) {
			ManifoldPoint p = _manifold._points[i];
			M.vec3_mulMat3(p._relPos1, p._localPos1, tf1._rotation);
			M.vec3_mulMat3(p._relPos2, p._localPos2, tf2._rotation);
			p._warmStarted = true; // set warm starting flag
		}
	}

	protected int findNearestContactPointIndex(DetectorResultPoint target, Transform tf1, Transform tf2) {
		float nearestSq = Setting.contactPersistenceThreshold * Setting.contactPersistenceThreshold;
		int idx = -1;
		for (int i=0;i< _manifold._numPoints;i++) {
			float d = distSq(_manifold._points[i], target, tf1, tf2);
			//trace("d is " + d);
			if (d < nearestSq) {
				nearestSq = d;
				idx = i;
			}
		}
		//trace("idx is " + idx);
		return idx;

	}

	private float distSq(ManifoldPoint mp, DetectorResultPoint result, Transform tf1, Transform tf2) {
		Vec3 rp1=new Vec3();
		Vec3 rp2=new Vec3();
		
		M.vec3_fromVec3(rp1, result.position1);
		M.vec3_fromVec3(rp2, result.position2);
		M.vec3_sub(rp1, rp1, tf1._position);
		M.vec3_sub(rp2, rp2, tf2._position);

		Vec3 diff1=rp1;//=new Vec3();
		Vec3 diff2=rp2;//=new Vec3();
		
		M.vec3_sub(diff1, mp._relPos1, rp1);
		M.vec3_sub(diff2, mp._relPos2, rp2);

		float sq1 = M.vec3_dot(diff1, diff1);
		float sq2 = M.vec3_dot(diff2, diff2);
		//trace("sq1: " + sq1 + ", sq2: " + sq2);
		return sq1 < sq2 ? sq1 : sq2;
	}

	public void  saveOldData() {
		numOldPoints = _manifold._numPoints;
		for (int i=0;i<numOldPoints;i++) {
			oldPoints[i]._copyFrom(_manifold._points[i]);
		}
	}

	public void updateContactPointById(ManifoldPoint cp) {
		for (int i=0;i<numOldPoints;i++) {
			ManifoldPoint ocp = oldPoints[i];
			if (cp._id == ocp._id) {
				cp._impulse.copyFrom(ocp._impulse);
				cp._warmStarted = true;
				break;
			}
		}
	}

	// --- internal ---

	public void totalUpdate(DetectorResult result, Transform tf1, Transform tf2) {
		saveOldData();

		int num = result.numPoints;
		_manifold._numPoints = num;
		for (int i=0;i<num;i++) {
			ManifoldPoint p = _manifold._points[i];
			DetectorResultPoint ref = result.points[i];
			p._initialize(ref, tf1, tf2);
			updateContactPointById(p);
		}
	}

	public void incrementalUpdate(DetectorResult result, Transform tf1, Transform tf2) {
		// update old data
		_manifold._updateDepthsAndPositions(tf1, tf2);

		// set warm started flag
		for (int i=0;i<_manifold._numPoints;i++) {
			_manifold._points[i]._warmStarted = true;
		}

		//?M.assert(result.numPoints == 1);
		DetectorResultPoint newPoint = result.points[0];

		// add or update point
		int index = findNearestContactPointIndex(newPoint, tf1, tf2);
		if (index == -1) {
			addManifoldPoint(newPoint, tf1, tf2);
		} else {
			ManifoldPoint cp = _manifold._points[index];
			cp._updateDepthAndPositions(newPoint, tf1, tf2);
		}

		// remove some points
		removeOutdatedPoints();
	}
}
