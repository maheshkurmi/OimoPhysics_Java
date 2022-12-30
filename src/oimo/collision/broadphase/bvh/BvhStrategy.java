package oimo.collision.broadphase.bvh;

/**
 * Internal class.
 *
 * BVH strategy for BVH tree
 */
public class BvhStrategy {
	public int _insertionStrategy;
	public boolean _balancingEnabled;

	public BvhStrategy() {
		_insertionStrategy = BvhInsertionStrategy.SIMPLE;
		_balancingEnabled = false;
	}

	// --- internal ---

	/**
	 * Returns the next step of leaf insertion.
	 * `0` or `1` to descend to corresponding child of current node.
	 * `-1` to stop descending and make common parent with current node.
	 */
	public int _decideInsertion(BvhNode currentNode, BvhNode leaf) {
		switch(_insertionStrategy) {
		case BvhInsertionStrategy.SIMPLE:
			return decideInsertionSimple(currentNode, leaf);
		case BvhInsertionStrategy.MINIMIZE_SURFACE_AREA:
			return decideInsertionMinimumSurfaceArea(currentNode, leaf);
		default:
			System.out.println("invalid BVH insertion strategy: " + _insertionStrategy);
			return -1;
		}
	}

	/**
	 * Sorts `leaves` and returns the split index `k` of the half-open interval [`from`, `until`).
	 * Leaves are separated into [`from`, `k`) and [`k`, `until`).
	 */
	public int _splitLeaves(BvhNode[] leaves, int from, int until) {
		return splitLeavesMean(leaves, from, until);
	}

	// --- private ---

	 int decideInsertionSimple(BvhNode currentNode,BvhNode leaf) {
		 	float centerX;
		 	float centerY;
		 	float centerZ;
			centerX = leaf._aabbMin.x + leaf._aabbMax.x;
			centerY = leaf._aabbMin.y + leaf._aabbMax.y;
			centerZ = leaf._aabbMin.z + leaf._aabbMax.z;
			BvhNode c1 = currentNode._children[0];
			BvhNode c2 = currentNode._children[1];
			float diff1X;
			float diff1Y;
			float diff1Z;
			float diff2X;
			float diff2Y;
			float diff2Z;
			diff1X = c1._aabbMin.x + c1._aabbMax.x;
			diff1Y = c1._aabbMin.y + c1._aabbMax.y;
			diff1Z = c1._aabbMin.z + c1._aabbMax.z;
			diff2X = c2._aabbMin.x + c2._aabbMax.x;
			diff2Y = c2._aabbMin.y + c2._aabbMax.y;
			diff2Z = c2._aabbMin.z + c2._aabbMax.z;
			diff1X -= centerX;
			diff1Y -= centerY;
			diff1Z -= centerZ;
			diff2X -= centerX;
			diff2Y -= centerY;
			diff2Z -= centerZ;
			if(diff1X * diff1X + diff1Y * diff1Y + diff1Z * diff1Z < diff2X * diff2X + diff2Y * diff2Y + diff2Z * diff2Z) {
				return 0;
			} else {
				return 1;
			}
	}

	int decideInsertionMinimumSurfaceArea(BvhNode currentNode, BvhNode leaf) {
		BvhNode c11 = currentNode._children[0];
		BvhNode c21 = currentNode._children[1];
		float ey = currentNode._aabbMax.y - currentNode._aabbMin.y;
		float ez = currentNode._aabbMax.z - currentNode._aabbMin.z;
		float combinedMinX;
		float combinedMinY;
		float combinedMinZ;
		float combinedMaxX;
		float combinedMaxY;
		float combinedMaxZ;
		combinedMinX = currentNode._aabbMin.x < leaf._aabbMin.x ? currentNode._aabbMin.x : leaf._aabbMin.x;
		combinedMinY = currentNode._aabbMin.y < leaf._aabbMin.y ? currentNode._aabbMin.y : leaf._aabbMin.y;
		combinedMinZ = currentNode._aabbMin.z < leaf._aabbMin.z ? currentNode._aabbMin.z : leaf._aabbMin.z;
		combinedMaxX = currentNode._aabbMax.x > leaf._aabbMax.x ? currentNode._aabbMax.x : leaf._aabbMax.x;
		combinedMaxY = currentNode._aabbMax.y > leaf._aabbMax.y ? currentNode._aabbMax.y : leaf._aabbMax.y;
		combinedMaxZ = currentNode._aabbMax.z > leaf._aabbMax.z ? currentNode._aabbMax.z : leaf._aabbMax.z;
		float ey1 = combinedMaxY - combinedMinY;
		float ez1 = combinedMaxZ - combinedMinZ;
		float newArea = ((combinedMaxX - combinedMinX) * (ey1 + ez1) + ey1 * ez1) * 2;
		float creatingCost = newArea * 2;
		float incrementalCost = (newArea - ((currentNode._aabbMax.x - currentNode._aabbMin.x) * (ey + ez) + ey * ez) * 2) * 2;
		float descendingCost1 = incrementalCost;
		combinedMinX = c11._aabbMin.x < leaf._aabbMin.x ? c11._aabbMin.x : leaf._aabbMin.x;
		combinedMinY = c11._aabbMin.y < leaf._aabbMin.y ? c11._aabbMin.y : leaf._aabbMin.y;
		combinedMinZ = c11._aabbMin.z < leaf._aabbMin.z ? c11._aabbMin.z : leaf._aabbMin.z;
		combinedMaxX = c11._aabbMax.x > leaf._aabbMax.x ? c11._aabbMax.x : leaf._aabbMax.x;
		combinedMaxY = c11._aabbMax.y > leaf._aabbMax.y ? c11._aabbMax.y : leaf._aabbMax.y;
		combinedMaxZ = c11._aabbMax.z > leaf._aabbMax.z ? c11._aabbMax.z : leaf._aabbMax.z;
		if(c11._height == 0) {
			 ey = combinedMaxY - combinedMinY;
			 ez = combinedMaxZ - combinedMinZ;
			descendingCost1 = incrementalCost + ((combinedMaxX - combinedMinX) * (ey + ez) + ey * ez) * 2;
		} else {
			 ey = combinedMaxY - combinedMinY;
			 ez = combinedMaxZ - combinedMinZ;
			 ey1 = c11._aabbMax.y - c11._aabbMin.y;
			 ez1 = c11._aabbMax.z - c11._aabbMin.z;
			descendingCost1 = incrementalCost + (((combinedMaxX - combinedMinX) * (ey + ez) + ey * ez) * 2 - ((c11._aabbMax.x - c11._aabbMin.x) * (ey1 + ez1) + ey1 * ez1) * 2);
		}
		float descendingCost2 = incrementalCost;
		combinedMinX = c21._aabbMin.x < leaf._aabbMin.x ? c21._aabbMin.x : leaf._aabbMin.x;
		combinedMinY = c21._aabbMin.y < leaf._aabbMin.y ? c21._aabbMin.y : leaf._aabbMin.y;
		combinedMinZ = c21._aabbMin.z < leaf._aabbMin.z ? c21._aabbMin.z : leaf._aabbMin.z;
		combinedMaxX = c21._aabbMax.x > leaf._aabbMax.x ? c21._aabbMax.x : leaf._aabbMax.x;
		combinedMaxY = c21._aabbMax.y > leaf._aabbMax.y ? c21._aabbMax.y : leaf._aabbMax.y;
		combinedMaxZ = c21._aabbMax.z > leaf._aabbMax.z ? c21._aabbMax.z : leaf._aabbMax.z;
		if(c21._height == 0) {
			 ey = combinedMaxY - combinedMinY;
			 ez = combinedMaxZ - combinedMinZ;
			descendingCost2 = incrementalCost + ((combinedMaxX - combinedMinX) * (ey + ez) + ey * ez) * 2;
		} else {
			 ey = combinedMaxY - combinedMinY;
			 ez = combinedMaxZ - combinedMinZ;
			 ey1 = c21._aabbMax.y - c21._aabbMin.y;
			 ez1 = c21._aabbMax.z - c21._aabbMin.z;
			descendingCost2 = incrementalCost + (((combinedMaxX - combinedMinX) * (ey + ez) + ey * ez) * 2 - ((c21._aabbMax.x - c21._aabbMin.x) * (ey1 + ez1) + ey1 * ez1) * 2);
		}
		if(creatingCost < descendingCost1) {
			if(creatingCost < descendingCost2) {
				return -1;
			} else {
				return 1;
			}
		} else if(descendingCost1 < descendingCost2) {
			return 0;
		} else {
			return 1;
		}
	}

	 int splitLeavesMean(BvhNode[] leaves, int from,int until) {
		float invN = 1.0f / (until - from);

		// mean := sum(min + max) / n
		float centerMeanX = 0;
		float centerMeanY = 0;
		float centerMeanZ = 0;
		int _g = from;
		while(_g < until) {
			BvhNode leaf = leaves[_g++];
			leaf._tmp.x = leaf._aabbMax.x + leaf._aabbMin.x;
			leaf._tmp.y = leaf._aabbMax.y + leaf._aabbMin.y;
			leaf._tmp.z = leaf._aabbMax.z + leaf._aabbMin.z;
			centerMeanX += leaf._tmp.x;
			centerMeanY += leaf._tmp.y;
			centerMeanZ += leaf._tmp.z;
		}
		centerMeanX *= invN;
		centerMeanY *= invN;
		centerMeanZ *= invN;

		float varianceX = 0;
		float varianceY = 0;
		float varianceZ = 0;
		for (int i=from;i<until;i++) {
			BvhNode leaf = leaves[i];
			float diffX = leaf._tmp.x - centerMeanX;
			float diffY = leaf._tmp.y - centerMeanY;
			float diffZ = leaf._tmp.z - centerMeanZ;
			diffX *= diffX;
			diffY *= diffY;
			diffZ *= diffZ;
			varianceX += diffX;
			varianceY += diffY;
			varianceZ += diffZ;
		}

		// sort and split
		float varX = varianceX;
		float varY = varianceY;
		float varZ = varianceZ;
		int l = from;
		int r = until - 1;
		if(varX > varY) {
			if(varX > varZ) {
				float mean = centerMeanX;
				while(true) {
					while(!(leaves[l]._tmp.x <= mean)) ++l;
					while(!(leaves[r]._tmp.x >= mean)) --r;
					if(l >= r) {
						break;
					}
					BvhNode tmp = leaves[l];
					leaves[l] = leaves[r];
					leaves[r] = tmp;
					++l;
					--r;
				}
			} else {
				float mean = centerMeanZ;
				while(true) {
					while(!(leaves[l]._tmp.z <= mean)) ++l;
					while(!(leaves[r]._tmp.z >= mean)) --r;
					if(l >= r) {
						break;
					}
					BvhNode tmp = leaves[l];
					leaves[l] = leaves[r];
					leaves[r] = tmp;
					++l;
					--r;
				}
			}
		} else if(varY > varZ) {
			float mean = centerMeanY;
			while(true) {
				while(!(leaves[l]._tmp.y <= mean)) ++l;
				while(!(leaves[r]._tmp.y >= mean)) --r;
				if(l >= r) {
					break;
				}
				BvhNode tmp = leaves[l];
				leaves[l] = leaves[r];
				leaves[r] = tmp;
				++l;
				--r;
			}
		} else {
			float mean = centerMeanZ;
			while(true) {
				while(!(leaves[l]._tmp.z <= mean)) ++l;
				while(!(leaves[r]._tmp.z >= mean)) --r;
				if(l >= r) {
					break;
				}
				BvhNode tmp = leaves[l];
				leaves[l] = leaves[r];
				leaves[r] = tmp;
				++l;
				--r;
			}
		}
		return l;
	}

}