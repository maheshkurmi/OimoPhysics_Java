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
		 	double centerX;
		 	double centerY;
		 	double centerZ;
			centerX = leaf._aabbMin.x + leaf._aabbMax.x;
			centerY = leaf._aabbMin.y + leaf._aabbMax.y;
			centerZ = leaf._aabbMin.z + leaf._aabbMax.z;
			BvhNode c1 = currentNode._children[0];
			BvhNode c2 = currentNode._children[1];
			double diff1X;
			double diff1Y;
			double diff1Z;
			double diff2X;
			double diff2Y;
			double diff2Z;
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
		double ey = currentNode._aabbMax.y - currentNode._aabbMin.y;
		double ez = currentNode._aabbMax.z - currentNode._aabbMin.z;
		double combinedMinX;
		double combinedMinY;
		double combinedMinZ;
		double combinedMaxX;
		double combinedMaxY;
		double combinedMaxZ;
		combinedMinX = currentNode._aabbMin.x < leaf._aabbMin.x ? currentNode._aabbMin.x : leaf._aabbMin.x;
		combinedMinY = currentNode._aabbMin.y < leaf._aabbMin.y ? currentNode._aabbMin.y : leaf._aabbMin.y;
		combinedMinZ = currentNode._aabbMin.z < leaf._aabbMin.z ? currentNode._aabbMin.z : leaf._aabbMin.z;
		combinedMaxX = currentNode._aabbMax.x > leaf._aabbMax.x ? currentNode._aabbMax.x : leaf._aabbMax.x;
		combinedMaxY = currentNode._aabbMax.y > leaf._aabbMax.y ? currentNode._aabbMax.y : leaf._aabbMax.y;
		combinedMaxZ = currentNode._aabbMax.z > leaf._aabbMax.z ? currentNode._aabbMax.z : leaf._aabbMax.z;
		double ey1 = combinedMaxY - combinedMinY;
		double ez1 = combinedMaxZ - combinedMinZ;
		double newArea = ((combinedMaxX - combinedMinX) * (ey1 + ez1) + ey1 * ez1) * 2;
		double creatingCost = newArea * 2;
		double incrementalCost = (newArea - ((currentNode._aabbMax.x - currentNode._aabbMin.x) * (ey + ez) + ey * ez) * 2) * 2;
		double descendingCost1 = incrementalCost;
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
		double descendingCost2 = incrementalCost;
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
		double invN = 1.0f / (until - from);

		// mean := sum(min + max) / n
		double centerMeanX = 0;
		double centerMeanY = 0;
		double centerMeanZ = 0;
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

		double varianceX = 0;
		double varianceY = 0;
		double varianceZ = 0;
		for (int i=from;i<until;i++) {
			BvhNode leaf = leaves[i];
			double diffX = leaf._tmp.x - centerMeanX;
			double diffY = leaf._tmp.y - centerMeanY;
			double diffZ = leaf._tmp.z - centerMeanZ;
			diffX *= diffX;
			diffY *= diffY;
			diffZ *= diffZ;
			varianceX += diffX;
			varianceY += diffY;
			varianceZ += diffZ;
		}

		// sort and split
		double varX = varianceX;
		double varY = varianceY;
		double varZ = varianceZ;
		int l = from;
		int r = until - 1;
		if(varX > varY) {
			if(varX > varZ) {
				double mean = centerMeanX;
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
				double mean = centerMeanZ;
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
			double mean = centerMeanY;
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
			double mean = centerMeanZ;
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