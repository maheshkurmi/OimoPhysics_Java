package oimo.collision.narrowphase.detector;

import oimo.collision.geometry.BoxGeometry;
import oimo.collision.geometry.Geometry;
import oimo.collision.narrowphase.DetectorResult;
import oimo.common.M;
import oimo.common.MathUtil;
import oimo.common.Setting;
import oimo.common.Transform;
import oimo.common.Vec3;

/**
 * Box vs Box detector.
 */
public class BoxBoxDetector extends Detector {
	private static  final float EDGE_BIAS_MULT=1.0f;

	FaceClipper clipper;

	/**
	 * Default constructor.
	 */
	public BoxBoxDetector() {
		super(false);
		clipper = new FaceClipper();
	}

	@Override 
	public void detectImpl(DetectorResult result, Geometry geom1, Geometry geom2, Transform tf1, Transform tf2, CachedDetectorData cachedData) {
		BoxGeometry b1 =  (BoxGeometry) geom1;
		BoxGeometry b2 =  (BoxGeometry) geom2;

		result.incremental = false;

		// basis of box1 := {x1, y1, z1}
		// basis of box2 := {x2, y2, z2}
		// half-extents of box1 := {w1, h1, d1}
		// half-extents of box2 := {w2, h2, d2}
		//
		// candidates of the separating axis:
		//     x1,
		//     y1,
		//     z1,
		//     x2,
		//     y2,
		//     z2,
		//     cross(x1, x2),
		//     cross(x1, y2),
		//     cross(x1, z2),
		//     cross(y1, x2),
		//     cross(y1, y2),
		//     cross(y1, z2),
		//     cross(z1, x2),
		//     cross(z1, y2),
		//     cross(z1, z2).
		//
		// projected length of box1:
		//   project to       | length
		//   -------------------------------
		//   x1               | w1
		//   y1               | h1
		//   z1               | d1
		//   a = cross(x1, _) | h1|y1.a| + d1|z1.a|
		//   a = cross(y1, _) | w1|x1.a| + d1|z1.a|
		//   a = cross(z1, _) | w1|x1.a| + h1|y1.a|
		//   a = _            | w1|x1.a| + h1|y1.a| + d1|z1.a|
		//
		// projected length of box2:
		//   project to       | length
		//   -------------------------------
		//   x2               | w2
		//   y2               | h2
		//   z2               | d2
		//   a = cross(x2, _) | h2|y2.a| + d2|z2.a|
		//   a = cross(y2, _) | w2|x2.a| + d2|z2.a|
		//   a = cross(z2, _) | w2|x2.a| + h2|y2.a|
		//   a = _            | w2|x2.a| + h2|y2.a| + d2|z2.a|

		Vec3 c1=tf1._position;
		Vec3 c2=tf2._position;
		Vec3 c12=c2.sub(c1); // from center1 to center2

		// bases
		Vec3 x1=new Vec3();
		Vec3 y1=new Vec3();
		Vec3 z1=new Vec3();
		Vec3 x2=new Vec3();
		Vec3 y2=new Vec3();
		Vec3 z2=new Vec3();
		M.mat3_getCol(x1, tf1._rotation, 0);
		M.mat3_getCol(y1, tf1._rotation, 1);
		M.mat3_getCol(z1, tf1._rotation, 2);
		M.mat3_getCol(x2, tf2._rotation, 0);
		M.mat3_getCol(y2, tf2._rotation, 1);
		M.mat3_getCol(z2, tf2._rotation, 2);

		// half extents of each box
		float w1=b1._halfExtents.x;
		float h1=b1._halfExtents.y;
		float d1=b1._halfExtents.y;
		float w2=b2._halfExtents.x;
		float h2=b2._halfExtents.x;
		float d2=b2._halfExtents.x;

		// scaled bases by half extents
		Vec3 sx1=x1.scale(w1);
		Vec3 sy1=y1.scale(h1);
		Vec3 sz1=z1.scale(d1);
		Vec3 sx2=x2.scale(w2);
		Vec3 sy2=y2.scale(h2);
		Vec3 sz2=z2.scale(d2);
		

		// --------------------- SAT check start ---------------------

		float proj1;
		float proj2;
		float projSum;
		float projC12;
		float projC12Abs;

		float mDepth = MathUtil.POSITIVE_INFINITY;
		int mId = -1;
		int mSign = 0;
		Vec3 mAxis=new Vec3();

		// --------------------- 6 faces ---------------------

		// try axis = x1
		proj1 = w1;
		proj2 = project(x1, sx2, sy2, sz2);
		projC12 = M.vec3_dot(x1, c12);
		//BoxBoxDetectorMacro.satCheck(mDepth, mId, mSign, mAxis, proj1, proj2, projC12, x1, 0, 1.0);
		float sum = proj1 + proj2;
		boolean neg = projC12 < 0;
		float abs = neg ? -projC12 : projC12;
		if(abs < sum) {
			float depth = sum - abs;
			if(depth < mDepth) {
				mDepth = depth;
				mId = 0;
				mAxis.set(x1);
				mSign = neg ? -1 : 1;
			}
		} else {
			return;
		}
		
		
		// try axis = y1
		proj1 = h1;
		proj2 = project(y1, sx2, sy2, sz2);
		projC12 = M.vec3_dot(y1, c12);
		//BoxBoxDetectorMacro.satCheck(mDepth, mId, mSign, mAxis, proj1, proj2, projC12, y1, 1, 1.0);
		sum = proj1 + proj2;
		neg = projC12 < 0;
		abs = neg ? -projC12 : projC12;
		if(abs < sum) {
			float depth = sum - abs;
			if(depth < mDepth) {
				mDepth = depth;
				mId = 1;
				mAxis.set(y1);
				mSign = neg ? -1 : 1;
			}
		} else {
			return;
		}
		
		// try axis = z1
		proj1 = d1;
		proj2 = project(z1, sx2, sy2, sz2);
		projC12 = M.vec3_dot(z1, c12);
		//BoxBoxDetectorMacro.satCheck(mDepth, mId, mSign, mAxis, proj1, proj2, projC12, z1, 2, 1.0);
		sum = proj1 + proj2;
		neg = projC12 < 0;
		abs = neg ? -projC12 : projC12;
		if(abs < sum) {
			float depth = sum - abs;
			if(depth < mDepth) {
				mDepth = depth;
				mId = 2;
				mAxis.set(z1);
				mSign = neg ? -1 : 1;
			}
		} else {
			return;
		}
		
		
		// apply bias to avoid jitting
		if (mDepth > Setting.linearSlop) {
			mDepth -= Setting.linearSlop;
		} else {
			mDepth = 0;
		}

		// try axis = x2
		proj1 = project(x2, sx1, sy1, sz1);
		proj2 = w2;
		projC12 = M.vec3_dot(x2, c12);
		//BoxBoxDetectorMacro.satCheck(mDepth, mId, mSign, mAxis, proj1, proj2, projC12, x2, 3, 1.0);
		sum = proj1 + proj2;
		neg = projC12 < 0;
		abs = neg ? -projC12 : projC12;
		if(abs < sum) {
			float depth = sum - abs;
			if(depth < mDepth) {
				mDepth = depth;
				mId = 3;
				mAxis.set(x2);
				mSign = neg ? -1 : 1;
			}
		} else {
			return;
		}
		
		
		// try axis = y2
		proj1 = project(y2, sx1, sy1, sz1);
		proj2 = h2;
		projC12 = M.vec3_dot(y2, c12);
		//BoxBoxDetectorMacro.satCheck(mDepth, mId, mSign, mAxis, proj1, proj2, projC12, y2, 4, 1.0);
		sum = proj1 + proj2;
		neg = projC12 < 0;
		abs = neg ? -projC12 : projC12;
		if(abs < sum) {
			float depth = sum - abs;
			if(depth < mDepth) {
				mDepth = depth;
				mId = 4;
				mAxis.set(y2);
				mSign = neg ? -1 : 1;
			}
		} else {
			return;
		}
		
		
		// try axis = z2
		proj1 = project(z2, sx1, sy1, sz1);
		proj2 = d2;
		projC12 = M.vec3_dot(z2, c12);
		//BoxBoxDetectorMacro.satCheck(mDepth, mId, mSign, mAxis, proj1, proj2, projC12, z2, 5, 1.0);
		sum = proj1 + proj2;
		neg = projC12 < 0;
		abs = neg ? -projC12 : projC12;
		if(abs < sum) {
			float depth = sum - abs;
			if(depth < mDepth) {
				mDepth = depth;
				mId = 5;
				mAxis.set(z2);
				mSign = neg ? -1 : 1;
			}
		} else {
			return;
		}
		
		
		// --------------------- 9 edges ---------------------

		// apply bias again to avoid jitting
		if (mDepth > Setting.linearSlop) {
			mDepth -= Setting.linearSlop;
		} else {
			mDepth = 0;
		}

		Vec3 edgeAxis=new Vec3();

		// try cross(x1, x2)
		M.vec3_cross(edgeAxis, x1, x2);
		if (!M.vec3_isZero(edgeAxis)) {
			M.vec3_normalize(edgeAxis, edgeAxis);
			proj1 = project2(edgeAxis, sy1, sz1);
			proj2 = project2(edgeAxis, sy2, sz2);
			projC12 = M.vec3_dot(edgeAxis, c12);
			//BoxBoxDetectorMacro.satCheck(mDepth, mId, mSign, mAxis, proj1, proj2, projC12, edgeAxis, 6, EDGE_BIAS_MULT);
			sum = proj1 + proj2;
			neg = projC12 < 0;
			abs = neg ? -projC12 : projC12;
			if(abs < sum) {
				float depth = sum - abs;
				if(depth < mDepth) {
					mDepth = depth;
					mId = 6;
					mAxis.set(edgeAxis);
					mSign = neg ? -1 : 1;
				}
			} else {
				return;
			}
		}

		// try cross(x1, y2)
		M.vec3_cross(edgeAxis, x1, y2);
		if (!M.vec3_isZero(edgeAxis)) {
			M.vec3_normalize(edgeAxis, edgeAxis);
			proj1 = project2(edgeAxis, sy1, sz1);
			proj2 = project2(edgeAxis, sx2, sz2);
			projC12 = M.vec3_dot(edgeAxis, c12);
			//BoxBoxDetectorMacro.satCheck(mDepth, mId, mSign, mAxis, proj1, proj2, projC12, edgeAxis, 7, EDGE_BIAS_MULT);
			sum = proj1 + proj2;
			neg = projC12 < 0;
			abs = neg ? -projC12 : projC12;
			if(abs < sum) {
				float depth = sum - abs;
				if(depth < mDepth) {
					mDepth = depth;
					mId = 7;
					mAxis.set(edgeAxis);
					mSign = neg ? -1 : 1;
				}
			} else {
				return;
			}
		}

		// try cross(x1, z2)
		M.vec3_cross(edgeAxis, x1, z2);
		if (!M.vec3_isZero(edgeAxis)) {
			M.vec3_normalize(edgeAxis, edgeAxis);
			proj1 = project2(edgeAxis, sy1, sz1);
			proj2 = project2(edgeAxis, sx2, sy2);
			projC12 = M.vec3_dot(edgeAxis, c12);
			//BoxBoxDetectorMacro.satCheck(mDepth, mId, mSign, mAxis, proj1, proj2, projC12, edgeAxis, 8, EDGE_BIAS_MULT);
			sum = proj1 + proj2;
			neg = projC12 < 0;
			abs = neg ? -projC12 : projC12;
			if(abs < sum) {
				float depth = sum - abs;
				if(depth < mDepth) {
					mDepth = depth;
					mId = 8;
					mAxis.set(edgeAxis);
					mSign = neg ? -1 : 1;
				}
			} else {
				return;
			}
		}

		// try cross(y1, x2)
		M.vec3_cross(edgeAxis, y1, x2);
		if (!M.vec3_isZero(edgeAxis)) {
			M.vec3_normalize(edgeAxis, edgeAxis);
			proj1 = project2(edgeAxis, sx1, sz1);
			proj2 = project2(edgeAxis, sy2, sz2);
			projC12 = M.vec3_dot(edgeAxis, c12);
			//BoxBoxDetectorMacro.satCheck(mDepth, mId, mSign, mAxis, proj1, proj2, projC12, edgeAxis, 9, EDGE_BIAS_MULT);
			sum = proj1 + proj2;
			neg = projC12 < 0;
			abs = neg ? -projC12 : projC12;
			if(abs < sum) {
				float depth = sum - abs;
				if(depth < mDepth) {
					mDepth = depth;
					mId = 9;
					mAxis.set(edgeAxis);
					mSign = neg ? -1 : 1;
				}
			} else {
				return;
			}
		}

		// try cross(y1, y2)
		M.vec3_cross(edgeAxis, y1, y2);
		if (!M.vec3_isZero(edgeAxis)) {
			M.vec3_normalize(edgeAxis, edgeAxis);
			proj1 = project2(edgeAxis, sx1, sz1);
			proj2 = project2(edgeAxis, sx2, sz2);
			projC12 = M.vec3_dot(edgeAxis, c12);
			//BoxBoxDetectorMacro.satCheck(mDepth, mId, mSign, mAxis, proj1, proj2, projC12, edgeAxis, 10, EDGE_BIAS_MULT);
			sum = proj1 + proj2;
			neg = projC12 < 0;
			abs = neg ? -projC12 : projC12;
			if(abs < sum) {
				float depth = sum - abs;
				if(depth < mDepth) {
					mDepth = depth;
					mId = 10;
					mAxis.set(edgeAxis);
					mSign = neg ? -1 : 1;
				}
			} else {
				return;
			}
		}

		// try cross(y1, z2)
		M.vec3_cross(edgeAxis, y1, z2);
		if (!M.vec3_isZero(edgeAxis)) {
			M.vec3_normalize(edgeAxis, edgeAxis);
			proj1 = project2(edgeAxis, sx1, sz1);
			proj2 = project2(edgeAxis, sx2, sy2);
			projC12 = M.vec3_dot(edgeAxis, c12);
			//BoxBoxDetectorMacro.satCheck(mDepth, mId, mSign, mAxis, proj1, proj2, projC12, edgeAxis, 11, EDGE_BIAS_MULT);
			sum = proj1 + proj2;
			neg = projC12 < 0;
			abs = neg ? -projC12 : projC12;
			if(abs < sum) {
				float depth = sum - abs;
				if(depth < mDepth) {
					mDepth = depth;
					mId = 11;
					mAxis.set(edgeAxis);
					mSign = neg ? -1 : 1;
				}
			} else {
				return;
			}
		}

		// try cross(z1, x2)
		M.vec3_cross(edgeAxis, z1, x2);
		if (!M.vec3_isZero(edgeAxis)) {
			M.vec3_normalize(edgeAxis, edgeAxis);
			proj1 = project2(edgeAxis, sx1, sy1);
			proj2 = project2(edgeAxis, sy2, sz2);
			projC12 = M.vec3_dot(edgeAxis, c12);
			//BoxBoxDetectorMacro.satCheck(mDepth, mId, mSign, mAxis, proj1, proj2, projC12, edgeAxis, 12, EDGE_BIAS_MULT);
			sum = proj1 + proj2;
			neg = projC12 < 0;
			abs = neg ? -projC12 : projC12;
			if(abs < sum) {
				float depth = sum - abs;
				if(depth < mDepth) {
					mDepth = depth;
					mId = 12;
					mAxis.set(edgeAxis);
					mSign = neg ? -1 : 1;
				}
			} else {
				return;
			}
		}

		// try cross(z1, y2)
		M.vec3_cross(edgeAxis, z1, y2);
		if (!M.vec3_isZero(edgeAxis)) {
			M.vec3_normalize(edgeAxis, edgeAxis);
			proj1 = project2(edgeAxis, sx1, sy1);
			proj2 = project2(edgeAxis, sx2, sz2);
			projC12 = M.vec3_dot(edgeAxis, c12);
			//BoxBoxDetectorMacro.satCheck(mDepth, mId, mSign, mAxis, proj1, proj2, projC12, edgeAxis, 13, EDGE_BIAS_MULT);
			sum = proj1 + proj2;
			neg = projC12 < 0;
			abs = neg ? -projC12 : projC12;
			if(abs < sum) {
				float depth = sum - abs;
				if(depth < mDepth) {
					mDepth = depth;
					mId = 13;
					mAxis.set(edgeAxis);
					mSign = neg ? -1 : 1;
				}
			} else {
				return;
			}
		}

		// try cross(z1, z2)
		M.vec3_cross(edgeAxis, z1, z2);
		if (!M.vec3_isZero(edgeAxis)) {
			M.vec3_normalize(edgeAxis, edgeAxis);
			proj1 = project2(edgeAxis, sx1, sy1);
			proj2 = project2(edgeAxis, sx2, sy2);
			projC12 = M.vec3_dot(edgeAxis, c12);
			//BoxBoxDetectorMacro.satCheck(mDepth, mId, mSign, mAxis, proj1, proj2, projC12, edgeAxis, 14, EDGE_BIAS_MULT);
			sum = proj1 + proj2;
			neg = projC12 < 0;
			abs = neg ? -projC12 : projC12;
			if(abs < sum) {
				float depth = sum - abs;
				if(depth < mDepth) {
					mDepth = depth;
					mId = 14;
					mAxis.set(edgeAxis);
					mSign = neg ? -1 : 1;
				}
			} else {
				return;
			}
		}

		// --------------------- edge-edge collision check ---------------------

		if (mId >= 6) {
			// flip axis so that it directs from box1 to box2
			M.vec3_scale(mAxis, mAxis, mSign);

			// direction of edges: 0 = x, 1 = y, 2 = z
			int id1 = ((int) (( ((double) ((( mId - 6 ))) ) / 3 )) );
			int id2 = ( ( mId - 6 ) - ( id1 * 3 ) );

			// points on the edges
			Vec3 p1=new Vec3();
			Vec3 p2=new Vec3();
			

			// directions
			Vec3 dir1=new Vec3();
			Vec3 dir2=new Vec3();
			

			switch (id1) {
			case 0: // use y and z
				M.vec3_assign(dir1, x1);
				supportingVertexRect(p1, sy1, sz1, mAxis);
				break;
			case 1: // use x and z
				M.vec3_assign(dir1, y1);
				supportingVertexRect(p1, sx1, sz1, mAxis);
				break;
			default: // use x and y
				M.vec3_assign(dir1, z1);
				supportingVertexRect(p1, sx1, sy1, mAxis);
			}
			M.vec3_add(p1, c1, p1);

			switch (id2) {
			case 0: // use y and z
				M.vec3_assign(dir2, x2);
				supportingVertexRect(p2, sy2, sz2, mAxis);
				break;
			case 1: // use x and z
				M.vec3_assign(dir2, y2);
				supportingVertexRect(p2, sx2, sz2, mAxis);
				break;
			default: // use x and y
				M.vec3_assign(dir2, z2);
				supportingVertexRect(p2, sx2, sy2, mAxis);
			}
			M.vec3_sub(p2, c2, p2);

			// compute params
			Vec3 r=new Vec3();
			M.vec3_sub(r, p1, p2);

			float dot12 = M.vec3_dot(dir1, dir2);
			float dot1r = M.vec3_dot(dir1, r);
			float dot2r = M.vec3_dot(dir2, r);

			float invDet = 1 / (1 - dot12 * dot12);
			float t1 = (dot12 * dot2r - dot1r) * invDet;
			float t2 = (dot2r - dot12 * dot1r) * invDet;

			// compute closest points and normal
			Vec3 cp1=new Vec3();
			Vec3 cp2=new Vec3();
			M.vec3_addRhsScaled(cp1, p1, dir1, t1);
			M.vec3_addRhsScaled(cp2, p2, dir2, t2);

			Vec3 normal =new Vec3(-mAxis.x,-mAxis.y,-mAxis.z);
			// add contact point
			setNormal(result, normal);
			addPoint(result, cp1, cp2, mDepth, 4);
			return;
		}

		// --------------------- face-face collision check ---------------------

		float tmpX;
		float tmpY;
		float tmpZ;
		boolean swapped;

		if (mId >= 3) { // swap box1 and box2
			mSign = -mSign;
			c12.negateEq();
			BoxGeometry tmp = b1;
			b1 = b2;
			b2 = tmp;
			float tmp1 = w1;
			w1 = w2;
			w2 = tmp1;
			float tmp2 = h1;
			h1 = h2;
			h2 = tmp2;
			float tmp3 = d1;
			d1 = d2;
			d2 = tmp3;
			M.vec3_swap( c1, c2);
			M.vec3_swap( x1, x2);
			M.vec3_swap( y1, y2);
			M.vec3_swap( z1, z2);
			M.vec3_swap( sx1, sx2);
			M.vec3_swap( sy1, sy2);
			M.vec3_swap( sz1, sz2);
			mId -= 3;
			swapped = true;
		} else {
			swapped = false;
		}

		// --------------------- find reference face ---------------------

		Vec3 refCenter=new Vec3();
		Vec3 refNormal=new Vec3();
		Vec3 refX=new Vec3();
		Vec3 refY=new Vec3();
		float refW;
		float refH;

		switch (mId) {
			case 0: // x+ or x-
				M.vec3_assign(refCenter, sx1);
				M.vec3_assign(refNormal, x1);
				M.vec3_assign(refX, y1);
				M.vec3_assign(refY, z1);
				refW = h1;
				refH = d1;
			case 1: // y+ or y-
				M.vec3_assign(refCenter, sy1);
				M.vec3_assign(refNormal, y1);
				M.vec3_assign(refX, z1);
				M.vec3_assign(refY, x1);
				refW = d1;
				refH = w1;
			default: // z+ or z-
				M.vec3_assign(refCenter, sz1);
				M.vec3_assign(refNormal, z1);
				M.vec3_assign(refX, x1);
				M.vec3_assign(refY, y1);
				refW = w1;
				refH = h1;
		}

		if (mSign < 0) { // x- or y- or z-
			refCenter.negateEq();
			refNormal.negateEq();
			M.vec3_swap( refX, refY);
			float tmp=refW;
			refW=refH;
			refH=tmp;
		}

		// translate reference center
		M.vec3_add(refCenter, refCenter, c1);

		// --------------------- find incident face ---------------------

		float minIncDot = 1;
		int incId = 0;

		float incDot;
		incDot = M.vec3_dot(refNormal, x2);
		if (incDot < minIncDot) { // x+
			minIncDot = incDot;
			incId = 0;
		}
		if (-incDot < minIncDot) { // x-
			minIncDot = -incDot;
			incId = 1;
		}
		incDot = M.vec3_dot(refNormal, y2);
		if (incDot < minIncDot) { // y+
			minIncDot = incDot;
			incId = 2;
		}
		if (-incDot < minIncDot) { // y-
			minIncDot = -incDot;
			incId = 3;
		}
		incDot = M.vec3_dot(refNormal, z2);
		if (incDot < minIncDot) { // y+
			minIncDot = incDot;
			incId = 4;
		}
		if (-incDot < minIncDot) { // y-
			minIncDot = -incDot;
			incId = 5;
		}

		Vec3 incV1=new Vec3();
		Vec3 incV2=new Vec3();
		Vec3 incV3=new Vec3();
		Vec3 incV4=new Vec3();

		switch (incId) {
			case 0:
				getBoxFace(incV1, incV2, incV3, incV4, sx2, sy2, sz2, "x+");
			case 1:
				getBoxFace(incV1, incV2, incV3, incV4, sx2, sy2, sz2, "x-");
			case 2:
				getBoxFace(incV1, incV2, incV3, incV4, sx2, sy2, sz2, "y+");
			case 3:
				getBoxFace(incV1, incV2, incV3, incV4, sx2, sy2, sz2, "y-");
			case 4:
				getBoxFace(incV1, incV2, incV3, incV4, sx2, sy2, sz2, "z+");
			default:
				getBoxFace(incV1, incV2, incV3, incV4, sx2, sy2, sz2, "z-");
		}

		M.vec3_add(incV1, incV1, c12);
		M.vec3_add(incV2, incV2, c12);
		M.vec3_add(incV3, incV3, c12);
		M.vec3_add(incV4, incV4, c12);

		// --------------------- clip incident face ---------------------

		clipper.init(refW, refH);
		clipper.addIncidentVertex(M.vec3_dot(incV1, refX), M.vec3_dot(incV1, refY),incV1.x, incV1.y,incV1.z);
		clipper.addIncidentVertex(M.vec3_dot(incV2, refX), M.vec3_dot(incV2, refY),incV2.x, incV2.y,incV2.z);
		clipper.addIncidentVertex(M.vec3_dot(incV3, refX), M.vec3_dot(incV3, refY),incV3.x, incV3.y,incV3.z);
		clipper.addIncidentVertex(M.vec3_dot(incV4, refX), M.vec3_dot(incV4, refY),incV4.x, incV4.y,incV4.z);
		clipper.clip();

		// --------------------- reduce vertices ---------------------

		clipper.reduce();

		// --------------------- add contact points ---------------------

		// set normal
		Vec3 normal=new Vec3();
		if (swapped) {
			normal.set(refNormal.x,refNormal.y,refNormal.z);
		} else {
			normal.set(-refNormal.x,-refNormal.y,-refNormal.z);
		}
		this.setNormal(result, normal);

		// add contact points
		for (int i=0; i<clipper.numVertices;i++) {
			float clippedVertexX;
			float clippedVertexY;
			float clippedVertexZ;
			IncidentVertex v = this.clipper.vertices[i];
			clippedVertexX = v.wx;
			clippedVertexY = v.wy;
			clippedVertexZ = v.wz;
			clippedVertexX += c1.x;
			clippedVertexY += c1.y;
			clippedVertexZ += c1.z;
			float clippedVertexToRefCenterX;
			float clippedVertexToRefCenterY;
			float clippedVertexToRefCenterZ;
			clippedVertexToRefCenterX = refCenter.x - clippedVertexX;
			clippedVertexToRefCenterY = refCenter.y - clippedVertexY;
			clippedVertexToRefCenterZ = refCenter.z - clippedVertexZ;
			float depth = clippedVertexToRefCenterX * refNormal.x + clippedVertexToRefCenterY * refNormal.y + clippedVertexToRefCenterZ * refNormal.z;
			float clippedVertexOnRefFaceX;
			float clippedVertexOnRefFaceY;
			float clippedVertexOnRefFaceZ;
			clippedVertexOnRefFaceX = clippedVertexX + refNormal.x * depth;
			clippedVertexOnRefFaceY = clippedVertexY + refNormal.y * depth;
			clippedVertexOnRefFaceZ = clippedVertexZ + refNormal.z * depth;
			if(depth > -Setting.contactPersistenceThreshold) {
				if(swapped) {
					this.addPoint(result,clippedVertexX,clippedVertexY,clippedVertexZ,clippedVertexOnRefFaceX,clippedVertexOnRefFaceY,clippedVertexOnRefFaceZ,depth,i);
				} else {
					this.addPoint(result,clippedVertexOnRefFaceX,clippedVertexOnRefFaceY,clippedVertexOnRefFaceZ,clippedVertexX,clippedVertexY,clippedVertexZ,depth,i);
				}
			}
		}
	}

	/**
	 * Returns half of the projected length of the box with scaled bases
	 * (`sx`, `sy`, `sz`) onto the normalized axis `axis`.
	 */
	private float project(Vec3 axis, Vec3 sx, Vec3 sy, Vec3 sz) {
		float dx = M.vec3_dot(axis, sx);
		float dy = M.vec3_dot(axis, sy);
		float dz = M.vec3_dot(axis, sz);
		if (dx < 0)
			dx = -dx;
		if (dy < 0)
			dy = -dy;
		if (dz < 0)
			dz = -dz;
		return dx + dy + dz;
	}

	/**
	 * 2D version of `project`.
	 */
	private float project2(Vec3 axis, Vec3 sx, Vec3 sy) {
		float dx = M.vec3_dot(axis, sx);
		float dy = M.vec3_dot(axis, sy);
		if (dx < 0)
			dx = -dx;
		if (dy < 0)
			dy = -dy;
		return dx + dy;
	}
	
	
	private void supportingVertexRect(Vec3 out, Vec3 halfExtX,Vec3 halfExtY, Vec3 axis) {
		boolean signX = M.vec3_dot(halfExtX, axis) > 0;
		boolean signY = M.vec3_dot(halfExtY, axis) > 0;
		if (signX) {
			if (signY) {
				M.vec3_mix2(out, halfExtX, halfExtY, 1, 1);
			} else {
				M.vec3_mix2(out, halfExtX, halfExtY, 1, -1);
			}
		} else {
			if (signY) {
				M.vec3_mix2(out, halfExtX, halfExtY, -1, 1);
			} else {
				M.vec3_mix2(out, halfExtX, halfExtY, -1, -1);
			}
		}
	}
	
	private void getBoxFace(Vec3 v1, Vec3 v2,Vec3 v3,Vec3 v4,Vec3 basisX,Vec3 basisY,Vec3 basisZ, final String face) {
		switch (face) {
		case "x+":
				M.vec3_mix3(v1, basisX, basisY, basisZ,  1,  1,  1);
				M.vec3_mix3(v2, basisX, basisY, basisZ,  1, -1,  1);
				M.vec3_mix3(v3, basisX, basisY, basisZ,  1, -1, -1);
				M.vec3_mix3(v4, basisX, basisY, basisZ,  1,  1, -1);
				break;
		case "x-":
			
				M.vec3_mix3(v1, basisX, basisY, basisZ, -1,  1,  1);
				M.vec3_mix3(v2, basisX, basisY, basisZ, -1,  1, -1);
				M.vec3_mix3(v3, basisX, basisY, basisZ, -1, -1, -1);
				M.vec3_mix3(v4, basisX, basisY, basisZ, -1, -1,  1);
				break;
		case "y+":
			
				M.vec3_mix3(v1, basisX, basisY, basisZ,  1,  1,  1);
				M.vec3_mix3(v2, basisX, basisY, basisZ,  1,  1, -1);
				M.vec3_mix3(v3, basisX, basisY, basisZ, -1,  1, -1);
				M.vec3_mix3(v4, basisX, basisY, basisZ, -1,  1,  1);
				break;
		case "y-":
			
				M.vec3_mix3(v1, basisX, basisY, basisZ,  1, -1,  1);
				M.vec3_mix3(v2, basisX, basisY, basisZ, -1, -1,  1);
				M.vec3_mix3(v3, basisX, basisY, basisZ, -1, -1, -1);
				M.vec3_mix3(v4, basisX, basisY, basisZ,  1, -1, -1);
				break;
		case "z+":
			
				M.vec3_mix3(v1, basisX, basisY, basisZ,  1,  1,  1);
				M.vec3_mix3(v2, basisX, basisY, basisZ, -1,  1,  1);
				M.vec3_mix3(v3, basisX, basisY, basisZ, -1, -1,  1);
				M.vec3_mix3(v4, basisX, basisY, basisZ,  1, -1,  1);
				break;
		case "z-":
				M.vec3_mix3(v1, basisX, basisY, basisZ,  1,  1, -1);
				M.vec3_mix3(v2, basisX, basisY, basisZ,  1, -1, -1);
				M.vec3_mix3(v3, basisX, basisY, basisZ, -1, -1, -1);
				M.vec3_mix3(v4, basisX, basisY, basisZ, -1,  1, -1);
				break;
		default:
			M.error("invalid face: " + face );
		}
	}
	
	private class IncidentVertex {
		// projected coord
		public float x;
		public float y;

		// world coord
		public float wx;
		public float wy;
		public float wz;

		public IncidentVertex() {
			x = 0;
			y = 0;
			wx = 0;
			wy = 0;
			wz = 0;
		}

		public void init(float x, float y, float wx,float wy,float wz) {
			this.x = x;
			this.y = y;
			this.wx = wx;
			this.wy = wy;
			this.wz = wz;
		}

		 public void copyFrom(IncidentVertex v) {
			x = v.x;
			y = v.y;
			wx = v.wx;
			wy = v.wy;
			wz = v.wz;
		}

		 public void interp(IncidentVertex v1, IncidentVertex v2, float t) {
			x = v1.x + (v2.x - v1.x) * t;
			y = v1.y + (v2.y - v1.y) * t;
			wx = v1.wx + (v2.wx - v1.wx) * t;
			wy = v1.wy + (v2.wy - v1.wy) * t;
			wz = v1.wz + (v2.wz - v1.wz) * t;
		}
	}

	private  class FaceClipper {
		public float w;
		public float h;
		public int numVertices;
		public IncidentVertex[] vertices;

		int numTmpVertices;
		IncidentVertex[]  tmpVertices;

		public FaceClipper() {
			w = 0;
			h = 0;
			numVertices = 0;
			numTmpVertices = 0;
			vertices = new IncidentVertex[8];
			tmpVertices = new IncidentVertex[8];
			for (int i=0;i<8;i++) {
				vertices[i] = new IncidentVertex();
				tmpVertices[i] = new IncidentVertex();
			}
		}

		 public void init(float w, float h) {
			this.w = w;
			this.h = h;
			numVertices = 0;
			numTmpVertices = 0;
		}

		 public void addIncidentVertex(float x, float y, float wx, float wy, float wz) {
			vertices[numVertices++].init(x, y, wx, wy, wz);
		}

		/**
		 * Clips the incident face by the reference face, generates up to eight vertices.
		 */
		public void clip() {
			clipL();
			flip();
			clipR();
			flip();
			clipT();
			flip();
			clipB();
			flip();
		}

		/**
		 * Reduces vertices up to four.
		 */
		public void reduce() {
			if (numVertices < 4)
				return;

			// TODO: maximize area
			float max1 = MathUtil.NEGATIVE_INFINITY;
			float min1 = MathUtil.POSITIVE_INFINITY;
			float max2 = MathUtil.NEGATIVE_INFINITY;
			float min2 = MathUtil.POSITIVE_INFINITY;
			IncidentVertex max1V = null;
			IncidentVertex min1V = null;
			IncidentVertex max2V = null;
			IncidentVertex min2V = null;
			float e1x = 1;
			float e1y = 1;
			float e2x = -1;
			float e2y = 1;

			for (int i=0;i<numVertices;i++) {
				IncidentVertex v = vertices[i];
				float dot1 = v.x * e1x + v.y * e1y;
				float dot2 = v.x * e2x + v.y * e2y;
				if (i == 0) { // issue #32
					max1 = dot1;
					max1V = v;
					min1 = dot1;
					min1V = v;
					max2 = dot2;
					max2V = v;
					min2 = dot2;
					min2V = v;
				} else {
					if (dot1 > max1) {
						max1 = dot1;
						max1V = v;
					}
					if (dot1 < min1) {
						min1 = dot1;
						min1V = v;
					}
					if (dot2 > max2) {
						max2 = dot2;
						max2V = v;
					}
					if (dot2 < min2) {
						min2 = dot2;
						min2V = v;
					}
				}
			}

			add(max1V);
			add(max2V);
			add(min1V);
			add(min2V);
			flip();
		}

		protected void clipL() {
			for (int i=0;i<numVertices;i++) {
				IncidentVertex v1 = vertices[i];
				IncidentVertex v2 = vertices[(i + 1) % numVertices];
				float s1 = w + v1.x;
				float s2 = w + v2.x;
				clipWithParam(v1, v2, s1, s2);
			}
		}

		protected void clipR() {
			for (int i=0;i<numVertices;i++) {
				IncidentVertex v1 = vertices[i];
				IncidentVertex v2 = vertices[(i + 1) % numVertices];
				float s1 = w - v1.x;
				float s2 = w - v2.x;
				clipWithParam(v1, v2, s1, s2);
			}
		}

		protected void clipT() {
			for (int i=0;i<numVertices;i++) {
				IncidentVertex v1 = vertices[i];
				IncidentVertex v2 = vertices[(i + 1) % numVertices];
				float s1 = h + v1.y;
				float s2 = h + v2.y;
				clipWithParam(v1, v2, s1, s2);
			}
		}

		protected void  clipB() {
			for (int i=0;i<numVertices;i++) {
				IncidentVertex v1 = vertices[i];
				IncidentVertex v2 = vertices[(i + 1) % numVertices];
				float s1 = h - v1.y;
				float s2 = h - v2.y;
				clipWithParam(v1, v2, s1, s2);
			}
		}

		protected void   flip() {
			//swap vertices and tmovertices
			IncidentVertex[] tmp=vertices;
			vertices=tmpVertices;
			tmpVertices=tmp;
			numVertices=numTmpVertices;
			numTmpVertices = 0;
		}

		protected void  clipWithParam(IncidentVertex v1, IncidentVertex v2, float s1, float s2) {
			if (s1 > 0 && s2 > 0) {
				add(v1);
			} else if (s1 > 0 && s2 <= 0) {
				// v2 is clipped
				add(v1);
				interp(v1, v2, s1 / (s1 - s2));
			} else if (s1 <= 0 && s2 > 0) {
				// v1 is clipped
				interp(v1, v2, s1 / (s1 - s2));
			}
		}

		protected void  add(IncidentVertex v) {
			tmpVertices[numTmpVertices++].copyFrom(v);
		}

		protected void  interp(IncidentVertex v1, IncidentVertex v2, float t) {
			tmpVertices[numTmpVertices++].interp(v1, v2, t);
		}
	}
}


