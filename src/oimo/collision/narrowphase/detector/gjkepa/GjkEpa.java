package oimo.collision.narrowphase.detector.gjkepa;
import oimo.collision.geometry.*;
import oimo.collision.narrowphase.detector.*;
import oimo.common.M;
import oimo.common.Mat3;
import oimo.common.Transform;
import oimo.common.Vec3;

/**
 * GJK algorithm and EPA for narrow-phase collision detection.
 */
public class GjkEpa {
	ConvexGeometry c1;
	ConvexGeometry c2;
	Transform tf1;
	Transform tf2;

	public static GjkEpa instance = new GjkEpa();

	// ------------------------------------------------------------- for GJK

	// simplex
	Vec3[] s;
	int simplexSize;

	// witness points
	Vec3[] w1;
	Vec3[] w2;

	Vec3[] tempVec3s;
	Transform tempTransform;

	// direction
	Vec3 dir;

	// closest point
	Vec3 closest;

	// base directions used to expand simplex
	Vec3[] baseDirs;

	// for convex casting
	Vec3 tl1;
	Vec3 tl2;
	Vec3 rayX;
	Vec3 rayR;

	// ------------------------------------------------------------- for EPA

	float depth;
	EpaPolyhedron polyhedron;

	// ------------------------------------------------------------- public vars

	/**
	 * Computed closest point of the first geometry in world coordinate system.
	 */
	public Vec3 closestPoint1;

	/**
	 * Computed closest point of the second geometry in world coordinate system.
	 */
	public Vec3 closestPoint2;

	/**
	 * Computed distance between two geometries. This value may be negative if two
	 * geometries are overlapping.
	 */
	public float distance;

	/**
	 * Default constructor. Consider using `GjkEpa.getInstance` instead of creating a new
	 * instance.
	 */
	public GjkEpa() {
		s = new Vec3[4];
		w1 = new Vec3[4];
		w2 = new Vec3[4];

		baseDirs = new Vec3[3];
		baseDirs[0] = new Vec3(1, 0, 0);
		baseDirs[1] = new Vec3(0, 1, 0);
		baseDirs[2] = new Vec3(0, 0, 1);

		tl1 = new Vec3();
		tl2 = new Vec3();
		rayX = new Vec3();
		rayR = new Vec3();
		tempTransform = new Transform();

		for (int i=0;i<4;i++) {
			s[i] = new Vec3();
			w1[i] = new Vec3();
			w2[i] = new Vec3();
		}

		dir = new Vec3();
		closest = new Vec3();

		closestPoint1 = new Vec3();
		closestPoint2 = new Vec3();

		polyhedron = new EpaPolyhedron();
	}

	// --- private ---

	public int computeClosestPointsImpl(ConvexGeometry c1, ConvexGeometry c2, Transform tf1, Transform tf2,
			CachedDetectorData cache, boolean useEpa) {
		GjkEpaLog.log("----------- GJK begin -----------");

		this.c1 = c1;
		this.c2 = c2;
		this.tf1 = tf1;
		this.tf2 = tf2;

		// simplex
		Vec3[] s = this.s;

		// witness points
		Vec3[] w1 = this.w1;
		Vec3[] w2 = this.w2;

		Vec3 closest = this.closest;

		Vec3 dir = this.dir;

		if (cache != null) {
			if (cache._gjkCache == null)
				cache._gjkCache = new GjkCache();
			loadCache(cache._gjkCache);
		} else {
			dir.zero();
		}

		if (dir.lengthSq() == 0) {
			// compute the first vertex of the simplex
			float firstDirX = tf2._position.x - tf1._position.x;
			float firstDirY = tf2._position.y - tf1._position.y;
			float firstDirZ = tf2._position.z - tf1._position.z;
			dir.x = firstDirX;
			dir.y = firstDirY;
			dir.z = firstDirZ;
			if (dir.x * dir.x + dir.y * dir.y + dir.z * dir.z < 1e-6) {
				dir.set(1, 0, 0);
			}
		}

		simplexSize = 0;
		computeSupportingVertex();
		simplexSize = 1;

		// loop count and max iteration of the loop
		int count = 0;
		int max = 40;

		float eps = 1e-4f;
		float eps2 = eps * eps;

		while (count < max) {
			// project the origin to the simplex and compute index of voronoi region of the
			// origin.
			int v = 0;
			GjkEpaLog.log("simplex size: $simplexSize");
			GjkEpaLog.log("projecting the origin to the simplex...");
			switch (simplexSize) {
			case 1:
				closest.copyFrom(s[0]);
				// GjkEpaLog.log('${s[0]}');
				v = 1;
			case 2:
				v = SimplexUtil.projectOrigin2(s[0], s[1], closest);
				// GjkEpaLog.log('${s[0]}');
				// GjkEpaLog.log('${s[1]}');
			case 3:
				v = SimplexUtil.projectOrigin3(s[0], s[1], s[2], closest);
				// GjkEpaLog.log('${s[0]}');
				// GjkEpaLog.log('${s[1]}');
				// GjkEpaLog.log('${s[2]}');
			case 4:
				v = SimplexUtil.projectOrigin4(s[0], s[1], s[2], s[3], closest);
				// GjkEpaLog.log('${s[0]}');
				// GjkEpaLog.log('${s[1]}');
				// GjkEpaLog.log('${s[2]}');
				// GjkEpaLog.log('${s[3]}');
				// }

				// check if the origin is touching or inside the simplex
				if (closest.lengthSq() < eps2) {
					if (!useEpa) {
						distance = 0;
						return GjkEpaResultState._SUCCEEDED;
					}

					// make the simplex to be a tetrahedron for EPA computation
					switch (simplexSize) {
					case 1:
						pointToTetrahedron();
						// GjkEpaLog.log("point -> tetrahedron");
					case 2:
						lineToTetrahedron();
						// GjkEpaLog.log("line -> tetrahedron");
					case 3:
						triangleToTetrahedron();
						// GjkEpaLog.log("triangle -> tetrahedron");
					}
					if (simplexSize == 4) {
						int epaState = computeDepth(c1, c2, tf1, tf2, s, w1, w2);
						if (epaState != GjkEpaResultState._SUCCEEDED) {
							distance = 0;
							return epaState;
						}
						distance = -depth;
						return GjkEpaResultState._SUCCEEDED;
					}
					// failed to make a tetrahedron
					distance = 0;
					return GjkEpaResultState._GJK_FAILED_TO_MAKE_TETRAHEDRON;
				}

				GjkEpaLog.log("projected origin: $v");

				// shrink the simplex according to the voronoi index of the origin
				shrinkSimplex(v);

				// compute the next vertex
				dir.copyFrom(closest).negateEq();
				computeSupportingVertex();

				if (dir.lengthSq() < eps2) {
					M.error("!?"); // this should never be happen
					return GjkEpaResultState.EPA_FAILED_TO_ADD_VERTEX;
				}

				float d1 = closest.dot(dir);
				float d2 = s[simplexSize].dot(dir);

				GjkEpaLog.log(
						"n: $simplexSize, prev: $closest, current: ${s[simplexSize]}, dir: $dir, iteration: $count, d2 - d1: ${d2 - d1}");

				if (d2 - d1 < eps2) { // terminate GJK; no improvement
					interpolateClosestPoints();

					GjkEpaLog.log("iteration: " + count);

					distance = closest.length(); // no improvement

					if (cache != null && cache._gjkCache != null) {
						saveCache(cache._gjkCache);
					}

					return GjkEpaResultState._SUCCEEDED;
				}

				simplexSize++;
				count++;
			}

		}
		GjkEpaLog.log("GJK failed: did not converge");
		return GjkEpaResultState._GJK_DID_NOT_CONVERGE;
	}

	// `c1` can be null
	public boolean convexCastImpl(ConvexGeometry c1, ConvexGeometry c2, Transform tf1, Transform tf2, Vec3 tl1, Vec3 tl2, RayCastHit hit) {
		GjkEpaLog.log("----------- GJK convex casting begin -----------");

		this.c1 = c1;
		this.c2 = c2;
		this.tf1 = tf1;
		this.tf2 = tf2;

		// simplex
		Vec3[] s = this.s;

		// witness points
		Vec3[] w1 = this.w1;
		Vec3[] w2 = this.w2;

		Vec3 closest = this.closest;

		Vec3 dir = this.dir;

		// compute the first vertex of the simplex
		Vec3 firstDir=new Vec3();
		M.vec3_sub(firstDir, tf2._position, tf1._position);
		M.vec3_toVec3(dir, firstDir);
		if (dir.lengthSq() < 1e-6) {
			dir.set(1, 0, 0);
		}

		simplexSize = 0;
		computeConvexCastSupportingVertex();
		simplexSize = 1;

		// loop count and max iteration of the loop
		int count = 0;
		int max = 40;

		float lambda = 0.0f;
		Vec3 rayX = this.rayX; // origin
		Vec3 rayR = this.rayR; // relative translation
		rayX.zero();
		rayR.copyFrom(tl2).subEq(tl1);

		float eps = 1e-4f;
		float eps2 = eps * eps;

		while (count < max) {
			// project the origin to the simplex and compute index of voronoi region of the origin.
			int v = 0;
			GjkEpaLog.log("simplex size: $simplexSize");
			GjkEpaLog.log("projecting the origin to the simplex...");
			GjkEpaLog.log("x: $rayX");
			GjkEpaLog.log("lambda: $lambda");
			switch (simplexSize) {
			case 1:
				closest.copyFrom(s[0]);
				//GjkEpaLog.log("${s[0]}");
				v = 1;
			case 2:
				v = SimplexUtil.projectOrigin2(s[0], s[1], closest);
				//GjkEpaLog.log('${s[0]}');
				//GjkEpaLog.log('${s[1]}');
			case 3:
				v = SimplexUtil.projectOrigin3(s[0], s[1], s[2], closest);
				//GjkEpaLog.log('${s[0]}');
				//GjkEpaLog.log('${s[1]}');
				//GjkEpaLog.log('${s[2]}');
			case 4:
				v = SimplexUtil.projectOrigin4(s[0], s[1], s[2], s[3], closest);
				//GjkEpaLog.log('${s[0]}');
				//GjkEpaLog.log('${s[1]}');
				//GjkEpaLog.log('${s[2]}');
				//GjkEpaLog.log('${s[3]}');
			}

			GjkEpaLog.log("projected origin: pos = $closest, voronoi index = $v");

			// shrink the simplex according to the voronoi index of the origin
			shrinkSimplex(v);

			// check if the origin is touching or inside the simplex
			if (closest.lengthSq() < eps2) {
				if (lambda == 0 || simplexSize == 4) {
					GjkEpaLog.log("overlapping... closest: " + closest);
					hit.fraction = lambda;
					return false; // overlapping
				}

				interpolateClosestPoints();

				hit.fraction = lambda;
				hit.normal.copyFrom(dir).normalize(); // previous dir
				hit.position.copyFrom(closestPoint1).addScaledEq(tl1, lambda);
				GjkEpaLog.log("GJK convex cast succeeded");
				return true;
			}

			// compute the next vertex
			dir.copyFrom(closest).negateEq();
			computeConvexCastSupportingVertex();
			s[simplexSize].subEq(rayX); // translate origin

			if (dir.lengthSq() < eps2) {
				//throw M.error("!?"); // this should never be happen
			}

			// n is the normal at the vertex p
			Vec3 p = s[simplexSize];
			Vec3 n = dir;
			//GjkEpaLog.log('new vertex p = $p');
			//GjkEpaLog.log('normal n = $n');
			//GjkEpaLog.log('ray dir r = $rayR');

			// check if a part of the ray can be rejected
			float pn = p.dot(n);
			GjkEpaLog.log("p dot n = $pn");
			if (pn < 0) {
				// check if entire the ray can be rejected
				if (rayR.dot(n) >= 0) {
					GjkEpaLog.log("rejected [0");
					return false;
				}
				float dLambda = pn / rayR.dot(n);
				lambda += dLambda;
				if (lambda >= 1) {
					GjkEpaLog.log("rejected 1]");
					return false;
				}
				GjkEpaLog.log("advanced: " + dLambda);
				rayX.addScaledEq(rayR, dLambda);

				// translate the simplex
				//for (int i =0;in 0...simplexSize + 1) {
				for (int i =0;i<simplexSize + 1;i++) {
					s[i].addScaledEq(rayR, -dLambda);
				}
			} else {
				GjkEpaLog.log("ray did not advance");
			}

			// do not add new vertex to the simplex if already exists
			 boolean duplicate = false;
			for (int i=0;i<simplexSize;i++) {
				float dx = s[i].x - s[simplexSize].x;
				float dy = s[i].y - s[simplexSize].y;
				float dz = s[i].z - s[simplexSize].z;
				if (dx * dx + dy * dy + dz * dz < eps2) {
					duplicate = true;
					GjkEpaLog.log("duplicate vertex ${s[i]} and ${s[simplexSize]}");
					break;
				}
			}
			if (!duplicate) {
				GjkEpaLog.log("added ${s[simplexSize]}");
				simplexSize++;
			}

			count++;

			GjkEpaLog.log("iteration: $count");
		}

		GjkEpaLog.log("GJK convex cast failed: did not converge");
		return false;
	}

	private void interpolateClosestPoints() {
		switch (simplexSize) {
		case 1: {
				closestPoint1.copyFrom(w1[0]);
				closestPoint2.copyFrom(w2[0]);
		}
		case 2: {
			float cX;
			float cY;
			float cZ;
			Vec3 v2 = this.closest;
			cX = v2.x;
			cY = v2.y;
			cZ = v2.z;
			float s0X;
			float s0Y;
			float s0Z;
			float w10X;
			float w10Y;
			float w10Z;
			float w20X;
			float w20Y;
			float w20Z;
			float s1X;
			float s1Y;
			float s1Z;
			float w11X;
			float w11Y;
			float w11Z;
			float w21X;
			float w21Y;
			float w21Z;
			Vec3 v3 = this.s[0];
			s0X = v3.x;
			s0Y = v3.y;
			s0Z = v3.z;
			Vec3 v4 = this.w1[0];
			w10X = v4.x;
			w10Y = v4.y;
			w10Z = v4.z;
			Vec3 v5 = this.w2[0];
			w20X = v5.x;
			w20Y = v5.y;
			w20Z = v5.z;
			Vec3 v6 = this.s[1];
			s1X = v6.x;
			s1Y = v6.y;
			s1Z = v6.z;
			Vec3 v7 = this.w1[1];
			w11X = v7.x;
			w11Y = v7.y;
			w11Z = v7.z;
			Vec3 v8 = this.w2[1];
			w21X = v8.x;
			w21Y = v8.y;
			w21Z = v8.z;
			float s01X;
			float s01Y;
			float s01Z;
			s01X = s1X - s0X;
			s01Y = s1Y - s0Y;
			s01Z = s1Z - s0Z;
			float invDet = s01X * s01X + s01Y * s01Y + s01Z * s01Z;
			if(invDet != 0) {
				invDet = 1 / invDet;
			}
			float s0cX;
			float s0cY;
			float s0cZ;
			s0cX = cX - s0X;
			s0cY = cY - s0Y;
			s0cZ = cZ - s0Z;
			float t = (s0cX * s01X + s0cY * s01Y + s0cZ * s01Z) * invDet;
			float diffX;
			float diffY;
			float diffZ;
			float cp1X;
			float cp1Y;
			float cp1Z;
			float cp2X;
			float cp2Y;
			float cp2Z;
			diffX = w11X - w10X;
			diffY = w11Y - w10Y;
			diffZ = w11Z - w10Z;
			cp1X = w10X + diffX * t;
			cp1Y = w10Y + diffY * t;
			cp1Z = w10Z + diffZ * t;
			diffX = w21X - w20X;
			diffY = w21Y - w20Y;
			diffZ = w21Z - w20Z;
			cp2X = w20X + diffX * t;
			cp2Y = w20Y + diffY * t;
			cp2Z = w20Z + diffZ * t;
			Vec3 v9 = this.closestPoint1;
			v9.x = cp1X;
			v9.y = cp1Y;
			v9.z = cp1Z;
			Vec3 v10 = this.closestPoint2;
			v10.x = cp2X;
			v10.y = cp2Y;
			v10.z = cp2Z;
			break;
		}
		case 3: {
			float cX1;
			float cY1;
			float cZ1;
			Vec3 v11 = this.closest;
			cX1 = v11.x;
			cY1 = v11.y;
			cZ1 = v11.z;
			float s0X1;
			float s0Y1;
			float s0Z1;
			float w10X1;
			float w10Y1;
			float w10Z1;
			float w20X1;
			float w20Y1;
			float w20Z1;
			float s1X1;
			float s1Y1;
			float s1Z1;
			float w11X1;
			float w11Y1;
			float w11Z1;
			float w21X1;
			float w21Y1;
			float w21Z1;
			float s2X;
			float s2Y;
			float s2Z;
			float w12X;
			float w12Y;
			float w12Z;
			float w22X;
			float w22Y;
			float w22Z;
			Vec3 v12 = this.s[0];
			s0X1 = v12.x;
			s0Y1 = v12.y;
			s0Z1 = v12.z;
			Vec3 v13 = this.w1[0];
			w10X1 = v13.x;
			w10Y1 = v13.y;
			w10Z1 = v13.z;
			Vec3 v14 = this.w2[0];
			w20X1 = v14.x;
			w20Y1 = v14.y;
			w20Z1 = v14.z;
			Vec3 v15 = this.s[1];
			s1X1 = v15.x;
			s1Y1 = v15.y;
			s1Z1 = v15.z;
			Vec3 v16 = this.w1[1];
			w11X1 = v16.x;
			w11Y1 = v16.y;
			w11Z1 = v16.z;
			Vec3 v17 = this.w2[1];
			w21X1 = v17.x;
			w21Y1 = v17.y;
			w21Z1 = v17.z;
			Vec3 v18 = this.s[2];
			s2X = v18.x;
			s2Y = v18.y;
			s2Z = v18.z;
			Vec3 v19 = this.w1[2];
			w12X = v19.x;
			w12Y = v19.y;
			w12Z = v19.z;
			Vec3 v20 = this.w2[2];
			w22X = v20.x;
			w22Y = v20.y;
			w22Z = v20.z;
			float s01X1;
			float s01Y1;
			float s01Z1;
			float s02X;
			float s02Y;
			float s02Z;
			float s0cX1;
			float s0cY1;
			float s0cZ1;
			s01X1 = s1X1 - s0X1;
			s01Y1 = s1Y1 - s0Y1;
			s01Z1 = s1Z1 - s0Z1;
			s02X = s2X - s0X1;
			s02Y = s2Y - s0Y1;
			s02Z = s2Z - s0Z1;
			s0cX1 = cX1 - s0X1;
			s0cY1 = cY1 - s0Y1;
			s0cZ1 = cZ1 - s0Z1;
			float d11 = s01X1 * s01X1 + s01Y1 * s01Y1 + s01Z1 * s01Z1;
			float d12 = s01X1 * s02X + s01Y1 * s02Y + s01Z1 * s02Z;
			float d22 = s02X * s02X + s02Y * s02Y + s02Z * s02Z;
			float d1c = s01X1 * s0cX1 + s01Y1 * s0cY1 + s01Z1 * s0cZ1;
			float d2c = s02X * s0cX1 + s02Y * s0cY1 + s02Z * s0cZ1;
			float invDet1 = d11 * d22 - d12 * d12;
			if(invDet1 != 0) {
				invDet1 = 1 / invDet1;
			}
			float s = (d1c * d22 - d2c * d12) * invDet1;
			float t1 = (-d1c * d12 + d2c * d11) * invDet1;
			float diffX1;
			float diffY1;
			float diffZ1;
			float cp1X1;
			float cp1Y1;
			float cp1Z1;
			float cp2X1;
			float cp2Y1;
			float cp2Z1;
			diffX1 = w11X1 - w10X1;
			diffY1 = w11Y1 - w10Y1;
			diffZ1 = w11Z1 - w10Z1;
			cp1X1 = w10X1 + diffX1 * s;
			cp1Y1 = w10Y1 + diffY1 * s;
			cp1Z1 = w10Z1 + diffZ1 * s;
			diffX1 = w12X - w10X1;
			diffY1 = w12Y - w10Y1;
			diffZ1 = w12Z - w10Z1;
			cp1X1 += diffX1 * t1;
			cp1Y1 += diffY1 * t1;
			cp1Z1 += diffZ1 * t1;
			diffX1 = w21X1 - w20X1;
			diffY1 = w21Y1 - w20Y1;
			diffZ1 = w21Z1 - w20Z1;
			cp2X1 = w20X1 + diffX1 * s;
			cp2Y1 = w20Y1 + diffY1 * s;
			cp2Z1 = w20Z1 + diffZ1 * s;
			diffX1 = w22X - w20X1;
			diffY1 = w22Y - w20Y1;
			diffZ1 = w22Z - w20Z1;
			cp2X1 += diffX1 * t1;
			cp2Y1 += diffY1 * t1;
			cp2Z1 += diffZ1 * t1;
			Vec3 v21 = this.closestPoint1;
			v21.x = cp1X1;
			v21.y = cp1Y1;
			v21.z = cp1Z1;
			Vec3 v22 = this.closestPoint2;
			v22.x = cp2X1;
			v22.y = cp2Y1;
			v22.z = cp2Z1;
			break;
		}
		default:
			//throw M.error("!?");
		}
	}

	public void loadCache(GjkCache gjkCache) {
		// copy simplex data from the cache
		dir.copyFrom(gjkCache.prevClosestDir);
	}

	public void  saveCache(GjkCache gjkCache) {
		// set GJK cache for the next computation
		gjkCache.prevClosestDir.copyFrom(closest).negateEq();
	}

	public void  shrinkSimplex(int vertexBits) {
		simplexSize = vertexBits;
		simplexSize = (simplexSize & 5) + (simplexSize >> 1 & 5);
		simplexSize = (simplexSize & 3) + (simplexSize >> 2 & 3);
		switch (vertexBits) {
		//case 0: // do nothing
		//case 1: // do nothing
		case 2: // 0 <- 1
			 s[0].copyFrom( s[1]);
			w1[0].copyFrom(w1[1]);
			w2[0].copyFrom(w2[1]);
		//case 3: // do nothing
		case 4: // 0 <- 2
			 s[0].copyFrom( s[2]);
			w1[0].copyFrom(w1[2]);
			w2[0].copyFrom(w2[2]);
		case 5: // 1 <- 2
			 s[1].copyFrom( s[2]);
			w1[1].copyFrom(w1[2]);
			w2[1].copyFrom(w2[2]);
		case 6: // 0 <- 2
			 s[0].copyFrom( s[2]);
			w1[0].copyFrom(w1[2]);
			w2[0].copyFrom(w2[2]);
		//case 7: // do nothing
		case 8: // 0 <- 3
			 s[0].copyFrom( s[3]);
			w1[0].copyFrom(w1[3]);
			w2[0].copyFrom(w2[3]);
		case 9: // 1 <- 3
			 s[1].copyFrom( s[3]);
			w1[1].copyFrom(w1[3]);
			w2[1].copyFrom(w2[3]);
		case 10: // 0 <- 3
			 s[0].copyFrom( s[3]);
			w1[0].copyFrom(w1[3]);
			w2[0].copyFrom(w2[3]);
		case 11: // 2 <- 3
			 s[2].copyFrom( s[3]);
			w1[2].copyFrom(w1[3]);
			w2[2].copyFrom(w2[3]);
		case 12: // 0 <- 2, 1 <- 3
			 s[0].copyFrom( s[2]);
			w1[0].copyFrom(w1[2]);
			w2[0].copyFrom(w2[2]);
			 s[1].copyFrom( s[3]);
			w1[1].copyFrom(w1[3]);
			w2[1].copyFrom(w2[3]);
		case 13: // 1 <- 3
			 s[1].copyFrom( s[3]);
			w1[1].copyFrom(w1[3]);
			w2[1].copyFrom(w2[3]);
		case 14: // 0 <- 3
			 s[0].copyFrom( s[3]);
			w1[0].copyFrom(w1[3]);
			w2[0].copyFrom(w2[3]);
		//case 15: // do nothing
		}
	}

	public void computeSupportingVertex() {
		computeWitnessPoint1(false);
		computeWitnessPoint2(false);
		s[simplexSize].copyFrom(w1[simplexSize]).subEq(w2[simplexSize]);
	}

	public void  computeConvexCastSupportingVertex() {
		if (c1 != null) {
			computeWitnessPoint1(true);
		} else {
			M.vec3_toVec3(w1[simplexSize], tf1._position);
		}
		computeWitnessPoint2(true);
		s[simplexSize].copyFrom(w1[simplexSize]).subEq(w2[simplexSize]);
	}

	public void  computeWitnessPoint1(boolean addMargin) {
		Vec3 tmp=new Vec3();
		Vec3 idir=new Vec3();
		M.vec3_fromVec3(idir, dir);

		// compute local dir
		Vec3 ldir1=new Vec3();
		M.vec3_mulMat3Transposed(ldir1, idir, tf1._rotation);

		// compute local witness point
		Vec3 iw1=new Vec3();
		M.vec3_toVec3(dir, ldir1);
		c1.computeLocalSupportingVertex(dir, w1[simplexSize]);
		if (addMargin) {
			dir.normalize();
			w1[simplexSize].addScaledEq(dir, c1._gjkMargin);
		}

		// compute world witness point
		M.vec3_fromVec3(tmp, w1[simplexSize]);
		M.vec3_mulMat3(iw1, tmp, tf1._rotation);
		M.vec3_add(iw1, iw1, tf1._position);
		M.vec3_toVec3(w1[simplexSize], iw1);

		dir.set(idir);
	}

	private void computeWitnessPoint2(boolean addMargin) {
		Vec3 tmp=new Vec3();
		Vec3 idir=new Vec3();
		M.vec3_fromVec3(idir, dir);

		// compute local dir
		Vec3 ldir2=new Vec3();
		M.vec3_mulMat3Transposed(ldir2, idir, tf2._rotation);
		//M.vec3_negate(ldir2, ldir2);
		ldir2.negateEq();

		// compute local witness point
		Vec3 iw2 =new Vec3();
		M.vec3_toVec3(dir, ldir2);
		c2.computeLocalSupportingVertex(dir, w2[simplexSize]);
		if (addMargin) {
			dir.normalize();
			w2[simplexSize].addScaledEq(dir, c2._gjkMargin);
		}

		// compute world witness point
		M.vec3_fromVec3(tmp, w2[simplexSize]);
		M.vec3_mulMat3(iw2, tmp, tf2._rotation);
		M.vec3_add(iw2, iw2, tf2._position);
		M.vec3_toVec3(w2[simplexSize], iw2);

		M.vec3_toVec3(dir, idir);
	}

	private void pointToTetrahedron() {
		for (int i=0;i<3;i++) {
			dir.copyFrom(baseDirs[i]);

			computeSupportingVertex();
			simplexSize++;
			lineToTetrahedron();
			if (simplexSize == 4) break;
			simplexSize--;

			dir.negateEq();

			computeSupportingVertex();
			simplexSize++;
			lineToTetrahedron();
			if (simplexSize == 4) break;
			simplexSize--;
		}
	}

	private void lineToTetrahedron() {
		Vec3 oldDir=new Vec3();
		M.vec3_fromVec3(oldDir, dir);

		Vec3 s0=s[0];
		Vec3 s1=s[1];
		Vec3 lineDir=new Vec3();
		M.vec3_fromVec3(s0, s[0]);
		M.vec3_fromVec3(s1, s[1]);
		M.vec3_sub(lineDir, s0, s1);
		for (int i =0;i<3;i++) {
			Vec3 baseDir=baseDirs[i];
			Vec3 newDir=new Vec3();
			M.vec3_cross(newDir, lineDir, baseDir);
			M.vec3_toVec3(dir, newDir);

			computeSupportingVertex();
			simplexSize++;
			triangleToTetrahedron();
			if (simplexSize == 4) break;
			simplexSize--;

			dir.negateEq();

			computeSupportingVertex();
			simplexSize++;
			triangleToTetrahedron();
			if (simplexSize == 4) break;
			simplexSize--;
		}

		M.vec3_toVec3(dir, oldDir);
	}

	private void triangleToTetrahedron() {
		Vec3 oldDir=dir.clone();

		do {
			Vec3 s0=s[0];
			Vec3 s1=s[1];
			Vec3 s2=s[2];
			Vec3 s01=new Vec3();
			Vec3 s02=new Vec3();
			M.vec3_fromVec3(s0, s[0]);
			M.vec3_fromVec3(s1, s[1]);
			M.vec3_fromVec3(s2, s[2]);
			M.vec3_sub(s01, s1, s0);
			M.vec3_sub(s02, s2, s0);
			M.vec3_cross(dir, s01, s02);

			computeSupportingVertex();
			simplexSize++;
			if (isValidTetrahedron()) break;
			simplexSize--;

			dir.negateEq();

			computeSupportingVertex();
			simplexSize++;
			if (isValidTetrahedron()) break;
			simplexSize--;
		} while (false);

		M.vec3_toVec3(dir, oldDir);
	}

	/*inline*/ 
	private boolean isValidTetrahedron() {
		float e00 = s[1].x - s[0].x;
		float e01 = s[1].y - s[0].y;
		float e02 = s[1].z - s[0].z;
		float e10 = s[2].x - s[0].x;
		float e11 = s[2].y - s[0].y;
		float e12 = s[2].z - s[0].z;
		float e20 = s[3].x - s[0].x;
		float e21 = s[3].y - s[0].y;
		float e22 = s[3].z - s[0].z;
		float det = e00 * (e11 * e22 - e12 * e21) - e01 * (e10 * e22 - e12 * e20) + e02 * (e10 * e21 - e11 * e20);
		return det > 1e-12 || det < -1e-12;
	}

	// EPA

	public int computeDepth(ConvexGeometry convex1, ConvexGeometry convex2, Transform tf1, Transform tf2, Vec3[] initialPolyhedron, Vec3[] initialPolyhedron1, Vec3[] initialPolyhedron2) {
		GjkEpaLog.log("----------- EPA begin ----------- ");

		EpaPolyhedron _this = this.polyhedron;
		while(_this._numTriangles > 0) {
			EpaTriangle t = _this._triangleList;
			_this._numTriangles--;
			EpaTriangle prev = t._prev;
			EpaTriangle next = t._next;
			if(prev != null) {
				prev._next = next;
			}
			if(next != null) {
				next._prev = prev;
			}
			if(t == _this._triangleList) {
				_this._triangleList = _this._triangleList._next;
			}
			if(t == _this._triangleListLast) {
				_this._triangleListLast = _this._triangleListLast._prev;
			}
			t._next = null;
			t._prev = null;
			t.removeReferences();
			t._next = _this._trianglePool;
			_this._trianglePool = t;
		}
		while(_this._numVertices > 0) {
			EpaVertex v = _this._vertices[--_this._numVertices];
			v.removeReferences();
			v._next = _this._vertexPool;
			_this._vertexPool = v;
		}
		EpaPolyhedron tmp = this.polyhedron;
		EpaPolyhedron _this1 = this.polyhedron;
		EpaVertex first = _this1._vertexPool;
		if(first != null) {
			_this1._vertexPool = first._next;
			first._next = null;
		} else {
			first = new oimo.collision.narrowphase.detector.gjkepa.EpaVertex();
		}
		EpaVertex tmp1 = first.init(initialPolyhedron[0],initialPolyhedron1[0],initialPolyhedron2[0]);
		EpaPolyhedron _this2 = this.polyhedron;
		EpaVertex first1 = _this2._vertexPool;
		if(first1 != null) {
			_this2._vertexPool = first1._next;
			first1._next = null;
		} else {
			first1 = new oimo.collision.narrowphase.detector.gjkepa.EpaVertex();
		}
		EpaVertex tmp2 = first1.init(initialPolyhedron[1],initialPolyhedron1[1],initialPolyhedron2[1]);
		EpaPolyhedron _this3 = this.polyhedron;
		EpaVertex first2 = _this3._vertexPool;
		if(first2 != null) {
			_this3._vertexPool = first2._next;
			first2._next = null;
		} else {
			first2 = new oimo.collision.narrowphase.detector.gjkepa.EpaVertex();
		}
		EpaVertex tmp3 = first2.init(initialPolyhedron[2],initialPolyhedron1[2],initialPolyhedron2[2]);
		EpaPolyhedron _this4 = this.polyhedron;
		EpaVertex first3 = _this4._vertexPool;
		if(first3 != null) {
			_this4._vertexPool = first3._next;
			first3._next = null;
		} else {
			first3 = new oimo.collision.narrowphase.detector.gjkepa.EpaVertex();
		}
		if(!tmp._init(tmp1,tmp2,tmp3,first3.init(initialPolyhedron[3],initialPolyhedron1[3],initialPolyhedron2[3]))) {
			return oimo.collision.narrowphase.detector.gjkepa.GjkEpaResultState.EPA_FAILED_TO_INIT;
		}
		simplexSize = 0;
		Vec3 supportingVertex = s[0];
		Vec3 witness1 = w1[0];
		Vec3 witness2 = w2[0];

		int count = 0;
		int maxIterations = 40;
		while (count < maxIterations) {
			EpaTriangle face = polyhedron._getBestTriangle();

			GjkEpaLog.log("nearest face:");
			//GjkEpaLog.run(face.dump());

			dir.copyFrom(face._normal).normalize();
			computeSupportingVertex();

			EpaVertex v0 = face._vertices[0];
			EpaVertex v1 = face._vertices[1];
			EpaVertex v2 = face._vertices[2];

			float dot1 = v0.v.dot(dir);
			float dot2 = supportingVertex.dot(dir);

			GjkEpaLog.log("got new vertex: " + supportingVertex);
			GjkEpaLog.log("improvement: " + dot1 + " -> " + dot2 + ", normal: " + dir.toString());

			if (dot2 - dot1 < 1e-6 || count == maxIterations - 1) { // no improvement
				closest.copyFrom(dir).scaleEq(dir.dot(v0.v) / dir.lengthSq());

				Vec3 c=new Vec3();
				M.vec3_fromVec3(c, closest);
				Vec3 s0=v0.v; Vec3 w10=v0.w1; Vec3 w20=v0.w2;
				Vec3 s1=v1.v; Vec3 w11=v1.w1; Vec3 w21=v1.w2;
				Vec3 s2=v2.v; Vec3 w12=v2.w1; Vec3 w22=v2.w2;
//				
//				M.vec3_fromVec3(s0, v0.v); M.vec3_fromVec3(w10, v0.w1); M.vec3_fromVec3(w20, v0.w2);
//				M.vec3_fromVec3(s1, v1.v); M.vec3_fromVec3(w11, v1.w1); M.vec3_fromVec3(w21, v1.w2);
//				M.vec3_fromVec3(s2, v2.v); M.vec3_fromVec3(w12, v2.w1); M.vec3_fromVec3(w22, v2.w2);

				Vec3 s01=new Vec3();
				Vec3 s02=new Vec3();
				Vec3 s0c=new Vec3();
				M.vec3_sub(s01, s1, s0);
				M.vec3_sub(s02, s2, s0);
				M.vec3_sub(s0c, c, s0);

				float d11 = M.vec3_dot(s01, s01);
				float d12 = M.vec3_dot(s01, s02);
				float d22 = M.vec3_dot(s02, s02);
				float d1c = M.vec3_dot(s01, s0c);
				float d2c = M.vec3_dot(s02, s0c);
				float invDet = d11 * d22 - d12 * d12;
				if (invDet != 0) invDet = 1 / invDet;
				float s = (d1c * d22 - d2c * d12) * invDet;
				float t = (-d1c * d12 + d2c * d11) * invDet;

				// compute closest points
				Vec3 diff=new Vec3();
				Vec3 cp1=new Vec3();
				Vec3 cp2=new Vec3();

				M.vec3_sub(diff, w11, w10);
				M.vec3_addRhsScaled(cp1, w10, diff, s);
				M.vec3_sub(diff, w12, w10);
				M.vec3_addRhsScaled(cp1, cp1, diff, t);

				M.vec3_sub(diff, w21, w20);
				M.vec3_addRhsScaled(cp2, w20, diff, s);
				M.vec3_sub(diff, w22, w20);
				M.vec3_addRhsScaled(cp2, cp2, diff, t);

				M.vec3_toVec3(closestPoint1, cp1);
				M.vec3_toVec3(closestPoint2, cp2);

				depth = closest.length();
				return GjkEpaResultState.SUCCEEDED;
			}
			EpaVertex epaVertex = polyhedron._pickVertex().init(supportingVertex, witness1, witness2);
			if (!polyhedron._addVertex(epaVertex, face)) {

				GjkEpaLog.log("EPA failed at vertex addition: " + polyhedron._status);
				//GjkEpaLog.run(polyhedron._dumpAsObjModel());

				return GjkEpaResultState.EPA_FAILED_TO_ADD_VERTEX;
			}
			count++;
		}

		GjkEpaLog.log("EPA failed: did not converge.");
		//GjkEpaLog.run(polyhedron._dumpAsObjModel());

		return GjkEpaResultState.EPA_DID_NOT_CONVERGE;
	}

	// --- public ---

	/**
	 * Returns an instance of `GjkEpa`.
	 */
	public static GjkEpa  getInstance() {
		return instance;
	}

	/**
	 * Computes the closest points of two convex geometries `c1` and `c2` with transforms `tf1` and `tf2`
	 * respectively, and returns the status of the result (see `GjkEpaResultState` for details). If cached
	 * data `cache` is not `null`, this tries to exploit the previous result in `cache` to improve performance,
	 * and stores the new result to `cache`.
	 *
	 * Set the compiler option `OIMO_GJK_EPA_DEBUG` for debugging (warning: massive logging).
	 */
	public int  computeClosestPoints(ConvexGeometry c1, ConvexGeometry c2, Transform tf1, Transform tf2, CachedDetectorData cache) {
		return computeClosestPointsImpl(c1, c2, tf1, tf2, cache, true);
	}

	/**
	 * Computes the distance between two convex geometries `c1` and `c2` with transforms `tf1` and `tf2`
	 * respectively, and returns the status of the result (see `GjkEpaResultState` for details). Different
	 * from `GjkEpa.computeClosestPoints`, this does not compute negative distances and closest points if
	 * two geometries are overlapping. If cached data `cache` is not `null`, this tries to exploit the
	 * previous result in `cache` to improve performance, and stores the new result to `cache`.
	 *
	 * Set the compiler option `OIMO_GJK_EPA_DEBUG` for debugging (warning: massive logging).
	 */
	public int computeDistance(ConvexGeometry c1,ConvexGeometry c2, Transform tf1, Transform tf2, CachedDetectorData cache) {
		return computeClosestPointsImpl(c1, c2, tf1, tf2, cache, false);
	}

	/**
	 * Performs a convex casting between `c1` and `c2`. Returns `true` and sets the result information
	 * to `hit` if the convex geometries intersect. Each convex geometries translates by `tl1` and `tl2`,
	 * starting from the beginning transforms `tf1` and `tf2` respectively.
	 *
	 * Set the compiler option `OIMO_GJK_EPA_DEBUG` for debugging (warning: massive logging).
	 */
	public boolean convexCast(ConvexGeometry c1, ConvexGeometry c2, Transform tf1, Transform tf2, Vec3 tl1, Vec3 tl2, RayCastHit hit) {
		return convexCastImpl(c1, c2, tf1, tf2, tl1, tl2, hit);
	}

	/**
	 * Performs ray cansting against the convex geometry `c` with transform `tf`. Returns `true` and sets
	 * the result information to `hit` if the line segment from `begin` to `end` intersects the convex
	 * geometry. Otherwise returns `false`.
	 *
	 * Set the compiler option `OIMO_GJK_EPA_DEBUG` for debugging (warning: massive logging).
	 */
	public boolean rayCast(ConvexGeometry c, Transform tf, Vec3 begin, Vec3 end, RayCastHit hit) {
		Transform tf1 = tempTransform;
		Transform tf2 = tf;
		tf1._position.x = begin.x;
		tf1._position.y = begin.y;
		tf1._position.z = begin.z;

		Vec3 tl1 = this.tl1;
		Vec3 tl2 = this.tl2;

		tl1.copyFrom(end).subEq(begin);
		tl2.zero();

		return convexCastImpl(null, c, tf1, tf2, tl1, tl2, hit);
	}

}