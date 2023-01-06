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

	double depth;
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
	public double distance;

	/**
	 * Default constructor. Consider using `GjkEpa.getInstance` instead of creating
	 * a new instance.
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

		for (int i = 0; i < 4; i++) {
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
		this.c1 = c1;
		this.c2 = c2;
		this.tf1 = tf1;
		this.tf2 = tf2;
		Vec3[] s = this.s;
		Vec3[] w1 = this.w1;
		Vec3[] w2 = this.w2;
		Vec3 closest = this.closest;
		Vec3 dir = this.dir;
		if(cache != null) {
			if(cache._gjkCache == null) {
				cache._gjkCache = new oimo.collision.narrowphase.detector.gjkepa.GjkCache();
			}
			this.loadCache(cache._gjkCache);
		} else {
			dir.zero();
		}
		if(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z == 0) {
			double firstDirX;
			double firstDirY;
			double firstDirZ;
			firstDirX = tf2._position.x - tf1._position.x;
			firstDirY = tf2._position.y - tf1._position.y;
			firstDirZ = tf2._position.z - tf1._position.z;
			dir.x = firstDirX;
			dir.y = firstDirY;
			dir.z = firstDirZ;
			if(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z < 1e-6) {
				dir.set(1,0,0);
			}
		}
		this.simplexSize = 0;
		this.computeWitnessPoint1(false);
		this.computeWitnessPoint2(false);
		Vec3 _this = this.s[this.simplexSize];
		Vec3 _v = this.w1[this.simplexSize];
		_this.x = _v.x;
		_this.y = _v.y;
		_this.z = _v.z;
		Vec3 _v1 = this.w2[this.simplexSize];
		_this.x -= _v1.x;
		_this.y -= _v1.y;
		_this.z -= _v1.z;
		this.simplexSize = 1;
		int count = 0;
		while(count < 40) {
			int v = 0;
			switch(this.simplexSize) {
			case 1:
				Vec3 v1 = s[0];
				closest.x = v1.x;
				closest.y = v1.y;
				closest.z = v1.z;
				v = 1;
				break;
			case 2:
				double v1X;
				double v1Y;
				double v1Z;
				double v2X;
				double v2Y;
				double v2Z;
				Vec3 v2 = s[0];
				v1X = v2.x;
				v1Y = v2.y;
				v1Z = v2.z;
				Vec3 v3 = s[1];
				v2X = v3.x;
				v2Y = v3.y;
				v2Z = v3.z;
				double v12X;
				double v12Y;
				double v12Z;
				v12X = v2X - v1X;
				v12Y = v2Y - v1Y;
				v12Z = v2Z - v1Z;
				double t = v12X * v1X + v12Y * v1Y + v12Z * v1Z;
				t = -t / (v12X * v12X + v12Y * v12Y + v12Z * v12Z);
				if(t < 0) {
					closest.x = v1X;
					closest.y = v1Y;
					closest.z = v1Z;
					v = 1;
				} else if(t > 1) {
					closest.x = v2X;
					closest.y = v2Y;
					closest.z = v2Z;
					v = 2;
				} else {
					double pX;
					double pY;
					double pZ;
					pX = v1X + v12X * t;
					pY = v1Y + v12Y * t;
					pZ = v1Z + v12Z * t;
					closest.x = pX;
					closest.y = pY;
					closest.z = pZ;
					v = 3;
				}
				break;
			case 3:
				Vec3 vec1 = s[0];
				Vec3 vec2 = s[1];
				Vec3 vec3 = s[2];
				double v1X1;
				double v1Y1;
				double v1Z1;
				double v2X1;
				double v2Y1;
				double v2Z1;
				double v3X;
				double v3Y;
				double v3Z;
				double v12X1;
				double v12Y1;
				double v12Z1;
				double v23X;
				double v23Y;
				double v23Z;
				double v31X;
				double v31Y;
				double v31Z;
				v1X1 = vec1.x;
				v1Y1 = vec1.y;
				v1Z1 = vec1.z;
				v2X1 = vec2.x;
				v2Y1 = vec2.y;
				v2Z1 = vec2.z;
				v3X = vec3.x;
				v3Y = vec3.y;
				v3Z = vec3.z;
				v12X1 = v2X1 - v1X1;
				v12Y1 = v2Y1 - v1Y1;
				v12Z1 = v2Z1 - v1Z1;
				v23X = v3X - v2X1;
				v23Y = v3Y - v2Y1;
				v23Z = v3Z - v2Z1;
				v31X = v1X1 - v3X;
				v31Y = v1Y1 - v3Y;
				v31Z = v1Z1 - v3Z;
				double nX;
				double nY;
				double nZ;
				nX = v12Y1 * v23Z - v12Z1 * v23Y;
				nY = v12Z1 * v23X - v12X1 * v23Z;
				nZ = v12X1 * v23Y - v12Y1 * v23X;
				double n12X;
				double n12Y;
				double n12Z;
				double n23X;
				double n23Y;
				double n23Z;
				double n31X;
				double n31Y;
				double n31Z;
				n12X = v12Y1 * nZ - v12Z1 * nY;
				n12Y = v12Z1 * nX - v12X1 * nZ;
				n12Z = v12X1 * nY - v12Y1 * nX;
				n23X = v23Y * nZ - v23Z * nY;
				n23Y = v23Z * nX - v23X * nZ;
				n23Z = v23X * nY - v23Y * nX;
				n31X = v31Y * nZ - v31Z * nY;
				n31Y = v31Z * nX - v31X * nZ;
				n31Z = v31X * nY - v31Y * nX;
				double mind = -1;
				double minvX;
				double minvY;
				double minvZ;
				int mini = 0;
				minvX = 0;
				minvY = 0;
				minvZ = 0;
				if(v1X1 * n12X + v1Y1 * n12Y + v1Z1 * n12Z < 0) {
//					double v1X;
//					double v1Y;
//					double v1Z;
//					double v2X;
//					double v2Y;
//					double v2Z;
					v1X = vec1.x;
					v1Y = vec1.y;
					v1Z = vec1.z;
					v2X = vec2.x;
					v2Y = vec2.y;
					v2Z = vec2.z;
//					double v12X;
//					double v12Y;
//					double v12Z;
					v12X = v2X - v1X;
					v12Y = v2Y - v1Y;
					v12Z = v2Z - v1Z;
					 t = v12X * v1X + v12Y * v1Y + v12Z * v1Z;
					t = -t / (v12X * v12X + v12Y * v12Y + v12Z * v12Z);
					int b;
					if(t < 0) {
						closest.x = v1X;
						closest.y = v1Y;
						closest.z = v1Z;
						b = 1;
					} else if(t > 1) {
						closest.x = v2X;
						closest.y = v2Y;
						closest.z = v2Z;
						b = 2;
					} else {
						double pX;
						double pY;
						double pZ;
						pX = v1X + v12X * t;
						pY = v1Y + v12Y * t;
						pZ = v1Z + v12Z * t;
						closest.x = pX;
						closest.y = pY;
						closest.z = pZ;
						b = 3;
					}
					mini = b;
					mind = closest.x * closest.x + closest.y * closest.y + closest.z * closest.z;
					minvX = closest.x;
					minvY = closest.y;
					minvZ = closest.z;
				}
				if(v2X1 * n23X + v2Y1 * n23Y + v2Z1 * n23Z < 0) {
//					let v1X;
//					let v1Y;
//					let v1Z;
//					let v2X;
//					let v2Y;
//					let v2Z;
					v1X = vec2.x;
					v1Y = vec2.y;
					v1Z = vec2.z;
					v2X = vec3.x;
					v2Y = vec3.y;
					v2Z = vec3.z;
//					let v12X;
//					let v12Y;
//					let v12Z;
					v12X = v2X - v1X;
					v12Y = v2Y - v1Y;
					v12Z = v2Z - v1Z;
					 t = v12X * v1X + v12Y * v1Y + v12Z * v1Z;
					t = -t / (v12X * v12X + v12Y * v12Y + v12Z * v12Z);
					int b;
					if(t < 0) {
						closest.x = v1X;
						closest.y = v1Y;
						closest.z = v1Z;
						b = 1;
					} else if(t > 1) {
						closest.x = v2X;
						closest.y = v2Y;
						closest.z = v2Z;
						b = 2;
					} else {
						double pX;
						double pY;
						double pZ;
						pX = v1X + v12X * t;
						pY = v1Y + v12Y * t;
						pZ = v1Z + v12Z * t;
						closest.x = pX;
						closest.y = pY;
						closest.z = pZ;
						b = 3;
					}
					double d = closest.x * closest.x + closest.y * closest.y + closest.z * closest.z;
					if(mind < 0 || d < mind) {
						mini = b << 1;
						mind = d;
						minvX = closest.x;
						minvY = closest.y;
						minvZ = closest.z;
					}
				}
				if(v3X * n31X + v3Y * n31Y + v3Z * n31Z < 0) {
//					double v1X;
//					double v1Y;
//					double v1Z;
//					double v2X;
//					double v2Y;
//					double v2Z;
					v1X = vec1.x;
					v1Y = vec1.y;
					v1Z = vec1.z;
					v2X = vec3.x;
					v2Y = vec3.y;
					v2Z = vec3.z;
//					double v12X;
//					double v12Y;
//					double v12Z;
					v12X = v2X - v1X;
					v12Y = v2Y - v1Y;
					v12Z = v2Z - v1Z;
					 t = v12X * v1X + v12Y * v1Y + v12Z * v1Z;
					t = -t / (v12X * v12X + v12Y * v12Y + v12Z * v12Z);
					int b;
					if(t < 0) {
						closest.x = v1X;
						closest.y = v1Y;
						closest.z = v1Z;
						b = 1;
					} else if(t > 1) {
						closest.x = v2X;
						closest.y = v2Y;
						closest.z = v2Z;
						b = 2;
					} else {
						double pX;
						double pY;
						double pZ;
						pX = v1X + v12X * t;
						pY = v1Y + v12Y * t;
						pZ = v1Z + v12Z * t;
						closest.x = pX;
						closest.y = pY;
						closest.z = pZ;
						b = 3;
					}
					double d = closest.x * closest.x + closest.y * closest.y + closest.z * closest.z;
					if(mind < 0 || d < mind) {
						mini = b & 1 | (b & 2) << 1;
						mind = d;
						minvX = closest.x;
						minvY = closest.y;
						minvZ = closest.z;
					}
				}
				if(mind > 0) {
					closest.x = minvX;
					closest.y = minvY;
					closest.z = minvZ;
					v = mini;
				} else {
					double l = nX * nX + nY * nY + nZ * nZ;
					if(l > 0) {
						l = 1 / Math.sqrt(l);
					}
					nX *= l;
					nY *= l;
					nZ *= l;
					double l2 = nX * nX + nY * nY + nZ * nZ;
					l2 = (v1X1 * nX + v1Y1 * nY + v1Z1 * nZ) / l2;
					minvX = nX * l2;
					minvY = nY * l2;
					minvZ = nZ * l2;
					closest.x = minvX;
					closest.y = minvY;
					closest.z = minvZ;
					v = 7;
				}
				break;
			case 4:
				Vec3 vec11 = s[0];
				Vec3 vec21 = s[1];
				Vec3 vec31 = s[2];
				Vec3 vec4 = s[3];
				double v1X2;
				double v1Y2;
				double v1Z2;
				double v2X2;
				double v2Y2;
				double v2Z2;
				double v3X1;
				double v3Y1;
				double v3Z1;
				double v4X;
				double v4Y;
				double v4Z;
				double v12X2;
				double v12Y2;
				double v12Z2;
				double v13X;
				double v13Y;
				double v13Z;
				double v14X;
				double v14Y;
				double v14Z;
				double v23X1;
				double v23Y1;
				double v23Z1;
				double v24X;
				double v24Y;
				double v24Z;
				v1X2 = vec11.x;
				v1Y2 = vec11.y;
				v1Z2 = vec11.z;
				v2X2 = vec21.x;
				v2Y2 = vec21.y;
				v2Z2 = vec21.z;
				v3X1 = vec31.x;
				v3Y1 = vec31.y;
				v3Z1 = vec31.z;
				v4X = vec4.x;
				v4Y = vec4.y;
				v4Z = vec4.z;
				v12X2 = v2X2 - v1X2;
				v12Y2 = v2Y2 - v1Y2;
				v12Z2 = v2Z2 - v1Z2;
				v13X = v3X1 - v1X2;
				v13Y = v3Y1 - v1Y2;
				v13Z = v3Z1 - v1Z2;
				v14X = v4X - v1X2;
				v14Y = v4Y - v1Y2;
				v14Z = v4Z - v1Z2;
				v23X1 = v3X1 - v2X2;
				v23Y1 = v3Y1 - v2Y2;
				v23Z1 = v3Z1 - v2Z2;
				v24X = v4X - v2X2;
				v24Y = v4Y - v2Y2;
				v24Z = v4Z - v2Z2;
				double n123X;
				double n123Y;
				double n123Z;
				double n134X;
				double n134Y;
				double n134Z;
				double n142X;
				double n142Y;
				double n142Z;
				double n243X;
				double n243Y;
				double n243Z;
				n123X = v12Y2 * v13Z - v12Z2 * v13Y;
				n123Y = v12Z2 * v13X - v12X2 * v13Z;
				n123Z = v12X2 * v13Y - v12Y2 * v13X;
				n134X = v13Y * v14Z - v13Z * v14Y;
				n134Y = v13Z * v14X - v13X * v14Z;
				n134Z = v13X * v14Y - v13Y * v14X;
				n142X = v14Y * v12Z2 - v14Z * v12Y2;
				n142Y = v14Z * v12X2 - v14X * v12Z2;
				n142Z = v14X * v12Y2 - v14Y * v12X2;
				n243X = v24Y * v23Z1 - v24Z * v23Y1;
				n243Y = v24Z * v23X1 - v24X * v23Z1;
				n243Z = v24X * v23Y1 - v24Y * v23X1;
				int sign = v12X2 * n243X + v12Y2 * n243Y + v12Z2 * n243Z > 0 ? 1 : -1;
				double mind1 = -1;
				double minvX1;
				double minvY1;
				double minvZ1;
				int mini1 = 0;
				minvX1 = 0;
				minvY1 = 0;
				minvZ1 = 0;
				if((v1X2 * n123X + v1Y2 * n123Y + v1Z2 * n123Z) * sign < 0) {
//					double v1X;
//					double v1Y;
//					double v1Z;
//					double v2X;
//					double v2Y;
//					double v2Z;
//					double v3X;
//					double v3Y;
//					double v3Z;
//					double v12X;
//					double v12Y;
//					double v12Z;
//					double v23X;
//					double v23Y;
//					double v23Z;
//					double v31X;
//					double v31Y;
//					double v31Z;
					v1X = vec11.x;
					v1Y = vec11.y;
					v1Z = vec11.z;
					v2X = vec21.x;
					v2Y = vec21.y;
					v2Z = vec21.z;
					v3X = vec31.x;
					v3Y = vec31.y;
					v3Z = vec31.z;
					v12X = v2X - v1X;
					v12Y = v2Y - v1Y;
					v12Z = v2Z - v1Z;
					v23X = v3X - v2X;
					v23Y = v3Y - v2Y;
					v23Z = v3Z - v2Z;
					v31X = v1X - v3X;
					v31Y = v1Y - v3Y;
					v31Z = v1Z - v3Z;
//					double nX;
//					double nY;
//					double nZ;
					nX = v12Y * v23Z - v12Z * v23Y;
					nY = v12Z * v23X - v12X * v23Z;
					nZ = v12X * v23Y - v12Y * v23X;
//					double n12X;
//					double n12Y;
//					double n12Z;
//					double n23X;
//					double n23Y;
//					double n23Z;
//					double n31X;
//					double n31Y;
//					double n31Z;
					n12X = v12Y * nZ - v12Z * nY;
					n12Y = v12Z * nX - v12X * nZ;
					n12Z = v12X * nY - v12Y * nX;
					n23X = v23Y * nZ - v23Z * nY;
					n23Y = v23Z * nX - v23X * nZ;
					n23Z = v23X * nY - v23Y * nX;
					n31X = v31Y * nZ - v31Z * nY;
					n31Y = v31Z * nX - v31X * nZ;
					n31Z = v31X * nY - v31Y * nX;
					 mind = -1;
//					double minvX;
//					double minvY;
//					double minvZ;
					 mini = 0;
					minvX = 0;
					minvY = 0;
					minvZ = 0;
					if(v1X * n12X + v1Y * n12Y + v1Z * n12Z < 0) {
//						let v1X;
//						let v1Y;
//						let v1Z;
//						let v2X;
//						let v2Y;
//						let v2Z;
						v1X = vec11.x;
						v1Y = vec11.y;
						v1Z = vec11.z;
						v2X = vec21.x;
						v2Y = vec21.y;
						v2Z = vec21.z;
//						let v12X;
//						let v12Y;
//						let v12Z;
						v12X = v2X - v1X;
						v12Y = v2Y - v1Y;
						v12Z = v2Z - v1Z;
						 t = v12X * v1X + v12Y * v1Y + v12Z * v1Z;
						t = -t / (v12X * v12X + v12Y * v12Y + v12Z * v12Z);
						int b;
						if(t < 0) {
							closest.x = v1X;
							closest.y = v1Y;
							closest.z = v1Z;
							b = 1;
						} else if(t > 1) {
							closest.x = v2X;
							closest.y = v2Y;
							closest.z = v2Z;
							b = 2;
						} else {
							double pX;
							double pY;
							double pZ;
							pX = v1X + v12X * t;
							pY = v1Y + v12Y * t;
							pZ = v1Z + v12Z * t;
							closest.x = pX;
							closest.y = pY;
							closest.z = pZ;
							b = 3;
						}
						mini = b;
						mind = closest.x * closest.x + closest.y * closest.y + closest.z * closest.z;
						minvX = closest.x;
						minvY = closest.y;
						minvZ = closest.z;
					}
					if(v2X * n23X + v2Y * n23Y + v2Z * n23Z < 0) {
//						let v1X;
//						let v1Y;
//						let v1Z;
//						let v2X;
//						let v2Y;
//						let v2Z;
						v1X = vec21.x;
						v1Y = vec21.y;
						v1Z = vec21.z;
						v2X = vec31.x;
						v2Y = vec31.y;
						v2Z = vec31.z;
//						let v12X;
//						let v12Y;
//						let v12Z;
						v12X = v2X - v1X;
						v12Y = v2Y - v1Y;
						v12Z = v2Z - v1Z;
						 t = v12X * v1X + v12Y * v1Y + v12Z * v1Z;
						t = -t / (v12X * v12X + v12Y * v12Y + v12Z * v12Z);
						int b;
						if(t < 0) {
							closest.x = v1X;
							closest.y = v1Y;
							closest.z = v1Z;
							b = 1;
						} else if(t > 1) {
							closest.x = v2X;
							closest.y = v2Y;
							closest.z = v2Z;
							b = 2;
						} else {
							double pX;
							double pY;
							double pZ;
							pX = v1X + v12X * t;
							pY = v1Y + v12Y * t;
							pZ = v1Z + v12Z * t;
							closest.x = pX;
							closest.y = pY;
							closest.z = pZ;
							b = 3;
						}
						double d = closest.x * closest.x + closest.y * closest.y + closest.z * closest.z;
						if(mind < 0 || d < mind) {
							mini = b << 1;
							mind = d;
							minvX = closest.x;
							minvY = closest.y;
							minvZ = closest.z;
						}
					}
					if(v3X * n31X + v3Y * n31Y + v3Z * n31Z < 0) {
//						let v1X;
//						let v1Y;
//						let v1Z;
//						let v2X;
//						let v2Y;
//						let v2Z;
						v1X = vec11.x;
						v1Y = vec11.y;
						v1Z = vec11.z;
						v2X = vec31.x;
						v2Y = vec31.y;
						v2Z = vec31.z;
//						let v12X;
//						let v12Y;
//						let v12Z;
						v12X = v2X - v1X;
						v12Y = v2Y - v1Y;
						v12Z = v2Z - v1Z;
						 t = v12X * v1X + v12Y * v1Y + v12Z * v1Z;
						t = -t / (v12X * v12X + v12Y * v12Y + v12Z * v12Z);
						int b;
						if(t < 0) {
							closest.x = v1X;
							closest.y = v1Y;
							closest.z = v1Z;
							b = 1;
						} else if(t > 1) {
							closest.x = v2X;
							closest.y = v2Y;
							closest.z = v2Z;
							b = 2;
						} else {
							double pX;
							double pY;
							double pZ;
							pX = v1X + v12X * t;
							pY = v1Y + v12Y * t;
							pZ = v1Z + v12Z * t;
							closest.x = pX;
							closest.y = pY;
							closest.z = pZ;
							b = 3;
						}
						double d = closest.x * closest.x + closest.y * closest.y + closest.z * closest.z;
						if(mind < 0 || d < mind) {
							mini = b & 1 | (b & 2) << 1;
							mind = d;
							minvX = closest.x;
							minvY = closest.y;
							minvZ = closest.z;
						}
					}
					int b;
					if(mind > 0) {
						closest.x = minvX;
						closest.y = minvY;
						closest.z = minvZ;
						b = mini;
					} else {
						double l = nX * nX + nY * nY + nZ * nZ;
						if(l > 0) {
							l = 1 / Math.sqrt(l);
						}
						nX *= l;
						nY *= l;
						nZ *= l;
						double l2 = nX * nX + nY * nY + nZ * nZ;
						l2 = (v1X * nX + v1Y * nY + v1Z * nZ) / l2;
						minvX = nX * l2;
						minvY = nY * l2;
						minvZ = nZ * l2;
						closest.x = minvX;
						closest.y = minvY;
						closest.z = minvZ;
						b = 7;
					}
					mini1 = b;
					mind1 = closest.x * closest.x + closest.y * closest.y + closest.z * closest.z;
					minvX1 = closest.x;
					minvY1 = closest.y;
					minvZ1 = closest.z;
				}
				if((v1X2 * n134X + v1Y2 * n134Y + v1Z2 * n134Z) * sign < 0) {
//					double v1X;
//					double v1Y;
//					double v1Z;
//					double v2X;
//					double v2Y;
//					double v2Z;
//					double v3X;
//					double v3Y;
//					double v3Z;
//					double v12X;
//					double v12Y;
//					double v12Z;
//					double v23X;
//					double v23Y;
//					double v23Z;
//					double v31X;
//					double v31Y;
//					double v31Z;
					v1X = vec11.x;
					v1Y = vec11.y;
					v1Z = vec11.z;
					v2X = vec31.x;
					v2Y = vec31.y;
					v2Z = vec31.z;
					v3X = vec4.x;
					v3Y = vec4.y;
					v3Z = vec4.z;
					v12X = v2X - v1X;
					v12Y = v2Y - v1Y;
					v12Z = v2Z - v1Z;
					v23X = v3X - v2X;
					v23Y = v3Y - v2Y;
					v23Z = v3Z - v2Z;
					v31X = v1X - v3X;
					v31Y = v1Y - v3Y;
					v31Z = v1Z - v3Z;
//					double nX;
//					double nY;
//					double nZ;
					nX = v12Y * v23Z - v12Z * v23Y;
					nY = v12Z * v23X - v12X * v23Z;
					nZ = v12X * v23Y - v12Y * v23X;
//					double n12X;
//					double n12Y;
//					double n12Z;
//					double n23X;
//					double n23Y;
//					double n23Z;
//					double n31X;
//					double n31Y;
//					double n31Z;
					n12X = v12Y * nZ - v12Z * nY;
					n12Y = v12Z * nX - v12X * nZ;
					n12Z = v12X * nY - v12Y * nX;
					n23X = v23Y * nZ - v23Z * nY;
					n23Y = v23Z * nX - v23X * nZ;
					n23Z = v23X * nY - v23Y * nX;
					n31X = v31Y * nZ - v31Z * nY;
					n31Y = v31Z * nX - v31X * nZ;
					n31Z = v31X * nY - v31Y * nX;
					 mind = -1;
//					double minvX;
//					double minvY;
//					double minvZ;
					 mini = 0;
					minvX = 0;
					minvY = 0;
					minvZ = 0;
					if(v1X * n12X + v1Y * n12Y + v1Z * n12Z < 0) {
//						double v1X;
//						double v1Y;
//						double v1Z;
//						double v2X;
//						double v2Y;
//						double v2Z;
						v1X = vec11.x;
						v1Y = vec11.y;
						v1Z = vec11.z;
						v2X = vec31.x;
						v2Y = vec31.y;
						v2Z = vec31.z;
//						double v12X;
//						double v12Y;
//						double v12Z;
						v12X = v2X - v1X;
						v12Y = v2Y - v1Y;
						v12Z = v2Z - v1Z;
						 t = v12X * v1X + v12Y * v1Y + v12Z * v1Z;
						t = -t / (v12X * v12X + v12Y * v12Y + v12Z * v12Z);
						int b;
						if(t < 0) {
							closest.x = v1X;
							closest.y = v1Y;
							closest.z = v1Z;
							b = 1;
						} else if(t > 1) {
							closest.x = v2X;
							closest.y = v2Y;
							closest.z = v2Z;
							b = 2;
						} else {
							double pX;
							double pY;
							double pZ;
							pX = v1X + v12X * t;
							pY = v1Y + v12Y * t;
							pZ = v1Z + v12Z * t;
							closest.x = pX;
							closest.y = pY;
							closest.z = pZ;
							b = 3;
						}
						mini = b;
						mind = closest.x * closest.x + closest.y * closest.y + closest.z * closest.z;
						minvX = closest.x;
						minvY = closest.y;
						minvZ = closest.z;
					}
					if(v2X * n23X + v2Y * n23Y + v2Z * n23Z < 0) {
//						let v1X;
//						let v1Y;
//						let v1Z;
//						let v2X;
//						let v2Y;
//						let v2Z;
						v1X = vec31.x;
						v1Y = vec31.y;
						v1Z = vec31.z;
						v2X = vec4.x;
						v2Y = vec4.y;
						v2Z = vec4.z;
//						let v12X;
//						let v12Y;
//						let v12Z;
						v12X = v2X - v1X;
						v12Y = v2Y - v1Y;
						v12Z = v2Z - v1Z;
						 t = v12X * v1X + v12Y * v1Y + v12Z * v1Z;
						t = -t / (v12X * v12X + v12Y * v12Y + v12Z * v12Z);
						int b;
						if(t < 0) {
							closest.x = v1X;
							closest.y = v1Y;
							closest.z = v1Z;
							b = 1;
						} else if(t > 1) {
							closest.x = v2X;
							closest.y = v2Y;
							closest.z = v2Z;
							b = 2;
						} else {
							double pX;
							double pY;
							double pZ;
							pX = v1X + v12X * t;
							pY = v1Y + v12Y * t;
							pZ = v1Z + v12Z * t;
							closest.x = pX;
							closest.y = pY;
							closest.z = pZ;
							b = 3;
						}
						double d = closest.x * closest.x + closest.y * closest.y + closest.z * closest.z;
						if(mind < 0 || d < mind) {
							mini = b << 1;
							mind = d;
							minvX = closest.x;
							minvY = closest.y;
							minvZ = closest.z;
						}
					}
					if(v3X * n31X + v3Y * n31Y + v3Z * n31Z < 0) {
//						double v1X;
//						double v1Y;
//						double v1Z;
//						double v2X;
//						double v2Y;
//						double v2Z;
						v1X = vec11.x;
						v1Y = vec11.y;
						v1Z = vec11.z;
						v2X = vec4.x;
						v2Y = vec4.y;
						v2Z = vec4.z;
//						double v12X;
//						double v12Y;
//						double v12Z;
						v12X = v2X - v1X;
						v12Y = v2Y - v1Y;
						v12Z = v2Z - v1Z;
						 t = v12X * v1X + v12Y * v1Y + v12Z * v1Z;
						t = -t / (v12X * v12X + v12Y * v12Y + v12Z * v12Z);
						int b;
						if(t < 0) {
							closest.x = v1X;
							closest.y = v1Y;
							closest.z = v1Z;
							b = 1;
						} else if(t > 1) {
							closest.x = v2X;
							closest.y = v2Y;
							closest.z = v2Z;
							b = 2;
						} else {
							double pX;
							double pY;
							double pZ;
							pX = v1X + v12X * t;
							pY = v1Y + v12Y * t;
							pZ = v1Z + v12Z * t;
							closest.x = pX;
							closest.y = pY;
							closest.z = pZ;
							b = 3;
						}
						double d = closest.x * closest.x + closest.y * closest.y + closest.z * closest.z;
						if(mind < 0 || d < mind) {
							mini = b & 1 | (b & 2) << 1;
							mind = d;
							minvX = closest.x;
							minvY = closest.y;
							minvZ = closest.z;
						}
					}
					int b;
					if(mind > 0) {
						closest.x = minvX;
						closest.y = minvY;
						closest.z = minvZ;
						b = mini;
					} else {
						double l = nX * nX + nY * nY + nZ * nZ;
						if(l > 0) {
							l = 1 / Math.sqrt(l);
						}
						nX *= l;
						nY *= l;
						nZ *= l;
						double l2 = nX * nX + nY * nY + nZ * nZ;
						l2 = (v1X * nX + v1Y * nY + v1Z * nZ) / l2;
						minvX = nX * l2;
						minvY = nY * l2;
						minvZ = nZ * l2;
						closest.x = minvX;
						closest.y = minvY;
						closest.z = minvZ;
						b = 7;
					}
					double d = closest.x * closest.x + closest.y * closest.y + closest.z * closest.z;
					if(mind1 < 0 || d < mind1) {
						mini1 = b & 1 | (b & 6) << 1;
						mind1 = d;
						minvX1 = closest.x;
						minvY1 = closest.y;
						minvZ1 = closest.z;
					}
				}
				if((v1X2 * n142X + v1Y2 * n142Y + v1Z2 * n142Z) * sign < 0) {
//					double v1X;
//					double v1Y;
//					double v1Z;
//					double v2X;
//					double v2Y;
//					double v2Z;
//					double v3X;
//					double v3Y;
//					double v3Z;
//					double v12X;
//					double v12Y;
//					double v12Z;
//					double v23X;
//					double v23Y;
//					double v23Z;
//					double v31X;
//					double v31Y;
//					double v31Z;
					v1X = vec11.x;
					v1Y = vec11.y;
					v1Z = vec11.z;
					v2X = vec21.x;
					v2Y = vec21.y;
					v2Z = vec21.z;
					v3X = vec4.x;
					v3Y = vec4.y;
					v3Z = vec4.z;
					v12X = v2X - v1X;
					v12Y = v2Y - v1Y;
					v12Z = v2Z - v1Z;
					v23X = v3X - v2X;
					v23Y = v3Y - v2Y;
					v23Z = v3Z - v2Z;
					v31X = v1X - v3X;
					v31Y = v1Y - v3Y;
					v31Z = v1Z - v3Z;
//					double nX;
//					double nY;
//					double nZ;
					nX = v12Y * v23Z - v12Z * v23Y;
					nY = v12Z * v23X - v12X * v23Z;
					nZ = v12X * v23Y - v12Y * v23X;
//					double n12X;
//					double n12Y;
//					double n12Z;
//					double n23X;
//					double n23Y;
//					double n23Z;
//					double n31X;
//					double n31Y;
//					double n31Z;
					n12X = v12Y * nZ - v12Z * nY;
					n12Y = v12Z * nX - v12X * nZ;
					n12Z = v12X * nY - v12Y * nX;
					n23X = v23Y * nZ - v23Z * nY;
					n23Y = v23Z * nX - v23X * nZ;
					n23Z = v23X * nY - v23Y * nX;
					n31X = v31Y * nZ - v31Z * nY;
					n31Y = v31Z * nX - v31X * nZ;
					n31Z = v31X * nY - v31Y * nX;
					 mind = -1;
//					double minvX;
//					double minvY;
//					double minvZ;
					 mini = 0;
					minvX = 0;
					minvY = 0;
					minvZ = 0;
					if(v1X * n12X + v1Y * n12Y + v1Z * n12Z < 0) {
//						double v1X;
//						double v1Y;
//						double v1Z;
//						double v2X;
//						double v2Y;
//						double v2Z;
						v1X = vec11.x;
						v1Y = vec11.y;
						v1Z = vec11.z;
						v2X = vec21.x;
						v2Y = vec21.y;
						v2Z = vec21.z;
//						double v12X;
//						double v12Y;
//						double v12Z;
						v12X = v2X - v1X;
						v12Y = v2Y - v1Y;
						v12Z = v2Z - v1Z;
						 t = v12X * v1X + v12Y * v1Y + v12Z * v1Z;
						t = -t / (v12X * v12X + v12Y * v12Y + v12Z * v12Z);
						int b;
						if(t < 0) {
							closest.x = v1X;
							closest.y = v1Y;
							closest.z = v1Z;
							b = 1;
						} else if(t > 1) {
							closest.x = v2X;
							closest.y = v2Y;
							closest.z = v2Z;
							b = 2;
						} else {
							double pX;
							double pY;
							double pZ;
							pX = v1X + v12X * t;
							pY = v1Y + v12Y * t;
							pZ = v1Z + v12Z * t;
							closest.x = pX;
							closest.y = pY;
							closest.z = pZ;
							b = 3;
						}
						mini = b;
						mind = closest.x * closest.x + closest.y * closest.y + closest.z * closest.z;
						minvX = closest.x;
						minvY = closest.y;
						minvZ = closest.z;
					}
					if(v2X * n23X + v2Y * n23Y + v2Z * n23Z < 0) {
//						double v1X;
//						double v1Y;
//						double v1Z;
//						double v2X;
//						double v2Y;
//						double v2Z;
						v1X = vec21.x;
						v1Y = vec21.y;
						v1Z = vec21.z;
						v2X = vec4.x;
						v2Y = vec4.y;
						v2Z = vec4.z;
//						double v12X;
//						double v12Y;
//						double v12Z;
						v12X = v2X - v1X;
						v12Y = v2Y - v1Y;
						v12Z = v2Z - v1Z;
						 t = v12X * v1X + v12Y * v1Y + v12Z * v1Z;
						t = -t / (v12X * v12X + v12Y * v12Y + v12Z * v12Z);
						int b;
						if(t < 0) {
							closest.x = v1X;
							closest.y = v1Y;
							closest.z = v1Z;
							b = 1;
						} else if(t > 1) {
							closest.x = v2X;
							closest.y = v2Y;
							closest.z = v2Z;
							b = 2;
						} else {
							double pX;
							double pY;
							double pZ;
							pX = v1X + v12X * t;
							pY = v1Y + v12Y * t;
							pZ = v1Z + v12Z * t;
							closest.x = pX;
							closest.y = pY;
							closest.z = pZ;
							b = 3;
						}
						double d = closest.x * closest.x + closest.y * closest.y + closest.z * closest.z;
						if(mind < 0 || d < mind) {
							mini = b << 1;
							mind = d;
							minvX = closest.x;
							minvY = closest.y;
							minvZ = closest.z;
						}
					}
					if(v3X * n31X + v3Y * n31Y + v3Z * n31Z < 0) {
//						double v1X;
//						double v1Y;
//						double v1Z;
//						double v2X;
//						double v2Y;
//						double v2Z;
						v1X = vec11.x;
						v1Y = vec11.y;
						v1Z = vec11.z;
						v2X = vec4.x;
						v2Y = vec4.y;
						v2Z = vec4.z;
//						double v12X;
//						double v12Y;
//						double v12Z;
						v12X = v2X - v1X;
						v12Y = v2Y - v1Y;
						v12Z = v2Z - v1Z;
						 t = v12X * v1X + v12Y * v1Y + v12Z * v1Z;
						t = -t / (v12X * v12X + v12Y * v12Y + v12Z * v12Z);
						int b;
						if(t < 0) {
							closest.x = v1X;
							closest.y = v1Y;
							closest.z = v1Z;
							b = 1;
						} else if(t > 1) {
							closest.x = v2X;
							closest.y = v2Y;
							closest.z = v2Z;
							b = 2;
						} else {
							double pX;
							double pY;
							double pZ;
							pX = v1X + v12X * t;
							pY = v1Y + v12Y * t;
							pZ = v1Z + v12Z * t;
							closest.x = pX;
							closest.y = pY;
							closest.z = pZ;
							b = 3;
						}
						double d = closest.x * closest.x + closest.y * closest.y + closest.z * closest.z;
						if(mind < 0 || d < mind) {
							mini = b & 1 | (b & 2) << 1;
							mind = d;
							minvX = closest.x;
							minvY = closest.y;
							minvZ = closest.z;
						}
					}
					int b;
					if(mind > 0) {
						closest.x = minvX;
						closest.y = minvY;
						closest.z = minvZ;
						b = mini;
					} else {
						double l = nX * nX + nY * nY + nZ * nZ;
						if(l > 0) {
							l = 1 / Math.sqrt(l);
						}
						nX *= l;
						nY *= l;
						nZ *= l;
						double l2 = nX * nX + nY * nY + nZ * nZ;
						l2 = (v1X * nX + v1Y * nY + v1Z * nZ) / l2;
						minvX = nX * l2;
						minvY = nY * l2;
						minvZ = nZ * l2;
						closest.x = minvX;
						closest.y = minvY;
						closest.z = minvZ;
						b = 7;
					}
					double d = closest.x * closest.x + closest.y * closest.y + closest.z * closest.z;
					if(mind1 < 0 || d < mind1) {
						mini1 = b & 3 | (b & 4) << 1;
						mind1 = d;
						minvX1 = closest.x;
						minvY1 = closest.y;
						minvZ1 = closest.z;
					}
				}
				if((v2X2 * n243X + v2Y2 * n243Y + v2Z2 * n243Z) * sign < 0) {
//					double v1X;
//					double v1Y;
//					double v1Z;
//					double v2X;
//					double v2Y;
//					double v2Z;
//					double v3X;
//					double v3Y;
//					double v3Z;
//					double v12X;
//					double v12Y;
//					double v12Z;
//					double v23X;
//					double v23Y;
//					double v23Z;
//					double v31X;
//					double v31Y;
//					double v31Z;
					v1X = vec21.x;
					v1Y = vec21.y;
					v1Z = vec21.z;
					v2X = vec31.x;
					v2Y = vec31.y;
					v2Z = vec31.z;
					v3X = vec4.x;
					v3Y = vec4.y;
					v3Z = vec4.z;
					v12X = v2X - v1X;
					v12Y = v2Y - v1Y;
					v12Z = v2Z - v1Z;
					v23X = v3X - v2X;
					v23Y = v3Y - v2Y;
					v23Z = v3Z - v2Z;
					v31X = v1X - v3X;
					v31Y = v1Y - v3Y;
					v31Z = v1Z - v3Z;
//					double nX;
//					double nY;
//					double nZ;
					nX = v12Y * v23Z - v12Z * v23Y;
					nY = v12Z * v23X - v12X * v23Z;
					nZ = v12X * v23Y - v12Y * v23X;
//					double n12X;
//					double n12Y;
//					double n12Z;
//					double n23X;
//					double n23Y;
//					double n23Z;
//					double n31X;
//					double n31Y;
//					double n31Z;
					n12X = v12Y * nZ - v12Z * nY;
					n12Y = v12Z * nX - v12X * nZ;
					n12Z = v12X * nY - v12Y * nX;
					n23X = v23Y * nZ - v23Z * nY;
					n23Y = v23Z * nX - v23X * nZ;
					n23Z = v23X * nY - v23Y * nX;
					n31X = v31Y * nZ - v31Z * nY;
					n31Y = v31Z * nX - v31X * nZ;
					n31Z = v31X * nY - v31Y * nX;
					 mind = -1;
//					double minvX;
//					double minvY;
//					double minvZ;
					 mini = 0;
					minvX = 0;
					minvY = 0;
					minvZ = 0;
					if(v1X * n12X + v1Y * n12Y + v1Z * n12Z < 0) {
//						double v1X;
//						double v1Y;
//						double v1Z;
//						double v2X;
//						double v2Y;
//						double v2Z;
						v1X = vec21.x;
						v1Y = vec21.y;
						v1Z = vec21.z;
						v2X = vec31.x;
						v2Y = vec31.y;
						v2Z = vec31.z;
//						double v12X;
//						double v12Y;
//						double v12Z;
						v12X = v2X - v1X;
						v12Y = v2Y - v1Y;
						v12Z = v2Z - v1Z;
						 t = v12X * v1X + v12Y * v1Y + v12Z * v1Z;
						t = -t / (v12X * v12X + v12Y * v12Y + v12Z * v12Z);
						int b;
						if(t < 0) {
							closest.x = v1X;
							closest.y = v1Y;
							closest.z = v1Z;
							b = 1;
						} else if(t > 1) {
							closest.x = v2X;
							closest.y = v2Y;
							closest.z = v2Z;
							b = 2;
						} else {
							double pX;
							double pY;
							double pZ;
							pX = v1X + v12X * t;
							pY = v1Y + v12Y * t;
							pZ = v1Z + v12Z * t;
							closest.x = pX;
							closest.y = pY;
							closest.z = pZ;
							b = 3;
						}
						mini = b;
						mind = closest.x * closest.x + closest.y * closest.y + closest.z * closest.z;
						minvX = closest.x;
						minvY = closest.y;
						minvZ = closest.z;
					}
					if(v2X * n23X + v2Y * n23Y + v2Z * n23Z < 0) {
//						double v1X;
//						double v1Y;
//						double v1Z;
//						double v2X;
//						double v2Y;
//						double v2Z;
						v1X = vec31.x;
						v1Y = vec31.y;
						v1Z = vec31.z;
						v2X = vec4.x;
						v2Y = vec4.y;
						v2Z = vec4.z;
//						double v12X;
//						double v12Y;
//						double v12Z;
						v12X = v2X - v1X;
						v12Y = v2Y - v1Y;
						v12Z = v2Z - v1Z;
						 t = v12X * v1X + v12Y * v1Y + v12Z * v1Z;
						t = -t / (v12X * v12X + v12Y * v12Y + v12Z * v12Z);
						int b;
						if(t < 0) {
							closest.x = v1X;
							closest.y = v1Y;
							closest.z = v1Z;
							b = 1;
						} else if(t > 1) {
							closest.x = v2X;
							closest.y = v2Y;
							closest.z = v2Z;
							b = 2;
						} else {
							double pX;
							double pY;
							double pZ;
							pX = v1X + v12X * t;
							pY = v1Y + v12Y * t;
							pZ = v1Z + v12Z * t;
							closest.x = pX;
							closest.y = pY;
							closest.z = pZ;
							b = 3;
						}
						double d = closest.x * closest.x + closest.y * closest.y + closest.z * closest.z;
						if(mind < 0 || d < mind) {
							mini = b << 1;
							mind = d;
							minvX = closest.x;
							minvY = closest.y;
							minvZ = closest.z;
						}
					}
					if(v3X * n31X + v3Y * n31Y + v3Z * n31Z < 0) {
//						double v1X;
//						double v1Y;
//						double v1Z;
//						double v2X;
//						double v2Y;
//						double v2Z;
						v1X = vec21.x;
						v1Y = vec21.y;
						v1Z = vec21.z;
						v2X = vec4.x;
						v2Y = vec4.y;
						v2Z = vec4.z;
//						double v12X;
//						double v12Y;
//						double v12Z;
						v12X = v2X - v1X;
						v12Y = v2Y - v1Y;
						v12Z = v2Z - v1Z;
						 t = v12X * v1X + v12Y * v1Y + v12Z * v1Z;
						t = -t / (v12X * v12X + v12Y * v12Y + v12Z * v12Z);
						int b;
						if(t < 0) {
							closest.x = v1X;
							closest.y = v1Y;
							closest.z = v1Z;
							b = 1;
						} else if(t > 1) {
							closest.x = v2X;
							closest.y = v2Y;
							closest.z = v2Z;
							b = 2;
						} else {
							double pX;
							double pY;
							double pZ;
							pX = v1X + v12X * t;
							pY = v1Y + v12Y * t;
							pZ = v1Z + v12Z * t;
							closest.x = pX;
							closest.y = pY;
							closest.z = pZ;
							b = 3;
						}
						double d = closest.x * closest.x + closest.y * closest.y + closest.z * closest.z;
						if(mind < 0 || d < mind) {
							mini = b & 1 | (b & 2) << 1;
							mind = d;
							minvX = closest.x;
							minvY = closest.y;
							minvZ = closest.z;
						}
					}
					int b;
					if(mind > 0) {
						closest.x = minvX;
						closest.y = minvY;
						closest.z = minvZ;
						b = mini;
					} else {
						double l = nX * nX + nY * nY + nZ * nZ;
						if(l > 0) {
							l = 1 / Math.sqrt(l);
						}
						nX *= l;
						nY *= l;
						nZ *= l;
						double l2 = nX * nX + nY * nY + nZ * nZ;
						l2 = (v1X * nX + v1Y * nY + v1Z * nZ) / l2;
						minvX = nX * l2;
						minvY = nY * l2;
						minvZ = nZ * l2;
						closest.x = minvX;
						closest.y = minvY;
						closest.z = minvZ;
						b = 7;
					}
					double d = closest.x * closest.x + closest.y * closest.y + closest.z * closest.z;
					if(mind1 < 0 || d < mind1) {
						mini1 = b << 1;
						mind1 = d;
						minvX1 = closest.x;
						minvY1 = closest.y;
						minvZ1 = closest.z;
					}
				}
				if(mind1 > 0) {
					closest.x = minvX1;
					closest.y = minvY1;
					closest.z = minvZ1;
					v = mini1;
				} else {
					closest.zero();
					v = 15;
				}
				break;
			}
			if(closest.x * closest.x + closest.y * closest.y + closest.z * closest.z < 1e-008) {
				if(!useEpa) {
					this.distance = 0;
					return 0;
				}
				switch(this.simplexSize) {
				case 1:
					this.pointToTetrahedron();
					break;
				case 2:
					this.lineToTetrahedron();
					break;
				case 3:
					this.triangleToTetrahedron();
					break;
				}
				if(this.simplexSize == 4) {
					int epaState = this.computeDepth(c1,c2,tf1,tf2,s,w1,w2);
					if(epaState != 0) {
						this.distance = 0;
						return epaState;
					}
					this.distance = -this.depth;
					return 0;
				}
				this.distance = 0;
				return 1;
			}
			this.shrinkSimplex(v);
			dir.x = closest.x;
			dir.y = closest.y;
			dir.z = closest.z;
			dir.x = -dir.x;
			dir.y = -dir.y;
			dir.z = -dir.z;
			this.computeWitnessPoint1(false);
			this.computeWitnessPoint2(false);
			 _this = this.s[this.simplexSize];
			Vec3 v4 = this.w1[this.simplexSize];
			_this.x = v4.x;
			_this.y = v4.y;
			_this.z = v4.z;
			Vec3 v5 = this.w2[this.simplexSize];
			_this.x -= v5.x;
			_this.y -= v5.y;
			_this.z -= v5.z;
			if(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z < 1e-008) {
				throw new Error("!?");
			}
			Vec3 _this1 = s[this.simplexSize];
			if(_this1.x * dir.x + _this1.y * dir.y + _this1.z * dir.z - (closest.x * dir.x + closest.y * dir.y + closest.z * dir.z) < 1e-008) {
				this.interpolateClosestPoints();
				this.distance = Math.sqrt(closest.x * closest.x + closest.y * closest.y + closest.z * closest.z);
				if(cache != null && cache._gjkCache != null) {
					this.saveCache(cache._gjkCache);
				}
				return 0;
			}
			this.simplexSize++;
			++count;
		}
		return 2;
	}

	// `c1` can be null
	public boolean convexCastImpl(ConvexGeometry c1, ConvexGeometry c2, Transform tf1, Transform tf2, Vec3 tl1,
			Vec3 tl2, RayCastHit hit) {
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
		Vec3 firstDir = new Vec3();
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

		double lambda = 0.0f;
		Vec3 rayX = this.rayX; // origin
		Vec3 rayR = this.rayR; // relative translation
		rayX.zero();
		rayR.copyFrom(tl2).subEq(tl1);

		double eps = 1e-4f;
		double eps2 = eps * eps;

		while (count < max) {
			// project the origin to the simplex and compute index of voronoi region of the
			// origin.
			int v = 0;
			GjkEpaLog.log("simplex size: $simplexSize");
			GjkEpaLog.log("projecting the origin to the simplex...");
			GjkEpaLog.log("x: $rayX");
			GjkEpaLog.log("lambda: $lambda");
			switch (simplexSize) {
			case 1:
				closest.copyFrom(s[0]);
				// GjkEpaLog.log("${s[0]}");
				v = 1;
				break;
			case 2:
				v = SimplexUtil.projectOrigin2(s[0], s[1], closest);
				// GjkEpaLog.log('${s[0]}');
				// GjkEpaLog.log('${s[1]}');
				break;
			case 3:
				v = SimplexUtil.projectOrigin3(s[0], s[1], s[2], closest);
				// GjkEpaLog.log('${s[0]}');
				// GjkEpaLog.log('${s[1]}');
				// GjkEpaLog.log('${s[2]}');
				break;
			case 4:
				v = SimplexUtil.projectOrigin4(s[0], s[1], s[2], s[3], closest);
				// GjkEpaLog.log('${s[0]}');
				// GjkEpaLog.log('${s[1]}');
				// GjkEpaLog.log('${s[2]}');
				// GjkEpaLog.log('${s[3]}');
				break;
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
				// throw M.error("!?"); // this should never be happen
			}

			// n is the normal at the vertex p
			Vec3 p = s[simplexSize];
			Vec3 n = dir;
			// GjkEpaLog.log('new vertex p = $p');
			// GjkEpaLog.log('normal n = $n');
			// GjkEpaLog.log('ray dir r = $rayR');

			// check if a part of the ray can be rejected
			double pn = p.dot(n);
			GjkEpaLog.log("p dot n = $pn");
			if (pn < 0) {
				// check if entire the ray can be rejected
				if (rayR.dot(n) >= 0) {
					GjkEpaLog.log("rejected [0");
					return false;
				}
				double dLambda = pn / rayR.dot(n);
				lambda += dLambda;
				if (lambda >= 1) {
					GjkEpaLog.log("rejected 1]");
					return false;
				}
				GjkEpaLog.log("advanced: " + dLambda);
				rayX.addScaledEq(rayR, dLambda);

				// translate the simplex
				// for (int i =0;in 0...simplexSize + 1) {
				for (int i = 0; i < simplexSize + 1; i++) {
					s[i].addScaledEq(rayR, -dLambda);
				}
			} else {
				GjkEpaLog.log("ray did not advance");
			}

			// do not add new vertex to the simplex if already exists
			boolean duplicate = false;
			for (int i = 0; i < simplexSize; i++) {
				double dx = s[i].x - s[simplexSize].x;
				double dy = s[i].y - s[simplexSize].y;
				double dz = s[i].z - s[simplexSize].z;
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
		case 1:
			closestPoint1.copyFrom(w1[0]);
			closestPoint2.copyFrom(w2[0]);
			break;
		case 2:
			double cX;
			double cY;
			double cZ;
			Vec3 v2 = this.closest;
			cX = v2.x;
			cY = v2.y;
			cZ = v2.z;
			double s0X;
			double s0Y;
			double s0Z;
			double w10X;
			double w10Y;
			double w10Z;
			double w20X;
			double w20Y;
			double w20Z;
			double s1X;
			double s1Y;
			double s1Z;
			double w11X;
			double w11Y;
			double w11Z;
			double w21X;
			double w21Y;
			double w21Z;
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
			double s01X;
			double s01Y;
			double s01Z;
			s01X = s1X - s0X;
			s01Y = s1Y - s0Y;
			s01Z = s1Z - s0Z;
			double invDet = s01X * s01X + s01Y * s01Y + s01Z * s01Z;
			if (invDet != 0) {
				invDet = 1 / invDet;
			}
			double s0cX;
			double s0cY;
			double s0cZ;
			s0cX = cX - s0X;
			s0cY = cY - s0Y;
			s0cZ = cZ - s0Z;
			double t = (s0cX * s01X + s0cY * s01Y + s0cZ * s01Z) * invDet;
			double diffX;
			double diffY;
			double diffZ;
			double cp1X;
			double cp1Y;
			double cp1Z;
			double cp2X;
			double cp2Y;
			double cp2Z;
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

		case 3:
			double cX1;
			double cY1;
			double cZ1;
			Vec3 v11 = this.closest;
			cX1 = v11.x;
			cY1 = v11.y;
			cZ1 = v11.z;
			double s0X1;
			double s0Y1;
			double s0Z1;
			double w10X1;
			double w10Y1;
			double w10Z1;
			double w20X1;
			double w20Y1;
			double w20Z1;
			double s1X1;
			double s1Y1;
			double s1Z1;
			double w11X1;
			double w11Y1;
			double w11Z1;
			double w21X1;
			double w21Y1;
			double w21Z1;
			double s2X;
			double s2Y;
			double s2Z;
			double w12X;
			double w12Y;
			double w12Z;
			double w22X;
			double w22Y;
			double w22Z;
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
			double s01X1;
			double s01Y1;
			double s01Z1;
			double s02X;
			double s02Y;
			double s02Z;
			double s0cX1;
			double s0cY1;
			double s0cZ1;
			s01X1 = s1X1 - s0X1;
			s01Y1 = s1Y1 - s0Y1;
			s01Z1 = s1Z1 - s0Z1;
			s02X = s2X - s0X1;
			s02Y = s2Y - s0Y1;
			s02Z = s2Z - s0Z1;
			s0cX1 = cX1 - s0X1;
			s0cY1 = cY1 - s0Y1;
			s0cZ1 = cZ1 - s0Z1;
			double d11 = s01X1 * s01X1 + s01Y1 * s01Y1 + s01Z1 * s01Z1;
			double d12 = s01X1 * s02X + s01Y1 * s02Y + s01Z1 * s02Z;
			double d22 = s02X * s02X + s02Y * s02Y + s02Z * s02Z;
			double d1c = s01X1 * s0cX1 + s01Y1 * s0cY1 + s01Z1 * s0cZ1;
			double d2c = s02X * s0cX1 + s02Y * s0cY1 + s02Z * s0cZ1;
			double invDet1 = d11 * d22 - d12 * d12;
			if (invDet1 != 0) {
				invDet1 = 1 / invDet1;
			}
			double s = (d1c * d22 - d2c * d12) * invDet1;
			double t1 = (-d1c * d12 + d2c * d11) * invDet1;
			double diffX1;
			double diffY1;
			double diffZ1;
			double cp1X1;
			double cp1Y1;
			double cp1Z1;
			double cp2X1;
			double cp2Y1;
			double cp2Z1;
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

		default:
			// throw M.error("!?");
		}
	}

	public void loadCache(GjkCache gjkCache) {
		// copy simplex data from the cache
		dir.copyFrom(gjkCache.prevClosestDir);
	}

	public void saveCache(GjkCache gjkCache) {
		// set GJK cache for the next computation
		gjkCache.prevClosestDir.copyFrom(closest).negateEq();
	}

	public void shrinkSimplex(int vertexBits) {
		simplexSize = vertexBits;
		simplexSize = (simplexSize & 5) + (simplexSize >> 1 & 5);
		simplexSize = (simplexSize & 3) + (simplexSize >> 2 & 3);
		switch (vertexBits) {
		// case 0: // do nothing
		// case 1: // do nothing
		case 2: // 0 <- 1
			s[0].copyFrom(s[1]);
			w1[0].copyFrom(w1[1]);
			w2[0].copyFrom(w2[1]);
			break;
		// case 3: // do nothing
		case 4: // 0 <- 2
			s[0].copyFrom(s[2]);
			w1[0].copyFrom(w1[2]);
			w2[0].copyFrom(w2[2]);
			break;
		case 5: // 1 <- 2
			s[1].copyFrom(s[2]);
			w1[1].copyFrom(w1[2]);
			w2[1].copyFrom(w2[2]);
			break;
		case 6: // 0 <- 2
			s[0].copyFrom(s[2]);
			w1[0].copyFrom(w1[2]);
			w2[0].copyFrom(w2[2]);
			break;
		// case 7: // do nothing
		case 8: // 0 <- 3
			s[0].copyFrom(s[3]);
			w1[0].copyFrom(w1[3]);
			w2[0].copyFrom(w2[3]);
			break;
		case 9: // 1 <- 3
			s[1].copyFrom(s[3]);
			w1[1].copyFrom(w1[3]);
			w2[1].copyFrom(w2[3]);
			break;
		case 10: // 0 <- 3
			s[0].copyFrom(s[3]);
			w1[0].copyFrom(w1[3]);
			w2[0].copyFrom(w2[3]);
			break;
		case 11: // 2 <- 3
			s[2].copyFrom(s[3]);
			w1[2].copyFrom(w1[3]);
			w2[2].copyFrom(w2[3]);
			break;
		case 12: // 0 <- 2, 1 <- 3
			s[0].copyFrom(s[2]);
			w1[0].copyFrom(w1[2]);
			w2[0].copyFrom(w2[2]);
			s[1].copyFrom(s[3]);
			w1[1].copyFrom(w1[3]);
			w2[1].copyFrom(w2[3]);
			break;
		case 13: // 1 <- 3
			s[1].copyFrom(s[3]);
			w1[1].copyFrom(w1[3]);
			w2[1].copyFrom(w2[3]);
			break;
		case 14: // 0 <- 3
			s[0].copyFrom(s[3]);
			w1[0].copyFrom(w1[3]);
			w2[0].copyFrom(w2[3]);
			break;
		// case 15: // do nothing
		}
	}

	public void computeSupportingVertex() {
		computeWitnessPoint1(false);
		computeWitnessPoint2(false);
		s[simplexSize].copyFrom(w1[simplexSize]).subEq(w2[simplexSize]);
	}

	public void computeConvexCastSupportingVertex() {
		if (c1 != null) {
			computeWitnessPoint1(true);
		} else {
			M.vec3_toVec3(w1[simplexSize], tf1._position);
		}
		computeWitnessPoint2(true);
		s[simplexSize].copyFrom(w1[simplexSize]).subEq(w2[simplexSize]);
	}

	public void computeWitnessPoint1(boolean addMargin) {
		Vec3 tmp = new Vec3();
		Vec3 idir = new Vec3();
		M.vec3_fromVec3(idir, dir);

		// compute local dir
		Vec3 ldir1 = new Vec3();
		M.vec3_mulMat3Transposed(ldir1, idir, tf1._rotation);

		// compute local witness point
		Vec3 iw1 = new Vec3();
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

		M.vec3_toVec3(dir, idir);
	}

	private void computeWitnessPoint2(boolean addMargin) {
		Vec3 tmp = new Vec3();
		Vec3 idir = new Vec3();
		M.vec3_fromVec3(idir, dir);

		// compute local dir
		Vec3 ldir2 = new Vec3();
		M.vec3_mulMat3Transposed(ldir2, idir, tf2._rotation);
		ldir2.negateEq();

		// compute local witness point
		Vec3 iw2 = new Vec3();
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
		for (int i = 0; i < 3; i++) {
			dir.copyFrom(baseDirs[i]);

			computeSupportingVertex();
			simplexSize++;
			lineToTetrahedron();
			if (simplexSize == 4)
				break;
			simplexSize--;

			dir.negateEq();

			computeSupportingVertex();
			simplexSize++;
			lineToTetrahedron();
			if (simplexSize == 4)
				break;
			simplexSize--;
		}
	}

	private void lineToTetrahedron() {
		Vec3 oldDir = new Vec3();
		M.vec3_fromVec3(oldDir, dir);

		Vec3 lineDir = new Vec3();
		M.vec3_sub(lineDir, s[0], s[1]);
		for (int i = 0; i < 3; i++) {
			M.vec3_cross(dir, lineDir, baseDirs[i]);

			computeSupportingVertex();
			simplexSize++;
			triangleToTetrahedron();
			if (simplexSize == 4)
				break;
			simplexSize--;

			dir.negateEq();

			computeSupportingVertex();
			simplexSize++;
			triangleToTetrahedron();
			if (simplexSize == 4)
				break;
			simplexSize--;
		}

		M.vec3_toVec3(dir, oldDir);
	}

	private void triangleToTetrahedron() {
		double oldDirX;
		double oldDirY;
		double oldDirZ;
		Vec3 v = this.dir;
		oldDirX = v.x;
		oldDirY = v.y;
		oldDirZ = v.z;
		while(true) {
			double s0X;
			double s0Y;
			double s0Z;
			double s1X;
			double s1Y;
			double s1Z;
			double s2X;
			double s2Y;
			double s2Z;
			double s01X;
			double s01Y;
			double s01Z;
			double s02X;
			double s02Y;
			double s02Z;
			v = this.s[0];
			s0X = v.x;
			s0Y = v.y;
			s0Z = v.z;
			Vec3 v1 = this.s[1];
			s1X = v1.x;
			s1Y = v1.y;
			s1Z = v1.z;
			Vec3 v2 = this.s[2];
			s2X = v2.x;
			s2Y = v2.y;
			s2Z = v2.z;
			s01X = s1X - s0X;
			s01Y = s1Y - s0Y;
			s01Z = s1Z - s0Z;
			s02X = s2X - s0X;
			s02Y = s2Y - s0Y;
			s02Z = s2Z - s0Z;
			double nX;
			double nY;
			double nZ;
			nX = s01Y * s02Z - s01Z * s02Y;
			nY = s01Z * s02X - s01X * s02Z;
			nZ = s01X * s02Y - s01Y * s02X;
			Vec3 v3 = this.dir;
			v3.x = nX;
			v3.y = nY;
			v3.z = nZ;
			this.computeWitnessPoint1(false);
			this.computeWitnessPoint2(false);
			Vec3 _this = this.s[this.simplexSize];
			Vec3 v4 = this.w1[this.simplexSize];
			_this.x = v4.x;
			_this.y = v4.y;
			_this.z = v4.z;
			Vec3 v5 = this.w2[this.simplexSize];
			_this.x -= v5.x;
			_this.y -= v5.y;
			_this.z -= v5.z;
			this.simplexSize++;
			if(this.isValidTetrahedron()) {
				break;
			}
			this.simplexSize--;
			Vec3 _this1 = this.dir;
			_this1.x = -_this1.x;
			_this1.y = -_this1.y;
			_this1.z = -_this1.z;
			this.computeWitnessPoint1(false);
			this.computeWitnessPoint2(false);
			Vec3 _this2 = this.s[this.simplexSize];
			Vec3 v6 = this.w1[this.simplexSize];
			_this2.x = v6.x;
			_this2.y = v6.y;
			_this2.z = v6.z;
			Vec3 v7 = this.w2[this.simplexSize];
			_this2.x -= v7.x;
			_this2.y -= v7.y;
			_this2.z -= v7.z;
			this.simplexSize++;
			if(this.isValidTetrahedron()) {
				break;
			}
			this.simplexSize--;
			break;
		}
		Vec3 v1 = this.dir;
		v1.x = oldDirX;
		v1.y = oldDirY;
		v1.z = oldDirZ;
	}

	/* inline */
	private boolean isValidTetrahedron() {
		double e00 = s[1].x - s[0].x;
		double e01 = s[1].y - s[0].y;
		double e02 = s[1].z - s[0].z;
		double e10 = s[2].x - s[0].x;
		double e11 = s[2].y - s[0].y;
		double e12 = s[2].z - s[0].z;
		double e20 = s[3].x - s[0].x;
		double e21 = s[3].y - s[0].y;
		double e22 = s[3].z - s[0].z;
		double det = e00 * (e11 * e22 - e12 * e21) - e01 * (e10 * e22 - e12 * e20) + e02 * (e10 * e21 - e11 * e20);
		return det > 1e-12 || det < -1e-12;
	}

	// EPA

	public int computeDepth(ConvexGeometry convex1, ConvexGeometry convex2, Transform tf1, Transform tf2,
			Vec3[] initialPolyhedron, Vec3[] initialPolyhedron1, Vec3[] initialPolyhedron2) {
		GjkEpaLog.log("----------- EPA begin ----------- ");

		EpaPolyhedron _this = this.polyhedron;
		while (_this._numTriangles > 0) {
			EpaTriangle t = _this._triangleList;
			_this._numTriangles--;
			EpaTriangle prev = t._prev;
			EpaTriangle next = t._next;
			if (prev != null) {
				prev._next = next;
			}
			if (next != null) {
				next._prev = prev;
			}
			if (t == _this._triangleList) {
				_this._triangleList = _this._triangleList._next;
			}
			if (t == _this._triangleListLast) {
				_this._triangleListLast = _this._triangleListLast._prev;
			}
			t._next = null;
			t._prev = null;
			t.removeReferences();
			t._next = _this._trianglePool;
			_this._trianglePool = t;
		}
		while (_this._numVertices > 0) {
			EpaVertex v = _this._vertices[--_this._numVertices];
			v.removeReferences();
			v._next = _this._vertexPool;
			_this._vertexPool = v;
		}
		EpaPolyhedron tmp = this.polyhedron;
		EpaPolyhedron _this1 = this.polyhedron;
		EpaVertex first = _this1._vertexPool;
		if (first != null) {
			_this1._vertexPool = first._next;
			first._next = null;
		} else {
			first = new oimo.collision.narrowphase.detector.gjkepa.EpaVertex();
		}
		EpaVertex tmp1 = first.init(initialPolyhedron[0], initialPolyhedron1[0], initialPolyhedron2[0]);
		EpaPolyhedron _this2 = this.polyhedron;
		EpaVertex first1 = _this2._vertexPool;
		if (first1 != null) {
			_this2._vertexPool = first1._next;
			first1._next = null;
		} else {
			first1 = new oimo.collision.narrowphase.detector.gjkepa.EpaVertex();
		}
		EpaVertex tmp2 = first1.init(initialPolyhedron[1], initialPolyhedron1[1], initialPolyhedron2[1]);
		EpaPolyhedron _this3 = this.polyhedron;
		EpaVertex first2 = _this3._vertexPool;
		if (first2 != null) {
			_this3._vertexPool = first2._next;
			first2._next = null;
		} else {
			first2 = new oimo.collision.narrowphase.detector.gjkepa.EpaVertex();
		}
		EpaVertex tmp3 = first2.init(initialPolyhedron[2], initialPolyhedron1[2], initialPolyhedron2[2]);
		EpaPolyhedron _this4 = this.polyhedron;
		EpaVertex first3 = _this4._vertexPool;
		if (first3 != null) {
			_this4._vertexPool = first3._next;
			first3._next = null;
		} else {
			first3 = new oimo.collision.narrowphase.detector.gjkepa.EpaVertex();
		}
		if (!tmp._init(tmp1, tmp2, tmp3,
				first3.init(initialPolyhedron[3], initialPolyhedron1[3], initialPolyhedron2[3]))) {
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
			// GjkEpaLog.run(face.dump());

			dir.copyFrom(face._normal).normalize();
			computeSupportingVertex();

			EpaVertex v0 = face._vertices[0];
			EpaVertex v1 = face._vertices[1];
			EpaVertex v2 = face._vertices[2];

			double dot1 = v0.v.dot(dir);
			double dot2 = supportingVertex.dot(dir);

			GjkEpaLog.log("got new vertex: " + supportingVertex);
			GjkEpaLog.log("improvement: " + dot1 + " -> " + dot2 + ", normal: " + dir.toString());

			if (dot2 - dot1 < 1e-6 || count == maxIterations - 1) { // no improvement
				closest.copyFrom(dir).scaleEq(dir.dot(v0.v) / dir.lengthSq());

				Vec3 c = new Vec3();
				M.vec3_fromVec3(c, closest);
				Vec3 s0 = v0.v;
				Vec3 w10 = v0.w1;
				Vec3 w20 = v0.w2;
				Vec3 s1 = v1.v;
				Vec3 w11 = v1.w1;
				Vec3 w21 = v1.w2;
				Vec3 s2 = v2.v;
				Vec3 w12 = v2.w1;
				Vec3 w22 = v2.w2;
//				
//				M.vec3_fromVec3(s0, v0.v); M.vec3_fromVec3(w10, v0.w1); M.vec3_fromVec3(w20, v0.w2);
//				M.vec3_fromVec3(s1, v1.v); M.vec3_fromVec3(w11, v1.w1); M.vec3_fromVec3(w21, v1.w2);
//				M.vec3_fromVec3(s2, v2.v); M.vec3_fromVec3(w12, v2.w1); M.vec3_fromVec3(w22, v2.w2);

				Vec3 s01 = new Vec3();
				Vec3 s02 = new Vec3();
				Vec3 s0c = new Vec3();
				M.vec3_sub(s01, s1, s0);
				M.vec3_sub(s02, s2, s0);
				M.vec3_sub(s0c, c, s0);

				double d11 = M.vec3_dot(s01, s01);
				double d12 = M.vec3_dot(s01, s02);
				double d22 = M.vec3_dot(s02, s02);
				double d1c = M.vec3_dot(s01, s0c);
				double d2c = M.vec3_dot(s02, s0c);
				double invDet = d11 * d22 - d12 * d12;
				if (invDet != 0)
					invDet = 1 / invDet;
				double s = (d1c * d22 - d2c * d12) * invDet;
				double t = (-d1c * d12 + d2c * d11) * invDet;

				// compute closest points
				Vec3 diff = new Vec3();
				Vec3 cp1 = new Vec3();
				Vec3 cp2 = new Vec3();

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
				// GjkEpaLog.run(polyhedron._dumpAsObjModel());

				return GjkEpaResultState.EPA_FAILED_TO_ADD_VERTEX;
			}
			count++;
		}

		GjkEpaLog.log("EPA failed: did not converge.");
		// GjkEpaLog.run(polyhedron._dumpAsObjModel());

		return GjkEpaResultState.EPA_DID_NOT_CONVERGE;
	}

	// --- public ---

	/**
	 * Returns an instance of `GjkEpa`.
	 */
	public static GjkEpa getInstance() {
		return instance;
	}

	/**
	 * Computes the closest points of two convex geometries `c1` and `c2` with
	 * transforms `tf1` and `tf2` respectively, and returns the status of the result
	 * (see `GjkEpaResultState` for details). If cached data `cache` is not `null`,
	 * this tries to exploit the previous result in `cache` to improve performance,
	 * and stores the new result to `cache`.
	 *
	 * Set the compiler option `OIMO_GJK_EPA_DEBUG` for debugging (warning: massive
	 * logging).
	 */
	public int computeClosestPoints(ConvexGeometry c1, ConvexGeometry c2, Transform tf1, Transform tf2,
			CachedDetectorData cache) {
		return computeClosestPointsImpl(c1, c2, tf1, tf2, cache, true);
	}

	/**
	 * Computes the distance between two convex geometries `c1` and `c2` with
	 * transforms `tf1` and `tf2` respectively, and returns the status of the result
	 * (see `GjkEpaResultState` for details). Different from
	 * `GjkEpa.computeClosestPoints`, this does not compute negative distances and
	 * closest points if two geometries are overlapping. If cached data `cache` is
	 * not `null`, this tries to exploit the previous result in `cache` to improve
	 * performance, and stores the new result to `cache`.
	 *
	 * Set the compiler option `OIMO_GJK_EPA_DEBUG` for debugging (warning: massive
	 * logging).
	 */
	public int computeDistance(ConvexGeometry c1, ConvexGeometry c2, Transform tf1, Transform tf2,
			CachedDetectorData cache) {
		return computeClosestPointsImpl(c1, c2, tf1, tf2, cache, false);
	}

	/**
	 * Performs a convex casting between `c1` and `c2`. Returns `true` and sets the
	 * result information to `hit` if the convex geometries intersect. Each convex
	 * geometries translates by `tl1` and `tl2`, starting from the beginning
	 * transforms `tf1` and `tf2` respectively.
	 *
	 * Set the compiler option `OIMO_GJK_EPA_DEBUG` for debugging (warning: massive
	 * logging).
	 */
	public boolean convexCast(ConvexGeometry c1, ConvexGeometry c2, Transform tf1, Transform tf2, Vec3 tl1, Vec3 tl2,
			RayCastHit hit) {
		return convexCastImpl(c1, c2, tf1, tf2, tl1, tl2, hit);
	}

	/**
	 * Performs ray cansting against the convex geometry `c` with transform `tf`.
	 * Returns `true` and sets the result information to `hit` if the line segment
	 * from `begin` to `end` intersects the convex geometry. Otherwise returns
	 * `false`.
	 *
	 * Set the compiler option `OIMO_GJK_EPA_DEBUG` for debugging (warning: massive
	 * logging).
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