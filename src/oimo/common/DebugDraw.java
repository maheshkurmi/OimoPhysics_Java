package oimo.common;
/**
 * The interface of debug drawer. This provides graphical information of a physics world
 * for debugging softwares. Users should override at least three methods `DebugDraw.point`,
 * `DebugDraw.triangle`, `DebugDraw.line`.
 */
public abstract class DebugDraw {
	/**
	 * Whether the shapes are drawn in wireframe mode.
	 */
	public boolean wireframe;

	/**
	 * Whether to draw the shapes.
	 */
	public boolean drawShapes;

	/**
	 * Whether to draw the bounding volume hierarchy of the broad-phase collision
	 * detection. If `BvhBroadPhase` is not used, nothing will be drawn regardless
	 * of the value of this parameter.
	 */
	public boolean drawBvh;

	/**
	 * The minimum depth of the BVH to be drawn. If `DebugDrawer.drawBvh` is set to
	 * `false`, the entire BVH will not be drawn.
	 */
	public int drawBvhMinLevel;

	/**
	 * The maximum depth of the BVH to be drawn. If `DebugDrawer.drawBvh` is set to
	 * `false`, the entire BVH will not be drawn.
	 */
	public int drawBvhMaxLevel;

	/**
	 * Whether to draw the AABBs.
	 */
	public boolean drawAabbs;

	/**
	 * Whether to draw the bases of the rigid bodies.
	 */
	public boolean drawBases;

	/**
	 * Whether to draw the overlapping pairs of the AABBs.
	 */
	public boolean drawPairs;

	/**
	 * Whether to draw the contacts.
	 */
	public boolean drawContacts;

	/**
	 * Whether to draw the bases (normals, tangents, and binormals) of the contacts.
	 */
	public boolean drawContactBases;

	/**
	 * Whether to draw the joints.
	 */
	public boolean drawJoints;

	/**
	 * Whether to draw the limits of the joints.
	 */
	public boolean drawJointLimits;

	/**
	 * The rendering style of the debug draw.
	 */
	public DebugDrawStyle style;

	Pool p;

	public static int SPHERE_PHI_DIVISION =8;
	public  static int SPHERE_THETA_DIVISION = 4;
	Vec3[][] sphereCoords;
	Vec3[][] tmpSphereVerts;
	Vec3[][] tmpSphereNorms;

	public  static  int CIRCLE_THETA_DIVISION = 8;
	Vec3[] circleCoords;
	Vec3[] circleCoordsShift;
	Vec3[] tmpCircleVerts1;
	Vec3[] tmpCircleVerts2;
	Vec3[] tmpCircleNorms;

	/**
	 * Default constructor.
	 */
	public DebugDraw() {
		p = new Pool();

		wireframe = false;
		drawShapes = true;
		drawBvh = false;
		drawBvhMinLevel = 0;
		drawBvhMaxLevel = 65536;
		drawAabbs = false;
		drawBases = false;
		drawPairs = false;
		drawContacts = false;
		drawJoints = true;
		drawJointLimits = false;

		initSphereCoords();
		initCircleCoords();

		style = new DebugDrawStyle();
	}

	// --- private ---

	private void initSphereCoords() {
		// theta
		int nt = SPHERE_THETA_DIVISION;
		double dt = MathUtil.PI / nt;

		// phi
		int np = SPHERE_PHI_DIVISION;
		double dp = MathUtil.TWO_PI / np;

		sphereCoords = new Vec3[nt + 1][];
		tmpSphereVerts = new Vec3[nt + 1][];//new Vector<Vector<Vec3>>(nt + 1);
		tmpSphereNorms = new Vec3[nt + 1][];//new Vector<Vector<Vec3>>(nt + 1);
		for (int i=0;i<nt + 1;i++) {
			int num = i == 0 || i == nt ? 1 : np;
			sphereCoords[i] = new Vec3[num];
			tmpSphereVerts[i] = new Vec3[num];//new Vector<Vec3>(num);
			tmpSphereNorms[i] = new Vec3[num];//new Vector<Vec3>(num);
			for (int j=0;j<num;j++) {
				double theta = i * dt;
				double phi = j * dp;
				sphereCoords[i][j] = new Vec3(
					MathUtil.sin(theta) * MathUtil.cos(phi),
					MathUtil.cos(theta),
					-MathUtil.sin(theta) * MathUtil.sin(phi)
				);
				tmpSphereVerts[i][j] = new Vec3();
				tmpSphereNorms[i][j] = new Vec3();
			}
		}
	}

	private void initCircleCoords() {
		circleCoords = new Vec3[CIRCLE_THETA_DIVISION];
		circleCoordsShift = new Vec3[CIRCLE_THETA_DIVISION];//new Vector<Vec3>(CIRCLE_THETA_DIVISION);
		tmpCircleVerts1 = new Vec3[CIRCLE_THETA_DIVISION];//new Vector<Vec3>(CIRCLE_THETA_DIVISION);
		tmpCircleVerts2 = new Vec3[CIRCLE_THETA_DIVISION];//new Vector<Vec3>(CIRCLE_THETA_DIVISION);
		tmpCircleNorms = new Vec3[CIRCLE_THETA_DIVISION];//new Vector<Vec3>(CIRCLE_THETA_DIVISION);

		double td = MathUtil.TWO_PI / CIRCLE_THETA_DIVISION;
		for (int i=0;i<CIRCLE_THETA_DIVISION;i++) {
			circleCoords[i] = new Vec3(MathUtil.cos(i * td), 0, -MathUtil.sin(i * td));
			circleCoordsShift[i] = new Vec3(MathUtil.cos((i + 0.5) * td), 0, -MathUtil.sin((i + 0.5) * td));
			tmpCircleVerts1[i] = new Vec3();
			tmpCircleVerts2[i] = new Vec3();
			tmpCircleNorms[i] = new Vec3();
		}
	}

	private Vec3  sphericalCoord(Vec3 origin,Vec3 x,Vec3 y,Vec3 z,double r, double theta, double phi) {
		Vec3 v = cartesianCoord(origin, x, y, z,
			r * MathUtil.sin(theta) * MathUtil.cos(phi),
			r * MathUtil.cos(theta),
			r * -MathUtil.sin(theta) * MathUtil.sin(phi)
		);
		return v;
	}

	private Vec3  polarCoord(Vec3 origin, Vec3 x, Vec3 y, double r, double theta) {
		Vec3 v = cartesianCoord2D(origin, x, y,
			r * MathUtil.cos(theta),
			r * MathUtil.sin(theta)
		);
		return v;
	}

	private Vec3  cartesianCoord(Vec3  origin,Vec3 x,Vec3 y,Vec3 z,double cx,double cy,double cz) {
		return new Vec3().copyFrom(origin)
			.addScaledEq(x, cx)
			.addScaledEq(y, cy)
			.addScaledEq(z, cz)
		;
	}

	private Vec3  cartesianCoord2D(Vec3  origin,Vec3 x,Vec3 y,double cx,double cy) {
		return new Vec3().copyFrom(origin)
			.addScaledEq(x, cx)
			.addScaledEq(y, cy)
		;
	}

	private Vec3 cartesianCoord1D(Vec3 origin, Vec3 x, double cx) {
		return new Vec3().copyFrom(origin).addScaledEq(x, cx);
	}

	
	
	// --- public ---
	Vec3 vec3() {
		return  p.vec3();
	}

	Mat3 mat3() {
		return  p.mat3();
	}

	Mat4  mat4() {
		return  p.mat4();
	}

	Quat quat() {
		return  p.quat();
	}

	void disp(Vec3 v) {
		  p.disposeVec3(v);
	}

	void disp(Mat3 m) {
		  p.disposeMat3(m);
	}

	void disp(Mat4 m) {
		  p.disposeMat4(m);
	}
	
	void disp(Quat q) {
		  p.disposeQuat(q);
	}

	
	/**
	 * Draws an axis-aligned bounding box.
	 *
	 * `min` is the minimum point of the AABB.
	 *
	 * `max` is the maximum point of the AABB.
	 *
	 * `color` is the color of the AABB.
	 */
	public void aabb(Vec3 min, Vec3 max, Vec3 color) {
		Vec3 v1  = vec3().set(min.x, min.y, min.z);
		Vec3 v2  = vec3().set(min.x, min.y, max.z);
		Vec3 v3  = vec3().set(min.x, max.y, min.z);
		Vec3 v4  = vec3().set(min.x, max.y, max.z);
		Vec3 v5  = vec3().set(max.x, min.y, min.z);
		Vec3 v6  = vec3().set(max.x, min.y, max.z);
		Vec3 v7  = vec3().set(max.x, max.y, min.z);
		Vec3 v8  = vec3().set(max.x, max.y, max.z);
		line(v1, v2, color);
		line(v3, v4, color);
		line(v5, v6, color);
		line(v7, v8, color);
		line(v1, v3, color);
		line(v2, v4, color);
		line(v5, v7, color);
		line(v6, v8, color);
		line(v1, v5, color);
		line(v2, v6, color);
		line(v3, v7, color);
		line(v4, v8, color);
		disp(v1);
		disp(v2);
		disp(v3);
		disp(v4);
		disp(v5);
		disp(v6);
		disp(v7);
		disp(v8);
	}

	/**
	 * Draws the basis of a transform `transform`.
	 *
	 * `length` is the length of the lines to be drawn.
	 *
	 * `colorX` is the color of the x-axis of the basis.
	 *
	 * `colorY` is the color of the y-axis of the basis.
	 *
	 * `colorZ` is the color of the z-axis of the basis.
	 */
	public void basis(Transform transform, double length,Vec3 colorX, Vec3 colorY,Vec3 colorZ) {
		Vec3 pos = vec3();
		Mat3 rot = mat3();
		Vec3 ex = vec3();
		Vec3 ey = vec3();
		Vec3 ez = vec3();

		M.vec3_toVec3(pos, transform._position);
		M.mat3_toMat3(rot, transform._rotation);
		rot.getColTo(0, ex);
		rot.getColTo(1, ey);
		rot.getColTo(2, ez);

		ex.scaleEq(length).addEq(pos);
		ey.scaleEq(length).addEq(pos);
		ez.scaleEq(length).addEq(pos);

		line(pos, ex, colorX);
		line(pos, ey, colorY);
		line(pos, ez, colorZ);

		disp(pos);
		disp(rot);
		disp(ex);
		disp(ey);
		disp(ez);
	}

	/**
	 * Draws an ellipse.
	 *
	 * `center` is the center of the ellipse.
	 *
	 * `ex` is the normalized x-axis vector of the ellipse.
	 *
	 * `ey` is the normalized y-axis vector of the ellipse.
	 *
	 * `radiusX` is the radius along the x-axis of the ellipse.
	 *
	 * `radiusY` is the radius along the y-axis of the ellipse.
	 *
	 * `color` is the color of the ellipse.
	 */
	public void ellipse(Vec3 center,Vec3 ex, Vec3 ey, double radiusX,double radiusY, Vec3 color) {
		arc(center, ex, ey, radiusX, radiusY, 0, MathUtil.TWO_PI, false, color);
	}

	/**
	 * Draws an arc.
	 *
	 * `center` is the center of the arc.
	 *
	 * `ex` is the normalized x-axis vector of the arc.
	 *
	 * `ey` is the normalized y-axis vector of the arc.
	 *
	 * `radiusX` is the radius along the x-axis of the arc.
	 *
	 * `radiusY` is the radius along the y-axis of the arc.
	 *
	 * `startAngle` is the start angle of the arc in radians.
	 *
	 * `endAngle` is the end angle of the arc in radians.
	 *
	 * `drawSector` is whether to draw line segments between start/end point and center point.
	 *
	 * `color` is the color of the arc.
	 */
	public void arc(Vec3 center, Vec3 ex, Vec3 ey, double radiusX, double radiusY, double startAngle, double endAngle, boolean drawSector, Vec3 color) {
		ex = vec3().copyFrom(ex).scaleEq(radiusX);
		ey = vec3().copyFrom(ey).scaleEq(radiusY);

		double step = MathUtil.PI / 6;
		double angDiff = endAngle - startAngle;
		if (angDiff < 0) angDiff = -angDiff;

		int n = (int) (angDiff / step + 0.5);
		if (n == 0) n = 1;

		double theta = startAngle;
		double dt = (endAngle - startAngle) / n;
		Vec3 prevV = polarCoord(center, ex, ey, 1, theta);

		if (drawSector) {
			line(center, prevV, color);
		}

		for (int i=0;i<n;i++) {
			theta += dt;
			Vec3 v = polarCoord(center, ex, ey, 1, theta);
			line(prevV, v, color);
			disp(prevV);
			prevV = v;
		}

		if (drawSector) {
			line(center, prevV, color);
		}

		disp(prevV);

		disp(ex);
		disp(ey);
	}

	/**
	 * Draws a cone locally along to the y-axis. The center of the cone is in the middle of
	 * the vertex and the center of the base circle.
	 *
	 * `tf` is the transformation of the cone.
	 *
	 * `radius` is the radius of the base circle of the cone.
	 *
	 * `halfHeight` is the half-height of the cone. The local position of the vertex of
	 * the cone is `(0, halfHeight, 0)`.
	 *
	 * `color` is the color of the cone.
	 */
	public void cone(Transform tf, double radius, double  halfHeight,  Vec3  color) {
		Vec3 ex = vec3();
		Vec3 ey = vec3();
		Vec3 ez = vec3();
		Vec3 o = vec3();
		Mat3 m = mat3();
		tf.getPositionTo(o);
		tf.getRotationTo(m);
		m.getColTo(0, ex);
		m.getColTo(1, ey);
		m.getColTo(2, ez);

		Vec3 top = cartesianCoord1D(o, ey, halfHeight);
		Vec3 bottom = cartesianCoord1D(o, ey, -halfHeight);

		if (wireframe) {
			Vec3 bottom1 = cartesianCoord2D(bottom, ex, ez, -radius, 0);
			Vec3 bottom2 = cartesianCoord2D(bottom, ex, ez, radius, 0);
			Vec3 bottom3 = cartesianCoord2D(bottom, ex, ez, 0, -radius);
			Vec3 bottom4 = cartesianCoord2D(bottom, ex, ez, 0, radius);

			ellipse(bottom, ex, ez, radius, radius, color);

			line(top, bottom1, color);
			line(top, bottom2, color);
			line(top, bottom3, color);
			line(top, bottom4, color);

			disp(bottom1);
			disp(bottom2);
			disp(bottom3);
			disp(bottom4);
		} else {
			double invDenom = 1 / MathUtil.sqrt(radius * radius + 4 * halfHeight * halfHeight);
			double cos = 2 * halfHeight * invDenom;
			double sin = radius * invDenom;
			double invDenom2 = 1 / MathUtil.sqrt(2 * (1 + cos));
			for (int i=0;i<CIRCLE_THETA_DIVISION;i++) {
				tmpCircleNorms[i].copyFrom(circleCoords[i]).scaleEq(cos).y += sin;
				tmpCircleNorms[i].mulMat3Eq(m);

				tmpCircleVerts1[i].copyFrom(circleCoordsShift[i]).scaleEq(cos).y += sin;
				tmpCircleVerts1[i].mulMat3Eq(m);

				tmpCircleVerts2[i].copyFrom(circleCoords[i]).mulMat3Eq(m).scaleEq(radius).addEq(o);
				tmpCircleVerts2[i].addScaledEq(ey, -halfHeight);
			}
			for (int i=0;i<CIRCLE_THETA_DIVISION;i++) {
				Vec3 v1;
				Vec3 v2;
				Vec3 v3;
				Vec3 n1;
				Vec3 n2;
				Vec3 n3;

				// side
				v1 = top;
				v2 = tmpCircleVerts2[i];
				v3 = tmpCircleVerts2[(i + 1) % CIRCLE_THETA_DIVISION];
				n1 = tmpCircleVerts1[i];
				n2 = tmpCircleNorms[i];
				n3 = tmpCircleNorms[(i + 1) % CIRCLE_THETA_DIVISION];
				triangle(v1, v2, v3, n1, n2, n3, color);

				// bottom
				v1 = bottom;
				v2 = tmpCircleVerts2[(i + 1) % CIRCLE_THETA_DIVISION];
				v3 = tmpCircleVerts2[i];
				n1 = vec3().copyFrom(ey).negateEq();
				triangle(v1, v2, v3, n1, n1, n1, color);
				disp(n1);
			}
		}

		disp(top);
		disp(bottom);

		disp(o);
		disp(m);
		disp(ex);
		disp(ey);
		disp(ez);
	}

	/**
	 * Draws a cylinder locally along to the y-axis.
	 *
	 * `tf` is the transformation of the cylinder.
	 *
	 * `radius` is the radius of the cylinder.
	 *
	 * `halfHeight` is the half-height of the cylinder.
	 *
	 * `color` is the color of the cylinder.
	 */
	public void cylinder(Transform tf,  double radius,  double  halfHeight, Vec3 color) {
		Vec3 ex = vec3();
		Vec3 ey = vec3();
		Vec3 ez = vec3();
		Vec3 o = vec3();
		Mat3 m = mat3();
		tf.getPositionTo(o);
		tf.getRotationTo(m);
		m.getColTo(0, ex);
		m.getColTo(1, ey);
		m.getColTo(2, ez);

		Vec3 top = cartesianCoord1D(o, ey, halfHeight);
		Vec3 bottom = cartesianCoord1D(o, ey, -halfHeight);

		if (wireframe) {
			Vec3 top1 = cartesianCoord2D(top, ex, ez, -radius, 0);
			Vec3 top2 = cartesianCoord2D(top, ex, ez, radius, 0);
			Vec3 top3 = cartesianCoord2D(top, ex, ez, 0, -radius);
			Vec3 top4 = cartesianCoord2D(top, ex, ez, 0, radius);

			Vec3 bottom1 = cartesianCoord2D(bottom, ex, ez, -radius, 0);
			Vec3 bottom2 = cartesianCoord2D(bottom, ex, ez, radius, 0);
			Vec3 bottom3 = cartesianCoord2D(bottom, ex, ez, 0, -radius);
			Vec3 bottom4 = cartesianCoord2D(bottom, ex, ez, 0, radius);

			ellipse(top, ex, ez, radius, radius, color);
			ellipse(bottom, ex, ez, radius, radius, color);

			line(top1, bottom1, color);
			line(top2, bottom2, color);
			line(top3, bottom3, color);
			line(top4, bottom4, color);

			disp(top1);
			disp(top2);
			disp(top3);
			disp(top4);
			disp(bottom1);
			disp(bottom2);
			disp(bottom3);
			disp(bottom4);
		} else {
			for (int i=0;i<CIRCLE_THETA_DIVISION;i++) {
				tmpCircleNorms[i].copyFrom(circleCoords[i]).mulMat3Eq(m);
				tmpCircleVerts1[i].copyFrom(tmpCircleNorms[i]).scaleEq(radius).addEq(o);
				tmpCircleVerts2[i].copyFrom(tmpCircleVerts1[i]);

				tmpCircleVerts1[i].addScaledEq(ey, halfHeight);
				tmpCircleVerts2[i].addScaledEq(ey, -halfHeight);
			}
			for (int i=0;i<CIRCLE_THETA_DIVISION;i++) {
				Vec3 v1;
				Vec3 v2;
				Vec3 v3;
				Vec3 v4;
				Vec3 n1;
				Vec3 n2;
				Vec3 n3;
				Vec3 n4;

				//top
				v1 = top;
				v2 = tmpCircleVerts1[i];
				v3 = tmpCircleVerts1[(i + 1) % CIRCLE_THETA_DIVISION];
				n1 = ey;
				triangle(v1, v2, v3, n1, n1, n1, color);

				//bottom
				v1 = bottom;
				v2 = tmpCircleVerts2[(i + 1) % CIRCLE_THETA_DIVISION];
				v3 = tmpCircleVerts2[i];
				n1 = vec3().copyFrom(ey).negateEq();
				triangle(v1, v2, v3, n1, n1, n1, color);
				disp(n1);

				//side
				v1 = tmpCircleVerts1[i];
				v2 = tmpCircleVerts2[i];
				v3 = tmpCircleVerts2[(i + 1) % CIRCLE_THETA_DIVISION];
				v4 = tmpCircleVerts1[(i + 1) % CIRCLE_THETA_DIVISION];
				n1 = tmpCircleNorms[i];
				n2 = tmpCircleNorms[(i + 1) % CIRCLE_THETA_DIVISION];
				rect(v1, v2, v3, v4, n1, n1, n2, n2, color);
			}
		}

		disp(top);
		disp(bottom);

		disp(o);
		disp(m);
		disp(ex);
		disp(ey);
		disp(ez);
	}

	/**
	 * Draws a capsule locally along to the y-axis.
	 *
	 * `tf` is the transformation of the capsule.
	 *
	 * `radius` is the radius of the capsule.
	 *
	 * `halfHeight` is the half-height of the capsule.
	 *
	 * `color` is the color of the capsule.
	 */
	public void capsule(Transform tf, double radius,  double halfHeight, Vec3 color) {
		Vec3 ex = vec3();
		Vec3 ey = vec3();
		Vec3 ez = vec3();
		Vec3 o = vec3();
		Mat3 m = mat3();
		tf.getPositionTo(o);
		tf.getRotationTo(m);
		m.getColTo(0, ex);
		m.getColTo(1, ey);
		m.getColTo(2, ez);

		// draw caps

		// theta
		int nt = SPHERE_THETA_DIVISION;

		// phi
		int np = SPHERE_PHI_DIVISION;

		Vec3[][] vs = tmpSphereVerts;
		Vec3[][] ns = tmpSphereNorms;

		 //build normals first
		for (int i2=0;i2<nt+1;i2++) {// in 0...nt + 1) {
			int n = tmpSphereVerts[i2].length;
			for (int j2=0;j2<n;j2++) {// in 0...n) {
				ns[i2][j2].copyFrom(sphereCoords[i2][j2]).mulMat3Eq(m);
			}
		}

		for (int i =0;i<nt;i++) {

			if (i == 0) {
				// build upper hemisphere
				int half = nt >> 1;
				for (int i2=0;i2<half + 1;i2++) {
					int n = tmpSphereVerts[i2].length;
					for (int j2=0;j2<n;j2++) {// in 0...n) {
						vs[i2][j2].copyFrom(ns[i2][j2]).scaleEq(radius).addEq(o).addScaledEq(ey, halfHeight);
					}
				}
			}

			if (i == (nt >> 1)) {
				// build lower hemisphere
				int half = nt >> 1;
				for (int i2 =half;i2<nt + 1;i2++) {
					int n = tmpSphereVerts[i2].length;
					for (int j2=0;j2<n;j2++) {
						vs[i2][j2].copyFrom(ns[i2][j2]).scaleEq(radius).addEq(o).addScaledEq(ey, -halfHeight);
					}
				}
			}

			for (int j=0;j<np;j++) {
				Vec3 v1;
				Vec3 v2;
				Vec3 v3;
				Vec3 v4;
				Vec3 n1;
				Vec3 n2;
				Vec3 n3;
				Vec3 n4;
				if (i == 0) {
					// top
					if (wireframe) {
						v1 = vs[0][0];
						v2 = vs[1][j];
						line(v1, v2, color);
					} else {
						v1 = vs[0][0];
						v2 = vs[1][j];
						v3 = vs[1][(j + 1) % np];
						n1 = ns[0][0];
						n2 = ns[1][j];
						n3 = ns[1][(j + 1) % np];
						triangle(v1, v2, v3, n1, n2, n3, color);
					}
				} else if (i == nt - 1) {
					// bottom
					if (wireframe) {
						v1 = vs[nt][0];
						v2 = vs[i][(j + 1) % np];
						v3 = vs[i][j];
						line(v1, v2, color);
						line(v2, v3, color);
					} else {
						v1 = vs[nt][0];
						v2 = vs[i][(j + 1) % np];
						v3 = vs[i][j];
						n1 = ns[nt][0];
						n2 = ns[i][(j + 1) % np];
						n3 = ns[i][j];
						triangle(v1, v2, v3, n1, n2, n3, color);
					}
				} else {
					// middle
					if (wireframe) {
						v1 = vs[i][j];
						v2 = vs[i][(j + 1) % np];
						v3 = vs[i + 1][j];
						line(v1, v2, color);
						line(v1, v3, color);
					} else {
						v1 = vs[i][j];
						v2 = vs[i][(j + 1) % np];
						v3 = vs[i + 1][j];
						v4 = vs[i + 1][(j + 1) % np];
						n1 = ns[i][j];
						n2 = ns[i][(j + 1) % np];
						n3 = ns[i + 1][j];
						n4 = ns[i + 1][(j + 1) % np];
						rect(v1, v3, v4, v2, n1, n3, n4, n2, color);
					}
				}
			}
		}

		// draw side

		Vec3 top = cartesianCoord1D(o, ey, halfHeight);
		Vec3 bottom = cartesianCoord1D(o, ey, -halfHeight);

		if (wireframe) {
			Vec3 top1 = cartesianCoord2D(top, ex, ez, -radius, 0);
			Vec3 top2 = cartesianCoord2D(top, ex, ez, radius, 0);
			Vec3 top3 = cartesianCoord2D(top, ex, ez, 0, -radius);
			Vec3 top4 = cartesianCoord2D(top, ex, ez, 0, radius);

			Vec3 bottom1 = cartesianCoord2D(bottom, ex, ez, -radius, 0);
			Vec3 bottom2 = cartesianCoord2D(bottom, ex, ez, radius, 0);
			Vec3 bottom3 = cartesianCoord2D(bottom, ex, ez, 0, -radius);
			Vec3 bottom4 = cartesianCoord2D(bottom, ex, ez, 0, radius);

			ellipse(top, ex, ez, radius, radius, color);
			ellipse(bottom, ex, ez, radius, radius, color);

			line(top1, bottom1, color);
			line(top2, bottom2, color);
			line(top3, bottom3, color);
			line(top4, bottom4, color);

			disp(top1);
			disp(top2);
			disp(top3);
			disp(top4);
			disp(bottom1);
			disp(bottom2);
			disp(bottom3);
			disp(bottom4);
		} else {
			for (int i=0;i< CIRCLE_THETA_DIVISION;i++) {
				tmpCircleNorms[i].copyFrom(circleCoords[i]).mulMat3Eq(m);
				tmpCircleVerts1[i].copyFrom(tmpCircleNorms[i]).scaleEq(radius).addEq(o);
				tmpCircleVerts2[i].copyFrom(tmpCircleVerts1[i]);

				tmpCircleVerts1[i].addScaledEq(ey, halfHeight);
				tmpCircleVerts2[i].addScaledEq(ey, -halfHeight);
			}
			for (int i=0;i< CIRCLE_THETA_DIVISION;i++) {
				Vec3 v1;
				Vec3 v2;
				Vec3 v3;
				Vec3 v4;
				Vec3 n1;
				Vec3 n2;

				// side
				v1 = tmpCircleVerts1[i];
				v2 = tmpCircleVerts2[i];
				v3 = tmpCircleVerts2[(i + 1) % CIRCLE_THETA_DIVISION];
				v4 = tmpCircleVerts1[(i + 1) % CIRCLE_THETA_DIVISION];
				n1 = tmpCircleNorms[i];
				n2 = tmpCircleNorms[(i + 1) % CIRCLE_THETA_DIVISION];
				rect(v1, v2, v3, v4, n1, n1, n2, n2, color);
			}
		}

		disp(top);
		disp(bottom);

		disp(o);
		disp(m);
		disp(ex);
		disp(ey);
		disp(ez);
	}

	/**
	 * Draws a sphere.
	 *
	 * `tf` is the transformation of the sphere.
	 *
	 * `radius` is the radius of the sphere.
	 *
	 * `color` is the color of the sphere.
	 */
	public void sphere(Transform tf, double radius, Vec3 color) {
		Vec3 o = vec3();
		Mat3 m = mat3();
		tf.getPositionTo(o);
		tf.getRotationTo(m);

		// theta
		int nt = SPHERE_THETA_DIVISION;

		// phi
		int np = SPHERE_PHI_DIVISION;

		Vec3[][] vs = tmpSphereVerts;
		Vec3[][] ns = tmpSphereNorms;

		for (int i=0;i<nt + 1;i++) {
			int n = tmpSphereVerts[i].length;
			for (int j=0;j<n;j++) {
				ns[i][j].copyFrom(sphereCoords[i][j]).mulMat3Eq(m);
				vs[i][j].copyFrom(ns[i][j]).scaleEq(radius).addEq(o);
			}
		}

		for (int i=0;i<nt;i++) {
			for (int j=0;j<np;j++) {
				Vec3 v1;
				Vec3 v2;
				Vec3 v3;
				Vec3 v4;
				Vec3 n1;
				Vec3 n2;
				Vec3 n3;
				Vec3 n4;
				if (i == 0) {
					// top
					if (wireframe) {
						v1 = vs[0][0];
						v2 = vs[1][j];
						line(v1, v2, color);
					} else {
						v1 = vs[0][0];
						v2 = vs[1][j];
						v3 = vs[1][(j + 1) % np];
						n1 = ns[0][0];
						n2 = ns[1][j];
						n3 = ns[1][(j + 1) % np];
						triangle(v1, v2, v3, n1, n2, n3, color);
					}
				} else if (i == nt - 1) {
					// bottom
					if (wireframe) {
						v1 = vs[nt][0];
						v2 = vs[i][(j + 1) % np];
						v3 = vs[i][j];
						line(v1, v2, color);
						line(v2, v3, color);
					} else {
						v1 = vs[nt][0];
						v2 = vs[i][(j + 1) % np];
						v3 = vs[i][j];
						n1 = ns[nt][0];
						n2 = ns[i][(j + 1) % np];
						n3 = ns[i][j];
						triangle(v1, v2, v3, n1, n2, n3, color);
					}
				} else {
					// middle
					if (wireframe) {
						v1 = vs[i][j];
						v2 = vs[i][(j + 1) % np];
						v3 = vs[i + 1][j];
						line(v1, v2, color);
						line(v1, v3, color);
					} else {
						v1 = vs[i][j];
						v2 = vs[i][(j + 1) % np];
						v3 = vs[i + 1][j];
						v4 = vs[i + 1][(j + 1) % np];
						n1 = ns[i][j];
						n2 = ns[i][(j + 1) % np];
						n3 = ns[i + 1][j];
						n4 = ns[i + 1][(j + 1) % np];
						rect(v1, v3, v4, v2, n1, n3, n4, n2, color);
					}
				}
			}
		}

		disp(o);
		disp(m);
	}

	/**
	 * Draws a box.
	 *
	 * `tf` is the transformation of the box.
	 *
	 * `halfExtents` is the half-extents of the box.
	 *
	 * `color` is the color of the box.
	 */
	public void box(Transform tf, Vec3 halfExtents, Vec3 color) {
		Vec3 v1;
		Vec3 v2;
		Vec3 v3;
		Vec3 v4;
		Vec3 v5;
		Vec3 v6;
		Vec3 v7;
		Vec3 v8;

		Vec3 ex = vec3();
		Vec3 ey = vec3();
		Vec3 ez = vec3();
		Vec3 o = vec3();
		Mat3 m = mat3();
		tf.getPositionTo(o);
		tf.getRotationTo(m);
		m.getColTo(0, ex);
		m.getColTo(1, ey);
		m.getColTo(2, ez);

		double hx = halfExtents.x;
		double hy = halfExtents.y;
		double hz = halfExtents.z;

		v1 = cartesianCoord(o, ex, ey, ez, -hx, -hy, -hz);
		v2 = cartesianCoord(o, ex, ey, ez, -hx, -hy, hz);
		v3 = cartesianCoord(o, ex, ey, ez, -hx, hy, -hz);
		v4 = cartesianCoord(o, ex, ey, ez, -hx, hy, hz);
		v5 = cartesianCoord(o, ex, ey, ez, hx, -hy, -hz);
		v6 = cartesianCoord(o, ex, ey, ez, hx, -hy, hz);
		v7 = cartesianCoord(o, ex, ey, ez, hx, hy, -hz);
		v8 = cartesianCoord(o, ex, ey, ez, hx, hy, hz);
		if (wireframe) {
			line(v1, v2, color);
			line(v3, v4, color);
			line(v5, v6, color);
			line(v7, v8, color);
			line(v1, v3, color);
			line(v2, v4, color);
			line(v5, v7, color);
			line(v6, v8, color);
			line(v1, v5, color);
			line(v2, v6, color);
			line(v3, v7, color);
			line(v4, v8, color);
		} else {
			Vec3 nex = vec3().copyFrom(ex).negateEq();
			Vec3 ney = vec3().copyFrom(ey).negateEq();
			Vec3 nez = vec3().copyFrom(ez).negateEq();
			// x-
			rect(v1, v2, v4, v3, nex, nex, nex, nex, color);
			// x+
			rect(v5, v7, v8, v6, ex, ex, ex, ex, color);
			// y-
			rect(v1, v5, v6, v2, ney, ney, ney, ney, color);
			// y+
			rect(v3, v4, v8, v7, ey, ey, ey, ey, color);
			// z-
			rect(v1, v3, v7, v5, nez, nez, nez, nez, color);
			// z+
			rect(v2, v6, v8, v4, ez, ez, ez, ez, color);
			disp(nex);
			disp(ney);
			disp(nez);
		}
		disp(v1);
		disp(v2);
		disp(v3);
		disp(v4);
		disp(v5);
		disp(v6);
		disp(v7);
		disp(v8);

		disp(o);
		disp(m);
		disp(ex);
		disp(ey);
		disp(ez);
	}

	/**
	 * Draws a rectangle.
	 *
	 * `v1`, `v2`, `v3`, `v4` are the rectangle's vertices in CCW order.
	 *
	 * `n1`, `n2`, `n3`, `n4` are the normals of the rectangle's vertices in CCW order.
	 *
	 * `color` is the color of the rectangle.
	 */
	public void rect(Vec3 v1,Vec3 v2, Vec3 v3, Vec3 v4, Vec3 n1, Vec3 n2,Vec3 n3,Vec3 n4, Vec3 color) {
		triangle(v1, v2, v3, n1, n2, n3, color);
		triangle(v1, v3, v4, n1, n3, n4, color);
	}

	
	/**
	 * Draws a point at `v`.
	 *
	 * `color` is the color of the point.
	 */
	public abstract void point(Vec3 v, Vec3 color);

	/**
	 * Draws a triangle.
	 *
	 * `v1`, `v2`, `v3` are the triangle's vertices in CCW order.
	 *
	 * `n1`, `n2`, `n3` are the normals of the triangle's vertices in CCW order.
	 *
	 * `color` is the color of the triangle.
	 */
	public abstract void triangle(Vec3 v1, Vec3 v2, Vec3 v3, Vec3 n1, Vec3 n2, Vec3 n3, Vec3 color);

	/**
	 * Draws a line segment between `v1` and `v2`.
	 *
	 * `color` is the color of the line segment.
	 */
	public abstract void line(Vec3 v1,Vec3 v2, Vec3 color);

	

}