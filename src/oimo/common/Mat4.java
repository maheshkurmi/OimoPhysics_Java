package oimo.common;

/**
 * 4x4 Matrix class.
 *
 * Note that columns and rows are 0-indexed.
 */
public class Mat4 {
	/**
	 * The number of instance creation.
	 */
	public static int numCreations = 0;

	/**
	 * The element at row 0 column 0.
	 */
	public double e00=1;

	/**
	 * The element at row 0 column 1.
	 */
	public double e01=0;

	/**
	 * The element at row 0 column 2.
	 */
	public double e02=0;

	/**
	 * The element at row 0 column 3.
	 */
	public double e03=0;

	/**
	 * The element at row 1 column 0.
	 */
	public double e10=0;

	/**
	 * The element at row 1 column 1.
	 */
	public double e11=1;

	/**
	 * The element at row 1 column 2.
	 */
	public double e12=0;

	/**
	 * The element at row 1 column 3.
	 */
	public double e13=0;

	/**
	 * The element at row 2 column 0.
	 */
	public double e20=0;

	/**
	 * The element at row 2 column 1.
	 */
	public double e21=0;

	/**
	 * The element at row 2 column 2.
	 */
	public double e22=1;

	/**
	 * The element at row 2 column 3.
	 */
	public double e23=0;

	/**
	 * The element at row 3 column 0.
	 */
	public double e30=0;

	/**
	 * The element at row 3 column 1.
	 */
	public double e31=0;

	/**
	 * The element at row 3 column 2.
	 */
	public double e32=0;

	/**
	 * The element at row 3 column 3.
	 */
	public double e33=1;

	/**
	 * Creates identity 4x4 matrix
	 */
	public Mat4() {
		numCreations++;
	}
	
	
	/**
	 * Creates a new matrix. The matrix is identity by default.
	 */
	public Mat4(
		double e00 ,double e01,double e02,double e03,
		double e10 ,double e11,double e12,double e13,
		double e20 ,double e21,double e22,double e23,
		double e30 ,double e31,double e32,double e33
	) {
		this.e00 = e00;
		this.e01 = e01;
		this.e02 = e02;
		this.e03 = e03;
		this.e10 = e10;
		this.e11 = e11;
		this.e12 = e12;
		this.e13 = e13;
		this.e20 = e20;
		this.e21 = e21;
		this.e22 = e22;
		this.e23 = e23;
		this.e30 = e30;
		this.e31 = e31;
		this.e32 = e32;
		this.e33 = e33;
		numCreations++;
	}

	public Mat4 set(
			double e00 ,double e01,double e02,double e03,
			double e10 ,double e11,double e12,double e13,
			double e20 ,double e21,double e22,double e23,
			double e30 ,double e31,double e32,double e33
		) {
			this.e00 = e00;
			this.e01 = e01;
			this.e02 = e02;
			this.e03 = e03;
			this.e10 = e10;
			this.e11 = e11;
			this.e12 = e12;
			this.e13 = e13;
			this.e20 = e20;
			this.e21 = e21;
			this.e22 = e22;
			this.e23 = e23;
			this.e30 = e30;
			this.e31 = e31;
			this.e32 = e32;
			this.e33 = e33;
			numCreations++;
			return this;
		}

	
	/**
	 * Sets this matrix to identity matrix and returns `this`.
	 */
	public Mat4 identity() {
		set(
			1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1
		);
		return this;
	}


	/**
	 * Copies values from `m` and returns `this`.
	 */
	public Mat4 copyFrom(Mat4 m) {
		e00 = m.e00;
		e01 = m.e01;
		e02 = m.e02;
		e03 = m.e03;
		e10 = m.e10;
		e11 = m.e11;
		e12 = m.e12;
		e13 = m.e13;
		e20 = m.e20;
		e21 = m.e21;
		e22 = m.e22;
		e23 = m.e23;
		e30 = m.e30;
		e31 = m.e31;
		e32 = m.e32;
		e33 = m.e33;
		return this;
	}


	/**
	 * Returns a clone of the matrix.
	 */
	public Mat4 clone() {
		return new Mat4(
			e00, e01, e02, e03,
			e10, e11, e12, e13,
			e20, e21, e22, e23,
			e30, e31, e32, e33
		);
	}

	
	
	/**
	 * Returns `this` + `m`
	 */
	public Mat4 add(Mat4 m) {
		return new Mat4(
			e00 + m.e00, e01 + m.e01, e02 + m.e02, e03 + m.e03,
			e10 + m.e10, e11 + m.e11, e12 + m.e12, e13 + m.e13,
			e20 + m.e20, e21 + m.e21, e22 + m.e22, e23 + m.e23,
			e30 + m.e30, e31 + m.e31, e32 + m.e32, e33 + m.e33
		);
	}

	/**
	 * Returns `this` - `m`
	 */
	public Mat4 sub(Mat4 m) {
		return new Mat4(
			e00 - m.e00, e01 - m.e01, e02 - m.e02, e03 - m.e03,
			e10 - m.e10, e11 - m.e11, e12 - m.e12, e13 - m.e13,
			e20 - m.e20, e21 - m.e21, e22 - m.e22, e23 - m.e23,
			e30 - m.e30, e31 - m.e31, e32 - m.e32, e33 - m.e33
		);
	}

	/**
	 * Returns `this` * `s`
	 */
	public Mat4 scale(double s) {
		return new Mat4(
			e00 * s, e01 * s, e02 * s, e03 * s,
			e10 * s, e11 * s, e12 * s, e13 * s,
			e20 * s, e21 * s, e22 * s, e23 * s,
			e30 * s, e31 * s, e32 * s, e33 * s
		);
	}

	/**
	 * Returns `this` * `m`
	 */
	public Mat4 mul(Mat4 m) {
		return new Mat4(
			e00 * m.e00 + e01 * m.e10 + e02 * m.e20 + e03 * m.e30,
			e00 * m.e01 + e01 * m.e11 + e02 * m.e21 + e03 * m.e31,
			e00 * m.e02 + e01 * m.e12 + e02 * m.e22 + e03 * m.e32,
			e00 * m.e03 + e01 * m.e13 + e02 * m.e23 + e03 * m.e33,
			e10 * m.e00 + e11 * m.e10 + e12 * m.e20 + e13 * m.e30,
			e10 * m.e01 + e11 * m.e11 + e12 * m.e21 + e13 * m.e31,
			e10 * m.e02 + e11 * m.e12 + e12 * m.e22 + e13 * m.e32,
			e10 * m.e03 + e11 * m.e13 + e12 * m.e23 + e13 * m.e33,
			e20 * m.e00 + e21 * m.e10 + e22 * m.e20 + e23 * m.e30,
			e20 * m.e01 + e21 * m.e11 + e22 * m.e21 + e23 * m.e31,
			e20 * m.e02 + e21 * m.e12 + e22 * m.e22 + e23 * m.e32,
			e20 * m.e03 + e21 * m.e13 + e22 * m.e23 + e23 * m.e33,
			e30 * m.e00 + e31 * m.e10 + e32 * m.e20 + e33 * m.e30,
			e30 * m.e01 + e31 * m.e11 + e32 * m.e21 + e33 * m.e31,
			e30 * m.e02 + e31 * m.e12 + e32 * m.e22 + e33 * m.e32,
			e30 * m.e03 + e31 * m.e13 + e32 * m.e23 + e33 * m.e33
		);
	}

	/**
	 * Sets this matrix to `this` + `m` and returns `this`.
	 */
	public Mat4 addEq(Mat4 m) {
		return set(
			e00 + m.e00, e01 + m.e01, e02 + m.e02, e03 + m.e03,
			e10 + m.e10, e11 + m.e11, e12 + m.e12, e13 + m.e13,
			e20 + m.e20, e21 + m.e21, e22 + m.e22, e23 + m.e23,
			e30 + m.e30, e31 + m.e31, e32 + m.e32, e33 + m.e33
		);
	}

	/**
	 * Sets this matrix to `this` - `m` and returns `this`.
	 */
	public Mat4 subEq(Mat4 m) {
		return set(
			e00 - m.e00, e01 - m.e01, e02 - m.e02, e03 - m.e03,
			e10 - m.e10, e11 - m.e11, e12 - m.e12, e13 - m.e13,
			e20 - m.e20, e21 - m.e21, e22 - m.e22, e23 - m.e23,
			e30 - m.e30, e31 - m.e31, e32 - m.e32, e33 - m.e33
		);
	}

	/**
	 * Sets this matrix to `this` * `s` and returns `this`.
	 */
	public Mat4 scaleEq(double s) {
		return set(
			e00 * s, e01 * s, e02 * s, e03 * s,
			e10 * s, e11 * s, e12 * s, e13 * s,
			e20 * s, e21 * s, e22 * s, e23 * s,
			e30 * s, e31 * s, e32 * s, e33 * s
		);
	}

	/**
	 * Sets this matrix to `this` * `m` and returns `this`.
	 */
	public Mat4 mulEq(Mat4 m) {
		return set(
			e00 * m.e00 + e01 * m.e10 + e02 * m.e20 + e03 * m.e30,
			e00 * m.e01 + e01 * m.e11 + e02 * m.e21 + e03 * m.e31,
			e00 * m.e02 + e01 * m.e12 + e02 * m.e22 + e03 * m.e32,
			e00 * m.e03 + e01 * m.e13 + e02 * m.e23 + e03 * m.e33,
			e10 * m.e00 + e11 * m.e10 + e12 * m.e20 + e13 * m.e30,
			e10 * m.e01 + e11 * m.e11 + e12 * m.e21 + e13 * m.e31,
			e10 * m.e02 + e11 * m.e12 + e12 * m.e22 + e13 * m.e32,
			e10 * m.e03 + e11 * m.e13 + e12 * m.e23 + e13 * m.e33,
			e20 * m.e00 + e21 * m.e10 + e22 * m.e20 + e23 * m.e30,
			e20 * m.e01 + e21 * m.e11 + e22 * m.e21 + e23 * m.e31,
			e20 * m.e02 + e21 * m.e12 + e22 * m.e22 + e23 * m.e32,
			e20 * m.e03 + e21 * m.e13 + e22 * m.e23 + e23 * m.e33,
			e30 * m.e00 + e31 * m.e10 + e32 * m.e20 + e33 * m.e30,
			e30 * m.e01 + e31 * m.e11 + e32 * m.e21 + e33 * m.e31,
			e30 * m.e02 + e31 * m.e12 + e32 * m.e22 + e33 * m.e32,
			e30 * m.e03 + e31 * m.e13 + e32 * m.e23 + e33 * m.e33
		);
	}

	
	/**
	 * Returns *scaling matrix* * `this`.
	 *
	 * Where *scaling matrix* is a matrix which scales `sx` times, `sy` times and
	 * `sz` times along the x-axis, y-axis and z-axis respectively.
	 */
	public Mat4 prependScale(double sx, double sy, double sz) {
		return new Mat4(
			e00 * sx, e01 * sx, e02 * sx, e03 * sx,
			e10 * sy, e11 * sy, e12 * sy, e13 * sy,
			e20 * sz, e21 * sz, e22 * sz, e23 * sz,
			e30, e31, e32, e33
		);
	}

	/**
	 * Returns `this` * *scaling matrix*.
	 *
	 * Where *scaling matrix* is a matrix which scales `sx` times, `sy` times and
	 * `sz` times along the x-axis, y-axis and z-axis respectively.
	 */
	public Mat4 appendScale(double sx, double sy, double sz) {
		return new Mat4(
			e00 * sx, e01 * sy, e02 * sz, e03,
			e10 * sx, e11 * sy, e12 * sz, e13,
			e20 * sx, e21 * sy, e22 * sz, e23,
			e30 * sx, e31 * sy, e32 * sz, e33
		);
	}

	/**
	 * Returns *rotation matrix* * `this`.
	 *
	 * Where *rotation matrix* is a matrix which rotates `rad` in radians around the **normalized**
	 * vector (`axisX`, `axisY`, `axisZ`).
	 */
	public Mat4 prependRotation(double rad, double axisX, double axisY, double axisZ) {
		double s = MathUtil.sin(rad);
		double c = MathUtil.cos(rad);
		double c1 = 1 - c;
		double r00 = axisX * axisX * c1 + c;
		double r01 = axisX * axisY * c1 - axisZ * s;
		double r02 = axisX * axisZ * c1 + axisY * s;
		double r10 = axisY * axisX * c1 + axisZ * s;
		double r11 = axisY * axisY * c1 + c;
		double r12 = axisY * axisZ * c1 - axisX * s;
		double r20 = axisZ * axisX * c1 - axisY * s;
		double r21 = axisZ * axisY * c1 + axisX * s;
		double r22 = axisZ * axisZ * c1 + c;
		return new Mat4(
			r00 * e00 + r01 * e10 + r02 * e20,
			r00 * e01 + r01 * e11 + r02 * e21,
			r00 * e02 + r01 * e12 + r02 * e22,
			r00 * e03 + r01 * e13 + r02 * e23,
			r10 * e00 + r11 * e10 + r12 * e20,
			r10 * e01 + r11 * e11 + r12 * e21,
			r10 * e02 + r11 * e12 + r12 * e22,
			r10 * e03 + r11 * e13 + r12 * e23,
			r20 * e00 + r21 * e10 + r22 * e20,
			r20 * e01 + r21 * e11 + r22 * e21,
			r20 * e02 + r21 * e12 + r22 * e22,
			r20 * e03 + r21 * e13 + r22 * e23,
			e30, e31, e32, e33
		);
	}

	/**
	 * Returns `this` * *rotation matrix*.
	 *
	 * Where *rotation matrix* is a matrix which rotates `rad` in radians around the **normalized**
	 * vector (`axisX`, `axisY`, `axisZ`).
	 */
	public Mat4 appendRotation(double rad, double axisX, double axisY, double axisZ) {
		double s = MathUtil.sin(rad);
		double c = MathUtil.cos(rad);
		double c1 = 1 - c;
		double r00 = axisX * axisX * c1 + c;
		double r01 = axisX * axisY * c1 - axisZ * s;
		double r02 = axisX * axisZ * c1 + axisY * s;
		double r10 = axisY * axisX * c1 + axisZ * s;
		double r11 = axisY * axisY * c1 + c;
		double r12 = axisY * axisZ * c1 - axisX * s;
		double r20 = axisZ * axisX * c1 - axisY * s;
		double r21 = axisZ * axisY * c1 + axisX * s;
		double r22 = axisZ * axisZ * c1 + c;
		return new Mat4(
			e00 * r00 + e01 * r10 + e02 * r20,
			e00 * r01 + e01 * r11 + e02 * r21,
			e00 * r02 + e01 * r12 + e02 * r22,
			e03,
			e10 * r00 + e11 * r10 + e12 * r20,
			e10 * r01 + e11 * r11 + e12 * r21,
			e10 * r02 + e11 * r12 + e12 * r22,
			e13,
			e20 * r00 + e21 * r10 + e22 * r20,
			e20 * r01 + e21 * r11 + e22 * r21,
			e20 * r02 + e21 * r12 + e22 * r22,
			e23,
			e30 * r00 + e31 * r10 + e32 * r20,
			e30 * r01 + e31 * r11 + e32 * r21,
			e30 * r02 + e31 * r12 + e32 * r22,
			e33
		);
	}

	/**
	 * Returns *translation matrix* * `this`.
	 *
	 * Where *translation matrix* is a matrix which translates `sx`, `sy` and `sz` along
	 * the x-axis, y-axis and z-axis respectively.
	 */
	public Mat4 prependTranslation(double tx,double ty,double tz) {
		return new Mat4(
			e00 + tx * e30, e01 + tx * e31, e02 + tx * e32, e03 + tx * e33,
			e10 + ty * e30, e11 + ty * e31, e12 + ty * e32, e13 + ty * e33,
			e20 + tz * e30, e21 + tz * e31, e22 + tz * e32, e23 + tz * e33,
			e30, e31, e32, e33
		);
	}

	/**
	 * Returns `this` * *translation matrix*.
	 *
	 * Where *translation matrix* is a matrix which translates `sx`, `sy` and `sz` along
	 * the x-axis, y-axis and z-axis respectively.
	 */
	public Mat4 appendTranslation(double tx,double ty,double tz) {
		return new Mat4(
			e00, e01, e02, e00 * tx + e01 * ty + e02 * tz + e03,
			e10, e11, e12, e10 * tx + e11 * ty + e12 * tz + e13,
			e20, e21, e22, e20 * tx + e21 * ty + e22 * tz + e23,
			e30, e31, e32, e30 * tx + e31 * ty + e32 * tz + e33
		);
	}
	
	/**
	 * Sets this matrix to *scaling matrix* * `this`, and returns `this`.
	 *
	 * Where *scaling matrix* is a matrix which scales `sx` times, `sy` times and
	 * `sz` times along the x-axis, y-axis and z-axis respectively.
	 */
	public Mat4 prependScaleEq(double sx, double sy,double sz) {
		return set(
			e00 * sx, e01 * sx, e02 * sx, e03 * sx,
			e10 * sy, e11 * sy, e12 * sy, e13 * sy,
			e20 * sz, e21 * sz, e22 * sz, e23 * sz,
			e30, e31, e32, e33
		);
	}

	/**
	 * Sets this matrix to `this` * *scaling matrix*, and returns `this`.
	 *
	 * Where *scaling matrix* is a matrix which scales `sx` times, `sy` times and
	 * `sz` times along the x-axis, y-axis and z-axis respectively.
	 */
	public Mat4 appendScaleEq(double sx,double sy,double sz) {
		return set(
			e00 * sx, e01 * sy, e02 * sz, e03,
			e10 * sx, e11 * sy, e12 * sz, e13,
			e20 * sx, e21 * sy, e22 * sz, e23,
			e30 * sx, e31 * sy, e32 * sz, e33
		);
	}

	/**
	 * Sets this matrix to *rotation matrix* * `this`, and returns `this`.
	 *
	 * Where *rotation matrix* is a matrix which rotates `rad` in radians around the **normalized**
	 * vector (`axisX`, `axisY`, `axisZ`).
	 */
	public Mat4 prependRotationEq(double rad,double axisX,double axisY,double axisZ) {
		double s = MathUtil.sin(rad);
		double c = MathUtil.cos(rad);
		double c1 = 1 - c;
		double r00 = axisX * axisX * c1 + c;
		double r01 = axisX * axisY * c1 - axisZ * s;
		double r02 = axisX * axisZ * c1 + axisY * s;
		double r10 = axisY * axisX * c1 + axisZ * s;
		double r11 = axisY * axisY * c1 + c;
		double r12 = axisY * axisZ * c1 - axisX * s;
		double r20 = axisZ * axisX * c1 - axisY * s;
		double r21 = axisZ * axisY * c1 + axisX * s;
		double r22 = axisZ * axisZ * c1 + c;
		return set(
			r00 * e00 + r01 * e10 + r02 * e20, r00 * e01 + r01 * e11 + r02 * e21, r00 * e02 + r01 * e12 + r02 * e22, r00 * e03 + r01 * e13 + r02 * e23,
			r10 * e00 + r11 * e10 + r12 * e20, r10 * e01 + r11 * e11 + r12 * e21, r10 * e02 + r11 * e12 + r12 * e22, r10 * e03 + r11 * e13 + r12 * e23,
			r20 * e00 + r21 * e10 + r22 * e20, r20 * e01 + r21 * e11 + r22 * e21, r20 * e02 + r21 * e12 + r22 * e22, r20 * e03 + r21 * e13 + r22 * e23,
			e30, e31, e32, e33
		);
	}

	/**
	 * Sets this matrix to `this` * *rotation matrix*, and returns `this`.
	 *
	 * Where *rotation matrix* is a matrix which rotates `rad` in radians around the **normalized**
	 * vector (`axisX`, `axisY`, `axisZ`).
	 */
	public Mat4 appendRotationEq(double rad,double axisX,double axisY,double axisZ) {
		double s = MathUtil.sin(rad);
		double c = MathUtil.cos(rad);
		double c1 = 1 - c;
		double r00 = axisX * axisX * c1 + c;
		double r01 = axisX * axisY * c1 - axisZ * s;
		double r02 = axisX * axisZ * c1 + axisY * s;
		double r10 = axisY * axisX * c1 + axisZ * s;
		double r11 = axisY * axisY * c1 + c;
		double r12 = axisY * axisZ * c1 - axisX * s;
		double r20 = axisZ * axisX * c1 - axisY * s;
		double r21 = axisZ * axisY * c1 + axisX * s;
		double r22 = axisZ * axisZ * c1 + c;
		return set(
			e00 * r00 + e01 * r10 + e02 * r20, e00 * r01 + e01 * r11 + e02 * r21, e00 * r02 + e01 * r12 + e02 * r22, e03,
			e10 * r00 + e11 * r10 + e12 * r20, e10 * r01 + e11 * r11 + e12 * r21, e10 * r02 + e11 * r12 + e12 * r22, e13,
			e20 * r00 + e21 * r10 + e22 * r20, e20 * r01 + e21 * r11 + e22 * r21, e20 * r02 + e21 * r12 + e22 * r22, e23,
			e30 * r00 + e31 * r10 + e32 * r20, e30 * r01 + e31 * r11 + e32 * r21, e30 * r02 + e31 * r12 + e32 * r22, e33
		);
	}

	/**
	 * Sets this matrix to *translation matrix* * `this`, and returns `this`.
	 *
	 * Where *translation matrix* is a matrix which translates `sx`, `sy` and `sz` along
	 * the x-axis, y-axis and z-axis respectively.
	 */
	public Mat4 prependTranslationEq(double tx, double ty,double tz) {
		return set(
			e00 + tx * e30, e01 + tx * e31, e02 + tx * e32, e03 + tx * e33,
			e10 + ty * e30, e11 + ty * e31, e12 + ty * e32, e13 + ty * e33,
			e20 + tz * e30, e21 + tz * e31, e22 + tz * e32, e23 + tz * e33,
			e30, e31, e32, e33
		);
	}

	/**
	 * Sets this matrix to `this` * *translation matrix*, and returns `this`.
	 *
	 * Where *translation matrix* is a matrix which translates `sx`, `sy` and `sz` along
	 * the x-axis, y-axis and z-axis respectively.
	 */
	public Mat4 appendTranslationEq(double tx,double ty,double tz) {
		return set(
			e00, e01, e02, e00 * tx + e01 * ty + e02 * tz + e03,
			e10, e11, e12, e10 * tx + e11 * ty + e12 * tz + e13,
			e20, e21, e22, e20 * tx + e21 * ty + e22 * tz + e23,
			e30, e31, e32, e30 * tx + e31 * ty + e32 * tz + e33
		);
	}
	
	/**
	 * Returns the transposed matrix.
	 */
	public Mat4 transpose() {
		return new Mat4(
			e00, e10, e20, e30,
			e01, e11, e21, e31,
			e02, e12, e22, e32,
			e03, e13, e23, e33
		);
	}

	/**
	 * Sets this matrix to the transposed matrix and returns `this`.
	 */
	public Mat4 transposeEq() {
		return set(
			e00, e10, e20, e30,
			e01, e11, e21, e31,
			e02, e12, e22, e32,
			e03, e13, e23, e33
		);
	}

	/**
	 * Returns the determinant.
	 */
	public double determinant() {
		double d23_01 = e20 * e31 - e21 * e30;
		double d23_02 = e20 * e32 - e22 * e30;
		double d23_03 = e20 * e33 - e23 * e30;
		double d23_12 = e21 * e32 - e22 * e31;
		double d23_13 = e21 * e33 - e23 * e31;
		double d23_23 = e22 * e33 - e23 * e32;
		return
			e00 * (e11 * d23_23 - e12 * d23_13 + e13 * d23_12) -
			e01 * (e10 * d23_23 - e12 * d23_03 + e13 * d23_02) +
			e02 * (e10 * d23_13 - e11 * d23_03 + e13 * d23_01) -
			e03 * (e10 * d23_12 - e11 * d23_02 + e12 * d23_01)
		;
	}

	/**
	 * Returns the trace.
	 */
	public double trace() {
		return e00 + e11 + e22 + e33;
	}

	/**
	 * Returns the inverse matrix.
	 *
	 * If the determinant is zero, zero matrix is returned.
	 */
	public Mat4 inverse() {
		double d01_01 = e00 * e11 - e01 * e10;
		double d01_02 = e00 * e12 - e02 * e10;
		double d01_03 = e00 * e13 - e03 * e10;
		double d01_12 = e01 * e12 - e02 * e11;
		double d01_13 = e01 * e13 - e03 * e11;
		double d01_23 = e02 * e13 - e03 * e12;
		double d23_01 = e20 * e31 - e21 * e30;
		double d23_02 = e20 * e32 - e22 * e30;
		double d23_03 = e20 * e33 - e23 * e30;
		double d23_12 = e21 * e32 - e22 * e31;
		double d23_13 = e21 * e33 - e23 * e31;
		double d23_23 = e22 * e33 - e23 * e32;
		double d00 = e11 * d23_23 - e12 * d23_13 + e13 * d23_12;
		double d01 = e10 * d23_23 - e12 * d23_03 + e13 * d23_02;
		double d02 = e10 * d23_13 - e11 * d23_03 + e13 * d23_01;
		double d03 = e10 * d23_12 - e11 * d23_02 + e12 * d23_01;
		double d10 = e01 * d23_23 - e02 * d23_13 + e03 * d23_12;
		double d11 = e00 * d23_23 - e02 * d23_03 + e03 * d23_02;
		double d12 = e00 * d23_13 - e01 * d23_03 + e03 * d23_01;
		double d13 = e00 * d23_12 - e01 * d23_02 + e02 * d23_01;
		double d20 = e31 * d01_23 - e32 * d01_13 + e33 * d01_12;
		double d21 = e30 * d01_23 - e32 * d01_03 + e33 * d01_02;
		double d22 = e30 * d01_13 - e31 * d01_03 + e33 * d01_01;
		double d23 = e30 * d01_12 - e31 * d01_02 + e32 * d01_01;
		double d30 = e21 * d01_23 - e22 * d01_13 + e23 * d01_12;
		double d31 = e20 * d01_23 - e22 * d01_03 + e23 * d01_02;
		double d32 = e20 * d01_13 - e21 * d01_03 + e23 * d01_01;
		double d33 = e20 * d01_12 - e21 * d01_02 + e22 * d01_01;
		double invDet = e00 * d00 - e01 * d01 + e02 * d02 - e03 * d03;
		if (invDet != 0) invDet = 1 / invDet;
		return new Mat4(
			d00 * invDet, -d10 * invDet, d20 * invDet, -d30 * invDet,
			-d01 * invDet, d11 * invDet, -d21 * invDet, d31 * invDet,
			d02 * invDet, -d12 * invDet, d22 * invDet, -d32 * invDet,
			-d03 * invDet, d13 * invDet, -d23 * invDet, d33 * invDet
		);
	}

	/**
	 * Sets this matrix to the inverse matrix and returns `this`.
	 *
	 * If the determinant is zero, this matrix is set to zero matrix.
	 */
	public Mat4 inverseEq() {
		double d01_01 = e00 * e11 - e01 * e10;
		double d01_02 = e00 * e12 - e02 * e10;
		double d01_03 = e00 * e13 - e03 * e10;
		double d01_12 = e01 * e12 - e02 * e11;
		double d01_13 = e01 * e13 - e03 * e11;
		double d01_23 = e02 * e13 - e03 * e12;
		double d23_01 = e20 * e31 - e21 * e30;
		double d23_02 = e20 * e32 - e22 * e30;
		double d23_03 = e20 * e33 - e23 * e30;
		double d23_12 = e21 * e32 - e22 * e31;
		double d23_13 = e21 * e33 - e23 * e31;
		double d23_23 = e22 * e33 - e23 * e32;
		double d00 = e11 * d23_23 - e12 * d23_13 + e13 * d23_12;
		double d01 = e10 * d23_23 - e12 * d23_03 + e13 * d23_02;
		double d02 = e10 * d23_13 - e11 * d23_03 + e13 * d23_01;
		double d03 = e10 * d23_12 - e11 * d23_02 + e12 * d23_01;
		double d10 = e01 * d23_23 - e02 * d23_13 + e03 * d23_12;
		double d11 = e00 * d23_23 - e02 * d23_03 + e03 * d23_02;
		double d12 = e00 * d23_13 - e01 * d23_03 + e03 * d23_01;
		double d13 = e00 * d23_12 - e01 * d23_02 + e02 * d23_01;
		double d20 = e31 * d01_23 - e32 * d01_13 + e33 * d01_12;
		double d21 = e30 * d01_23 - e32 * d01_03 + e33 * d01_02;
		double d22 = e30 * d01_13 - e31 * d01_03 + e33 * d01_01;
		double d23 = e30 * d01_12 - e31 * d01_02 + e32 * d01_01;
		double d30 = e21 * d01_23 - e22 * d01_13 + e23 * d01_12;
		double d31 = e20 * d01_23 - e22 * d01_03 + e23 * d01_02;
		double d32 = e20 * d01_13 - e21 * d01_03 + e23 * d01_01;
		double d33 = e20 * d01_12 - e21 * d01_02 + e22 * d01_01;
		double invDet = e00 * d00 - e01 * d01 + e02 * d02 - e03 * d03;
		if (invDet != 0) invDet = 1 / invDet;
		return set(
			d00 * invDet, -d10 * invDet, d20 * invDet, -d30 * invDet,
			-d01 * invDet, d11 * invDet, -d21 * invDet, d31 * invDet,
			d02 * invDet, -d12 * invDet, d22 * invDet, -d32 * invDet,
			-d03 * invDet, d13 * invDet, -d23 * invDet, d33 * invDet
		);
	}

	/**
	 * Sets this matrix to *view matrix* and returns `this`.
	 *
	 * Where *view matrix* is a matrix which represents the viewing transformation with
	 * eyes at (`eyeX`, `eyeY`, `eyeZ`), fixation point at (`atX`, `atY`, `atZ`), and
	 * up vector (`upX`, `upY`, `upZ`).
	 */
	public Mat4 lookAt(double eyeX, double eyeY, double eyeZ, double atX, double atY, double atZ, double upX, double upY, double upZ) {
		double zx = eyeX - atX;
		double zy = eyeY - atY;
		double zz = eyeZ - atZ;
		double tmp = 1 / MathUtil.sqrt(zx * zx + zy * zy + zz * zz);
		zx *= tmp;
		zy *= tmp;
		zz *= tmp;
		double xx = upY * zz - upZ * zy;
		double xy = upZ * zx - upX * zz;
		double xz = upX * zy - upY * zx;
		tmp = 1 / MathUtil.sqrt(xx * xx + xy * xy + xz * xz);
		xx *= tmp;
		xy *= tmp;
		xz *= tmp;
		double yx = zy * xz - zz * xy;
		double yy = zz * xx - zx * xz;
		double yz = zx * xy - zy * xx;
		e00 = xx;
		e01 = xy;
		e02 = xz;
		e03 = -(xx * eyeX + xy * eyeY + xz * eyeZ);
		e10 = yx;
		e11 = yy;
		e12 = yz;
		e13 = -(yx * eyeX + yy * eyeY + yz * eyeZ);
		e20 = zx;
		e21 = zy;
		e22 = zz;
		e23 = -(zx * eyeX + zy * eyeY + zz * eyeZ);
		e30 = 0;
		e31 = 0;
		e32 = 0;
		e33 = 1;
		return this;
	}

	/**
	 * Sets this matrix to *perspecive projection matrix* and returns `this`.
	 *
	 * Where *perspecive projection matrix* is a matrix which represents the perspective
	 * projection transformation with field of view in the y direction `fovY` in radians,
	 * aspect ratio `aspect`, and z-value of near and far clipping plane `near`, `far`.
	 */
	public Mat4 perspective(double fovY, double aspect, double near, double far) {
		double h = 1 / MathUtil.tan(fovY * 0.5);
		double fnf = far / (near - far);
		e00 = h / aspect;
		e01 = 0;
		e02 = 0;
		e03 = 0;
		e10 = 0;
		e11 = h;
		e12 = 0;
		e13 = 0;
		e20 = 0;
		e21 = 0;
		e22 = fnf;
		e23 = near * fnf;
		e30 = 0;
		e31 = 0;
		e32 = -1;
		e33 = 0;
		return this;
	}

	/**
	 * Sets this matrix to *orthogonal projection matrix* and returns `this`.
	 *
	 * Where *orthogonal projection matrix* is a matrix which represents the orthogonal
	 * projection transformation with screen width and height `width`, `height`, and
	 * z-value of near and far clipping plane `near`, `far`.
	 */
	public Mat4  ortho(double width, double height,double near, double far) {
		double nf = 1 / (near - far);
		e00 = 2 / width;
		e01 = 0;
		e02 = 0;
		e03 = 0;
		e10 = 0;
		e11 = 2 / height;
		e12 = 0;
		e13 = 0;
		e20 = 0;
		e21 = 0;
		e22 = nf;
		e23 = near * nf;
		e30 = 0;
		e31 = 0;
		e32 = 0;
		e33 = 1;
		return this;
	}

	/**
	 * Returns an array of the elements of this matrix.
	 *
	 * If `columnMajor` is true, the array is arranged in column-major order.
	 * Otherwise, the array is arranged in row-major order.
	 */
	public float[] toArray(boolean columnMajor) {
		if (columnMajor) {
			return new float[] {
				(float) e00, (float)e10, (float)e20, (float)e30,
				(float)e01, (float)e11,(float) e21, (float)e31,
				(float)e02, (float)e12, (float)e22, (float)e32,
				(float)e03,(float) e13, (float)e23, (float)e33
			};
		} else {
			return new float[] {
				(float)e00, (float)e01,(float) e02, (float)e03,
				(float)e10, (float)e11, (float)e12, (float)e13,
				(float)e20, (float)e21, (float)e22, (float)e23,
				(float)e30, (float)e31, (float)e32, (float)e33
			};
		}
	}
	


	/**
	 * Sets this matrix to the extension of `m` and returns `this`.
	 *
	 * `this.e33` is set to `1` and other components don't exist in `m` are set to `0`.
	 */
	public Mat4 fromMat3(Mat3 m) {
		return set(
			m.e00, m.e01, m.e02, 0,
			m.e10, m.e11, m.e12, 0,
			m.e20, m.e21, m.e22, 0,
			0, 0, 0, 1
		);
	}

	/**
	 * Sets this matrix to the representation of `transform` and returns `this`.
	 */
	public Mat4 fromTransform(Transform transform) {
		M.transform_toMat4(this, transform._position, transform._rotation);
		return this;
	}

	/**
	 * Returns the string representation of the matrix.
	 */
	public String toString() {
		return
			"Mat4[" + M.toFixed8(e00) + ", " + M.toFixed8(e01) + ", " + M.toFixed8(e02) + ", " + M.toFixed8(e03) + ",\n" +
			"    " + M.toFixed8(e10) + ", " + M.toFixed8(e11) + ", " + M.toFixed8(e12) + ", " + M.toFixed8(e13) + ",\n" +
			"    " + M.toFixed8(e20) + ", " + M.toFixed8(e21) + ", " + M.toFixed8(e22) + ", " + M.toFixed8(e23) + ",\n" +
			"    " + M.toFixed8(e30) + ", " + M.toFixed8(e31) + ", " + M.toFixed8(e32) + ", " + M.toFixed8(e33) + "]"
		;
	}
}