package oimo.common;

/**
 * 3x3 Matrix class.
 *
 * Note that columns and rows are 0-indexed.
 */
public class Mat3 {
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
	 * Creates identity Matrix 
	 */
	public Mat3() {
		numCreations++;
	}

	/**
	 * Creates a new matrix. The matrix is identity by default.
	 */
	public Mat3(double e00,double e01,double e02,
			double e10,double e11,double e12,
			double e20,double e21,double e22) {
		this.e00 = e00;
		this.e01 = e01;
		this.e02 = e02;
		this.e10 = e10;
		this.e11 = e11;
		this.e12 = e12;
		this.e20 = e20;
		this.e21 = e21;
		this.e22 = e22;
		numCreations++;
	}

	public Mat3 set(double e00,double e01,double e02,
			double e10,double e11,double e12,
			double e20,double e21,double e22) {
		this.e00 = e00;
		this.e01 = e01;
		this.e02 = e02;
		this.e10 = e10;
		this.e11 = e11;
		this.e12 = e12;
		this.e20 = e20;
		this.e21 = e21;
		this.e22 = e22;
		return this;
	}

	/**
	 * Sets this matrix to null matrix and returns `this`.
	 */
	public Mat3 zero() {
		set(
			0, 0, 0,
			0, 0, 0,
			0, 0, 0
		);
		return this;
	}
	
	/**
	 * Sets this matrix to identity matrix and returns `this`.
	 */
	public Mat3 identity() {
		set(
			1, 0, 0,
			0, 1, 0,
			0, 0, 1
		);
		return this;
	}

	/**
	 * Returns new  `this` + `m`
	 */
	public Mat3 add(Mat3 m) {
		return new Mat3(
			e00 + m.e00, e01 + m.e01, e02 + m.e02,
			e10 + m.e10, e11 + m.e11, e12 + m.e12,
			e20 + m.e20, e21 + m.e21, e22 + m.e22
		);
	}

	/**
	 * Returns new `this` - `m`
	 */
	public Mat3 sub(Mat3 m) {
		return new Mat3(
			e00 - m.e00, e01 - m.e01, e02 - m.e02,
			e10 - m.e10, e11 - m.e11, e12 - m.e12,
			e20 - m.e20, e21 - m.e21, e22 - m.e22
		);
	}

	/**
	 * Returns new `this` * `s`
	 */
	public Mat3 scale(double s) {
		return new Mat3(
			e00 * s, e01 * s, e02 * s,
			e10 * s, e11 * s, e12 * s,
			e20 * s, e21 * s, e22 * s
		);
	}

	/**
	 * Returns new `this` * `m`
	 */
	public Mat3 mul(Mat3 m) {
		return new Mat3(
			e00 * m.e00 + e01 * m.e10 + e02 * m.e20,
			e00 * m.e01 + e01 * m.e11 + e02 * m.e21,
			e00 * m.e02 + e01 * m.e12 + e02 * m.e22,
			e10 * m.e00 + e11 * m.e10 + e12 * m.e20,
			e10 * m.e01 + e11 * m.e11 + e12 * m.e21,
			e10 * m.e02 + e11 * m.e12 + e12 * m.e22,
			e20 * m.e00 + e21 * m.e10 + e22 * m.e20,
			e20 * m.e01 + e21 * m.e11 + e22 * m.e21,
			e20 * m.e02 + e21 * m.e12 + e22 * m.e22
		);
	}
	
	/**
	 * Returns new `this` * `m transpose`
	 */
	public Mat3 mulTranspose(Mat3 m) {
		return new Mat3(
			e00 * m.e00 + e01 * m.e01 + e02 * m.e02,
			e00 * m.e10 + e01 * m.e11 + e02 * m.e12,
			e00 * m.e20 + e01 * m.e21 + e02 * m.e22,
			e10 * m.e00 + e11 * m.e01 + e12 * m.e02,
			e10 * m.e10 + e11 * m.e11 + e12 * m.e12,
			e10 * m.e20 + e11 * m.e21 + e12 * m.e22,
			e20 * m.e00 + e21 * m.e01 + e22 * m.e02,
			e20 * m.e10 + e21 * m.e11 + e22 * m.e12,
			e20 * m.e20 + e21 * m.e21 + e22 * m.e22
		);
	}

	/**
	 * Sets this matrix to `this` + `m` and returns `this`.
	 */
	public Mat3 addEq(Mat3 m) {
		return set(
			e00 + m.e00, e01 + m.e01, e02 + m.e02,
			e10 + m.e10, e11 + m.e11, e12 + m.e12,
			e20 + m.e20, e21 + m.e21, e22 + m.e22
		);
	}

	/**
	 * Sets this matrix to `this` - `m` and returns `this`.
	 */
	public Mat3 subEq(Mat3 m) {
		return set(
			e00 - m.e00, e01 - m.e01, e02 - m.e02,
			e10 - m.e10, e11 - m.e11, e12 - m.e12,
			e20 - m.e20, e21 - m.e21, e22 - m.e22
		);
	}

	/**
	 * Sets this matrix to `this` * `s` and returns `this`.
	 */
	public Mat3 scaleEq(double s) {
		return set(
			e00 * s, e01 * s, e02 * s,
			e10 * s, e11 * s, e12 * s,
			e20 * s, e21 * s, e22 * s
		);
	}

	/**
	 * Sets this matrix to `this` * `m` and returns `this`.
	 */
	public Mat3 mulEq(Mat3 m) {
		return set(
			e00 * m.e00 + e01 * m.e10 + e02 * m.e20,
			e00 * m.e01 + e01 * m.e11 + e02 * m.e21,
			e00 * m.e02 + e01 * m.e12 + e02 * m.e22,
			e10 * m.e00 + e11 * m.e10 + e12 * m.e20,
			e10 * m.e01 + e11 * m.e11 + e12 * m.e21,
			e10 * m.e02 + e11 * m.e12 + e12 * m.e22,
			e20 * m.e00 + e21 * m.e10 + e22 * m.e20,
			e20 * m.e01 + e21 * m.e11 + e22 * m.e21,
			e20 * m.e02 + e21 * m.e12 + e22 * m.e22
		);
	}

	/**
	 * Sets this matrix to `m'*'this` and returns `this`.
	 */
	public Mat3 preMulEq(Mat3 m) {
		return set(
			m.e00 * e00 + m.e01 * e10 + m.e02 * e20,
			m.e00 * e01 + m.e01 * e11 + m.e02 * e21,
			m.e00 * e02 + m.e01 * e12 + m.e02 * e22,
			m.e10 * e00 + m.e11 * e10 + m.e12 * e20,
			m.e10 * e01 + m.e11 * e11 + m.e12 * e21,
			m.e10 * e02 + m.e11 * e12 + m.e12 * e22,
			m.e20 * e00 + m.e21 * e10 + m.e22 * e20,
			m.e20 * e01 + m.e21 * e11 + m.e22 * e21,
			m.e20 * e02 + m.e21 * e12 + m.e22 * e22
		);
	}
	
	/**
	 * Sets this matrix to `this` * `m'` and returns `this`.
	 */
	public Mat3 mulTransposeEq(Mat3 m) {
		return set(
				e00 * m.e00 + e01 * m.e01 + e02 * m.e02,
				e00 * m.e10 + e01 * m.e11 + e02 * m.e12,
				e00 * m.e20 + e01 * m.e21 + e02 * m.e22,
				e10 * m.e00 + e11 * m.e01 + e12 * m.e02,
				e10 * m.e10 + e11 * m.e11 + e12 * m.e12,
				e10 * m.e20 + e11 * m.e21 + e12 * m.e22,
				e20 * m.e00 + e21 * m.e01 + e22 * m.e02,
				e20 * m.e10 + e21 * m.e11 + e22 * m.e12,
				e20 * m.e20 + e21 * m.e21 + e22 * m.e22
		);
	}
	
	
	/**
	 * Returns New Mat3 = scaling matrix *`this`.
	 *
	 * Where *scaling matrix* is a matrix which scales `sx` times, `sy` times and
	 * `sz` times along the x-axis, y-axis and z-axis respectively.
	 */
	public Mat3 prependScale(double sx, double sy, double sz) {
		return new Mat3(
			e00 * sx, e01 * sx, e02 * sx,
			e10 * sy, e11 * sy, e12 * sy,
			e20 * sz, e21 * sz, e22 * sz
		);
	}

	/**
	 * Returns new Mat3 =`this` * scaling matrix*.
	 *
	 * Where *scaling matrix* is a matrix which scales `sx` times, `sy` times and
	 * `sz` times along the x-axis, y-axis and z-axis respectively.
	 */
	public Mat3 appendScale(double sx,double sy,double sz) {
		return new Mat3(
			e00 * sx, e01 * sy, e02 * sz,
			e10 * sx, e11 * sy, e12 * sz,
			e20 * sx, e21 * sy, e22 * sz
		);
	}

	/**
	 * Returns new Mat3 = rotation matrix* * `this`.
	 *
	 * Where *rotation matrix* is a matrix which rotates `rad` in radians around the **normalized**
	 * vector (`axisX`, `axisY`, `axisZ`).
	 */
	public Mat3 prependRotation(double rad, double axisX, double axisY, double axisZ) {
		double s = (float) MathUtil.sin(rad);
		double c = (float) MathUtil.cos(rad);
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
		return new Mat3(
			r00 * e00 + r01 * e10 + r02 * e20,
			r00 * e01 + r01 * e11 + r02 * e21,
			r00 * e02 + r01 * e12 + r02 * e22,
			r10 * e00 + r11 * e10 + r12 * e20,
			r10 * e01 + r11 * e11 + r12 * e21,
			r10 * e02 + r11 * e12 + r12 * e22,
			r20 * e00 + r21 * e10 + r22 * e20,
			r20 * e01 + r21 * e11 + r22 * e21,
			r20 * e02 + r21 * e12 + r22 * e22
		);
	}

	/**
	 * Returns new Mat3 = `this` * rotation matrix*.
	 *
	 * Where *rotation matrix* is a matrix which rotates `rad` in radians around the **normalized**
	 * vector (`axisX`, `axisY`, `axisZ`).
	 */
	public Mat3 appendRotation(double rad, double axisX, double axisY, double axisZ) {
		double s = (float) MathUtil.sin(rad);
		double c = (float) MathUtil.cos(rad);
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
		return new Mat3(
			e00 * r00 + e01 * r10 + e02 * r20,
			e00 * r01 + e01 * r11 + e02 * r21,
			e00 * r02 + e01 * r12 + e02 * r22,
			e10 * r00 + e11 * r10 + e12 * r20,
			e10 * r01 + e11 * r11 + e12 * r21,
			e10 * r02 + e11 * r12 + e12 * r22,
			e20 * r00 + e21 * r10 + e22 * r20,
			e20 * r01 + e21 * r11 + e22 * r21,
			e20 * r02 + e21 * r12 + e22 * r22
		);
	}

	/**
	 * Sets this matrix to this = scaling matrix* `this`, and returns `this`.
	 *
	 * Where *scaling matrix* is a matrix which scales `sx` times, `sy` times and
	 * `sz` times along the x-axis, y-axis and z-axis respectively.
	 */
	public Mat3 prependScaleEq(double sx,double sy,double sz) {
		return set(
			e00 * sx, e01 * sx, e02 * sx,
			e10 * sy, e11 * sy, e12 * sy,
			e20 * sz, e21 * sz, e22 * sz
		);
	}

	/**
	 * Sets this matrix to this= `this` * scaling matrix*, and returns `this`.
	 *
	 * Where *scaling matrix* is a matrix which scales `sx` times, `sy` times and
	 * `sz` times along the x-axis, y-axis and z-axis respectively.
	 */
	public Mat3 appendScaleEq(double sx,double sy,double sz) {
		return set(
			e00 * sx, e01 * sy, e02 * sz,
			e10 * sx, e11 * sy, e12 * sz,
			e20 * sx, e21 * sy, e22 * sz
		);
	}

	/**
	 * Sets this matrix to *rotation matrix* * `this`, and returns `this`.
	 *
	 * Where *rotation matrix* is a matrix which rotates `rad` in radians around the **normalized**
	 * vector (`axisX`, `axisY`, `axisZ`).
	 */
	public Mat3 prependRotationEq(double rad, double axisX, double axisY, double axisZ) {
		double s = (float) MathUtil.sin(rad);
		double c = (float) MathUtil.cos(rad);
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
			r00 * e00 + r01 * e10 + r02 * e20,
			r00 * e01 + r01 * e11 + r02 * e21,
			r00 * e02 + r01 * e12 + r02 * e22,
			r10 * e00 + r11 * e10 + r12 * e20,
			r10 * e01 + r11 * e11 + r12 * e21,
			r10 * e02 + r11 * e12 + r12 * e22,
			r20 * e00 + r21 * e10 + r22 * e20,
			r20 * e01 + r21 * e11 + r22 * e21,
			r20 * e02 + r21 * e12 + r22 * e22
		);
	}

	/**
	 * Sets this matrix to `this` * *rotation matrix*, and returns `this`.
	 *
	 * Where *rotation matrix* is a matrix which rotates `rad` in radians around the **normalized**
	 * vector (`axisX`, `axisY`, `axisZ`).
	 */
	public Mat3 appendRotationEq(double rad, double axisX, double axisY, double axisZ) {
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
			e00 * r00 + e01 * r10 + e02 * r20,
			e00 * r01 + e01 * r11 + e02 * r21,
			e00 * r02 + e01 * r12 + e02 * r22,
			e10 * r00 + e11 * r10 + e12 * r20,
			e10 * r01 + e11 * r11 + e12 * r21,
			e10 * r02 + e11 * r12 + e12 * r22,
			e20 * r00 + e21 * r10 + e22 * r20,
			e20 * r01 + e21 * r11 + e22 * r21,
			e20 * r02 + e21 * r12 + e22 * r22
		);
	}

	/**
	 * Returns the transposed matrix.
	 */
	public Mat3 transpose() {
		return new Mat3(
			e00, e10, e20,
			e01, e11, e21,
			e02, e12, e22
		);
	}

	/**
	 * Sets this matrix to the transposed matrix and returns `this`.
	 */
	public Mat3 transposeEq() {
		return set(
			e00, e10, e20,
			e01, e11, e21,
			e02, e12, e22
		);
	}

	/**
	 * Returns the determinant.
	 */
	public double determinant() {
		return e00 * (e11 * e22 - e12 * e21) - e01 * (e10 * e22 - e12 * e20) + e02 * (e10 * e21 - e11 * e20);
	}

	/**
	 * Returns the trace.
	 */
	public double trace() {
		return e00 + e11 + e22;
	}

	/**
	 * Returns the inverse matrix.
	 *
	 * If the determinant is zero, zero matrix is returned.
	 */
	public Mat3 inverse() {
		double d00 = e11 * e22 - e12 * e21;
		double d01 = e10 * e22 - e12 * e20;
		double d02 = e10 * e21 - e11 * e20;
		double d10 = e01 * e22 - e02 * e21;
		double d11 = e00 * e22 - e02 * e20;
		double d12 = e00 * e21 - e01 * e20;
		double d20 = e01 * e12 - e02 * e11;
		double d21 = e00 * e12 - e02 * e10;
		double d22 = e00 * e11 - e01 * e10;
		double invDet = e00 * d00 - e01 * d01 + e02 * d02;
		if (invDet != 0) invDet = 1 / invDet;
		return new Mat3(
			d00 * invDet, -d10 * invDet, d20 * invDet,
			-d01 * invDet, d11 * invDet, -d21 * invDet,
			d02 * invDet, -d12 * invDet, d22 * invDet
		);
	}

	/**
	 * Sets this matrix to the inverse matrix and returns `this`.
	 *
	 * If the determinant is zero, this matrix is set to zero matrix.
	 */
	public Mat3 inverseEq() {
		double d00 = e11 * e22 - e12 * e21;
		double d01 = e10 * e22 - e12 * e20;
		double d02 = e10 * e21 - e11 * e20;
		double d10 = e01 * e22 - e02 * e21;
		double d11 = e00 * e22 - e02 * e20;
		double d12 = e00 * e21 - e01 * e20;
		double d20 = e01 * e12 - e02 * e11;
		double d21 = e00 * e12 - e02 * e10;
		double d22 = e00 * e11 - e01 * e10;
		double invDet = e00 * d00 - e01 * d01 + e02 * d02;
		if (invDet != 0) invDet = 1 / invDet;
		return set(
			d00 * invDet, -d10 * invDet, d20 * invDet,
			-d01 * invDet, d11 * invDet, -d21 * invDet,
			d02 * invDet, -d12 * invDet, d22 * invDet
		);
	}

	/**
	 * Returns an array of the elements of this matrix.
	 *
	 * If `columnMajor` is true, the array is arranged in column-major order.
	 * Otherwise, the array is arranged in row-major order.
	 */
	public double[] toArray(boolean columnMajor) {
		if (columnMajor) {
			return new double[] {
				e00, e10, e20,
				e01, e11, e21,
				e02, e12, e22
			};
		} else {
			return new double[] {
				e00, e01, e02,
				e10, e11, e12,
				e20, e21, e22
			};
		}
	}

	/**
	 * Copies values from `m` and returns `this`.
	 */
	public Mat3 copyFrom(Mat3 m) {
		e00 = m.e00;
		e01 = m.e01;
		e02 = m.e02;
		e10 = m.e10;
		e11 = m.e11;
		e12 = m.e12;
		e20 = m.e20;
		e21 = m.e21;
		e22 = m.e22;
		return this;
	}

	/**
	 * Returns a clone of the matrix.
	 */
	public Mat3 clone() {
		return new Mat3(
			e00, e01, e02,
			e10, e11, e12,
			e20, e21, e22
		);
	}

	/**
	 * Sets this matrix to the representation of the quaternion `q`, and returns `this`.
	 */
	public Mat3 fromQuat(Quat q) {
		double x = q.x;
		double y = q.y;
		double z = q.z;
		double w = q.w;
		double x2 = 2 * x;
		double y2 = 2 * y;
		double z2 = 2 * z;
		double xx = x * x2;
		double yy = y * y2;
		double zz = z * z2;
		double xy = x * y2;
		double yz = y * z2;
		double xz = x * z2;
		double wx = w * x2;
		double wy = w * y2;
		double wz = w * z2;
		e00 = 1 - yy - zz;
		e01 = xy - wz;
		e02 = xz + wy;
		e10 = xy + wz;
		e11 = 1 - xx - zz;
		e12 = yz - wx;
		e20 = xz - wy;
		e21 = yz + wx;
		e22 = 1 - xx - yy;
	
		return this;
	}

	/**
	 * Returns a new quaternion which represents this matrix.
	 *
	 * This matrix must be a rotation matrix, that is, must be orthogonalized and have determinant 1.
	 */
	public Quat toQuat() {
		return new Quat().fromMat3(this);
	}

	/**
	 * Sets this matrix to the rotation matrix represented by Euler angles `eulerAngles`, and returns `this`.
	 * Rotation order is first X-axis, then rotated Y-axis, finally rotated Z-axis.
	 * @param radX, radY, radZ angles in radians
	 */
	public Mat3 fromEulerXyz(double radX, double radY, double radZ) {
		double sx =  MathUtil.sin(radX);
		double sy =  MathUtil.sin(radY);
		double sz =   MathUtil.sin(radZ);
		double cx = MathUtil.cos(radX);
		double cy =  MathUtil.cos(radY);
		double cz = MathUtil.cos(radZ);
		return set(
			cy * cz,               -cy * sz,                 sy,
			cx * sz + cz * sx * sy, cx * cz - sx * sy * sz, -cy * sx,
			sx * sz - cx * cz * sy, cz * sx + cx * sy * sz,  cx * cy
		);
	}

	/**
	 * Sets this matrix to the rotation matrix represented by Euler angles `eulerAngles`, and returns `this`.
	 * Rotation order is first X-axis, then rotated Y-axis, finally rotated Z-axis.
	 */
	public Mat3 fromEulerXyz(Vec3  eulerAngles) {
		return fromEulerXyz(eulerAngles.x,eulerAngles.y,eulerAngles.z);
	}

	/**
	 * Returns a vector `(angleX, angleY, angleZ)` represents the Euler angles of this matrix.
	 * Rotation order is first X-axis, then rotated Y-axis, finally rotated Z-axis.
	 * Note that `angleX`, `angleY`, and `angleZ` are in range of -PI to PI, -PI/2 to PI/2, and -PI to PI respectively.
	 */
	public Vec3 toEulerXyz() {
		// | cos(y)cos(z)                      -cos(y)sin(z)                        sin(y)       |
		// | cos(x)sin(z) + cos(z)sin(x)sin(y)  cos(x)cos(z) - sin(x)sin(y)sin(z)  -cos(y)sin(x) |
		// | sin(x)sin(z) - cos(x)cos(z)sin(y)  cos(z)sin(x) + cos(x)sin(y)sin(z)   cos(x)cos(y) |

		// TODO:
		// XZY:
		//   [                        cos(y)*cos(z),       -sin(z),                        cos(z)*sin(y)]
		//   [ sin(x)*sin(y) + cos(x)*cos(y)*sin(z), cos(x)*cos(z), cos(x)*sin(y)*sin(z) - cos(y)*sin(x)]
		//   [ cos(y)*sin(x)*sin(z) - cos(x)*sin(y), cos(z)*sin(x), cos(x)*cos(y) + sin(x)*sin(y)*sin(z)]
		//
		// YXZ:
		//   [ cos(y)*cos(z) + sin(x)*sin(y)*sin(z), cos(z)*sin(x)*sin(y) - cos(y)*sin(z), cos(x)*sin(y)]
		//   [                        cos(x)*sin(z),                        cos(x)*cos(z),       -sin(x)]
		//   [ cos(y)*sin(x)*sin(z) - cos(z)*sin(y), sin(y)*sin(z) + cos(y)*cos(z)*sin(x), cos(x)*cos(y)]
		//
		// YZX:
		//   [  cos(y)*cos(z), sin(x)*sin(y) - cos(x)*cos(y)*sin(z), cos(x)*sin(y) + cos(y)*sin(x)*sin(z)]
		//   [         sin(z),                        cos(x)*cos(z),                       -cos(z)*sin(x)]
		//   [ -cos(z)*sin(y), cos(y)*sin(x) + cos(x)*sin(y)*sin(z), cos(x)*cos(y) - sin(x)*sin(y)*sin(z)]
		//
		// ZXY:
		//   [ cos(y)*cos(z) - sin(x)*sin(y)*sin(z), -cos(x)*sin(z), cos(z)*sin(y) + cos(y)*sin(x)*sin(z)]
		//   [ cos(y)*sin(z) + cos(z)*sin(x)*sin(y),  cos(x)*cos(z), sin(y)*sin(z) - cos(y)*cos(z)*sin(x)]
		//   [                       -cos(x)*sin(y),         sin(x),                        cos(x)*cos(y)]
		//
		// ZYX:
		//   [ cos(y)*cos(z), cos(z)*sin(x)*sin(y) - cos(x)*sin(z), sin(x)*sin(z) + cos(x)*cos(z)*sin(y)]
		//   [ cos(y)*sin(z), cos(x)*cos(z) + sin(x)*sin(y)*sin(z), cos(x)*sin(y)*sin(z) - cos(z)*sin(x)]
		//   [       -sin(y),                        cos(y)*sin(x),                        cos(x)*cos(y)]

		double sy = e02;

		if (sy <= -1) { // y = -PI / 2
			// |  0           0          -1 |
			// | -sin(x - z)  cos(x - z)  0 |
			// |  cos(x - z)  sin(x - z)  0 |

			double xSubZ = MathUtil.atan2(e21, e11);

			// not unique, minimize x^2 + z^2
			//   x =  xSubZ/2
			//   z = -xSubZ/2
			return new Vec3(xSubZ * 0.5f, -MathUtil.HALF_PI, -xSubZ * 0.5f);
		}
		if (sy >= 1) { // y = PI / 2
			// |  0           0           1 |
			// |  sin(x + z)  cos(x + z)  0 |
			// | -cos(x + z)  sin(x + z)  0 |

			double xAddZ = MathUtil.atan2(e21, e11);

			// not unique, minimize x^2 + z^2
			//   x = xAddZ/2
			//   z = xAddZ/2
			return new Vec3(xAddZ * 0.5f, MathUtil.HALF_PI, xAddZ * 0.5f);
		}else {
			return new Vec3( MathUtil.atan2(-e12, e22),MathUtil.asin(sy), MathUtil.atan2(-e01, e00));
		}

	
	}

	/**
	 * Returns the `index`th row vector of the matrix.
	 *
	 * If `index` is less than `0` or greater than `2`, `null` will be returned.
	 */
	public Vec3 getRow(int index) {
			if (index == 0) return new Vec3(e00, e01, e02);
			else if (index == 1)return new Vec3(e10, e11, e12);
			else if (index == 2) return new Vec3(e20, e21, e22);
			return null;
	}

	/**
	 * Returns the `index`th column vector of the matrix.
	 *
	 * If `index` is less than `0` or greater than `2`, `null` will be returned.
	 */
	public Vec3 getCol(int index) {
			if (index == 0) return new Vec3(e00, e10, e20);
			else if (index == 1)return new Vec3(e01, e11, e21);
			else if (index == 2)return new Vec3(e02, e12, e22);
			return null;
	}

	/**
	 * Sets `dst` to the `index`th row vector of the matrix.
	 *
	 * If `index` is less than `0` or greater than `2`, `dst` will be set to the zero vector.
	 */
	public void getRowTo(int index,Vec3 dst) {
		if (index == 0) dst.set(e00, e01, e02);
		else if (index == 1) dst.set(e10, e11, e12);
		else if (index == 2) dst.set(e20, e21, e22);
		else dst.zero();
	}

	/**
	 * Sets `dst` to the `index`th column vector of the matrix.
	 *
	 * If `index` is less than `0` or greater than `2`, `dst` will be set to the zero vector.
	 */
	public void getColTo(int index, Vec3 dst) {
		if (index == 0) dst.set(e00, e10, e20);
		else if (index == 1) dst.set(e01, e11, e21);
		else if (index == 2) dst.set(e02, e12, e22);
		else dst.zero();
	}

	/**
	 * Sets this matrix by row vectors and returns `this`.
	 */
	public Mat3 fromRows( Vec3 row0, Vec3 row1, Vec3 row2) {
		return set(
			row0.x, row0.y, row0.z,
			row1.x, row1.y, row1.z,
			row2.x, row2.y, row2.z
		);
	}

	/**
	 * Sets this matrix by column vectors and returns `this`.
	 */
	public Mat3 fromCols(Vec3 col0, Vec3 col1,Vec3 col2) {
		return set(
			col0.x, col1.x, col2.x,
			col0.y, col1.y, col2.y,
			col0.z, col1.z, col2.z
		);
	}

//	/**
//	 * Transforms vector by this matrix
//	 * @param position
//	 * @return specified position vector after transformation
//	 */
//	public Vec3 transform(Vec3 position) {
//		double x
//		position.x = e00 * v.x + e01 * v.y + e02 * v.z ;
//		position.y = e10 * v.x + e11 * v.y + e12 * v.z ;
//		position.z = e20 * v.x + e21 * v.y + e22 * v.z ;
//		return position;
//	}
	
	
	/**
	 * Returns the string representation of the matrix.
	 */
	public String toString() {
		return String.format(
				"Mat3[ %.3f %.3f %.3f \n" +
				"    [ %.3f %.3f %.3f \n" +
				"    [ %.3f %.3f %.3f \n" 
				,e00, e01, e02,
				e10, e11, e12,
				e20, e21, e22
				)
			;
	}

}

