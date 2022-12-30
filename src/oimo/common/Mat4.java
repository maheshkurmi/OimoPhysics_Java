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
	public float e00=1;

	/**
	 * The element at row 0 column 1.
	 */
	public float e01=0;

	/**
	 * The element at row 0 column 2.
	 */
	public float e02=0;

	/**
	 * The element at row 0 column 3.
	 */
	public float e03=0;

	/**
	 * The element at row 1 column 0.
	 */
	public float e10=0;

	/**
	 * The element at row 1 column 1.
	 */
	public float e11=1;

	/**
	 * The element at row 1 column 2.
	 */
	public float e12=0;

	/**
	 * The element at row 1 column 3.
	 */
	public float e13=0;

	/**
	 * The element at row 2 column 0.
	 */
	public float e20=0;

	/**
	 * The element at row 2 column 1.
	 */
	public float e21=0;

	/**
	 * The element at row 2 column 2.
	 */
	public float e22=1;

	/**
	 * The element at row 2 column 3.
	 */
	public float e23=0;

	/**
	 * The element at row 3 column 0.
	 */
	public float e30=0;

	/**
	 * The element at row 3 column 1.
	 */
	public float e31=0;

	/**
	 * The element at row 3 column 2.
	 */
	public float e32=0;

	/**
	 * The element at row 3 column 3.
	 */
	public float e33=1;

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
		float e00 ,float e01,float e02,float e03,
		float e10 ,float e11,float e12,float e13,
		float e20 ,float e21,float e22,float e23,
		float e30 ,float e31,float e32,float e33
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
			float e00 ,float e01,float e02,float e03,
			float e10 ,float e11,float e12,float e13,
			float e20 ,float e21,float e22,float e23,
			float e30 ,float e31,float e32,float e33
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