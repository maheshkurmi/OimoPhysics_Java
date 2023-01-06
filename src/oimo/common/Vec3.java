package oimo.common;
/**
 * 3D vector class.
 */
public class Vec3 {
	/**
	 * The number of instance creation.
	 */
	public static int numCreations = 0;

	/**
	 * The x-value of the vector.
	 */
	public double x=0;

	/**
	 * The y-value of the vector.
	 */
	public double y=0;

	/**
	 * The z-value of the vector.
	 */
	public double z=0;

	/**
	 * Creates a new vector as origin or zero vector
	 */
	public Vec3() {
		numCreations++;
	}
	
	/**
	 * Creates a new vector. The vector is zero vector by default.
	 */
	public Vec3(double x,double y, double z) {
		this.x = x;
		this.y = y;
		this.z = z;
		numCreations++;
	}



	/**
	 * Sets all values at once and returns 'this'.
	 */
	public Vec3 set(double x,double y, double z) {
		this.x = x;
		this.y = y;
		this.z = z;
		return this;
	}
	
	/**
	 * Copies Vector 'v' into this vector
	 * @param v
	 */
	public Vec3 set(Vec3 v) {
		this.x = v.x;
		this.y = v.y;
		this.z = v.z;
		return this;
	}
	
	/**
	 * Sets this vector to zero vector and returns 'this'.
	 */
	public Vec3 zero() {
		return set(0, 0, 0);
	}

	
	/**
	 * Returns new 'this' + 'v'.
	 */
	public Vec3 add(Vec3 v) {
		return new Vec3(x + v.x, y + v.y, z + v.z);
	}

	/**
	 * Returns new ('this.x' + 'vx', 'this.y' + 'vy', 'this.z' + 'vz').
	 */
	public Vec3 add(double vx, double vy, double vz) {
		return new Vec3(x + vx, y + vy, z + vz);
	}

	/**
	 * Returns new 'this' + 'v' * 's'.
	 */
	public Vec3 addScaled(Vec3 v, double s) {
		return new Vec3(x + v.x * s, y + v.y * s, z + v.z * s);
	}

	/**
	 * Returns new 'this' - 'v'.
	 */
	public Vec3 sub( Vec3 v) {
		return new Vec3(x - v.x, y - v.y, z - v.z);
	}

	/**
	 * Returns ('this.x' - 'vx', 'this.y' - 'vy', 'this.z' - 'vz').
	 */
	public Vec3 sub(double vx, double vy, double vz) {
		return new Vec3(x - vx, y - vy, z - vz);
	}

	/**
	 * Returns new 'this' * 's'.
	 */
	public Vec3 scale(double s) {
		return new Vec3(x * s, y * s, z * s);
	}

	/**
	 * Returns new ('this.x' * 'sx', 'this.y' * 'sy', 'this.z' * 'sz').
	 */
	public Vec3 scale(double sx, double sy, double sz) {
		return new Vec3(x * sx, y * sy, z * sz);
	}

	/**
	 * Returns the dot product of 'this' and 'v'.
	 */
	public double dot(Vec3 v) {
		return x * v.x + y * v.y + z * v.z;
	}

	/**
	 * Returns the new Vector  as cross product of 'this' and 'v'.
	 */
	public Vec3 cross( Vec3 v){
		return new Vec3(
			y * v.z - z * v.y,
			z * v.x - x * v.z,
			x * v.y - y * v.x
		);
	}

	
	/**
	 * Sets this vector to 'this' + 'v' and returns 'this'.
	 */
	public Vec3 addEq(Vec3 v) {
		return set(x + v.x, y + v.y, z + v.z);
	}

	/**
	 * Sets this vector to ('this.x' + 'vx', 'this.y' + 'vy', 'this.z' + 'vz') and returns 'this'.
	 */
	public Vec3 add3Eq(double vx, double vy, double vz) {
		return set(x + vx, y + vy, z + vz);
	}

	/**
	 * Sets this vector to 'this' + 'v' * 's' and returns 'this'.
	 */
	public Vec3 addScaledEq(Vec3 v, double s) {
		return set(x + v.x * s, y + v.y * s, z + v.z * s);
	}

	/**
	 * Sets this vector to 'this' - 'v' and returns 'this'.
	 */
	public Vec3 subEq(Vec3 v) {
		return set(x - v.x, y - v.y, z - v.z);
	}

	/**
	 * Sets this vector to ('this.x' - 'vx', 'this.y' - 'vy', 'this.z' - 'vz') and returns 'this'.
	 */
	public Vec3 sub3Eq(double vx, double vy, double vz) {
		return set(x - vx, y - vy, z - vz);
	}

	/**
	 * Sets this vector to 'this' * 's' and returns 'this'.
	 */
	public Vec3 scaleEq(double s) {
		return set(x * s, y * s, z * s);
	}

	/**
	 * Sets this vector to ('this.x' * 'sx', 'this.y' * 'sy', 'this.z' * 'sz') and returns 'this'.
	 */
	public Vec3 scale3Eq(double sx, double sy, double sz) {
		return set(x * sx, y * sy, z * sz);
	}

	/**
	 * Sets this vector to the cross product of 'this' and 's', and returns 'this'.
	 */
	public Vec3 crossEq(Vec3 v) {
		return set(
			y * v.z - z * v.y,
			z * v.x - x * v.z,
			x * v.y - y * v.x
		);
	}

	/**
	 * Returns the new transformed vector by 'm'.
	 */
	public Vec3 mulMat3( Mat3 m) {
		return new Vec3(
			x * m.e00 + y * m.e01 + z * m.e02,
			x * m.e10 + y * m.e11 + z * m.e12,
			x * m.e20 + y * m.e21 + z * m.e22
		);
	}

	/**
	 * Returns the new transformed vector by 'm'.
	 */
	public Vec3 mulMat4(Mat4 m) {
		return new Vec3(
			x * m.e00 + y * m.e01 + z * m.e02 + m.e03,
			x * m.e10 + y * m.e11 + z * m.e12 + m.e13,
			x * m.e20 + y * m.e21 + z * m.e22 + m.e23
		);
	}

	/**
	 * Returns the new vector after this vector is transformed by 'tf'.
	 */
	public Vec3 mulTransform(Transform tf) {
		
		Vec3 v=new Vec3();
		M.vec3_fromVec3(v, this);
		M.vec3_mulMat3(v, v, tf._rotation);
		M.vec3_add(v, v, tf._position);
		return v;
//		
//		Vec3 v=new Vec3();
//		
//		M.vec3_mulMat3(this, v, tf._rotation);
//		this.addEq(tf._position);
//		return this;
	}

	/**
	 * Sets this vector to the transformed vector by 'm' and returns 'this'.
	 */
	public Vec3 mulMat3Eq(Mat3 m) {
		return set(
			x * m.e00 + y * m.e01 + z * m.e02,
			x * m.e10 + y * m.e11 + z * m.e12,
			x * m.e20 + y * m.e21 + z * m.e22
		);
	}

	/**
	 * Sets this vector to the transformed vector by 'm' and returns 'this'.
	 */
	public Vec3 mulMat4Eq(Mat4 m) {
		return set(
			x * m.e00 + y * m.e01 + z * m.e02 + m.e03,
			x * m.e10 + y * m.e11 + z * m.e12 + m.e13,
			x * m.e20 + y * m.e21 + z * m.e22 + m.e23
		);
	}

	/**
	 * Sets this vector to the transformed vector by 'tf' and returns 'this'.
	 */
	public Vec3 mulTransformEq(Transform tf) {
//		var v:IVec3;
//		M.vec3_fromVec3(v, this);
//		M.vec3_mulMat3(v, v, tf._rotation);
//		M.vec3_add(v, v, tf._position);
//		M.vec3_toVec3(this, v);
//		Vec3 v=new Vec3();
		M.vec3_mulMat3(this, this, tf._rotation);
		this.addEq(tf._position);
		return this;
	}

	/**
	 * Returns the length of the vector.
	 */
	public double length() {
		return MathUtil.sqrt(x * x + y * y + z * z);
	}

	/**
	 * Returns the squared length of the vector.
	 */
	public double lengthSq() {
		return x * x + y * y + z * z;
	}

	/**
	 * Returns the new  normalized vector.
	 *
	 * If the length is zero, zero vector is returned.
	 */
	public Vec3 normalized() {
		double invLen = length();
		if (invLen > 0) invLen = 1 / invLen;
		return new Vec3(x * invLen, y * invLen, z * invLen);
	}

	/**
	 * Normalize this vector and returns 'this'.
	 *
	 * If the length is zero, this vector is set to zero vector.
	 */
	public Vec3 normalize() {
		double invLen = length();
		if (invLen > 0) invLen = 1 / invLen;
		return set(x * invLen, y * invLen, z * invLen);
	}

	/**
	 * Returns the nagated vector.
	 */
	public Vec3 negate() {
		return new Vec3(-x, -y, -z);
	}

	/**
	 * Negate the vector and returns 'this'.
	 */
	public Vec3 negateEq() {
		return set(-x, -y, -z);
	}

	/**
	 * Copies values from 'v' and returns 'this'.
	 */
	public Vec3 copyFrom(Vec3 v) {
		x = v.x;
		y = v.y;
		z = v.z;
		return this;
	}

	/**
	 * Returns a clone of the vector.
	 */
	public Vec3 clone() {
		return new Vec3(x, y, z);
	}

	/**
	 * Returns the string representation of the vector.
	 */
	public String toString() {
		return String.format("Vec3[%.3f,%.3f,%.3f]",this.x,this.y,this.z);
	}

	



}
