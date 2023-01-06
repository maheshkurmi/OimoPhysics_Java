package oimo.common;

/**
 * Quaternion class.
 */
public class Quat {
	/**
	 * The number of instance creation.
	 */
	public static int numCreations;

	/**
	 * The x-value of the imaginary part of the quaternion.
	 */
	public double x=0;

	/**
	 * The y-value of the imaginary part of the quaternion.
	 */
	public double y=0;

	/**
	 * The z-value of the imaginary part of the quaternion.
	 */
	public double z=0;

	/**
	 * The real part of the quaternion.
	 */
	public double w=1;

	/**
	 * Constructor - initializes Quat as identity (real part=1 and imaginary part as zero).
	 */
	public Quat() {
		
	}
	/**
	 * Creates a new quaternion  in form w+ i(x+y+z) 
	 */
	public Quat(double x, double y, double z, double w) {
		this.x = x;
		this.y = y;
		this.z = z;
		this.w = w;
		numCreations++;
	}

	public Quat set(double x, double y, double z, double w) {
		this.x = x;
		this.y = y;
		this.z = z;
		this.w = w;
		return this;
	}

	/**
	 * Sets the quaternion to identity quaternion and returns `this`.
	 */
	public Quat identity() {
		return set(0, 0, 0, 1);
	}

	/**
	 * Sets all values at once and returns `this`.
	 */
	public Quat init(double x, double y, double z, double w) {
		this.x = x;
		this.y = y;
		this.z = z;
		this.w = w;
		return this;
	}

	/**
	 * Returns `this` + `v`.
	 */
	public Quat add(Quat q) {
		return new Quat(x + q.x, y + q.y, z + q.z, w + q.w);
	}

	/**
	 * Returns `this` - `v`.
	 */
	public Quat sub(Quat q) {
		return new Quat(x - q.x, y - q.y, z - q.z, w - q.w);
	}

	/**
	 * Returns `this` * `s`.
	 */
	public Quat scale(double s) {
		return new Quat(x * s, y * s, z * s, w * s);
	}

	/**
	 * Sets this quaternion to `this` + `v` and returns `this`.
	 */
	public Quat addEq(Quat q) {
		return set(x + q.x, y + q.y, z + q.z, w + q.w);
	}

	/**
	 * Sets this quaternion to `this` - `v` and returns `this`.
	 */
	public Quat subEq(Quat q) {
		return set(x - q.x, y - q.y, z - q.z, w - q.w);
	}

	/**
	 * Sets this quaternion to `this` * `s` and returns `this`.
	 */
	public Quat scaleEq(double s) {
		return set(x * s, y * s, z * s, w * s);
	}

	/**
	 * Returns the length of the quaternion.
	 */
	public double length() {
		return MathUtil.sqrt(x * x + y * y + z * z + w * w);
	}

	/**
	 * Returns the squared length of the quaternion.
	 */
	public double lengthSq() {
		return x * x + y * y + z * z + w * w;
	}

	/**
	 * Returns the dot product of `this` and `q`.
	 */
	public double dot(Quat q) {
		return x * q.x + y * q.y + z * q.z + w * q.w;
	}

	/**
	 * Returns the normalized quaternion.
	 *
	 * If the length is zero, zero quaterinon is returned.
	 */
	public Quat normalized() {
		double invLen = length();
		if (invLen > 0)
			invLen = 1 / invLen;
		return new Quat(x * invLen, y * invLen, z * invLen, w * invLen);
	}

	/**
	 * Sets this quaternion to the normalized quaternion and returns `this`.
	 *
	 * If the length is zero, this quaternion is set to zero quaternion.
	 */
	public Quat normalize() {
		double invLen = length();
		if (invLen > 0)
			invLen = 1 / invLen;
		return set(x * invLen, y * invLen, z * invLen, w * invLen);
	}

	/**
	 * Sets this quaternion to the quaternion representing the shortest arc rotation
	 * from `v1` to `v2`, and return `this`.
	 */
	public Quat setArc(Vec3 v1, Vec3 v2) {
		double x1 = v1.x;
		double y1 = v1.y;
		double z1 = v1.z;
		double x2 = v2.x;
		double y2 = v2.y;
		double z2 = v2.z;
		double d = x1 * x2 + y1 * y2 + z1 * z2;

		this.w = MathUtil.sqrt((((1 + d)) * 0.5f));
		if ((this.w == 0)) {
			x2 = (x1 * x1);
			y2 = (y1 * y1);
			z2 = (z1 * z1);
			if ((x2 < y2)) {
				if ((x2 < z2)) {
					// |x1| is the smallest, use x-axis
					d = (1 / MathUtil.sqrt((y2 + z2)));
					this.x = 0;
					this.y = (z1 * d);
					this.z = (-(y1) * d);
				} else {
					// |z1| is the smallest, use z-axis
					d = (1 / MathUtil.sqrt((x2 + y2)));
					this.z = 0;
					this.x = (y1 * d);
					this.y = (-(x1) * d);
				}

			} else {
				if ((y2 < z2)) {
					// |y1| is the smallest, use y-axis
					d = (1 / MathUtil.sqrt((z2 + x2)));
					this.y = 0;
					this.z = (x1 * d);
					this.x = (-(z1) * d);
				} else {
					// |z1| is the smallest, use z-axis
					d = (1 / MathUtil.sqrt((x2 + y2)));
					this.z = 0;
					this.x = (y1 * d);
					this.y = (-(x1) * d);
				}

			}
			return this;
		}

		d = (0.5f / this.w);
		double cx = ((y1 * z2) - (z1 * y2));
		double cy = ((z1 * x2) - (x1 * z2));
		double cz = ((x1 * y2) - (y1 * x2));
		this.x = (cx * d);
		this.y = (cy * d);
		this.z = (cz * d);
		return this;
	}

	/**
	 * Returns the spherical linear interpolation between two quaternions `this` and
	 * `q` with interpolation paraeter `t`. Both quaternions `this` and `q` must be
	 * normalized.
	 */
	public Quat slerp(Quat q, double t) {
		double qx;
		double qy;
		double qz;
		double qw;
		double d = dot(q);
		if (d < 0) {
			d = -d;
			qx = -q.x;
			qy = -q.y;
			qz = -q.z;
			qw = -q.w;
		} else {
			qx = q.x;
			qy = q.y;
			qz = q.z;
			qw = q.w;
		}
		if (d > 1 - 1e-6) {
			// two quaternions are too close, returns lerp instead
			return new Quat(x + (qx - x) * t, y + (qy - y) * t, z + (qz - z) * t, w + (qw - w) * t).normalize();
		}
		// target angle
		double theta = t * MathUtil.acos(d);

		// make q orthogonal to this
		qx -= x * d;
		qy -= y * d;
		qz -= z * d;
		qw -= w * d;
		double invLen = 1 / MathUtil.sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
		qx *= invLen;
		qy *= invLen;
		qz *= invLen;
		qw *= invLen;

		// mix them
		double sin = MathUtil.sin(theta);
		double cos = MathUtil.cos(theta);
		return new Quat(x * cos + qx * sin, y * cos + qy * sin, z * cos + qz * sin, w * cos + qw * sin);
	}

	/**
	 * Copies values from `q` and returns `this`.
	 */
	public Quat copyFrom(Quat q) {
		x = q.x;
		y = q.y;
		z = q.z;
		w = q.w;
		return this;
	}

	/**
	 * Returns a clone of the quaternion.
	 */
	public Quat clone() {
		return new Quat(x, y, z, w);
	}

	/**
	 * Sets this quaternion to the representation of the matrix `m`, and returns
	 * `this`.
	 *
	 * The matrix `m` must be a rotation matrix, that is, must be orthogonalized and
	 * have determinant 1.
	 */
	public Quat fromMat3(Mat3 m) {
		double e00 = m.e00;
		double e11 = m.e11;
		double e22 = m.e22;
		double trace = e00 + e11 + e22;
		double s = 0;
		if ((trace > 0)) {
			s = MathUtil.sqrt((trace + 1));
			this.w = (0.5f * s);
			s = (0.5f / s);
			this.x = (((m.e21 - m.e12)) * s);
			this.y = (((m.e02 - m.e20)) * s);
			this.z = (((m.e10 - m.e01)) * s);
		} else {
			if ((e00 > e11)) {
				if ((e00 > e22)) {
					s = MathUtil.sqrt((((e00 - e11) - e22) + 1));
					this.x = (0.5f * s);
					s = (0.5f / s);
					this.y = (((m.e01 + m.e10)) * s);
					this.z = (((m.e02 + m.e20)) * s);
					this.w = (((m.e21 - m.e12)) * s);
				} else {
					s = MathUtil.sqrt((((e22 - e00) - e11) + 1));
					this.z = (0.5f * s);
					s = (0.5f / s);
					this.x = (((m.e02 + m.e20)) * s);
					this.y = (((m.e12 + m.e21)) * s);
					this.w = (((m.e10 - m.e01)) * s);
				}

			} else {
				if ((e11 > e22)) {
					s = MathUtil.sqrt((((e11 - e22) - e00) + 1));
					this.y = (0.5f * s);
					s = (0.5f / s);
					this.x = (((m.e01 + m.e10)) * s);
					this.z = (((m.e12 + m.e21)) * s);
					this.w = (((m.e02 - m.e20)) * s);
				} else {
					s = MathUtil.sqrt((((e22 - e00) - e11) + 1));
					this.z = (0.5f * s);
					s = (0.5f / s);
					this.x = (((m.e02 + m.e20)) * s);
					this.y = (((m.e12 + m.e21)) * s);
					this.w = (((m.e10 - m.e01)) * s);
				}

			}

		}
		return this;
	}

	/**
	 * Returns a rotation matrix which represents this quaternion.
	 */
	public Mat3 toMat3() {
		return new Mat3().fromQuat(this);
	}

	/**
	 * Returns the string representation of the quaternion.
	 */
	public String toString() {
		return String.format("Quat[%.3f + i(%.3f,%.3f,%.3f)] ",this.w,this.x,this.y,this.z);
	}

}