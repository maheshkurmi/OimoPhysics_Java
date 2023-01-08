package oimo.common;

public class MathUtil {
	public static double POSITIVE_INFINITY = Float.POSITIVE_INFINITY;
	public static double NEGATIVE_INFINITY = Float.NEGATIVE_INFINITY;
	public static double PI = 3.14159265358979f;
	public static double TWO_PI = 6.28318530717958f;
	public static double HALF_PI = 1.570796326794895f;
	public static double TO_RADIANS = 0.0174532925199432781f;
	public static double TO_DEGREES = 57.2957795130823797f;

	public static double abs(double x) {
		if ((x > 0)) {
			return x;
		} else {
			return -(x);
		}

	}

	public static double sin(double x) {
		return Math.sin(x);
	}

	public static double cos(double x) {
		return Math.cos(x);
	}

	public static double tan(double x) {
		return Math.tan(x);
	}

	public static double asin(double x) {
		return Math.asin(x);
	}

	public static double acos(double x) {
		return Math.acos(x);
	}

	public static double atan(double x) {
		return Math.atan(x);
	}

	public static double safeAsin(double x) {
		if ((x <= -1)) {
			return -HALF_PI;
		}

		if ((x >= 1)) {
			return HALF_PI;
		}

		return Math.asin(x);
	}

	public static double safeAcos(double x) {
		if ((x <= -1)) {
			return PI;
		}

		if ((x >= 1)) {
			return 0;
		}

		return Math.acos(x);
	}

	public static double atan2(double y, double x) {
		return Math.atan2(y, x);
	}

	public static double sqrt(double x) {
		if (x < 0) {
			//System.out.println("Error in Math.sqrt " + x);
			return 0;
		}
		return Math.sqrt(x);
	}

	/**
	 * Clamsps x between min and max
	 * 
	 * @param x
	 * @param min
	 * @param max
	 * @return
	 */
	public static double clamp(double x, double min, double max) {
		if ((x < min)) {
			return min;
		} else {
			if ((x > max)) {
				return max;
			} else {
				return x;
			}

		}

	}

	/**
	 * 
	 * @return
	 */
	public static double rand() {
		return Math.random();
	}

	/**
	 * Returns random number in [-1,1]
	 * 
	 * @return
	 */
	public static double randIn(double min, double max) {
		return (min + (rand() * ((max - min))));
	}

	/**
	 * Returns new random Vec3 with each component in [min,max]
	 * 
	 * @return
	 */
	public static Vec3 randVec3In(double min, double max) {
		return new Vec3(randIn(min, max), randIn(min, max), randIn(min, max));
	}

	/**
	 * Returns new random Vec3 with each component in [-1,1]
	 * 
	 * @return
	 */
	public static Vec3 randVec3() {
		return randVec3In(-1, 1);
	}

	public static double min(double a, double b) {
		return a < b ? a : b;

	}

	public static double max(double a, double b) {
		return a > b ? a : b;
	}

}
