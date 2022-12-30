package oimo.common;


public class MathUtil
{
	public static float POSITIVE_INFINITY = Float.POSITIVE_INFINITY;
	public static float NEGATIVE_INFINITY = Float.NEGATIVE_INFINITY;
	public static float PI = 3.14159265358979f;
	public static float TWO_PI = 6.28318530717958f;
	public static float HALF_PI = 1.570796326794895f;
	public static float TO_RADIANS = 0.0174532925199432781f;
	public static float TO_DEGREES = 57.2957795130823797f;

	
	public static float abs(float x)
	{
		if (( x > 0 )) 
		{
			return x;
		}
		else
		{
			return  - (x) ;
		}
		
	}
	
	
	public static float sin(float x)
	{
		return (float) Math.sin(x);
	}
	
	
	public static float cos(float x)
	{
		return (float) Math.cos(x);
	}
	
	
	public static float tan(float x)
	{
		return (float) Math.tan(x);
	}
	
	
	public static float asin(float x)
	{
		return (float)Math.asin(x);
	}
	
	
	public static float acos(float x)
	{
		return (float)Math.acos(x);
	}
	
	
	public static float atan(float x)
	{
		return (float)Math.atan(x);
	}
	
	
	public static float safeAsin(float x)
	{
		if (( x <= -1 )) 
		{
			return -HALF_PI;
		}
		
		if (( x >= 1 )) 
		{
			return HALF_PI;
		}
		
		return (float)Math.asin(x);
	}
	
	
	public static float safeAcos(float x)
	{
		if (( x <= -1 )) 
		{
			return PI;
		}
		
		if (( x >= 1 )) 
		{
			return 0;
		}
		
		return  (float) Math.acos(x);
	}
	
	
	public static float atan2(float y, float x)
	{
		return (float) Math.atan2(y, x);
	}
	
	
	public static float sqrt(float x)
	{
		return (float) Math.sqrt(x);
	}
	
	
	/**
	 * Clamsps x between min and max
	 * @param x
	 * @param min
	 * @param max
	 * @return
	 */
	public static float clamp(float x, float min, float max)
	{
		if (( x < min )) 
		{
			return min;
		}
		else
		{
			if (( x > max )) 
			{
				return max;
			}
			else
			{
				return x;
			}
			
		}
		
	}
	
	/**
	 * 
	 * @return
	 */
	public static float rand()
	{
		return (float) Math.random();
	}
	
	/**
	 * Returns random number in [-1,1]
	 * @return
	 */
	public static float randIn(float min, float max)
	{
		return (float) ( min + ( rand() * (( max - min )) ) );
	}
	
	/**
	 * Returns new random Vec3 with each component in [min,max]
	 * @return
	 */
	public static Vec3 randVec3In(float min, float max)
	{
		return new Vec3(randIn( min, max ),randIn( min, max ),randIn( min, max ));
	}
	
	/**
	 * Returns new random Vec3 with each component in [-1,1]
	 * @return
	 */
	public static Vec3 randVec3()
	{
		return randVec3In(-1,1);
	}


	public static float min(float a, float b) {
		return a<b?a:b;
	
	}
	
	public static float max(float a, float b) {
		return a>b?a:b;
	}
	
	
}


