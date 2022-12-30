package oimo.common;

public class M {
	
	private static Mat3 _M3 =new Mat3();
	private static Vec3 _V3 =new Vec3();
	private static Quat _Q =new Quat();

	public static void vec3_zero(Vec3 dst) {
		 dst.zero();
	}
	
	public static void vec3_set(Vec3 dst,float x, float y, float z) {
		 dst.set(x,y,z);
	}
	
	public static void vec3_toVec3(Vec3 dst,Vec3 src) {
		 dst.set(src.x,src.y,src.z);
	}
	
	public static void vec3_fromVec3(Vec3 dst, Vec3 src) {
		 dst.set(src.x,src.y,src.z);
	}
	
	/**
	 * dst= v1+v2*t
	 * @param dst
	 * @param v1
	 * @param v2
	 */
	public static void vec3_addRhsScaled(Vec3 dst, Vec3 v1, Vec3 v2, float t) {
		dst.set(v1.x+v2.x*t, v1.y+v2.y*t, v1.z+v2.z*t);
	}

	
	
	public static void  mat3_id(Mat3 dst){
		 dst.identity();
	}

	/**
	 * Copies 'src' Matrix to 'dst' matrix
	 * @param src
	 * @param dst
	 */
	public static void mat3_toMat3(Mat3 dst, Mat3 src) {
		dst.copyFrom(src);
	}

	/**
	 * Creates diagonal Matrix and stores result in 'dst' 
	 * @param dst
	 * @param e00 
	 * @param e11
	 * @param e22
	 */
	public static void mat3_diagonal(Mat3 dst, float e00, float e11, float e22) {
		dst.set(e00, 0, 0, 0, e11, 0, 0, 0, e22);
	}

	/**
	 * Creates Matrix from rotation angles and stores result in dst matrix
	 * @param dst
	 * @param xyz
	 */
	public static void mat3_fromEulerXyz(Mat3 dst, Vec3 xyz) {
		dst.fromEulerXyz(xyz);
	}

	/**
	 * Creates Matrix from rotation angles and stores result in dst matrix
	 * @param dst
	 * @param xyz
	 */
	public static void mat3_toEulerXyz( Vec3 dst, Mat3 rot) {
		dst.set(rot.toEulerXyz());
		
	}

	
	/**
	 * Multiplies src1 and src2 and stores in dst
	 * dst=src1*src2
	 * @param dst
	 * @param src1
	 * @param src2
	 */
	public static void mat3_mul(Mat3 dst, Mat3 src1, Mat3 src2) {
		_M3.copyFrom(src1).mulEq(src2);
		dst.copyFrom(_M3);
	}
	
	/**
	 * dst =src1*src2'
	 * @param dst
	 * @param src1
	 * @param src2
	 */
	public static void mat3_mulRhsTransposed(Mat3 dst, Mat3 src1, Mat3 src2) {
		//_M3.copyFrom(src2).transpose();
		//dst.copyFrom(src1).mul(_M3);
		dst.copyFrom(src1).mulTransposeEq(src2);
	}
	
	/**
	 * dst =src1'*src2
	 * @param dst
	 * @param src1
	 * @param src2
	 */
	public static void mat3_mulLhsTransposed(Mat3 dst, Mat3 src1, Mat3 src2) {
		//_M3.copyFrom(src2).transpose();
		//dst.copyFrom(src1).mul(_M3);
		dst.copyFrom(src1).transposeEq().mulEq(src2);
	}
	
	/**
	 * Rotates destination matrix by euler angles (in radians)
	 * dst=eulerMatrix*dst
	 * @param dst
	 * @param eulerXYZ Euler angles in radians
	 */
	public static void mat3_rotateXYZ(Mat3 dst, Vec3 eulerXYZ) {
		_M3.fromEulerXyz(eulerXYZ).mulEq(dst);
		dst.copyFrom(_M3);
	}

	public static void quat_fromMat3(Quat dest, Mat3 src) {
		dest.fromMat3(src);
	}

	public static String toFixed8(float e00) {
		return String.format("%.3f", e00);
	}

	

	public static void vec3_min(Vec3 dst, Vec3 src1, Vec3 src2) {
		dst.set(MathUtil.min(src1.x, src2.x), Math.min(src1.y, src2.y),Math.min(src1.z, src2.z));
	}
	
	public static void vec3_max(Vec3 dst, Vec3 src1, Vec3 src2) {
		dst.set(MathUtil.max(src1.x, src2.x), Math.max(src1.y, src2.y),Math.max(src1.z, src2.z));
	}

	
	/**
	 * Returns whether the pair of `aabb1` and `aabb2` is overlapping.
	 */
	public static boolean aabb_overlap(Vec3 aabbMin1, Vec3 aabbMax1, Vec3 aabbMin2, Vec3 aabbMax2) {
		return
			(aabbMin1.x < aabbMax2.x ) && (aabbMax1.x > aabbMin2.x) && 
			(aabbMin1.y < aabbMax2.y ) && (aabbMax1.y > aabbMin2.y) && 
			(aabbMin1.z < aabbMax2.z ) && (aabbMax1.z > aabbMin2.z) ;
			
	}

	public static void vec3_sub(Vec3 dst, Vec3 src1, Vec3 src2) {
		dst.x = src1.x-src2.x;
		dst.y = src1.y-src2.y;
		dst.z = src1.z-src2.z;
	}

	public static void vec3_add(Vec3 dst, Vec3 src1, Vec3 src2) {
		dst.x = src1.x+src2.x;
		dst.y = src1.y+src2.y;
		dst.z = src1.z+src2.z;
	}
	
	/**
	 * Sets dst = v1*f1 + v2*f2
	 * @param dst
	 * @param v1
	 * @param v2
	 * @param f1
	 * @param f2
	 */
	public static void vec3_mix2(Vec3 dst, Vec3 v1, Vec3 v2, float f1, float f2) {
		dst.x = f1*v1.x+f2*v2.x;
		dst.y = f1*v1.y+f2*v2.y;
		dst.z = f1*v1.z+f2*v2.z;
	}
	
	/**
	 * Sets dst = v1*f1 + v2*f2
	 * @param dst
	 * @param v1
	 * @param v2
	 * @param f1
	 * @param f2
	 */
	public static void vec3_mix3(Vec3 dst, Vec3 v1, Vec3 v2, Vec3 v3, float f1, float f2,float f3) {
		dst.x = f1*v1.x+f2*v2.x+f3*v3.x;
		dst.y = f1*v1.y+f2*v2.y+f3*v3.y;
		dst.z = f1*v1.z+f2*v2.z+f3*v3.z;
	}
	
	/**
	 * Swaps two vectors
	 * @param v1
	 * @param v2
	 */
	public static void vec3_swap(Vec3 v1, Vec3 v2) {
		float tmp=v1.x;
		v1.x=v2.x;
		v2.x=tmp;
		tmp=v1.y;
		v1.y=v2.y;
		v2.y=tmp;
	}
	
	
	/**
	 * Tansforms the vector 'v' by specified 'transform' and stores result in Vector 'dst'
	 * dst = v*transform
	 * @param dst
	 * @param v
	 * @param transform
	 */
	public static void vec3_mulMat3(Vec3 dst, Vec3 v, Mat3 transform) {
		_V3.copyFrom(v);
		dst.x = transform.e00 * _V3.x + transform.e01 * _V3.y + transform.e02 * _V3.z ;
		dst.y = transform.e10 * _V3.x + transform.e11 * _V3.y + transform.e12 * _V3.z ;
		dst.z = transform.e20 * _V3.x + transform.e21 * _V3.y + transform.e22 * _V3.z ;
	}
	
	public static void vec3_mulMat3Transposed(Vec3 dst, Vec3 v, Mat3 transform) {
		_V3.copyFrom(v);
		dst.x = transform.e00 * _V3.x + transform.e10 * _V3.y + transform.e20 * _V3.z ;
		dst.y = transform.e01 * _V3.x + transform.e11 * _V3.y + transform.e21 * _V3.z ;
		dst.z = transform.e02 * _V3.x + transform.e12 * _V3.y + transform.e22 * _V3.z ;
	}

	/**
	 * Return product of components of vector
	 * @param v
	 * @return
	 */
	public static float vec3_mulHorizontal(Vec3 v) {
		return v.x*v.y*v.z;
	}

	/**
	 * Returns vector after multiplication of components of v1 to the components of v2
	 * dst =v1*v2
	 * 
	 * @param dst
	 * @param v1
	 * @param v2
	 */
	public static void vec3_compWiseMul(Vec3 dst, Vec3 v1, Vec3 v2) {
		dst.set(v1.x*v2.x, v1.y*v2.y, v1.z*v2.z);
	}

	/**
	 * Sets dst components as absolute values of components of src
	 * @param dst
	 * @param src
	 */
	public static void vec3_abs(Vec3 dst, Vec3 src) {
		dst.set(Math.abs(src.x),Math.abs(src.y),Math.abs(src.z));
	}

	/**
	 * Extract column 'colIndex' from Matrix 'm' in Vector 'dst'
	 * @param dst
	 * @param m
	 * @param colIndex
	 */
	public static void mat3_getCol(Vec3 dst, Mat3 m, int colIndex) {
		m.getColTo(colIndex, dst);
	}

	/**
	 * Extract column 'rowIndex' from Matrix 'm' in Vector 'dst'
	 * @param dst
	 * @param m
	 * @param rowIndex
	 */
	public static void mat3_getRow(Vec3 dst, Mat3 m, int rowIndex) {
		m.getRowTo(rowIndex, dst);
	}

	
	
	/**
	 * Sets dst = src*s
	 * @param dst
	 * @param src
	 * @param s
	 */
	public static void vec3_scale(Vec3 dst, Vec3 src, float s) {
		dst.set(src.x*s,src.y*s,src.z*s);
	}

	/**
	 * returns a.b
	 * @param a
	 * @param b
	 * @return
	 */
	public static float vec3_dot(Vec3 a, Vec3 b) {
		return a.dot(b);
	}


	
	public static void vec3_cross(Vec3 dst, Vec3 v1, Vec3 v2) {
		dst.x = v1.y * v2.z - v1.z * v2.y;
		dst.y = v1.z * v2.x - v1.x * v2.z;
		dst.z = v1.x * v2.y - v1.y * v2.x;

	}

	public static void error(String string) {
		System.err.println(string);
		//throw new Exception(string);
	}

	public static void mat3_fromMat3(Mat3 dst, Mat3 src) {
		dst.copyFrom(src);
	}

	public static void transform_assign(Transform dst, Transform src) {
		dst.copyFrom(src);
	}


	/**
	 * dst=src1*src2
	 * @param dst
	 * @param src1
	 * @param src2
	 */
	public static void transform_mul(Transform dst, Transform src1, Transform src2) {
		M.mat3_mul(dst._rotation, src2._rotation, src1._rotation);
		M.vec3_mulMat3(dst._position, src1._position, src2._rotation);
		M.vec3_add(dst._position, dst._position, src2._position);
	}

	public static void mat3_transformInertia(Mat3 dst,Mat3 inertia, Mat3 rotation) {
		M.mat3_mul(dst, rotation, inertia);
		M.mat3_mulRhsTransposed(dst, dst, rotation);
	}
	
	public static Mat3 mat3_inertiaFromCOM(Mat3 dst, Vec3 com, float mass) {
		float xx = mass * com.x * com.x;// ${as[0].f()} * ${as[0].f()};
		float yy = mass * com.y * com.y;// ${as[1].f()} * ${as[1].f()};
		float zz = mass * com.z * com.z;// ${as[2].f()} * ${as[2].f()};
		float xy = -mass * com.x * com.y;// -${as[0].f()} * ${as[1].f()};
		float yz = -mass * com.y * com.z;// -${as[1].f()} * ${as[2].f()};
		float zx = -mass * com.z * com.x;// -${as[2].f()} * ${as[0].f()};
		return new Mat3(yy + zz, 	  xy,           zx, 
							 xy,      xx + zz,      yz, 
							 zx,      yz,           xx + yy);

	}
	
	public static void mat3_zero(Mat3 mat) {
		mat.zero();
	}
	
	public static void mat3_scaleRows(Mat3 dst, Mat3 src, float sx, float sy, float sz) {
		dst.e00=src.e00*sx;
		dst.e01=src.e01*sx;
		dst.e02=src.e02*sx;
		
		dst.e10=src.e10*sy;
		dst.e11=src.e11*sy;
		dst.e12=src.e12*sy;
		
		dst.e20=src.e20*sz;
		dst.e21=src.e21*sz;
		dst.e22=src.e22*sz;
	}

	public static void vec3_assign(Vec3 dst, Vec3 src) {
		dst.set(src);
	}

	public static void mat3_assign(Mat3 dst, Mat3 src) {
		dst.copyFrom(src);
	}

	public static float vec3_length(Vec3 rotation) {
		return rotation.length();
	}

	public static void quat_fromVec3AndFloat(Quat dq, Vec3 sinAxis, float cosHalfTheta) {
		dq.w=cosHalfTheta;
		dq.x=sinAxis.x;
		dq.y=sinAxis.y;
		dq.z=sinAxis.z;
		
	}

	/**
	 * 
	 * @param q
	 * @param a
	 * @param b
	 */
	public static void quat_mul(Quat dst, Quat a, Quat b) {
		float qax = a.x, qay = a.y, qaz = a.z, qaw = a.w;
		float qbx = b.x, qby = b.y, qbz = b.z, qbw = b.w;
		_Q.x = qax * qbw + qaw * qbx + qay * qbz - qaz * qby;
		_Q.y = qay * qbw + qaw * qby + qaz * qbx - qax * qbz;
		_Q.z = qaz * qbw + qaw * qbz + qax * qby - qay * qbx;
		_Q.w = qaw * qbw - qax * qbx - qay * qby - qaz * qbz;
		 dst.copyFrom(_Q);

	}

	public static boolean vec3_isZero(Vec3 v) {
		return v.x==0&&v.y==0&&v.z==0;
	}

	public static void trace(String string) {
		System.out.println(string);
	}

	/**
	 * Sets Quat to identity
	 * @param idQ
	 */
	public static void quat_id(Quat dst) {
		dst.set(0, 0, 0, 1);
	}

	/**
	 * Sets dst quaternion to the quaternion representing the shortest arc rotation from `v1` to `v2`, and return `this`.
	 * @param q
	 * @param v1
	 * @param v2
	 */
	public static void quat_arc(Quat dst, Vec3 v1, Vec3 v2) {
		dst.setArc(v1, v2);
	}

	/**
	 * the spherical linear interpolation between two quaternions `q1` and `q2` with interpolation parameter `t`. Both quaternions `q1` and `q2` must be normalized.
	 * @param dst to holde result
	 * @param q1
	 * @param q2
	 * @param t
	 */
	public static void quat_slerp(Quat dst, Quat q1, Quat q2, float t) {
		_Q.copyFrom(q1).slerp(q2, t);
	}

	/**
	 * 
	 * @param dst
	 * @param q
	 */
	public static void mat3_fromQuat(Mat3 dst, Quat q) {
		dst.fromQuat(q);
	}

	/**
	 * computes a normalized vector perpendicular to the src and stores result in dst
	 */
	public static void vec3_perp(Vec3 dst, Vec3 src) {
		float x1 = src.x;
		float y1 = src.y;
		float z1 = src.z;
		float x2 = x1*x1;
		float y2 = y1*y1;
		float z2 = z1*z1;
		
		if(x2<=y2 && x2<=z2) {
			// |x1| is the smallest, use x-axis as the RHS of the cross product
			float d = 1 / MathUtil.sqrt(y2 + z2);
			dst.set(0, z1 * d, -y1 * d);
		}else if(y2<=x2 && y2<=z2) {
			// |y1| is the smallest, use y-axis as the RHS of the cross product
			float d = 1 / MathUtil.sqrt(z2 + x2);
			dst.set( -z1 * d, 0, x1 * d);
		}else {
			// |z1| is the smallest, use z-axis as the RHS of the cross product
			float d = 1 / MathUtil.sqrt(x2 + y2);
			dst.set( y1 * d, -x1 * d, 0);
		}
	
	}

	public static void vec3_normalize(Vec3 dst, Vec3 src) {
		dst.set(src).normalize();
	}

	public static boolean aabb_contains(Vec3 _aabbMin, Vec3 _aabbMax, Vec3 _min, Vec3 _max) {
		if(_aabbMin.x <= _min.x && _aabbMax.x >= _max.x && _aabbMin.y <= _min.y && _aabbMax.y >= _max.y && _aabbMin.z <= _min.z && _aabbMax.z >= _max.z) {
			return true;
		}
		return false;
	}

	public static void mat3_fromCols(Mat3 mat, Vec3 col1, Vec3 col2, Vec3 col3) {
		mat.fromCols(col1, col2, col3);
	}

	public static float quat_getReal(Quat relQ) {
		// TODO Auto-generated method stub
		return 0;
	}

	public static void vec3_fromQuat(Vec3 dst, Quat q) {
		dst.set(q.x,q.y,q.z);
	}

	public static void vec3_toCrossMatrix(Mat3 dst, Vec3 v) {
		dst.set(0, -v.z, v.y, v.z, 0, -v.x, -v.y, v.x, 0);
	}

	
	

}
