package oimo.collision.narrowphase.detector.gjkepa;
import oimo.common.M;
import oimo.common.Vec3;

/**
 * Simplex utilities for GJK/EPA computations.
 */
public class SimplexUtil {
	/**
	 * Sets `out` to the minimum length point on the line (`vec1`, `vec2`)
	 * and returns the index of the voronoi region.
	 */
	public static int projectOrigin2(Vec3 vec1, Vec3 vec2, Vec3 out) {
		Vec3 v1=vec1;
		Vec3 v2=vec2;
	
		float v12x=v2.x-v1.x;
		float v12y=v2.y-v1.y;
		
		float d=v12x*v12x+v12y*v12y; //M.vec3_dot(v12, v12);
		float t=v12x*v1.x+v12y*v1.y; //M.vec3_dot(v12, v1);
		t = -t / d;

		if (t < 0) {
			out.set(v1);
			return 1;
		}
		if (t > 1) {
			out.set(v2);
			return 2;
		}

		out.x=v1.x+v12x*t;
		out.y=v1.y+v12y*t;
		
		return 3;
	}

	/**
	 * Sets `out` to the minimum length point on the triangle (`vec1`, `vec2`, `vec3`)
	 * and returns the index of the voronoi region.
	 */
	public static int projectOrigin3(Vec3 vec1, Vec3 vec2, Vec3 vec3, Vec3 out) {
		Vec3 v1=vec1;
		Vec3 v2=vec2;
		Vec3 v3=vec3;
		
		Vec3 v12=new Vec3();
		Vec3 v23=new Vec3();
		Vec3 v31=new Vec3();
		
		M.vec3_sub(v12, v2, v1);
		M.vec3_sub(v23, v3, v2);
		M.vec3_sub(v31, v1, v3);

		Vec3 n=new Vec3();
		
		M.vec3_cross(n, v12, v23);
		
		Vec3 n12=new Vec3();
		Vec3 n23=new Vec3();
		Vec3 n31=new Vec3();
	
		M.vec3_cross(n12, v12, n);
		M.vec3_cross(n23, v23, n);
		M.vec3_cross(n31, v31, n);
		float d12 = M.vec3_dot(v1, n12);
		float d23 = M.vec3_dot(v2, n23);
		float d31 = M.vec3_dot(v3, n31);

		float mind = -1;
		Vec3 minv=new Vec3();
		int mini = 0; // index of voronoi region

		if (d12 < 0) {
			int b = projectOrigin2(vec1, vec2, out);
			float d = out.x * out.x + out.y * out.y + out.z * out.z;
			mini = b;
			mind = d;
			minv.set(out);
			//M.vec3_fromVec3(minv, out);
		}
		if (d23 < 0) {
			int b = projectOrigin2(vec2, vec3, out);
			float d = out.x * out.x + out.y * out.y + out.z * out.z;
			if (mind < 0 || d < mind) {
				mini = b << 1; // 00000021 -> 00000210
				mind = d;
				minv.set(out);
				//M.vec3_fromVec3(minv, out);
			}
		}
		if (d31 < 0) {
			int b = projectOrigin2(vec1, vec3, out);
			float d = out.x * out.x + out.y * out.y + out.z * out.z;
			if (mind < 0 || d < mind) {
				mini = b & 1 | (b & 2) << 1; // 00000021 -> 00000201
				mind = d;
				minv.set(out);
				//M.vec3_fromVec3(minv, out);
			}
		}
		if (mind > 0) {
			out.set(minv);
			//M.vec3_toVec3(out, minv);
			return mini;
		}
		n.normalize();
		float dn = M.vec3_dot(v1, n);
		float l2 = M.vec3_dot(n, n);
		l2 = dn / l2;
		M.vec3_scale(minv, n, l2);
		M.vec3_toVec3(out, minv);
		return 7;
	}

	/**
	 * Sets `out` to the minimum length point on the tetrahedron (`vec1`, `vec2`, `vec3`, `vec4`)
	 * and returns the index of the voronoi region.
	 */
	public static int projectOrigin4(Vec3 vec1, Vec3 vec2, Vec3 vec3, Vec3 vec4, Vec3 out) {
		Vec3 v1=vec1;
		Vec3 v2=vec2;
		Vec3 v3=vec3;
		Vec3 v4=vec4;
		Vec3 v12=v2.sub(v1);
		Vec3 v13=v3.sub(v1);
		Vec3 v14=v4.sub(v1);
		Vec3 v23=v3.sub(v2);
		Vec3 v24=v4.sub(v2);
		Vec3 v34=v4.sub(v3);
		

		boolean rev;
		Vec3 n123=new Vec3();
		Vec3 n134=new Vec3();
		Vec3 n142=new Vec3();
		Vec3 n243=new Vec3();
		Vec3 n=new Vec3();
		M.vec3_cross(n123, v12, v13);
		M.vec3_cross(n134, v13, v14);
		M.vec3_cross(n142, v14, v12);
		M.vec3_cross(n243, v24, v23);

		int sign = M.vec3_dot(v12, n243) > 0 ? 1 : -1;
		float d123 = M.vec3_dot(v1, n123);
		float d134 = M.vec3_dot(v1, n134);
		float d142 = M.vec3_dot(v1, n142);
		float d243 = M.vec3_dot(v2, n243);

		float mind = -1;
		Vec3 minv=new Vec3();
		int mini = 0; // index of voronoi region

		if (d123 * sign < 0) {
			int b = projectOrigin3(vec1, vec2, vec3, out);
			float d = out.x * out.x + out.y * out.y + out.z * out.z;
			mini = b;
			mind = d;
			minv.set(out);
			//M.vec3_fromVec3(minv, out);
		}
		if (d134 * sign < 0) {
			int b = projectOrigin3(vec1, vec3, vec4, out);
			float d = out.x * out.x + out.y * out.y + out.z * out.z;
			if (mind < 0 || d < mind) {
				mini = b & 1 | (b & 6) << 1; // 00000321 -> 00003201
				mind = d;
				minv.set(out);
			}
		}
		if (d142 * sign < 0) {
			int b = projectOrigin3(vec1, vec2, vec4, out);
			float d = out.x * out.x + out.y * out.y + out.z * out.z;
			if (mind < 0 || d < mind) {
				mini = b & 3 | (b & 4) << 1; // 00000321 -> 00003021
				mind = d;
				minv.set(out);
			}
		}
		if (d243 * sign < 0) {
			int b = projectOrigin3(vec2, vec3, vec4, out);
			float d = out.x * out.x + out.y * out.y + out.z * out.z;
			if (mind < 0 || d < mind) {
				mini = b << 1; // 00000321 -> 00003210
				mind = d;
				minv.set(out);
			}
		}

		if (mind > 0) {
			M.vec3_toVec3(out, minv);
			return mini;
		}

		// the origin is inside the tetrahedron
		out.zero();
		return 15;
	}
}
