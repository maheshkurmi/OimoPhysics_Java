package oimo.collision.geometry;
import oimo.common.Transform;
import oimo.common.Vec3;
import oimo.common.M;
/**
 * A box collision geometry.
 */
public class BoxGeometry extends ConvexGeometry {
	public Vec3 _halfExtents;
	public Vec3 _halfAxisX;
	public Vec3 _halfAxisY;
	public Vec3 _halfAxisZ;

	/**
	 * Creates a box collision geometry of half-extents `halfExtents`.
	 */
	public BoxGeometry(Vec3 halfExtents) {
		super(GeometryType.BOX);
		_halfExtents=halfExtents.clone();
		_halfAxisX.set(halfExtents.x, 0, 0);
		_halfAxisY.set(0,halfExtents.y,0);
		_halfAxisZ.set(0,0,halfExtents.z);
		
		_updateMass();

		//float minHalfExtents = ( (( halfExtents.x < halfExtents.y )) ? (( (( halfExtents.z < halfExtents.x )) ? (halfExtents.z) : (halfExtents.x) )) : (( (( halfExtents.z < halfExtents.y )) ? (halfExtents.z) : (halfExtents.y) )) );
		float minHalfExtents=0;
			if (halfExtents.x < halfExtents.y) {
				if (halfExtents.z < halfExtents.x) {
					minHalfExtents=halfExtents.z;
				} else {
					minHalfExtents=halfExtents.x;
				}
			} else {
				if (halfExtents.z < halfExtents.y) {
					minHalfExtents=halfExtents.z;
				} else {
					minHalfExtents=halfExtents.y;
				}
			}
		;

		if (_gjkMargin > minHalfExtents * 0.2) _gjkMargin = minHalfExtents * 0.2f;
	}

	/**
	 * Returns the copy of half-extents of the box.
	 */
	public Vec3 getHalfExtents() {
		return  _halfExtents.clone();
	}

	/**
	 * Sets `halfExtents` to the half-extents of the box.
	 */
	public void getHalfExtentsTo(Vec3 halfExtents) {
		halfExtents.copyFrom(_halfExtents);
		//M.vec3_toVec3(halfExtents, _halfExtents);
	}

	@Override 
	public void _updateMass() {
		_volume = 8 * M.vec3_mulHorizontal(_halfExtents);
		Vec3 sq=_TMP_V1;
		M.vec3_compWiseMul(sq, _halfExtents, _halfExtents);
		
		M.mat3_diagonal(_inertiaCoeff,
			1 / 3 * (sq.y + sq.z),
			1 / 3 * (sq.z + sq.x),
			1 / 3 * (sq.x + sq.y)
		);
	}

	@Override 
	public void _computeAabb(Aabb aabb, Transform tf) {
		
		Vec3  tfx=_TMP_V1;
		Vec3  tfy=_TMP_V2;
		Vec3  tfz=new Vec3();
		
		M.vec3_mulMat3(tfx, _halfAxisX, tf._rotation);
		M.vec3_mulMat3(tfy, _halfAxisY, tf._rotation);
		M.vec3_mulMat3(tfz, _halfAxisZ, tf._rotation);
		M.vec3_abs(tfx, tfx);
		M.vec3_abs(tfy, tfy);
		M.vec3_abs(tfz, tfz);
		Vec3 tfs=new Vec3();
		
		M.vec3_add(tfs, tfx, tfy);
		M.vec3_add(tfs, tfs, tfz);

		M.vec3_sub(aabb._min, tf._position, tfs);
		M.vec3_add(aabb._max, tf._position, tfs);
	}
	

	@Override 
	public void computeLocalSupportingVertex(Vec3 dir,Vec3 out) {
		Vec3 gjkMargins=_TMP_V1;;
		Vec3 coreExtents=_TMP_V2;
		gjkMargins.set(_gjkMargin, _gjkMargin, _gjkMargin);
		M.vec3_min(gjkMargins, gjkMargins, _halfExtents); // avoid making core extents negative
		M.vec3_sub(coreExtents, _halfExtents, gjkMargins);
		out.x = dir.x > 0 ? coreExtents.x : -coreExtents.x;
		out.y = dir.y > 0 ? coreExtents.y : -coreExtents.y;
		out.z = dir.z > 0 ? coreExtents.z : -coreExtents.z;
		
		//out.y = dir.y > 0 ? M.vec3_get(coreExtents, 1) : -M.vec3_get(coreExtents, 1);
		//out.z = dir.z > 0 ? M.vec3_get(coreExtents, 2) : -M.vec3_get(coreExtents, 2);
	}

	@Override 
	public  boolean _rayCastLocal(Vec3 begin,Vec3 end,RayCastHit result) {
		
		float p1x = begin.x;
		float p1y = begin.y;
		float p1z = begin.z;
		float p2x = end.x;
		float p2y = end.y;
		float p2z = end.z;
		float halfW = _halfExtents.x;
		float halfH = _halfExtents.y;
		float halfD = _halfExtents.z;
		float dx = p2x - p1x;
		float dy = p2y - p1y;
		float dz = p2z - p1z;
		float tminx = 0;
		float tminy = 0;
		float tminz = 0;
		float tmaxx = 1;
		float tmaxy = 1;
		float tmaxz = 1;
		if (dx > -1e-6 && dx < 1e-6) {
			if (p1x <= -halfW || p1x >= halfW) {
				return false;
			}
		} else {
			float invDx = 1 / dx;
			float t1 = (-halfW - p1x) * invDx;
			float t2 = (halfW - p1x) * invDx;
			if (t1 > t2) {
				float tmp = t1;
				t1 = t2;
				t2 = tmp;
			}
			if (t1 > 0) tminx = t1;
			if (t2 < 1) tmaxx = t2;
		}

		if (dy > -1e-6 && dy < 1e-6) {
			if (p1y <= -halfH || p1y >= halfH) {
				return false;
			}
		} else {
			float invDy = 1 / dy;
			float t1 = (-halfH - p1y) * invDy;
			float t2 = (halfH - p1y) * invDy;
			if (t1 > t2) {
				float tmp = t1;
				t1 = t2;
				t2 = tmp;
			}
			if (t1 > 0) tminy = t1;
			if (t2 < 1) tmaxy = t2;
		}

		if (dz > -1e-6 && dz < 1e-6) {
			if (p1z <= -halfD || p1z >= halfD) {
				return false;
			}
		} else {
			float invDz = 1 / dz;
			float t1 = (-halfD - p1z) * invDz;
			float t2 = (halfD - p1z) * invDz;
			if (t1 > t2) {
				float tmp = t1;
				t1 = t2;
				t2 = tmp;
			}
			if (t1 > 0) tminz = t1;
			if (t2 < 1) tmaxz = t2;
		}

		if (tminx >= 1 || tminy >= 1 || tminz >= 1 || tmaxx <= 0 || tmaxy <= 0 || tmaxz <= 0) return false;
		float min = tminx;
		float max = tmaxx;
		int hitDirection = 0;
		if (tminy > min) {
			min = tminy;
			hitDirection = 1;
		}
		if (tminz > min) {
			min = tminz;
			hitDirection = 2;
		}
		if (tmaxy < max) {
			max = tmaxy;
		}
		if (tmaxz < max) {
			max = tmaxz;
		}
		if (min > max) return false;
		if (min == 0) return false; // the ray starts from inside
		switch (hitDirection) {
		case 0:
			result.normal.set(dx > 0 ? -1 : 1, 0, 0);
		case 1:
			result.normal.set(0, dy > 0 ? -1 : 1, 0);
		case 2:
			result.normal.set(0, 0, dz > 0 ? -1 : 1);
		}
		result.position.set(p1x + min * dx, p1y + min * dy, p1z + min * dz);
		result.fraction = min;
		return true;
	}

}