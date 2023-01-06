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
		_halfAxisX=new Vec3(halfExtents.x, 0, 0);
		_halfAxisY=new Vec3(0,halfExtents.y,0);
		_halfAxisZ=new Vec3(0,0,halfExtents.z);
		
		_updateMass();

		//double minHalfExtents = ( (( halfExtents.x < halfExtents.y )) ? (( (( halfExtents.z < halfExtents.x )) ? (halfExtents.z) : (halfExtents.x) )) : (( (( halfExtents.z < halfExtents.y )) ? (halfExtents.z) : (halfExtents.y) )) );
		double minHalfExtents=0;
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
		Vec3 sq=new Vec3();
		M.vec3_compWiseMul(sq, _halfExtents, _halfExtents);
		
		M.mat3_diagonal(_inertiaCoeff,
			1 / 3.0 * (sq.y + sq.z),
			1 / 3.0 * (sq.z + sq.x),
			1 / 3.0 * (sq.x + sq.y)
		);
		//System.out.println(_inertiaCoeff);
	}

	@Override 
	public void _computeAabb(Aabb aabb, Transform tf) {
		
		Vec3  tfx=new Vec3();
		Vec3  tfy=new Vec3();
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
		Vec3 gjkMargins=new Vec3();
		Vec3 coreExtents=new Vec3();
		M.vec3_set(gjkMargins, _gjkMargin, _gjkMargin, _gjkMargin);
		M.vec3_min(gjkMargins, gjkMargins, _halfExtents); // avoid making core extents negative
		M.vec3_sub(coreExtents, _halfExtents, gjkMargins);
		out.x = dir.x > 0 ? coreExtents.x : -coreExtents.x;
		out.y = dir.y > 0 ? coreExtents.y : -coreExtents.y;
		out.z = dir.z > 0 ? coreExtents.z : -coreExtents.z;
		
	}

	@Override 
	public  boolean _rayCastLocal(Vec3 begin,Vec3 end,RayCastHit result) {
		
		double p1x = begin.x;
		double p1y = begin.y;
		double p1z = begin.z;
		double p2x = end.x;
		double p2y = end.y;
		double p2z = end.z;
		double halfW = _halfExtents.x;
		double halfH = _halfExtents.y;
		double halfD = _halfExtents.z;
		double dx = p2x - p1x;
		double dy = p2y - p1y;
		double dz = p2z - p1z;
		double tminx = 0;
		double tminy = 0;
		double tminz = 0;
		double tmaxx = 1;
		double tmaxy = 1;
		double tmaxz = 1;
		if (dx > -1e-6 && dx < 1e-6) {
			if (p1x <= -halfW || p1x >= halfW) {
				return false;
			}
		} else {
			double invDx = 1 / dx;
			double t1 = (-halfW - p1x) * invDx;
			double t2 = (halfW - p1x) * invDx;
			if (t1 > t2) {
				double tmp = t1;
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
			double invDy = 1 / dy;
			double t1 = (-halfH - p1y) * invDy;
			double t2 = (halfH - p1y) * invDy;
			if (t1 > t2) {
				double tmp = t1;
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
			double invDz = 1 / dz;
			double t1 = (-halfD - p1z) * invDz;
			double t2 = (halfD - p1z) * invDz;
			if (t1 > t2) {
				double tmp = t1;
				t1 = t2;
				t2 = tmp;
			}
			if (t1 > 0) tminz = t1;
			if (t2 < 1) tmaxz = t2;
		}

		if (tminx >= 1 || tminy >= 1 || tminz >= 1 || tmaxx <= 0 || tmaxy <= 0 || tmaxz <= 0) return false;
		double min = tminx;
		double max = tmaxx;
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
			break;
		case 1:
			result.normal.set(0, dy > 0 ? -1 : 1, 0);
			break;
		case 2:
			result.normal.set(0, 0, dz > 0 ? -1 : 1);
			break;
		}
		result.position.set(p1x + min * dx, p1y + min * dy, p1z + min * dz);
		result.fraction = min;
		return true;
	}

}