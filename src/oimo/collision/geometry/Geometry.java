package oimo.collision.geometry;

import oimo.common.M;
import oimo.common.Mat3;
import oimo.common.Transform;
import oimo.common.Vec3;

/**
 * Abstract collision geometry.
 */
public abstract class Geometry {
	public int _type;
	public float _volume;
	public Mat3 _inertiaCoeff; // I / mass
	protected Vec3 _TMP_V1=new Vec3(),_TMP_V2=new Vec3();
	
	public Geometry (int type) {
		_type = type;
		_volume = 0;
	}

	// --- internal ---
	/**
	 * Calculate volume, mass and inertia tensors for this geometry
	 */
	public abstract void _updateMass();

	/**
	 * Calculate AABB for this geometry when tranformed by specified transform
	 * @param result AABB to hold result
	 * @param tf Transform to apply on shape
	 * @return aabb
	 */
	public abstract void _computeAabb(Aabb result, Transform tf);

	/**
	 * Performs ray cast in local space of shape and stores result in 'hit' object
	 * @param begin Starting point of ray in local space of shape
	 * @param end Ending point of ray in local space of shape
	 * @param result Result of ray cast
	 * @return true if ray hits shape else returns false
	 */
	public abstract boolean _rayCastLocal(Vec3 begin, Vec3 end, RayCastHit result);

	// --- public ---

	/**
	 * Returns the type of the collision geometry.
	 *
	 * See `GeometryType` for details.
	 */
	public int getType() {
		return _type;
	}

	/**
	 * Returns the volume of the collision geometry.
	 */
	public float getVolume() {
		return _volume;
	}

	/**
	 * Performs ray casting. Returns `true` and sets the result information to `hit` if
	 * the line segment from `begin` to `end` and the geometry transformed by `transform`
	 * intersect. Returns `false` if the line segment and the geometry do not intersect.
	 * @param begin Starting point of ray in world space
	 * @param end End point of ray in world space
	 * @param result Transform of this shape in World Space
	 */
	public boolean rayCast(Vec3 begin, Vec3 end, Transform transform, RayCastHit result) {
		Vec3 beginLocal=_TMP_V1.copyFrom(begin);
		Vec3 endLocal=_TMP_V2.copyFrom(end);
		
		//transform points in Shapes local frame by multiplying with inverse transform
		M.vec3_sub(beginLocal, beginLocal, transform._position);
		M.vec3_sub(endLocal, endLocal, transform._position);
		M.vec3_mulMat3Transposed(beginLocal, beginLocal, transform._rotation);
		M.vec3_mulMat3Transposed(endLocal, endLocal, transform._rotation);
		
		//Perform Ray cast in local space of shape
		if (this._rayCastLocal(beginLocal, endLocal, result)) {
			//Ray has hit the shape and we got values in local space of shape
			Vec3 localPos=result.position.clone();
			Vec3 localNormal=result.normal.clone();
			
			// transform from local space to world spacce
			M.vec3_mulMat3(localPos, localPos, transform._rotation);
			M.vec3_mulMat3(localNormal, localNormal, transform._rotation);
			M.vec3_add(localPos, localPos, transform._position);
			// assign global result
			result.position.copyFrom(localPos);
			result.normal.copyFrom(localNormal);
	
			return true;
		}
		
		// ray does't hit shape
		return false;
	}

}
