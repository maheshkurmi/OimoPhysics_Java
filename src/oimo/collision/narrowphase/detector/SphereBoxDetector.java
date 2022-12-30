package oimo.collision.narrowphase.detector;
import oimo.collision.geometry.*;
import oimo.collision.narrowphase.*;
import oimo.common.MathUtil;
import oimo.common.Transform;
import oimo.common.M;

/**
 * Sphere vs Box collision detector.
 */
public class SphereBoxDetector extends Detector {
	/**
	 * If `swapped` is `true`, the collision detector expects `BoxGeometry` and `SphereGeometry` for the
	 * first and second argument of `SphereBoxDetector.detect`. If `swapped` is `false`, the collision detector expects
	 * `SphereGeometry` and `BoxGeometry` instead.
	 */
	public SphereBoxDetector(boolean swapped) {
		super(swapped);
	}

	@Override 
	public void detectImpl(DetectorResult result, Geometry geom1, Geometry geom2, Transform tf1, Transform tf2, CachedDetectorData cachedData) {
		SphereGeometry s =  (SphereGeometry) geom1;
		BoxGeometry b =  (BoxGeometry) geom2;

		result.incremental = false;

		float halfExtX = b._halfExtents.x;
		float halfExtY = b._halfExtents.y;
		float halfExtZ = b._halfExtents.z;
		float negHalfExtX = -halfExtX;
		float negHalfExtY = -halfExtY;
		float negHalfExtZ = -halfExtZ;
		
		float r = s._radius;
		float boxToSphereX = tf1._position.x - tf2._position.x;
		float boxToSphereY = tf1._position.y - tf2._position.y;
		float boxToSphereZ = tf1._position.z - tf2._position.z;
		
		//vec3_mulMat3Transposed(boxToSphereInBox, boxToSphere, tf2._rotation);
		float boxToSphereInBoxX = tf2._rotation.e00 * boxToSphereX + tf2._rotation.e10 * boxToSphereY + tf2._rotation.e20 * boxToSphereZ;
		float boxToSphereInBoxY = tf2._rotation.e01 * boxToSphereX + tf2._rotation.e11 * boxToSphereY + tf2._rotation.e21 * boxToSphereZ;
		float boxToSphereInBoxZ = tf2._rotation.e02 * boxToSphereX + tf2._rotation.e12 * boxToSphereY + tf2._rotation.e22 * boxToSphereZ;

	
		// is the center of the sphere inside the box?
		//boolean insideBox = M.aabb_overlap(negHalfExt, halfExt, boxToSphereInBox, boxToSphereInBox);
		if(negHalfExtX < boxToSphereInBoxX && halfExtX > boxToSphereInBoxX && negHalfExtY < boxToSphereInBoxY && halfExtY > boxToSphereInBoxY && negHalfExtZ < boxToSphereInBoxZ && halfExtZ > boxToSphereInBoxZ) {
			// compute minimum distance between the center of the sphere and the box's surfaces that are perpendicular to each axis
			float sphereToBoxSurfaceX = boxToSphereInBoxX < 0 ? -boxToSphereInBoxX : boxToSphereInBoxX;
			float sphereToBoxSurfaceY = boxToSphereInBoxY < 0 ? -boxToSphereInBoxY : boxToSphereInBoxY;
			float sphereToBoxSurfaceZ = boxToSphereInBoxZ < 0 ? -boxToSphereInBoxZ : boxToSphereInBoxZ;
			sphereToBoxSurfaceX = halfExtX - sphereToBoxSurfaceX;
			sphereToBoxSurfaceY = halfExtY - sphereToBoxSurfaceY;
			sphereToBoxSurfaceZ = halfExtZ - sphereToBoxSurfaceZ;
			
			float  normalInBoxX;
			float  normalInBoxY;
			float  normalInBoxZ;
			
			float  distX = sphereToBoxSurfaceX;
			float  distY = sphereToBoxSurfaceY;
			float  distZ = sphereToBoxSurfaceZ;
			float depth;
			
			float  projectionMaskX;
			float  projectionMaskY;
			float  projectionMaskZ;
		
			// ... and select the smallest one, setting normal
			if(distX < distY) {
				if(distX < distZ) {
					if(boxToSphereInBoxX > 0) {
						normalInBoxX = 1;
						normalInBoxY = 0;
						normalInBoxZ = 0;
					} else {
						normalInBoxX = -1;
						normalInBoxY = 0;
						normalInBoxZ = 0;
					}
					projectionMaskX = 0;
					projectionMaskY = 1;
					projectionMaskZ = 1;
					depth = distX;
				} else {
					if(boxToSphereInBoxZ > 0) {
						normalInBoxX = 0;
						normalInBoxY = 0;
						normalInBoxZ = 1;
					} else {
						normalInBoxX = 0;
						normalInBoxY = 0;
						normalInBoxZ = -1;
					}
					projectionMaskX = 1;
					projectionMaskY = 1;
					projectionMaskZ = 0;
					depth = distZ;
				}
			} else if(distY < distZ) {
				if(boxToSphereInBoxY > 0) {
					normalInBoxX = 0;
					normalInBoxY = 1;
					normalInBoxZ = 0;
				} else {
					normalInBoxX = 0;
					normalInBoxY = -1;
					normalInBoxZ = 0;
				}
				projectionMaskX = 1;
				projectionMaskY = 0;
				projectionMaskZ = 1;
				depth = distY;
			} else {
				if(boxToSphereInBoxZ > 0) {
					normalInBoxX = 0;
					normalInBoxY = 0;
					normalInBoxZ = 1;
				} else {
					normalInBoxX = 0;
					normalInBoxY = 0;
					normalInBoxZ = -1;
				}
				projectionMaskX = 1;
				projectionMaskY = 1;
				projectionMaskZ = 0;
				depth = distZ;
			}

			
			// compute the closest point
			float baseX = projectionMaskX * boxToSphereInBoxX;
			float baseY = projectionMaskY * boxToSphereInBoxY;
			float baseZ = projectionMaskZ * boxToSphereInBoxZ;
			
			float boxToClosestPointInBoxX = normalInBoxX * halfExtX;
			float boxToClosestPointInBoxY = normalInBoxY * halfExtY;
			float boxToClosestPointInBoxZ = normalInBoxZ * halfExtZ;
			boxToClosestPointInBoxX += baseX;
			boxToClosestPointInBoxY += baseY;
			boxToClosestPointInBoxZ += baseZ;
			
			// bring them back to the world coordinate system
			float boxToClosestPointX;
			float boxToClosestPointY;
			float boxToClosestPointZ;
			float normalX;
			float normalY;
			float normalZ;
			// M.vec3_mulMat3(boxToClosestPoint, boxToClosestPointInBox, tf2._rotation);
			boxToClosestPointX = tf2._rotation.e00 * boxToClosestPointInBoxX + tf2._rotation.e01 * boxToClosestPointInBoxY + tf2._rotation.e02 * boxToClosestPointInBoxZ;
			boxToClosestPointY = tf2._rotation.e10 * boxToClosestPointInBoxX + tf2._rotation.e11 * boxToClosestPointInBoxY + tf2._rotation.e12 * boxToClosestPointInBoxZ;
			boxToClosestPointZ = tf2._rotation.e20 * boxToClosestPointInBoxX + tf2._rotation.e21 * boxToClosestPointInBoxY + tf2._rotation.e22 * boxToClosestPointInBoxZ;
			// M.vec3_mulMat3(normal, normalInBox, tf2._rotation);
			normalX = tf2._rotation.e00 * normalInBoxX + tf2._rotation.e01 * normalInBoxY + tf2._rotation.e02 * normalInBoxZ;
			normalY = tf2._rotation.e10 * normalInBoxX + tf2._rotation.e11 * normalInBoxY + tf2._rotation.e12 * normalInBoxZ;
			normalZ = tf2._rotation.e20 * normalInBoxX + tf2._rotation.e21 * normalInBoxY + tf2._rotation.e22 * normalInBoxZ;
		
			this.setNormal(result,normalX,normalY,normalZ);
			float pos1X = tf1._position.x + normalX * -r;
			float pos1Y = tf1._position.y + normalY * -r;
			float pos1Z = tf1._position.z + normalZ * -r;
			float pos2X = tf2._position.x + boxToClosestPointX;
			float pos2Y = tf2._position.y + boxToClosestPointY;
			float pos2Z = tf2._position.z + boxToClosestPointZ;
			this.addPoint(result,pos1X,pos1Y,pos1Z,pos2X,pos2Y,pos2Z,depth,0);
			return;
		}

		// compute the closest point to the center of the sphere; just clamp the coordinate
		halfExtX -= 1e-9;
		halfExtY -= 1e-9;
		halfExtZ -= 1e-9;
		// avoid division by zero
		negHalfExtX += 1e-9;
		negHalfExtY += 1e-9;
		negHalfExtZ += 1e-9;
		float boxToClosestPointInBoxX = boxToSphereInBoxX < halfExtX ? boxToSphereInBoxX : halfExtX;
		float boxToClosestPointInBoxY = boxToSphereInBoxY < halfExtY ? boxToSphereInBoxY : halfExtY;
		float boxToClosestPointInBoxZ = boxToSphereInBoxZ < halfExtZ ? boxToSphereInBoxZ : halfExtZ;
		if(!(boxToClosestPointInBoxX > negHalfExtX)) {
			boxToClosestPointInBoxX = negHalfExtX;
		}
		if(!(boxToClosestPointInBoxY > negHalfExtY)) {
			boxToClosestPointInBoxY = negHalfExtY;
		}
		if(!(boxToClosestPointInBoxZ > negHalfExtZ)) {
			boxToClosestPointInBoxZ = negHalfExtZ;
		}
		
		float closestPointToSphereInBoxX = boxToSphereInBoxX - boxToClosestPointInBoxX;
		float closestPointToSphereInBoxY = boxToSphereInBoxY - boxToClosestPointInBoxY;
		float closestPointToSphereInBoxZ = boxToSphereInBoxZ - boxToClosestPointInBoxZ;
		//dist =M.vec3_dot(closestPointToSphereInBox, closestPointToSphereInBox);
		float dist = closestPointToSphereInBoxX * closestPointToSphereInBoxX + closestPointToSphereInBoxY * closestPointToSphereInBoxY + closestPointToSphereInBoxZ * closestPointToSphereInBoxZ;
		if(dist >= r * r) {
			return;
		}
		
		dist = MathUtil.sqrt(dist);
		// bring them back to the world coordinate system
		float boxToClosestPointX = tf2._rotation.e00 * boxToClosestPointInBoxX + tf2._rotation.e01 * boxToClosestPointInBoxY + tf2._rotation.e02 * boxToClosestPointInBoxZ;
		float boxToClosestPointY = tf2._rotation.e10 * boxToClosestPointInBoxX + tf2._rotation.e11 * boxToClosestPointInBoxY + tf2._rotation.e12 * boxToClosestPointInBoxZ;
		float boxToClosestPointZ = tf2._rotation.e20 * boxToClosestPointInBoxX + tf2._rotation.e21 * boxToClosestPointInBoxY + tf2._rotation.e22 * boxToClosestPointInBoxZ;
	
		float closestPointToSphereX = tf2._rotation.e00 * closestPointToSphereInBoxX + tf2._rotation.e01 * closestPointToSphereInBoxY + tf2._rotation.e02 * closestPointToSphereInBoxZ;
		float closestPointToSphereY = tf2._rotation.e10 * closestPointToSphereInBoxX + tf2._rotation.e11 * closestPointToSphereInBoxY + tf2._rotation.e12 * closestPointToSphereInBoxZ;
		float closestPointToSphereZ = tf2._rotation.e20 * closestPointToSphereInBoxX + tf2._rotation.e21 * closestPointToSphereInBoxY + tf2._rotation.e22 * closestPointToSphereInBoxZ;
	
		float l = closestPointToSphereX * closestPointToSphereX + closestPointToSphereY * closestPointToSphereY + closestPointToSphereZ * closestPointToSphereZ;
		if(l > 0) {
			l = 1 / MathUtil.sqrt(l);
		}
		
		float normalX = closestPointToSphereX * l;
		float normalY = closestPointToSphereY * l;
		float normalZ = closestPointToSphereZ * l;
		
		this.setNormal(result,normalX,normalY,normalZ);
		
		float pos1X = tf1._position.x + normalX * -r;
		float pos1Y = tf1._position.y + normalY * -r;
		float pos1Z = tf1._position.z + normalZ * -r;
		float pos2X = tf2._position.x + boxToClosestPointX;
		float pos2Y = tf2._position.y + boxToClosestPointY;
		float pos2Z = tf2._position.z + boxToClosestPointZ;
		this.addPoint(result,pos1X,pos1Y,pos1Z,pos2X,pos2Y,pos2Z,r - dist,0);
	}
}