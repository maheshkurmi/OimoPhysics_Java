package oimo.collision.narrowphase.detector;

import oimo.collision.geometry.*;
import oimo.collision.narrowphase.*;
import oimo.common.MathUtil;
import oimo.common.Transform;
import oimo.common.Vec3;
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
	protected void detectImpl(DetectorResult result, Geometry geom1, Geometry geom2, Transform tf1, Transform tf2, CachedDetectorData cachedData) {
		SphereGeometry s =  (SphereGeometry) geom1;
		BoxGeometry b =  (BoxGeometry) geom2;

		result.incremental = false;
		
		double r = s._radius;
		
		//find min and max half extents of box
		Vec3 halfExt=b._halfExtents.clone();
		Vec3 negHalfExt=b._halfExtents.clone().negateEq();
			
		//find center of sphere in boxframe	
		Vec3 boxToSphereInBox= new Vec3();
		M.vec3_sub(boxToSphereInBox, tf1._position, tf2._position);
		M.vec3_mulMat3Transposed(boxToSphereInBox, boxToSphereInBox, tf2._rotation);
		
		// is the center of the sphere inside the box?
		boolean insideBox = M.aabb_overlap(negHalfExt, halfExt, boxToSphereInBox, boxToSphereInBox);
		if(insideBox) {
			// compute minimum distance between the center of the sphere and the box's surfaces that are perpendicular to each axis
			double  distX = halfExt.x-Math.abs(boxToSphereInBox.x);
			double  distY = halfExt.y-Math.abs(boxToSphereInBox.y);
			double  distZ = halfExt.z-Math.abs(boxToSphereInBox.z);
			
			double  normalInBoxX;
			double  normalInBoxY;
			double  normalInBoxZ;
			double depth;
			
			double  projectionMaskX;
			double  projectionMaskY;
			double  projectionMaskZ;
			// ... and select the smallest one, setting normal
			if(distX < distY) {
				if(distX < distZ) {
					if(boxToSphereInBox.x > 0) {
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
					if(boxToSphereInBox.z > 0) {
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
				if(boxToSphereInBox.y > 0) {
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
				if(boxToSphereInBox.z > 0) {
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
			double baseX = projectionMaskX * boxToSphereInBox.x;
			double baseY = projectionMaskY * boxToSphereInBox.y;
			double baseZ = projectionMaskZ * boxToSphereInBox.z;
			
			double boxToClosestPointInBoxX = normalInBoxX * halfExt.x;
			double boxToClosestPointInBoxY = normalInBoxY * halfExt.y;
			double boxToClosestPointInBoxZ = normalInBoxZ * halfExt.z;
			boxToClosestPointInBoxX += baseX;
			boxToClosestPointInBoxY += baseY;
			boxToClosestPointInBoxZ += baseZ;
			
			// bring them back to the world coordinate system
			double boxToClosestPointX;
			double boxToClosestPointY;
			double boxToClosestPointZ;
			double normalX;
			double normalY;
			double normalZ;
			
			// M.vec3_mulMat3(boxToClosestPoint, boxToClosestPointInBox, tf2._rotation);
			boxToClosestPointX = tf2._rotation.e00 * boxToClosestPointInBoxX + tf2._rotation.e01 * boxToClosestPointInBoxY + tf2._rotation.e02 * boxToClosestPointInBoxZ;
			boxToClosestPointY = tf2._rotation.e10 * boxToClosestPointInBoxX + tf2._rotation.e11 * boxToClosestPointInBoxY + tf2._rotation.e12 * boxToClosestPointInBoxZ;
			boxToClosestPointZ = tf2._rotation.e20 * boxToClosestPointInBoxX + tf2._rotation.e21 * boxToClosestPointInBoxY + tf2._rotation.e22 * boxToClosestPointInBoxZ;
			// M.vec3_mulMat3(normal, normalInBox, tf2._rotation);
			normalX = tf2._rotation.e00 * normalInBoxX + tf2._rotation.e01 * normalInBoxY + tf2._rotation.e02 * normalInBoxZ;
			normalY = tf2._rotation.e10 * normalInBoxX + tf2._rotation.e11 * normalInBoxY + tf2._rotation.e12 * normalInBoxZ;
			normalZ = tf2._rotation.e20 * normalInBoxX + tf2._rotation.e21 * normalInBoxY + tf2._rotation.e22 * normalInBoxZ;
		
			this.setNormal(result,normalX,normalY,normalZ);
			double pos1X = tf1._position.x + normalX * -r;
			double pos1Y = tf1._position.y + normalY * -r;
			double pos1Z = tf1._position.z + normalZ * -r;
			double pos2X = tf2._position.x + boxToClosestPointX;
			double pos2Y = tf2._position.y + boxToClosestPointY;
			double pos2Z = tf2._position.z + boxToClosestPointZ;
			this.addPoint(result,pos1X,pos1Y,pos1Z,pos2X,pos2Y,pos2Z,depth,0);
			return;
		}

		// compute the closest point to the center of the sphere; just clamp the coordinate of sphere to box AABB
		double eps = 1e-9;
		halfExt.sub3Eq(eps,eps,eps);
		negHalfExt.add3Eq(eps,eps,eps);
		//clamp 
		double boxToClosestPointInBoxX=MathUtil.clamp(boxToSphereInBox.x, negHalfExt.x,halfExt.x);
		double boxToClosestPointInBoxY=MathUtil.clamp(boxToSphereInBox.y, negHalfExt.y,halfExt.y);
		double boxToClosestPointInBoxZ=MathUtil.clamp(boxToSphereInBox.z, negHalfExt.z,halfExt.z);
		
		double closestPointToSphereInBoxX = boxToSphereInBox.x - boxToClosestPointInBoxX;
		double closestPointToSphereInBoxY = boxToSphereInBox.y - boxToClosestPointInBoxY;
		double closestPointToSphereInBoxZ = boxToSphereInBox.z - boxToClosestPointInBoxZ;
		
		//check if closest point to sphere lies inside sphere
		double dist = closestPointToSphereInBoxX * closestPointToSphereInBoxX + closestPointToSphereInBoxY * closestPointToSphereInBoxY + closestPointToSphereInBoxZ * closestPointToSphereInBoxZ;
		if(dist >= r * r) {
			return;
		}
		
		dist = MathUtil.sqrt(dist);
		// bring them back to the world coordinate system
		double boxToClosestPointX = tf2._rotation.e00 * boxToClosestPointInBoxX + tf2._rotation.e01 * boxToClosestPointInBoxY + tf2._rotation.e02 * boxToClosestPointInBoxZ;
		double boxToClosestPointY = tf2._rotation.e10 * boxToClosestPointInBoxX + tf2._rotation.e11 * boxToClosestPointInBoxY + tf2._rotation.e12 * boxToClosestPointInBoxZ;
		double boxToClosestPointZ = tf2._rotation.e20 * boxToClosestPointInBoxX + tf2._rotation.e21 * boxToClosestPointInBoxY + tf2._rotation.e22 * boxToClosestPointInBoxZ;
	
		double closestPointToSphereX = tf2._rotation.e00 * closestPointToSphereInBoxX + tf2._rotation.e01 * closestPointToSphereInBoxY + tf2._rotation.e02 * closestPointToSphereInBoxZ;
		double closestPointToSphereY = tf2._rotation.e10 * closestPointToSphereInBoxX + tf2._rotation.e11 * closestPointToSphereInBoxY + tf2._rotation.e12 * closestPointToSphereInBoxZ;
		double closestPointToSphereZ = tf2._rotation.e20 * closestPointToSphereInBoxX + tf2._rotation.e21 * closestPointToSphereInBoxY + tf2._rotation.e22 * closestPointToSphereInBoxZ;
	
		double l = closestPointToSphereX * closestPointToSphereX + closestPointToSphereY * closestPointToSphereY + closestPointToSphereZ * closestPointToSphereZ;
		if(l > 0) {
			l = 1 / MathUtil.sqrt(l);
		}
		
		double normalX = closestPointToSphereX * l;
		double normalY = closestPointToSphereY * l;
		double normalZ = closestPointToSphereZ * l;
		
		this.setNormal(result,normalX,normalY,normalZ);
		
		double pos1X = tf1._position.x + normalX * -r;
		double pos1Y = tf1._position.y + normalY * -r;
		double pos1Z = tf1._position.z + normalZ * -r;
		double pos2X = tf2._position.x + boxToClosestPointX;
		double pos2Y = tf2._position.y + boxToClosestPointY;
		double pos2Z = tf2._position.z + boxToClosestPointZ;
		this.addPoint(result,pos1X,pos1Y,pos1Z,pos2X,pos2Y,pos2Z,r - dist,0);
	}
}