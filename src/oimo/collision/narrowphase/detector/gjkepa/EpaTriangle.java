package oimo.collision.narrowphase.detector.gjkepa;
import oimo.common.M;
import oimo.common.MathUtil;
import oimo.common.Vec3;

/**
 * Internal class.
 */
public class EpaTriangle {
	public EpaTriangle _next;
	public EpaTriangle _prev;

	public EpaVertex[] _vertices;
	public EpaTriangle[] _adjacentTriangles;
	public int[] _adjacentPairIndex;
	public Vec3 _normal;
	public double _distanceSq;

	public int[] _nextIndex; // (0, 1, 2) -> (1, 2, 0)

	public int _tmpDfsId;
	public boolean _tmpDfsVisible;

	Vec3 tmp;

	public static int count;
	public int id = ++count;

	public EpaTriangle() {
		_next = null;
		_prev = null;
		_normal = new Vec3();
		_distanceSq = 0;
		_tmpDfsId = 0;
		_tmpDfsVisible = false;

		_vertices = new EpaVertex[3];
		_adjacentTriangles = new EpaTriangle[3];
		_adjacentPairIndex = new int[3];
		tmp = new Vec3();
		_nextIndex = new int[3];
		_nextIndex[0] = 1;
		_nextIndex[1] = 2;
		_nextIndex[2] = 0;
	}

   public boolean  checkVisible(int id, Vec3 from) {
		// if (id == _tmpDfsId) return _tmpDfsVisible;
		tmp.copyFrom(from).subEq(_vertices[0].v);
		_tmpDfsVisible = tmp.dot(_normal) > 0;
		return _tmpDfsVisible;
	}

    /**
     * 
     * @param vertex1
     * @param vertex2
     * @param vertex3
     * @param center
     * @param autoCheck
     * @return
     */
	public boolean init(EpaVertex vertex1, EpaVertex vertex2, EpaVertex vertex3, Vec3 center, boolean autoCheck) {
		Vec3 v1=vertex1.v;
		Vec3 v2=vertex2.v;
		Vec3 v3=vertex3.v;
		Vec3 vc=center;
	
		var v12=new Vec3(); // 1 to 2
		var v13=new Vec3(); // 1 to 3
		var vc1=new Vec3(); // c to 1
		M.vec3_sub(v12, v2, v1);
		M.vec3_sub(v13, v3, v1);
		M.vec3_sub(vc1, v1, vc);
		Vec3 inor=new Vec3();
		M.vec3_cross(inor, v12, v13);
		boolean inverted = false;
		double d = M.vec3_dot(vc1, inor);
		if (d < 0) {
			if (autoCheck) {
				GjkEpaLog.log("found the triangle inverted, but it does not matter.");
				// vertices must be CCW
				var tmp = vertex2;
				vertex2 = vertex3;
				vertex3 = tmp;
				M.vec3_scale(inor, inor, -1);
			} else {
				GjkEpaLog.log("the triangle is inverted!");
				inverted = true;
				//return false;
			}
		}
		_vertices[0] = vertex1;
		_vertices[1] = vertex2;
		_vertices[2] = vertex3;
		M.vec3_toVec3(_normal, inor);
		SimplexUtil.projectOrigin3(vertex1.v, vertex2.v, vertex3.v, tmp);
		_distanceSq = tmp.lengthSq();

		_adjacentTriangles[0] = null;
		_adjacentTriangles[1] = null;
		_adjacentTriangles[2] = null;
		_adjacentPairIndex[0] = -1;
		_adjacentPairIndex[1] = -1;
		_adjacentPairIndex[2] = -1;
		return !inverted;
	}

	public boolean setAdjacentTriangle(EpaTriangle triangle) {
		int count = 0;
		for (int i=0;i<3;i++) {
			for (int j=0;j<3;j++) {
				int i2 = _nextIndex[i];
				int j2 = _nextIndex[j];
				if (_vertices[i] == triangle._vertices[j2] && _vertices[i2] == triangle._vertices[j]) {
					_adjacentTriangles[i] = triangle;
					_adjacentPairIndex[i] = j;
					triangle._adjacentTriangles[j] = this;
					triangle._adjacentPairIndex[j] = i;
					count++;
				}
			}
		}
		if (count != 1) {
			GjkEpaLog.log(_vertices[0].randId + " " + _vertices[1].randId + " " + _vertices[2].randId);
			GjkEpaLog.log(triangle._vertices[0].randId + " " + triangle._vertices[1].randId + " " + triangle._vertices[2].randId);
			return false; // invalid polyhedron
		}
		return true;
	}

	public void removeAdjacentTriangles() {
		for (int i=0;i<3;i++) {
			EpaTriangle triangle = _adjacentTriangles[i];
			if (triangle != null) {
				int pairIndex = _adjacentPairIndex[i];
				triangle._adjacentTriangles[pairIndex] = null;
				triangle._adjacentPairIndex[pairIndex] = -1;
				_adjacentTriangles[i] = null;
				_adjacentPairIndex[i] = -1;
			}
		}
	}

	public void removeReferences() {
		_next = null;
		_prev = null;
		_tmpDfsId = 0;
		_tmpDfsVisible = false;
		_distanceSq = 0;
		_vertices[0] = null;
		_vertices[1] = null;
		_vertices[2] = null;
		_adjacentTriangles[0] = null;
		_adjacentTriangles[1] = null;
		_adjacentTriangles[2] = null;
		_adjacentPairIndex[0] = 0;
		_adjacentPairIndex[1] = 0;
		_adjacentPairIndex[2] = 0;
	}

	public void dump() {
//		GjkEpaLog.log(
//					'Face data:
//				id:$id
//				v1:${_vertices[0].v}
//				v2:${_vertices[1].v}
//				v3:${_vertices[2].v}
//				n:$_normal
//				distSq:$_distanceSq
//				'
//		);
	}

}