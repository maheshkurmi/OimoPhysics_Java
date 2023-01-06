package oimo.collision.narrowphase.detector.gjkepa;
import oimo.common.MathUtil;
import oimo.common.Setting;
import oimo.common.Vec3;
import oimo.common.M;

/**
 * Internal class.
 */
public class EpaPolyhedron {
	public EpaVertex[] _vertices;
	public int _numVertices;

	public EpaTriangle _triangleList;
	public EpaTriangle _triangleListLast;
	public int _numTriangles;

	public EpaTriangle _trianglePool;
	public EpaVertex _vertexPool;

	public Vec3 _center;
	public int _status;

	public EpaPolyhedron() {
		_vertices = new EpaVertex[Setting.maxEPAVertices];
		_center = new Vec3();
		_numVertices = 0;
		_triangleList = null;
		_triangleListLast = null;
		_numTriangles = 0;
		_trianglePool = null;
		_vertexPool = null;
	}


	public void dumpHoleEdge(EpaVertex first) {
//		GjkEpaLog.run({
//			var v:EpaVertex = first;
//			var vs = "";
//			var fs = "";
//			var cnt = 0;
//			do {
//				cnt += 2;
//				vs += "v " + v.v.x + " " + v.v.y + " " + v.v.z + "\n";
//				vs += "v " + v.v.x + " " + v.v.y + " " + v.v.z + "\n";
//				fs += "f " + (cnt - 1) + " " + cnt + " " + (cnt + 1) + "\n";
//				v = v._tmpEdgeLoopNext;
//			} while (v != first);
//			vs += "v " + v.v.x + " " + v.v.y + " " + v.v.z + "\n";
//			trace("edge loop data:\n" + vs + "\n" + fs);
//		});
	}

	public boolean validate() {
		EpaTriangle t = _triangleList;
		while(t != null) {
			for (int i=0 ;i<3;i++) {
				t._vertices[i]._tmpEdgeLoopOuterTriangle = null;
				t._vertices[i]._tmpEdgeLoopNext = null;
				if (t._adjacentPairIndex[i] == -1) {
					_status = EpaPolyhedronState.NO_ADJACENT_PAIR_INDEX;
					return false;
					//throw M.error("!?"));
				}
				if (t._adjacentTriangles[i] == null) {
					_status = EpaPolyhedronState.NO_ADJACENT_TRIANGLE;
					return false;
					//throw M.error("!?"));
				}
			}	
			t = t._next;
		}
		return true;
	}

	public void findEdgeLoop(int id,EpaTriangle base, Vec3 from) {
		if (base._tmpDfsId == id) return;
		base._tmpDfsId = id;
		GjkEpaLog.log("DFS: " + base.id);
		if (!base.checkVisible(id, from)) {
			_status = EpaPolyhedronState.TRIANGLE_INVISIBLE;
			GjkEpaLog.log("tri " + base.id + " is invisible!");
			return;
		}

		// find edges of the hole
		for (int i=0;i<3;i++) {
			EpaTriangle t = base._adjacentTriangles[i];
			if (t == null) continue;
			if (t.checkVisible(id, from)) {
				GjkEpaLog.log("tri " + t.id + " is visible.");
				findEdgeLoop(id, t, from);
			} else {
				// triangle `base` can be seen from `from`, but triangle `t` cannot.
				GjkEpaLog.log("tri " + t.id + " is invisible.");
				GjkEpaLog.log("added edge: " + base.id + " " + t.id);
				EpaVertex v1 = base._vertices[i];
				v1._tmpEdgeLoopNext = base._vertices[base._nextIndex[i]];
				v1._tmpEdgeLoopOuterTriangle = t;
			}
		}

		// expand the hole
		base.removeAdjacentTriangles();
		this._numTriangles--;
		EpaTriangle prev = base._prev;
		EpaTriangle next = base._next;
		if(prev != null) {
			prev._next = next;
		}
		if(next != null) {
			next._prev = prev;
		}
		if(base == this._triangleList) {
			this._triangleList = this._triangleList._next;
		}
		if(base == this._triangleListLast) {
			this._triangleListLast = this._triangleListLast._prev;
		}
		base._next = null;
		base._prev = null;
		base.removeReferences();
		base._next = this._trianglePool;
		this._trianglePool = base;
	}



	public boolean _init(EpaVertex v1, EpaVertex v2, EpaVertex v3, EpaVertex v4) {
		this._status = 0;
		this._numVertices = 4;
		this._vertices[0] = v1;
		this._vertices[1] = v2;
		this._vertices[2] = v3;
		this._vertices[3] = v4;
		Vec3 _this = this._center;
		Vec3 v = v1.v;
		_this.x = v.x;
		_this.y = v.y;
		_this.z = v.z;
		Vec3 v5 = v2.v;
		_this.x += v5.x;
		_this.y += v5.y;
		_this.z += v5.z;
		Vec3 v6 = v3.v;
		_this.x += v6.x;
		_this.y += v6.y;
		_this.z += v6.z;
		Vec3 v7 = v4.v;
		_this.x += v7.x;
		_this.y += v7.y;
		_this.z += v7.z;
		_this.x *= 0.25;
		_this.y *= 0.25;
		_this.z *= 0.25;
		EpaTriangle first = this._trianglePool;
		if(first != null) {
			this._trianglePool = first._next;
			first._next = null;
		} else {
			first = new oimo.collision.narrowphase.detector.gjkepa.EpaTriangle();
		}
		EpaTriangle t1 = first;
		EpaTriangle first1 = this._trianglePool;
		if(first1 != null) {
			this._trianglePool = first1._next;
			first1._next = null;
		} else {
			first1 = new oimo.collision.narrowphase.detector.gjkepa.EpaTriangle();
		}
		EpaTriangle t2 = first1;
		EpaTriangle first2 = this._trianglePool;
		if(first2 != null) {
			this._trianglePool = first2._next;
			first2._next = null;
		} else {
			first2 = new oimo.collision.narrowphase.detector.gjkepa.EpaTriangle();
		}
		EpaTriangle t3 = first2;
		EpaTriangle first3 = this._trianglePool;
		if(first3 != null) {
			this._trianglePool = first3._next;
			first3._next = null;
		} else {
			first3 = new oimo.collision.narrowphase.detector.gjkepa.EpaTriangle();
		}
		EpaTriangle t4 = first3;
		if(!t1.init(v1,v2,v3,this._center,true)) {
			this._status = 1;
		}
		if(!t2.init(v1,v2,v4,this._center,true)) {
			this._status = 1;
		}
		if(!t3.init(v1,v3,v4,this._center,true)) {
			this._status = 1;
		}
		if(!t4.init(v2,v3,v4,this._center,true)) {
			this._status = 1;
		}
		if(!t1.setAdjacentTriangle(t2)) {
			this._status = 1;
		}
		if(!t1.setAdjacentTriangle(t3)) {
			this._status = 1;
		}
		if(!t1.setAdjacentTriangle(t4)) {
			this._status = 1;
		}
		if(!t2.setAdjacentTriangle(t3)) {
			this._status = 1;
		}
		if(!t2.setAdjacentTriangle(t4)) {
			this._status = 1;
		}
		if(!t3.setAdjacentTriangle(t4)) {
			this._status = 1;
		}
		this._numTriangles++;
		if(this._triangleList == null) {
			this._triangleList = t1;
			this._triangleListLast = t1;
		} else {
			this._triangleListLast._next = t1;
			t1._prev = this._triangleListLast;
			this._triangleListLast = t1;
		}
		this._numTriangles++;
		if(this._triangleList == null) {
			this._triangleList = t2;
			this._triangleListLast = t2;
		} else {
			this._triangleListLast._next = t2;
			t2._prev = this._triangleListLast;
			this._triangleListLast = t2;
		}
		this._numTriangles++;
		if(this._triangleList == null) {
			this._triangleList = t3;
			this._triangleListLast = t3;
		} else {
			this._triangleListLast._next = t3;
			t3._prev = this._triangleListLast;
			this._triangleListLast = t3;
		}
		this._numTriangles++;
		if(this._triangleList == null) {
			this._triangleList = t4;
			this._triangleListLast = t4;
		} else {
			this._triangleListLast._next = t4;
			t4._prev = this._triangleListLast;
			this._triangleListLast = t4;
		}
		return _status == EpaPolyhedronState.OK;
	}

	
	public boolean _addVertex(EpaVertex vertex, EpaTriangle base) {
		GjkEpaLog.log("vertex added " + _numVertices + " " + vertex.v);
		GjkEpaLog.log("begin polyhedron modifying...");

		this._vertices[this._numVertices++] = vertex;
		EpaVertex v1 = base._vertices[0];
		GjkEpaLog.log("trying to find a edge loop... v=" + vertex.v);
		// make a hole on the polyhedron finding its edge loop

		this.findEdgeLoop(this._numVertices,base,vertex.v);
		if(this._status != 0) {
			return false;
		}
		EpaVertex v = v1;
		EpaTriangle prevT = null;
		EpaTriangle firstT = null;
		while(true) {
			if(v._tmpEdgeLoopNext == null) {
				GjkEpaLog.log("edge loop is broken:");
				
				this._dumpAsObjModel();
				this._status = 4;
				return false;
			}
			if(v._tmpEdgeLoopOuterTriangle == null) {
				this._status = 5;
				return false;
			}
			EpaTriangle first = this._trianglePool;
			if(first != null) {
				this._trianglePool = first._next;
				first._next = null;
			} else {
				first = new oimo.collision.narrowphase.detector.gjkepa.EpaTriangle();
			}
			EpaTriangle t = first;
			if(firstT == null) {
				firstT = t;
			}
			GjkEpaLog.log("patching...");

			if(!t.init(v,v._tmpEdgeLoopNext,vertex,this._center,false)) {
				this._status = 1;
			}
			if(this._status != 0) {
				return false;
			}
			this._numTriangles++;
			if(this._triangleList == null) {
				this._triangleList = t;
				this._triangleListLast = t;
			} else {
				this._triangleListLast._next = t;
				t._prev = this._triangleListLast;
				this._triangleListLast = t;
			}
			if(!t.setAdjacentTriangle(v._tmpEdgeLoopOuterTriangle)) {
				this._status = 1;
			}
			if(prevT != null) {
				if(!t.setAdjacentTriangle(prevT)) {
					this._status = 1;
				}
			}
			prevT = t;
			v = v._tmpEdgeLoopNext;
			if(!(v != v1)) {
				break;
			}
		}
		if(!prevT.setAdjacentTriangle(firstT)) {
			this._status = 1;
		}
		
		return _status == EpaPolyhedronState.OK && validate();
	}

	public void _dumpAsObjModel() {
//		GjkEpaLog.run({
//			trace("dumping .obj model of the polyhedron...");
//			var f:EpaTriangle = _triangleList;
//			var vs:String = "";
//			var fs:String = "";
//			var c:Int = 0;
//			M.list_foreach(f, _next, {
//				vs += "v " + f._vertices[0].v.x + " " + f._vertices[0].v.y + " " + f._vertices[0].v.z + "\n";
//				vs += "v " + f._vertices[1].v.x + " " + f._vertices[1].v.y + " " + f._vertices[1].v.z + "\n";
//				vs += "v " + f._vertices[2].v.x + " " + f._vertices[2].v.y + " " + f._vertices[2].v.z + "\n";
//				fs += "f " + ++c + " " + ++c + " " + ++c + "\n";
//			});
//			trace("\n\n#EPAPolyhedron\n" + vs + "\n" + fs + "\n\n");
//		});
	}



	public EpaTriangle _getBestTriangle() {
		EpaTriangle f = _triangleList;
		double mind = MathUtil.POSITIVE_INFINITY;
		EpaTriangle minf = null;
		while(f != null) {
			if(f._distanceSq < mind) {
				mind = f._distanceSq;
				minf = f;
			}
			f = f._next;
		}
		return minf;
	}


	public EpaVertex _pickVertex() {
		EpaVertex first = this._vertexPool;
		if(first != null) {
			this._vertexPool = first._next;
			first._next = null;
		} else {
			first = new EpaVertex();
		}
		return first;
	}
	
	
	public void _clear() {
		while (_numTriangles > 0) {
			EpaTriangle t = _triangleList;
			_numTriangles--;
			EpaTriangle prev = t._prev;
			EpaTriangle next = t._next;
			if (prev != null) {
				prev._next = next;
			}
			if (next != null) {
				next._prev = prev;
			}
			if (t ==_triangleList) {
				_triangleList = _triangleList._next;
			}
			if (t == _triangleListLast) {
				_triangleListLast = _triangleListLast._prev;
			}
			t._next = null;
			t._prev = null;
			t.removeReferences();
			t._next = _trianglePool;
			_trianglePool = t;
		}
		while (_numVertices > 0) {
			EpaVertex v = _vertices[--_numVertices];
			v.removeReferences();
			v._next = _vertexPool;
			_vertexPool = v;
		}
	}
	
}