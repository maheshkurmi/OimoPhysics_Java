package oimo.collision.narrowphase.detector.gjkepa;
import oimo.common.M;
import oimo.common.MathUtil;
import oimo.common.Setting;
import oimo.common.Vec3;

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

	// --- private ---
	private EpaTriangle pickTriangle() {
		EpaTriangle first = this._trianglePool;
		if(first != null) {
			this._trianglePool = first._next;
			first._next = null;
		} else {
			first = new EpaTriangle();
		}
		return first;
		//return M.singleList_pick(_trianglePool, _next, new EpaTriangle());
	}

	private void poolTriangle(EpaTriangle t) {
		t.removeReferences();
		t._next = _trianglePool;
		_trianglePool = t;
		//t.removeReferences();
		//M.singleList_pool(_trianglePool, _next, t);
	}

	private void  setAdjacentTriangle(EpaTriangle t1, EpaTriangle t2) {
		if (!t1.setAdjacentTriangle(t2)) {
			_status = EpaPolyhedronState.INVALID_TRIANGLE;
		}
	}

	private void initTriangle(EpaTriangle t, EpaVertex vertex1, EpaVertex vertex2,EpaVertex vertex3, Vec3 center, boolean autoCheck ) {
		if (!t.init(vertex1, vertex2, vertex3, center, autoCheck)) {
			_status = EpaPolyhedronState.INVALID_TRIANGLE;
		}
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
			EpaTriangle n=t._next;
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
			t =n;
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
				int i2 = base._nextIndex[i];
				EpaVertex v1 = base._vertices[i];
				EpaVertex v2 = base._vertices[i2];
				v1._tmpEdgeLoopNext = v2;
				v1._tmpEdgeLoopOuterTriangle = t;
			}
		}

		// expand the hole
		base.removeAdjacentTriangles();
		removeTriangle(base);
	}

	private void addTriangle(EpaTriangle t) {
		this._numTriangles++;
		if(this._triangleList == null) {
			this._triangleList = t;
			this._triangleListLast = t;
		} else {
			this._triangleListLast._next = t;
			t._prev = this._triangleListLast;
			this._triangleListLast = t;
		}
//		_numTriangles++;
//		GjkEpaLog.log("triangle added " + _numTriangles + ", id: " + t.id);
//		GjkEpaLog.run(t.dump());
//		M.list_push(_triangleList, _triangleListLast, _prev, _next, t);
	}
	
	
	private void removeTriangle(EpaTriangle t){
		this._numTriangles--;
		GjkEpaLog.log("triangle removed " + _numTriangles + ", id: " + t.id);
		
		EpaTriangle prev = t._prev;
		EpaTriangle next = t._next;
		if(prev != null) {
			prev._next = next;
		}
		if(next != null) {
			next._prev = prev;
		}
		if(t == this._triangleList) {
			this._triangleList = this._triangleList._next;
		}
		if(t == this._triangleListLast) {
			this._triangleListLast = this._triangleListLast._prev;
		}
		t._next = null;
		t._prev = null;
		poolTriangle(t);	
		
//		_numTriangles--;
//		GjkEpaLog.log("triangle removed " + _numTriangles + ", id: " + t.id);
//		M.list_remove(_triangleList, _triangleListLast, _prev, _next, t);
//		poolTriangle(t);
		
	}
	
	EpaVertex _pickVertex() {
		EpaVertex first = this._vertexPool;
		if(first != null) {
			this._vertexPool = first._next;
			first._next = null;
		} else {
			first = new EpaVertex();
		}
		return first;
	}
	
	private void _poolVertex(EpaVertex v) {
		v.removeReferences();
		v._next = _vertexPool;
		_vertexPool = v;
//		v.removeReferences();
//		M.singleList_pool(_vertexPool, _next, v);
	}
	
	
	public void _clear() {
		while (_numTriangles > 0) {
			removeTriangle(_triangleList);
		}
		//M.assert(_triangleList == null);
		//M.assert(_triangleListLast == null);
		while (_numVertices > 0) {
			_poolVertex(_vertices[--_numVertices]);
		}
		
//		while (_numTriangles > 0) {
//			EpaTriangle t = _triangleList;
//			_numTriangles--;
//			EpaTriangle prev = t._prev;
//			EpaTriangle next = t._next;
//			if (prev != null) {
//				prev._next = next;
//			}
//			if (next != null) {
//				next._prev = prev;
//			}
//			if (t ==_triangleList) {
//				_triangleList = _triangleList._next;
//			}
//			if (t == _triangleListLast) {
//				_triangleListLast = _triangleListLast._prev;
//			}
//			t._next = null;
//			t._prev = null;
//			t.removeReferences();
//			t._next = _trianglePool;
//			_trianglePool = t;
//		}
//		while (_numVertices > 0) {
//			EpaVertex v = _vertices[--_numVertices];
//			v.removeReferences();
//			v._next = _vertexPool;
//			_vertexPool = v;
//		}
	}
	
	
	public boolean _init(EpaVertex v1, EpaVertex v2, EpaVertex v3, EpaVertex v4) {
		_status = EpaPolyhedronState.OK;
		_numVertices = 4;
		_vertices[0] = v1;
		_vertices[1] = v2;
		_vertices[2] = v3;
		_vertices[3] = v4;
		_center.copyFrom(v1.v).addEq(v2.v).addEq(v3.v).addEq(v4.v).scaleEq(0.25);
		EpaTriangle t1;
		EpaTriangle t2;
		EpaTriangle t3;
		EpaTriangle t4;
		t1 = pickTriangle();
		t2 = pickTriangle();
		t3 = pickTriangle();
		t4 = pickTriangle();
		initTriangle(t1, v1, v2, v3, _center, true);
		initTriangle(t2, v1, v2, v4, _center, true);
		initTriangle(t3, v1, v3, v4, _center, true);
		initTriangle(t4, v2, v3, v4, _center, true);
		setAdjacentTriangle(t1, t2);
		setAdjacentTriangle(t1, t3);
		setAdjacentTriangle(t1, t4);
		setAdjacentTriangle(t2, t3);
		setAdjacentTriangle(t2, t4);
		setAdjacentTriangle(t3, t4);
		addTriangle(t1);
		addTriangle(t2);
		addTriangle(t3);
		addTriangle(t4);
		return _status == EpaPolyhedronState.OK;
//		this._status = 0;
//		this._numVertices = 4;
//		this._vertices[0] = v1;
//		this._vertices[1] = v2;
//		this._vertices[2] = v3;
//		this._vertices[3] = v4;
//		Vec3 _this = this._center;
//		Vec3 v = v1.v;
//		_this.x = v.x;
//		_this.y = v.y;
//		_this.z = v.z;
//		Vec3 v5 = v2.v;
//		_this.x += v5.x;
//		_this.y += v5.y;
//		_this.z += v5.z;
//		Vec3 v6 = v3.v;
//		_this.x += v6.x;
//		_this.y += v6.y;
//		_this.z += v6.z;
//		Vec3 v7 = v4.v;
//		_this.x += v7.x;
//		_this.y += v7.y;
//		_this.z += v7.z;
//		_this.x *= 0.25;
//		_this.y *= 0.25;
//		_this.z *= 0.25;
//		EpaTriangle first = this._trianglePool;
//		if(first != null) {
//			this._trianglePool = first._next;
//			first._next = null;
//		} else {
//			first = new oimo.collision.narrowphase.detector.gjkepa.EpaTriangle();
//		}
//		EpaTriangle t1 = first;
//		EpaTriangle first1 = this._trianglePool;
//		if(first1 != null) {
//			this._trianglePool = first1._next;
//			first1._next = null;
//		} else {
//			first1 = new oimo.collision.narrowphase.detector.gjkepa.EpaTriangle();
//		}
//		EpaTriangle t2 = first1;
//		EpaTriangle first2 = this._trianglePool;
//		if(first2 != null) {
//			this._trianglePool = first2._next;
//			first2._next = null;
//		} else {
//			first2 = new oimo.collision.narrowphase.detector.gjkepa.EpaTriangle();
//		}
//		EpaTriangle t3 = first2;
//		EpaTriangle first3 = this._trianglePool;
//		if(first3 != null) {
//			this._trianglePool = first3._next;
//			first3._next = null;
//		} else {
//			first3 = new oimo.collision.narrowphase.detector.gjkepa.EpaTriangle();
//		}
//		EpaTriangle t4 = first3;
//		if(!t1.init(v1,v2,v3,this._center,true)) {
//			this._status = 1;
//		}
//		if(!t2.init(v1,v2,v4,this._center,true)) {
//			this._status = 1;
//		}
//		if(!t3.init(v1,v3,v4,this._center,true)) {
//			this._status = 1;
//		}
//		if(!t4.init(v2,v3,v4,this._center,true)) {
//			this._status = 1;
//		}
//		if(!t1.setAdjacentTriangle(t2)) {
//			this._status = 1;
//		}
//		if(!t1.setAdjacentTriangle(t3)) {
//			this._status = 1;
//		}
//		if(!t1.setAdjacentTriangle(t4)) {
//			this._status = 1;
//		}
//		if(!t2.setAdjacentTriangle(t3)) {
//			this._status = 1;
//		}
//		if(!t2.setAdjacentTriangle(t4)) {
//			this._status = 1;
//		}
//		if(!t3.setAdjacentTriangle(t4)) {
//			this._status = 1;
//		}
//		this._numTriangles++;
//		if(this._triangleList == null) {
//			this._triangleList = t1;
//			this._triangleListLast = t1;
//		} else {
//			this._triangleListLast._next = t1;
//			t1._prev = this._triangleListLast;
//			this._triangleListLast = t1;
//		}
//		this._numTriangles++;
//		if(this._triangleList == null) {
//			this._triangleList = t2;
//			this._triangleListLast = t2;
//		} else {
//			this._triangleListLast._next = t2;
//			t2._prev = this._triangleListLast;
//			this._triangleListLast = t2;
//		}
//		this._numTriangles++;
//		if(this._triangleList == null) {
//			this._triangleList = t3;
//			this._triangleListLast = t3;
//		} else {
//			this._triangleListLast._next = t3;
//			t3._prev = this._triangleListLast;
//			this._triangleListLast = t3;
//		}
//		this._numTriangles++;
//		if(this._triangleList == null) {
//			this._triangleList = t4;
//			this._triangleListLast = t4;
//		} else {
//			this._triangleListLast._next = t4;
//			t4._prev = this._triangleListLast;
//			this._triangleListLast = t4;
//		}
//		return _status == EpaPolyhedronState.OK;
	}

	
	public EpaTriangle _getBestTriangle() {
		EpaTriangle f = _triangleList;
		double mind = MathUtil.POSITIVE_INFINITY;
		EpaTriangle minf = null;
		while(f != null) {
			EpaTriangle n=f._next;
			if(f._distanceSq < mind) {
				mind = f._distanceSq;
				minf = f;
			}
			f = n;
		}
		return minf;
	}

	public boolean _addVertex(EpaVertex vertex, EpaTriangle base) {
		this._vertices[this._numVertices++] = vertex;
		GjkEpaLog.log("vertex added " + _numVertices + " " + vertex.v);
		GjkEpaLog.log("begin polyhedron modifying...");

		EpaVertex v1 = base._vertices[0];
		
		GjkEpaLog.log("trying to find a edge loop... v=" + vertex.v);
		// make a hole on the polyhedron finding its edge loop
		this.findEdgeLoop(this._numVertices,base,vertex.v);
		if(this._status != 0) {
			return false;
		}
		dumpHoleEdge(v1);
		
		// ... and "patch" the hole
		EpaVertex v = v1;
		EpaVertex firstV = v1;
		EpaTriangle prevT = null;
		EpaTriangle firstT = null;
		do {
			if(v._tmpEdgeLoopNext == null) {
				GjkEpaLog.log("edge loop is broken:");
				this._dumpAsObjModel();
				this._status = EpaPolyhedronState.EDGE_LOOP_BROKEN;
				return false;
			}
			if(v._tmpEdgeLoopOuterTriangle == null) {
				this._status = EpaPolyhedronState.NO_OUTER_TRIANGLE;
				return false;
			}
			EpaTriangle t = pickTriangle();
			if (firstT == null) firstT = t;
			GjkEpaLog.log("patching...");

			initTriangle(t, v, v._tmpEdgeLoopNext, vertex, _center,false);
			if (_status != EpaPolyhedronState.OK) return false;
			addTriangle(t);

			setAdjacentTriangle(t, v._tmpEdgeLoopOuterTriangle);
			if (prevT != null) setAdjacentTriangle(t, prevT);

			prevT = t;

			v = v._tmpEdgeLoopNext;
		} while (v != firstV);
		
		setAdjacentTriangle(prevT, firstT);
		
		return this._status == EpaPolyhedronState.OK && validate();
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

	
	
	
}