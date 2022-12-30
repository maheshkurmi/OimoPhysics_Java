package oimo.collision.narrowphase.detector.gjkepa;
import oimo.common.MathUtil;
import oimo.common.Vec3;

/**
 * Internal class.
 */
public class EpaVertex {
	public EpaVertex _next; // for object pooling

	public Vec3 v;
	public Vec3 w1;
	public Vec3 w2;

	public EpaVertex _tmpEdgeLoopNext;
	public EpaTriangle _tmpEdgeLoopOuterTriangle;

	public int randId=(int) (MathUtil.rand()*100000);

	public EpaVertex() {
		v = new Vec3();
		w1 = new Vec3();
		w2 = new Vec3();
	}

	public EpaVertex init(Vec3 v, Vec3 w1, Vec3 w2) {
		this.v.copyFrom(v);
		this.w1.copyFrom(w1);
		this.w2.copyFrom(w2);
		_next = null;
		_tmpEdgeLoopNext = null;
		_tmpEdgeLoopOuterTriangle = null;
		return this;
	}

	public void removeReferences() {
		_next = null;
		_tmpEdgeLoopNext = null;
		_tmpEdgeLoopOuterTriangle = null;
	}

}
