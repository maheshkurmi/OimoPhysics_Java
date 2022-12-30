package oimo.dynamics.constraint.solver.direct;

/**
 * Internal class
 */
public class BoundaryBuildInfo {
	public int size; // dimension

	public int numBounded;
	public int[] iBounded; // indices
	public int[] signs; // signs

	public int numUnbounded;
	public int[] iUnbounded; // indices

	// numBounded + numUnbounded <= n

	public BoundaryBuildInfo(int size) {
		this.size = size;
		numBounded = 0;
		iBounded = new int[size];
		signs = new int[size];
		numUnbounded = 0;
		iUnbounded = new int[size];
	}

	public void clear() {
		numBounded = 0;
		numUnbounded = 0;
	}

	public void pushBounded(int idx, int sign) {
		iBounded[numBounded] = idx;
		signs[numBounded] = sign;
		numBounded++;
	}

	public void pushUnbounded(int idx) {
		iUnbounded[numUnbounded] = idx;
		numUnbounded++;
	}

	public void popBounded() {
		numBounded--;
	}

	public void popUnbounded() {
		numUnbounded--;
	}

}
