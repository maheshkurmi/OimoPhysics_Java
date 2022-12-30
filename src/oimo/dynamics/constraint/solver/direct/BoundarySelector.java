package oimo.dynamics.constraint.solver.direct;

/**
 * Internal Class
 */
public class BoundarySelector {
	int n;
	int[] indices;
	int[] tmpIndices;

	public BoundarySelector(int n) {
		this.n = n;
		indices = new int[n];
		tmpIndices = new int[n];
		for (int i = 0; i < n; i++) {
			indices[i] = i;
		}
	}

	public int getIndex(int i) {
		return indices[i];
	}

	public void select(int index) {
		int i = 0;
		while (indices[i] != index) {
			i++;
		}
		while (i > 0) {
			int tmp = indices[i];
			indices[i] = indices[i - 1];
			indices[i - 1] = tmp;
			i--;
		}
		// validate();
	}

	/**
	 * Makes first n elements the permutation of {0, 1, ... , n-1}
	 */
	public void setSize(int size) {
		int numSmaller = 0;
		int numGreater = 0;
		for (int i = 0; i < n; i++) {
			int idx = indices[i];
			if (idx < size) {
				tmpIndices[numSmaller] = idx;
				numSmaller++;
			} else {
				tmpIndices[size + numGreater] = idx;
				numGreater++;
			}
		}
		int[] tmp = indices;
		indices = tmpIndices;
		tmpIndices = tmp;
		// validate();
	}

	/*
	 * public function validate():Void { var po = new Vector<Bool>(n); for (i in
	 * 0...n) { if (po[indices[i]]) { throw M.error("???")); } else { po[indices[i]]
	 * = true; } } for (i in 0...9) { if (!po[i]) { throw M.error("???")); } } }
	 */

}
