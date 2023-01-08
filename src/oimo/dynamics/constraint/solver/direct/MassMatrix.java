package oimo.dynamics.constraint.solver.direct;

import java.util.Arrays;

import oimo.common.M;
import oimo.common.Mat3;
import oimo.dynamics.constraint.info.JacobianRow;
import oimo.dynamics.constraint.info.joint.JointSolverInfo;
import oimo.dynamics.constraint.solver.common.JointSolverMassDataRow;
import oimo.dynamics.rigidbody.RigidBody;

/**
 * Internal class
 */
public class MassMatrix {
	// matrix size
	public int _size;
	// inverse matrix elements
	public double[][] _invMass;
	public double[][] _invMassWithoutCfm;

	public JointSolverMassDataRow[] _massData;

	public double[][][] _cachedSubmatrices;
	public boolean[] _cacheComputed;
	public int _maxSubmatrixId;

	// temp matrix used for computing a inverse matrix
	double[][] tmpMatrix;

	public MassMatrix(int size) {
		_size = size;

		tmpMatrix = new double[size][size];// new Vector<Vector<Float>>(_size);
		_invMass = new double[size][size];// Vector<Vector<Float>>(_size);
		_invMassWithoutCfm = new double[size][size];// new Vector<Vector<Float>>(_size);
		for (int i = 0; i < _size; i++) {
			tmpMatrix[i] = new double[size];// Vector<Float>(_size);
			_invMass[i] = new double[size];// new Vector<Float>(_size);
			_invMassWithoutCfm[i] = new double[size];// new Vector<Float>(_size);
			for (int j = 0; j < _size; j++) {
				tmpMatrix[i][j] = 0;
				_invMass[i][j] = 0;
				_invMassWithoutCfm[i][j] = 0;
			}
		}
		_maxSubmatrixId = 1 << _size;
		_cacheComputed = new boolean[_maxSubmatrixId];// new Vector<Bool>(_maxSubmatrixId);
		_cachedSubmatrices = new double[_maxSubmatrixId][][];// Vector<Vector<Vector<Float>>>(_maxSubmatrixId);
		for (int i = 0; i < _maxSubmatrixId; i++) {
			// popcount (assuming the size of the matrix is less than 0x100 = 256)
			int t = i;
			t = (t & 0x55) + (t >> 1 & 0x55);
			t = (t & 0x33) + (t >> 2 & 0x33);
			t = (t & 0xf) + (t >> 4 & 0xf);
			int matrixSize = t;

			double[][] subMatrix = new double[matrixSize][];
			for (int j = 0; j < matrixSize; j++) {
				subMatrix[j] = new double[matrixSize];
				for (int k = 0; k < matrixSize; k++) {
					subMatrix[j][k] = 0;
				}
			}
			_cacheComputed[i] = false;
			_cachedSubmatrices[i] = subMatrix;
		}
	}

	// --- private ---

	protected void computeSubmatrix(int id, int[] indices, int size) {
		// compute the inverse matrix of the submatrix of the inverse mass matrix
		//
		// | invData[i[0]][i[0]] ... invData[i[0]][i[n-1]] |
		// inv( | ... ... ... | )
		// | invData[i[n-1]][i[0]] ... invData[i[n-1]][i[n-1]] |
		for (int i = 0; i < size; i++) {
			int ii = indices[i];
			for (int j = 0; j < size; j++) {
				tmpMatrix[i][j] = _invMass[ii][indices[j]];
			}
		}

		double[][] src = tmpMatrix;
		double[][] dst = _cachedSubmatrices[id];
		double[] srci;
		double[] dsti;
		double[] srcj;
		double[] dstj;
		double diag;

		switch (size) {
		case 4:
			srci = src[0];
			dsti = dst[0];
			diag = 1 / srci[0];
			dsti[0] = diag;
			srci[1] = srci[1] * diag;
			srci[2] = srci[2] * diag;
			srci[3] = srci[3] * diag;
			srcj = src[1];
			dstj = dst[1];
			dstj[0] = -diag * srcj[0];
			srcj[1] = srcj[1] - srci[1] * srcj[0];
			srcj[2] = srcj[2] - srci[2] * srcj[0];
			srcj[3] = srcj[3] - srci[3] * srcj[0];
			srcj = src[2];
			dstj = dst[2];
			dstj[0] = -diag * srcj[0];
			srcj[1] = srcj[1] - srci[1] * srcj[0];
			srcj[2] = srcj[2] - srci[2] * srcj[0];
			srcj[3] = srcj[3] - srci[3] * srcj[0];
			srcj = src[3];
			dstj = dst[3];
			dstj[0] = -diag * srcj[0];
			srcj[1] = srcj[1] - srci[1] * srcj[0];
			srcj[2] = srcj[2] - srci[2] * srcj[0];
			srcj[3] = srcj[3] - srci[3] * srcj[0];
			srci = src[1];
			dsti = dst[1];
			diag = 1 / srci[1];
			dsti[1] = diag;
			dsti[0] = dsti[0] * diag;
			srci[2] = srci[2] * diag;
			srci[3] = srci[3] * diag;
			srcj = src[0];
			dstj = dst[0];
			dstj[0] = dstj[0] - dsti[0] * srcj[1];
			srcj[2] = srcj[2] - srci[2] * srcj[1];
			srcj[3] = srcj[3] - srci[3] * srcj[1];
			srcj = src[2];
			dstj = dst[2];
			dstj[0] = dstj[0] - dsti[0] * srcj[1];
			dstj[1] = -diag * srcj[1];
			srcj[2] = srcj[2] - srci[2] * srcj[1];
			srcj[3] = srcj[3] - srci[3] * srcj[1];
			srcj = src[3];
			dstj = dst[3];
			dstj[0] = dstj[0] - dsti[0] * srcj[1];
			dstj[1] = -diag * srcj[1];
			srcj[2] = srcj[2] - srci[2] * srcj[1];
			srcj[3] = srcj[3] - srci[3] * srcj[1];
			srci = src[2];
			dsti = dst[2];
			diag = 1 / srci[2];
			dsti[2] = diag;
			dsti[0] = dsti[0] * diag;
			dsti[1] = dsti[1] * diag;
			srci[3] = srci[3] * diag;
			srcj = src[0];
			dstj = dst[0];
			dstj[0] = dstj[0] - dsti[0] * srcj[2];
			srcj[3] = srcj[3] - srci[3] * srcj[2];
			srcj = src[1];
			dstj = dst[1];
			dstj[0] = dstj[0] - dsti[0] * srcj[2];
			dstj[1] = dstj[1] - dsti[1] * srcj[2];
			srcj[3] = srcj[3] - srci[3] * srcj[2];
			srcj = src[3];
			dstj = dst[3];
			dstj[0] = dstj[0] - dsti[0] * srcj[2];
			dstj[1] = dstj[1] - dsti[1] * srcj[2];
			dstj[2] = -diag * srcj[2];
			srcj[3] = srcj[3] - srci[3] * srcj[2];
			srci = src[3];
			dsti = dst[3];
			diag = 1 / srci[3];
			dsti[3] = diag;
			dsti[0] = dsti[0] * diag;
			dsti[1] = dsti[1] * diag;
			dsti[2] = dsti[2] * diag;
			srcj = src[0];
			dstj = dst[0];
			dstj[0] = dstj[0] - dsti[0] * srcj[3];
			srcj = src[1];
			dstj = dst[1];
			dstj[0] = dstj[0] - dsti[0] * srcj[3];
			dstj[1] = dstj[1] - dsti[1] * srcj[3];
			srcj = src[2];
			dstj = dst[2];
			dstj[0] = dstj[0] - dsti[0] * srcj[3];
			dstj[1] = dstj[1] - dsti[1] * srcj[3];
			dstj[2] = dstj[2] - dsti[2] * srcj[3];
			dsti = dst[1];
			dst[0][1] = dsti[0];
			dsti = dst[2];
			dst[0][2] = dsti[0];
			dst[1][2] = dsti[1];
			dsti = dst[3];
			dst[0][3] = dsti[0];
			dst[1][3] = dsti[1];
			dst[2][3] = dsti[2];
			break;
		case 5:
			srci = src[0];
			dsti = dst[0];
			diag = 1 / srci[0];
			dsti[0] = diag;
			srci[1] = srci[1] * diag;
			srci[2] = srci[2] * diag;
			srci[3] = srci[3] * diag;
			srci[4] = srci[4] * diag;
			srcj = src[1];
			dstj = dst[1];
			dstj[0] = -diag * srcj[0];
			srcj[1] = srcj[1] - srci[1] * srcj[0];
			srcj[2] = srcj[2] - srci[2] * srcj[0];
			srcj[3] = srcj[3] - srci[3] * srcj[0];
			srcj[4] = srcj[4] - srci[4] * srcj[0];
			srcj = src[2];
			dstj = dst[2];
			dstj[0] = -diag * srcj[0];
			srcj[1] = srcj[1] - srci[1] * srcj[0];
			srcj[2] = srcj[2] - srci[2] * srcj[0];
			srcj[3] = srcj[3] - srci[3] * srcj[0];
			srcj[4] = srcj[4] - srci[4] * srcj[0];
			srcj = src[3];
			dstj = dst[3];
			dstj[0] = -diag * srcj[0];
			srcj[1] = srcj[1] - srci[1] * srcj[0];
			srcj[2] = srcj[2] - srci[2] * srcj[0];
			srcj[3] = srcj[3] - srci[3] * srcj[0];
			srcj[4] = srcj[4] - srci[4] * srcj[0];
			srcj = src[4];
			dstj = dst[4];
			dstj[0] = -diag * srcj[0];
			srcj[1] = srcj[1] - srci[1] * srcj[0];
			srcj[2] = srcj[2] - srci[2] * srcj[0];
			srcj[3] = srcj[3] - srci[3] * srcj[0];
			srcj[4] = srcj[4] - srci[4] * srcj[0];
			srci = src[1];
			dsti = dst[1];
			diag = 1 / srci[1];
			dsti[1] = diag;
			dsti[0] = dsti[0] * diag;
			srci[2] = srci[2] * diag;
			srci[3] = srci[3] * diag;
			srci[4] = srci[4] * diag;
			srcj = src[0];
			dstj = dst[0];
			dstj[0] = dstj[0] - dsti[0] * srcj[1];
			srcj[2] = srcj[2] - srci[2] * srcj[1];
			srcj[3] = srcj[3] - srci[3] * srcj[1];
			srcj[4] = srcj[4] - srci[4] * srcj[1];
			srcj = src[2];
			dstj = dst[2];
			dstj[0] = dstj[0] - dsti[0] * srcj[1];
			dstj[1] = -diag * srcj[1];
			srcj[2] = srcj[2] - srci[2] * srcj[1];
			srcj[3] = srcj[3] - srci[3] * srcj[1];
			srcj[4] = srcj[4] - srci[4] * srcj[1];
			srcj = src[3];
			dstj = dst[3];
			dstj[0] = dstj[0] - dsti[0] * srcj[1];
			dstj[1] = -diag * srcj[1];
			srcj[2] = srcj[2] - srci[2] * srcj[1];
			srcj[3] = srcj[3] - srci[3] * srcj[1];
			srcj[4] = srcj[4] - srci[4] * srcj[1];
			srcj = src[4];
			dstj = dst[4];
			dstj[0] = dstj[0] - dsti[0] * srcj[1];
			dstj[1] = -diag * srcj[1];
			srcj[2] = srcj[2] - srci[2] * srcj[1];
			srcj[3] = srcj[3] - srci[3] * srcj[1];
			srcj[4] = srcj[4] - srci[4] * srcj[1];
			srci = src[2];
			dsti = dst[2];
			diag = 1 / srci[2];
			dsti[2] = diag;
			dsti[0] = dsti[0] * diag;
			dsti[1] = dsti[1] * diag;
			srci[3] = srci[3] * diag;
			srci[4] = srci[4] * diag;
			srcj = src[0];
			dstj = dst[0];
			dstj[0] = dstj[0] - dsti[0] * srcj[2];
			srcj[3] = srcj[3] - srci[3] * srcj[2];
			srcj[4] = srcj[4] - srci[4] * srcj[2];
			srcj = src[1];
			dstj = dst[1];
			dstj[0] = dstj[0] - dsti[0] * srcj[2];
			dstj[1] = dstj[1] - dsti[1] * srcj[2];
			srcj[3] = srcj[3] - srci[3] * srcj[2];
			srcj[4] = srcj[4] - srci[4] * srcj[2];
			srcj = src[3];
			dstj = dst[3];
			dstj[0] = dstj[0] - dsti[0] * srcj[2];
			dstj[1] = dstj[1] - dsti[1] * srcj[2];
			dstj[2] = -diag * srcj[2];
			srcj[3] = srcj[3] - srci[3] * srcj[2];
			srcj[4] = srcj[4] - srci[4] * srcj[2];
			srcj = src[4];
			dstj = dst[4];
			dstj[0] = dstj[0] - dsti[0] * srcj[2];
			dstj[1] = dstj[1] - dsti[1] * srcj[2];
			dstj[2] = -diag * srcj[2];
			srcj[3] = srcj[3] - srci[3] * srcj[2];
			srcj[4] = srcj[4] - srci[4] * srcj[2];
			srci = src[3];
			dsti = dst[3];
			diag = 1 / srci[3];
			dsti[3] = diag;
			dsti[0] = dsti[0] * diag;
			dsti[1] = dsti[1] * diag;
			dsti[2] = dsti[2] * diag;
			srci[4] = srci[4] * diag;
			srcj = src[0];
			dstj = dst[0];
			dstj[0] = dstj[0] - dsti[0] * srcj[3];
			srcj[4] = srcj[4] - srci[4] * srcj[3];
			srcj = src[1];
			dstj = dst[1];
			dstj[0] = dstj[0] - dsti[0] * srcj[3];
			dstj[1] = dstj[1] - dsti[1] * srcj[3];
			srcj[4] = srcj[4] - srci[4] * srcj[3];
			srcj = src[2];
			dstj = dst[2];
			dstj[0] = dstj[0] - dsti[0] * srcj[3];
			dstj[1] = dstj[1] - dsti[1] * srcj[3];
			dstj[2] = dstj[2] - dsti[2] * srcj[3];
			srcj[4] = srcj[4] - srci[4] * srcj[3];
			srcj = src[4];
			dstj = dst[4];
			dstj[0] = dstj[0] - dsti[0] * srcj[3];
			dstj[1] = dstj[1] - dsti[1] * srcj[3];
			dstj[2] = dstj[2] - dsti[2] * srcj[3];
			dstj[3] = -diag * srcj[3];
			srcj[4] = srcj[4] - srci[4] * srcj[3];
			srci = src[4];
			dsti = dst[4];
			diag = 1 / srci[4];
			dsti[4] = diag;
			dsti[0] = dsti[0] * diag;
			dsti[1] = dsti[1] * diag;
			dsti[2] = dsti[2] * diag;
			dsti[3] = dsti[3] * diag;
			srcj = src[0];
			dstj = dst[0];
			dstj[0] = dstj[0] - dsti[0] * srcj[4];
			srcj = src[1];
			dstj = dst[1];
			dstj[0] = dstj[0] - dsti[0] * srcj[4];
			dstj[1] = dstj[1] - dsti[1] * srcj[4];
			srcj = src[2];
			dstj = dst[2];
			dstj[0] = dstj[0] - dsti[0] * srcj[4];
			dstj[1] = dstj[1] - dsti[1] * srcj[4];
			dstj[2] = dstj[2] - dsti[2] * srcj[4];
			srcj = src[3];
			dstj = dst[3];
			dstj[0] = dstj[0] - dsti[0] * srcj[4];
			dstj[1] = dstj[1] - dsti[1] * srcj[4];
			dstj[2] = dstj[2] - dsti[2] * srcj[4];
			dstj[3] = dstj[3] - dsti[3] * srcj[4];
			dsti = dst[1];
			dst[0][1] = dsti[0];
			dsti = dst[2];
			dst[0][2] = dsti[0];
			dst[1][2] = dsti[1];
			dsti = dst[3];
			dst[0][3] = dsti[0];
			dst[1][3] = dsti[1];
			dst[2][3] = dsti[2];
			dsti = dst[4];
			dst[0][4] = dsti[0];
			dst[1][4] = dsti[1];
			dst[2][4] = dsti[2];
			dst[3][4] = dsti[3];
			break;
		case 6:
			srci = src[0];
			dsti = dst[0];
			diag = 1 / srci[0];
			dsti[0] = diag;
			srci[1] = srci[1] * diag;
			srci[2] = srci[2] * diag;
			srci[3] = srci[3] * diag;
			srci[4] = srci[4] * diag;
			srci[5] = srci[5] * diag;
			srcj = src[1];
			dstj = dst[1];
			dstj[0] = -diag * srcj[0];
			srcj[1] = srcj[1] - srci[1] * srcj[0];
			srcj[2] = srcj[2] - srci[2] * srcj[0];
			srcj[3] = srcj[3] - srci[3] * srcj[0];
			srcj[4] = srcj[4] - srci[4] * srcj[0];
			srcj[5] = srcj[5] - srci[5] * srcj[0];
			srcj = src[2];
			dstj = dst[2];
			dstj[0] = -diag * srcj[0];
			srcj[1] = srcj[1] - srci[1] * srcj[0];
			srcj[2] = srcj[2] - srci[2] * srcj[0];
			srcj[3] = srcj[3] - srci[3] * srcj[0];
			srcj[4] = srcj[4] - srci[4] * srcj[0];
			srcj[5] = srcj[5] - srci[5] * srcj[0];
			srcj = src[3];
			dstj = dst[3];
			dstj[0] = -diag * srcj[0];
			srcj[1] = srcj[1] - srci[1] * srcj[0];
			srcj[2] = srcj[2] - srci[2] * srcj[0];
			srcj[3] = srcj[3] - srci[3] * srcj[0];
			srcj[4] = srcj[4] - srci[4] * srcj[0];
			srcj[5] = srcj[5] - srci[5] * srcj[0];
			srcj = src[4];
			dstj = dst[4];
			dstj[0] = -diag * srcj[0];
			srcj[1] = srcj[1] - srci[1] * srcj[0];
			srcj[2] = srcj[2] - srci[2] * srcj[0];
			srcj[3] = srcj[3] - srci[3] * srcj[0];
			srcj[4] = srcj[4] - srci[4] * srcj[0];
			srcj[5] = srcj[5] - srci[5] * srcj[0];
			srcj = src[5];
			dstj = dst[5];
			dstj[0] = -diag * srcj[0];
			srcj[1] = srcj[1] - srci[1] * srcj[0];
			srcj[2] = srcj[2] - srci[2] * srcj[0];
			srcj[3] = srcj[3] - srci[3] * srcj[0];
			srcj[4] = srcj[4] - srci[4] * srcj[0];
			srcj[5] = srcj[5] - srci[5] * srcj[0];
			srci = src[1];
			dsti = dst[1];
			diag = 1 / srci[1];
			dsti[1] = diag;
			dsti[0] = dsti[0] * diag;
			srci[2] = srci[2] * diag;
			srci[3] = srci[3] * diag;
			srci[4] = srci[4] * diag;
			srci[5] = srci[5] * diag;
			srcj = src[0];
			dstj = dst[0];
			dstj[0] = dstj[0] - dsti[0] * srcj[1];
			srcj[2] = srcj[2] - srci[2] * srcj[1];
			srcj[3] = srcj[3] - srci[3] * srcj[1];
			srcj[4] = srcj[4] - srci[4] * srcj[1];
			srcj[5] = srcj[5] - srci[5] * srcj[1];
			srcj = src[2];
			dstj = dst[2];
			dstj[0] = dstj[0] - dsti[0] * srcj[1];
			dstj[1] = -diag * srcj[1];
			srcj[2] = srcj[2] - srci[2] * srcj[1];
			srcj[3] = srcj[3] - srci[3] * srcj[1];
			srcj[4] = srcj[4] - srci[4] * srcj[1];
			srcj[5] = srcj[5] - srci[5] * srcj[1];
			srcj = src[3];
			dstj = dst[3];
			dstj[0] = dstj[0] - dsti[0] * srcj[1];
			dstj[1] = -diag * srcj[1];
			srcj[2] = srcj[2] - srci[2] * srcj[1];
			srcj[3] = srcj[3] - srci[3] * srcj[1];
			srcj[4] = srcj[4] - srci[4] * srcj[1];
			srcj[5] = srcj[5] - srci[5] * srcj[1];
			srcj = src[4];
			dstj = dst[4];
			dstj[0] = dstj[0] - dsti[0] * srcj[1];
			dstj[1] = -diag * srcj[1];
			srcj[2] = srcj[2] - srci[2] * srcj[1];
			srcj[3] = srcj[3] - srci[3] * srcj[1];
			srcj[4] = srcj[4] - srci[4] * srcj[1];
			srcj[5] = srcj[5] - srci[5] * srcj[1];
			srcj = src[5];
			dstj = dst[5];
			dstj[0] = dstj[0] - dsti[0] * srcj[1];
			dstj[1] = -diag * srcj[1];
			srcj[2] = srcj[2] - srci[2] * srcj[1];
			srcj[3] = srcj[3] - srci[3] * srcj[1];
			srcj[4] = srcj[4] - srci[4] * srcj[1];
			srcj[5] = srcj[5] - srci[5] * srcj[1];
			srci = src[2];
			dsti = dst[2];
			diag = 1 / srci[2];
			dsti[2] = diag;
			dsti[0] = dsti[0] * diag;
			dsti[1] = dsti[1] * diag;
			srci[3] = srci[3] * diag;
			srci[4] = srci[4] * diag;
			srci[5] = srci[5] * diag;
			srcj = src[0];
			dstj = dst[0];
			dstj[0] = dstj[0] - dsti[0] * srcj[2];
			srcj[3] = srcj[3] - srci[3] * srcj[2];
			srcj[4] = srcj[4] - srci[4] * srcj[2];
			srcj[5] = srcj[5] - srci[5] * srcj[2];
			srcj = src[1];
			dstj = dst[1];
			dstj[0] = dstj[0] - dsti[0] * srcj[2];
			dstj[1] = dstj[1] - dsti[1] * srcj[2];
			srcj[3] = srcj[3] - srci[3] * srcj[2];
			srcj[4] = srcj[4] - srci[4] * srcj[2];
			srcj[5] = srcj[5] - srci[5] * srcj[2];
			srcj = src[3];
			dstj = dst[3];
			dstj[0] = dstj[0] - dsti[0] * srcj[2];
			dstj[1] = dstj[1] - dsti[1] * srcj[2];
			dstj[2] = -diag * srcj[2];
			srcj[3] = srcj[3] - srci[3] * srcj[2];
			srcj[4] = srcj[4] - srci[4] * srcj[2];
			srcj[5] = srcj[5] - srci[5] * srcj[2];
			srcj = src[4];
			dstj = dst[4];
			dstj[0] = dstj[0] - dsti[0] * srcj[2];
			dstj[1] = dstj[1] - dsti[1] * srcj[2];
			dstj[2] = -diag * srcj[2];
			srcj[3] = srcj[3] - srci[3] * srcj[2];
			srcj[4] = srcj[4] - srci[4] * srcj[2];
			srcj[5] = srcj[5] - srci[5] * srcj[2];
			srcj = src[5];
			dstj = dst[5];
			dstj[0] = dstj[0] - dsti[0] * srcj[2];
			dstj[1] = dstj[1] - dsti[1] * srcj[2];
			dstj[2] = -diag * srcj[2];
			srcj[3] = srcj[3] - srci[3] * srcj[2];
			srcj[4] = srcj[4] - srci[4] * srcj[2];
			srcj[5] = srcj[5] - srci[5] * srcj[2];
			srci = src[3];
			dsti = dst[3];
			diag = 1 / srci[3];
			dsti[3] = diag;
			dsti[0] = dsti[0] * diag;
			dsti[1] = dsti[1] * diag;
			dsti[2] = dsti[2] * diag;
			srci[4] = srci[4] * diag;
			srci[5] = srci[5] * diag;
			srcj = src[0];
			dstj = dst[0];
			dstj[0] = dstj[0] - dsti[0] * srcj[3];
			srcj[4] = srcj[4] - srci[4] * srcj[3];
			srcj[5] = srcj[5] - srci[5] * srcj[3];
			srcj = src[1];
			dstj = dst[1];
			dstj[0] = dstj[0] - dsti[0] * srcj[3];
			dstj[1] = dstj[1] - dsti[1] * srcj[3];
			srcj[4] = srcj[4] - srci[4] * srcj[3];
			srcj[5] = srcj[5] - srci[5] * srcj[3];
			srcj = src[2];
			dstj = dst[2];
			dstj[0] = dstj[0] - dsti[0] * srcj[3];
			dstj[1] = dstj[1] - dsti[1] * srcj[3];
			dstj[2] = dstj[2] - dsti[2] * srcj[3];
			srcj[4] = srcj[4] - srci[4] * srcj[3];
			srcj[5] = srcj[5] - srci[5] * srcj[3];
			srcj = src[4];
			dstj = dst[4];
			dstj[0] = dstj[0] - dsti[0] * srcj[3];
			dstj[1] = dstj[1] - dsti[1] * srcj[3];
			dstj[2] = dstj[2] - dsti[2] * srcj[3];
			dstj[3] = -diag * srcj[3];
			srcj[4] = srcj[4] - srci[4] * srcj[3];
			srcj[5] = srcj[5] - srci[5] * srcj[3];
			srcj = src[5];
			dstj = dst[5];
			dstj[0] = dstj[0] - dsti[0] * srcj[3];
			dstj[1] = dstj[1] - dsti[1] * srcj[3];
			dstj[2] = dstj[2] - dsti[2] * srcj[3];
			dstj[3] = -diag * srcj[3];
			srcj[4] = srcj[4] - srci[4] * srcj[3];
			srcj[5] = srcj[5] - srci[5] * srcj[3];
			srci = src[4];
			dsti = dst[4];
			diag = 1 / srci[4];
			dsti[4] = diag;
			dsti[0] = dsti[0] * diag;
			dsti[1] = dsti[1] * diag;
			dsti[2] = dsti[2] * diag;
			dsti[3] = dsti[3] * diag;
			srci[5] = srci[5] * diag;
			srcj = src[0];
			dstj = dst[0];
			dstj[0] = dstj[0] - dsti[0] * srcj[4];
			srcj[5] = srcj[5] - srci[5] * srcj[4];
			srcj = src[1];
			dstj = dst[1];
			dstj[0] = dstj[0] - dsti[0] * srcj[4];
			dstj[1] = dstj[1] - dsti[1] * srcj[4];
			srcj[5] = srcj[5] - srci[5] * srcj[4];
			srcj = src[2];
			dstj = dst[2];
			dstj[0] = dstj[0] - dsti[0] * srcj[4];
			dstj[1] = dstj[1] - dsti[1] * srcj[4];
			dstj[2] = dstj[2] - dsti[2] * srcj[4];
			srcj[5] = srcj[5] - srci[5] * srcj[4];
			srcj = src[3];
			dstj = dst[3];
			dstj[0] = dstj[0] - dsti[0] * srcj[4];
			dstj[1] = dstj[1] - dsti[1] * srcj[4];
			dstj[2] = dstj[2] - dsti[2] * srcj[4];
			dstj[3] = dstj[3] - dsti[3] * srcj[4];
			srcj[5] = srcj[5] - srci[5] * srcj[4];
			srcj = src[5];
			dstj = dst[5];
			dstj[0] = dstj[0] - dsti[0] * srcj[4];
			dstj[1] = dstj[1] - dsti[1] * srcj[4];
			dstj[2] = dstj[2] - dsti[2] * srcj[4];
			dstj[3] = dstj[3] - dsti[3] * srcj[4];
			dstj[4] = -diag * srcj[4];
			srcj[5] = srcj[5] - srci[5] * srcj[4];
			srci = src[5];
			dsti = dst[5];
			diag = 1 / srci[5];
			dsti[5] = diag;
			dsti[0] = dsti[0] * diag;
			dsti[1] = dsti[1] * diag;
			dsti[2] = dsti[2] * diag;
			dsti[3] = dsti[3] * diag;
			dsti[4] = dsti[4] * diag;
			srcj = src[0];
			dstj = dst[0];
			dstj[0] = dstj[0] - dsti[0] * srcj[5];
			srcj = src[1];
			dstj = dst[1];
			dstj[0] = dstj[0] - dsti[0] * srcj[5];
			dstj[1] = dstj[1] - dsti[1] * srcj[5];
			srcj = src[2];
			dstj = dst[2];
			dstj[0] = dstj[0] - dsti[0] * srcj[5];
			dstj[1] = dstj[1] - dsti[1] * srcj[5];
			dstj[2] = dstj[2] - dsti[2] * srcj[5];
			srcj = src[3];
			dstj = dst[3];
			dstj[0] = dstj[0] - dsti[0] * srcj[5];
			dstj[1] = dstj[1] - dsti[1] * srcj[5];
			dstj[2] = dstj[2] - dsti[2] * srcj[5];
			dstj[3] = dstj[3] - dsti[3] * srcj[5];
			srcj = src[4];
			dstj = dst[4];
			dstj[0] = dstj[0] - dsti[0] * srcj[5];
			dstj[1] = dstj[1] - dsti[1] * srcj[5];
			dstj[2] = dstj[2] - dsti[2] * srcj[5];
			dstj[3] = dstj[3] - dsti[3] * srcj[5];
			dstj[4] = dstj[4] - dsti[4] * srcj[5];
			dsti = dst[1];
			dst[0][1] = dsti[0];
			dsti = dst[2];
			dst[0][2] = dsti[0];
			dst[1][2] = dsti[1];
			dsti = dst[3];
			dst[0][3] = dsti[0];
			dst[1][3] = dsti[1];
			dst[2][3] = dsti[2];
			dsti = dst[4];
			dst[0][4] = dsti[0];
			dst[1][4] = dsti[1];
			dst[2][4] = dsti[2];
			dst[3][4] = dsti[3];
			dsti = dst[5];
			dst[0][5] = dsti[0];
			dst[1][5] = dsti[1];
			dst[2][5] = dsti[2];
			dst[3][5] = dsti[3];
			dst[4][5] = dsti[4];
			break;
		default:
			for (int i = 0; i < size; i++) {
				srci = src[i];
				dsti = dst[i];
				diag = 1 / srci[i];
				dsti[i] = diag;
				for (int j = 0; j < i; j++) {
					dsti[j] = dsti[j] * diag;
				}
				for (int j = i + 1; j < size; j++) {// j in i + 1...size) {
					srci[j] = srci[j] * diag;
				}
				for (int j = 0; j < i; j++) { // j in 0...i) {
					srcj = src[j];
					dstj = dst[j];
					for (int k = 0; k < j + 1; k++) {// k in 0...j + 1) {
						dstj[k] = dstj[k] - dsti[k] * srcj[i];
					}
					for (int k = i + 1; k < size; k++) {// k in i + 1...size) {
						srcj[k] = srcj[k] - srci[k] * srcj[i];
					}
				}
				for (int j = i + 1; j < size; j++) {// j in i + 1...size) {
					srcj = src[j];
					dstj = dst[j];
					for (int k = 0; k < i; k++) {// k in 0...i) {
						dstj[k] = dstj[k] - dsti[k] * srcj[i];
					}
					dstj[i] = -diag * srcj[i];
					for (int k = i + 1; k < size; k++) {// k in i + 1...size) {
						srcj[k] = srcj[k] - srci[k] * srcj[i];
					}
				}
			}
			for (int i = 1; i < size; i++) {// i in 1...size) {
				dsti = dst[i];
				for (int j = 0; j < i; j++) {//// j in 0...i) {
					dst[j][i] = dsti[j];
				}
			}
		}
	}

	// --- internal ---

	public void computeInvMass(JointSolverInfo info, JointSolverMassDataRow[] massData) {
		double[][] invMass = this._invMass;
		double[][] invMassWithoutCfm = this._invMassWithoutCfm;

		int numRows = info.numRows;

		RigidBody b1 = info.b1;
		RigidBody b2 = info.b2;

		double invM1;
		double invM2;
		invM1 = b1._invMass;
		invM2 = b2._invMass;
		Mat3 invI1 = b1._invInertia;
		Mat3 invI2 = b2._invInertia;

		// M.mat3_assign(invI1, b1._invInertia);
		// M.mat3_assign(invI2, b2._invInertia);

		// compute invM*trpJ
		//
		// | 1/m 0 0 0 0 0 | | row1 ... rowN |
		// | 0 1/m 0 0 0 0 |
		// | 0 0 1/m 0 0 0 |
		// | 0 0 0 invI00 invI01 invI02 |
		// | 0 0 0 invI10 invI11 invI12 |
		// | 0 0 0 invI20 invI21 invI22 |
		for (int i = 0; i < numRows; i++) {
			JacobianRow j = info.rows[i].jacobian;
			JointSolverMassDataRow md = massData[i];
			j.updateSparsity();

			if (j.isLinearSet()) {
				M.vec3_scale(md.invMLin1, j.lin1, invM1);
				M.vec3_scale(md.invMLin2, j.lin2, invM2);
			} else {
				M.vec3_zero(md.invMLin1);
				M.vec3_zero(md.invMLin2);
			}
			if (j.isAngularSet()) {
				M.vec3_mulMat3(md.invMAng1, j.ang1, invI1);
				M.vec3_mulMat3(md.invMAng2, j.ang2, invI2);
			} else {
				M.vec3_zero(md.invMAng1);
				M.vec3_zero(md.invMAng2);
			}
		}

		// compute J*(invM*trpJ)
		//
		// | row1^T | | 1/m 0 0 0 0 0 | | row1 ... rowN |
		// | ... | | 0 1/m 0 0 0 0 |
		// | rowN^T | | 0 0 1/m 0 0 0 |
		// | 0 0 0 invI00 invI01 invI02 |
		// | 0 0 0 invI10 invI11 invI12 |
		// | 0 0 0 invI20 invI21 invI22 |
		for (int i = 0; i < numRows; i++) {
			// rows of LHS matrix
			JacobianRow j1 = info.rows[i].jacobian;

			for (int j = 0; j < numRows; j++) { // compute only elements in upper triangle
				// cols of RHS matrix
				JacobianRow j2 = info.rows[j].jacobian;
				JointSolverMassDataRow md2 = massData[j];

				double val = M.vec3_dot(j1.lin1, md2.invMLin1) + M.vec3_dot(j1.ang1, md2.invMAng1)
						+ M.vec3_dot(j1.lin2, md2.invMLin2) + M.vec3_dot(j1.ang2, md2.invMAng2);
				if (i == j) {
					invMass[i][j] = val + info.rows[i].cfm;
					invMassWithoutCfm[i][j] = val;

					md2.mass = val + info.rows[i].cfm;
					md2.massWithoutCfm = val;

					if (md2.mass != 0)
						md2.mass = 1 / md2.mass;
					if (md2.massWithoutCfm != 0)
						md2.massWithoutCfm = 1 / md2.massWithoutCfm;
				} else {
					invMass[i][j] = val;
					invMass[j][i] = val;
					invMassWithoutCfm[i][j] = val;
					invMassWithoutCfm[j][i] = val;
				}
			}
		}

		// clear cached submatrices
		clearCache();
	}

	public double[][] getSubmatrix(int[] indices, int n) {
		int id = 0;
		for (int i = 0; i < n; i++) {
			id |= 1 << indices[i];
		}
		if (_cacheComputed[id]) {
			// the submatrix has been computed after the last change of the inverse mass
			// matrix
			return _cachedSubmatrices[id];
		}
		// there's no cache, compute the submatrix
		computeSubmatrix(id, indices, n);
		_cacheComputed[id] = true;
		return _cachedSubmatrices[id];
	}

	public void clearCache() {
		for (int i = 0; i < _maxSubmatrixId; i++) {
			_cacheComputed[i] = false;
		}
	}

	public void dump() {
		System.out.println("inverse mass matrix:");
		dumpMatrix(_invMass);
		for (int i = 0; i < _maxSubmatrixId; i++) {
			System.out.println("mass submatrix of id " + i + ":");
			int[] indices = new int[_cachedSubmatrices[i].length];
			int cnt = 0;
			for (int j = 0; j < _size; j++) {
				if ((i & 1 << j) != 0) {
					indices[cnt++] = j;
				}
			}
			System.out.println("(indices: " + indices + ")");
			double[][] mat = getSubmatrix(indices, indices.length);
			dumpMatrix(mat);
		}
	}

	public void dumpMatrix(double[][] data) {
		if (data.length == 0) {
			System.out.println("| |");
			return;
		}
		for (int i = 0; i < data.length; i++) {
			System.out.println(Arrays.toString(data[i]));
		}
		/*
		 * String[][] strs:Array<Array<String>> = []; int row = data.length; int col =
		 * data[0].length; for (int i=0;i< row;i++) { strs.push([]); for (j in 0...col)
		 * { strs[i].push("" + data[i][j]); } }
		 * 
		 * for (j in 0...col) { var maxChar = 0; for (i in 0...row) { maxChar =
		 * strs[i][j].length > maxChar ? strs[i][j].length : maxChar; } for (i in
		 * 0...row) { while (strs[i][j].length < maxChar) strs[i][j] = " " + strs[i][j];
		 * } } for (i in 0...row) { var str = "| "; for (j in 0...col) { str += (j == 0
		 * ? "" : ", ") + strs[i][j]; } str += " |"; trace(str); }
		 */
	}

}
