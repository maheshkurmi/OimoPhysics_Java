package oimo.common;

/**
 * The object pool system of `Vec3`, `Mat3`, `Mat4`, and `Quat`.
 * It keeps track of free objects which can be obtained by calling corresponding functions from Pool , 
 * objects no longer needed should be disposed by calling corresponding Pool.dispose function
 */
public class Pool {
	Vec3[] stackVec3;
	int sizeVec3;
	Mat3[] stackMat3;
	int sizeMat3;
	Mat4[] stackMat4;
	int sizeMat4;
	Quat[] stackQuat;
	int sizeQuat;

	/**
	 * Default constructor.
	 */
	public Pool() {
		stackVec3 = new Vec3[256];
		sizeVec3 = 0;
		stackMat3 = new Mat3[256];
		sizeMat3 = 0;
		stackMat4 = new Mat4[256];
		sizeMat4 = 0;
		stackQuat = new Quat[256];
		sizeQuat = 0;
	}

	/**
	 * Returns a `Vec3` object. If an unused object of `Vec3` is pooled, this does
	 * not create a new instance.
	 */
	public Vec3 vec3() {
		if ((this.sizeVec3 == 0)) {
			//there is no free/disposed object so create new one
			return new Vec3();
		} else {
			return stackVec3[--this.sizeVec3];
		}
	}

	/**
	 * Returns a `Mat3` object. If an unused object of `Mat3` is pooled, this does
	 * not create a new instance.
	 */
	public Mat3 mat3() {
		if ((this.sizeMat3 == 0)) {
			//there is no free/disposed object so create new one
			return new Mat3();
		} else {
			return stackMat3[--this.sizeMat3];
		}
	}


	/**
	 * Returns a `Mat4` object. If an unused object of `Vec3` is pooled, this does
	 * not create a new instance.
	 */
	public Mat4 Mat4() {
		if ((this.sizeMat4 == 0)) {
			//there is no free/disposed object so create new one
			return new Mat4();
		} else {
			//we have some disposed/free object,lets return them 
			return stackMat4[--this.sizeMat4];
		}
	}


	/**
	 * Returns a `Quat` object. If an unused object of `Quat` is pooled, this does
	 * not create a new instance.
	 */
	public Quat quat() {
		if ((this.sizeQuat == 0)) {
			//there is no free/disposed object so create new one
			return new Quat();
		} else {
			return stackQuat[--this.sizeQuat];
		}
	}
	

	/**
	 * Disposes an object got from `Pool.vec3`, `Pool.mat3`, `Pool.mat4`, or `Pool.quat`.
	 */
	public void dispose(Vec3 vec3, Mat3 mat3, Mat4 mat4, Quat quat) {
		if (vec3 != null) {
			disposeVec3(vec3);
		}
		if (mat3 != null) {
			disposeMat3(mat3);
		}
		if (mat4 != null) {
			disposeMat4(mat4);
		}
		if (quat != null) {
			disposeQuat(quat);
		}
	}

	/**
	 * Disposes an `Vec3` object got from `Pool.vec3`.
	 */
	public void disposeVec3(Vec3 v) {
		v.zero();
		if (this.sizeVec3 == this.stackVec3.length) {
			//increase size to double
			int newLength = (this.sizeVec3 << 1);
			Vec3[] newArray = new Vec3[newLength];
			int i=0;
			while (i++ < this.sizeVec3) {
				newArray[i] = this.stackVec3[i];
				this.stackVec3[i] = null;
			}
			this.stackVec3 = newArray;
		}
		//add object to free pool
		this.stackVec3[this.sizeVec3++] = v;
	}

	
	/**
	 * Disposes an `Mat3` object got from `Pool.mat3`.
	 */
	public void disposeMat3(Mat3 m) {
		m.identity();
		if (this.sizeMat3 == this.stackMat3.length) {
			//increase size to double
			int newLength = (this.sizeMat3 << 1);
			Mat3[] newArray = new Mat3[newLength];
			int i=0;
			while (i++ < this.sizeMat3) {
				newArray[i] = this.stackMat3[i];
				this.stackMat3[i] = null;
			}
			this.stackMat3 = newArray;
		}
		//add object to free pool
		this.stackMat3[this.sizeMat3++] = m;
	}

	/**
	 * Disposes an `Mat4` object got from `Pool.mat4`.
	 */
	public void disposeMat4(Mat4 m) {
		m.identity();
		if (this.sizeMat4 == this.stackMat4.length) {
			//increase size to double
			int newLength = (this.sizeMat4 << 1);
			Mat4[] newArray = new Mat4[newLength];
			int i=0;
			while (i++ < this.sizeMat4) {
				newArray[i] = this.stackMat4[i];
				this.stackMat4[i] = null;
			}
			this.stackMat4 = newArray;
		}
		//add object to free pool
		this.stackMat4[this.sizeMat4++] = m;
	}

	
	/**
	 * Disposes an `Quat` object got from `Pool.quat`.
	 */
	public void disposeQuat(Quat q) {
		q.identity();
		if (this.sizeQuat == this.stackQuat.length) {
			//increase size to double
			int newLength = (this.sizeQuat << 1);
			Quat[] newArray = new Quat[newLength];
			int i=0;
			while (i++ < this.sizeQuat) {
				newArray[i] = this.stackQuat[i];
				this.stackQuat[i] = null;
			}
			this.stackQuat = newArray;
		}
		//add object to free pool
		this.stackQuat[this.sizeMat4++] = q;
	}


}