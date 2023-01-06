package demo.gl2renderer;

import java.nio.IntBuffer;

import com.jogamp.common.nio.Buffers;
import com.jogamp.opengl.GL2;

class OIndexBuffer {
	GL2 gl;
	int vboId;
	int count;
	static final int INT_SIZE_BITS=4;
	public OIndexBuffer(GL2 gl) {
		this.gl = gl;
		int[] bufferID = new int[1]; // Get buffer IDs for the data.
		gl.glGenBuffers(1,bufferID,0);
		vboId = bufferID[0];
	}

	/**
	 * @param
	 * @param staticDraw usage GL.STATIC_DRAW|GL.DYNAMIC_DRAW
	 * @return
	 */
	public void setData(int[] array, boolean staticDraw) {
		gl.glBindBuffer(GL2.GL_ELEMENT_ARRAY_BUFFER, vboId);
		gl.glBufferData(GL2.GL_ELEMENT_ARRAY_BUFFER, array.length*INT_SIZE_BITS,Buffers.newDirectIntBuffer(array), staticDraw?GL2.GL_STATIC_DRAW:GL2.GL_DYNAMIC_DRAW);
		gl.glBindBuffer(GL2.GL_ELEMENT_ARRAY_BUFFER, 0);
		count = array.length;
	}

	public void updateData(int[] array, boolean staticDraw) {
		gl.glBindBuffer(GL2.GL_ELEMENT_ARRAY_BUFFER, vboId);
		gl.glBufferSubData(GL2.GL_ELEMENT_ARRAY_BUFFER, 0,array.length*INT_SIZE_BITS, Buffers.newDirectIntBuffer(array));
		gl.glBindBuffer(GL2.GL_ELEMENT_ARRAY_BUFFER, 0);
	}

	public void updateDataInt32Array(IntBuffer array) {
		gl.glBindBuffer(GL2.GL_ELEMENT_ARRAY_BUFFER, vboId);
		gl.glBufferSubData(GL2.GL_ELEMENT_ARRAY_BUFFER, 0, array.capacity()*INT_SIZE_BITS,array);
		gl.glBindBuffer(GL2.GL_ELEMENT_ARRAY_BUFFER, 0);
	}

	/**
	 * 
	 * @param drawMode GL2.GL_TRIANGLES|GL2.GL_LINES|GL2.GL_POINTS
	 * @param count pass -1 to draw all points
	 */
	public void draw(int drawMode, int count) {
		gl.glBindBuffer(GL2.GL_ELEMENT_ARRAY_BUFFER, vboId);
		gl.glDrawElements(drawMode, count >= 0 ? count : this.count, GL2.GL_UNSIGNED_INT, 0);
	}

}
