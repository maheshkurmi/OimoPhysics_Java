package demo.gl2renderer;

import java.nio.FloatBuffer;

import com.jogamp.opengl.GL2;
import com.jogamp.common.nio.Buffers;

import oimo.common.M;

public class OVertexBuffer {
	public int numVertices;//(default, null):Int;
	OVertexAttribute[] attribs;
	int[] indices;
	int[] offsets;
	/** Size of one vertex in byte including color and normal data*/
	int stride;
	GL2 gl;
	int vboId;
	int dataLength;
	// 32bit float
	static final int FLOAT_SIZE_BITS=4;
	public OVertexBuffer(GL2 gl) {
		this.gl = gl;
		int[] bufferID = new int[1]; // Get 4 buffer IDs for the data.
		gl.glGenBuffers(1,bufferID,0);
		vboId = bufferID[0];
	}

	public void setData(float[] array, boolean staticDraw) {
		if (attribs == null) {
			M.error("OVertexBuffer  : set attributes first");
		}
		numVertices =  array.length / (stride / 4);
		gl.glBindBuffer(GL2.GL_ARRAY_BUFFER, vboId);
		gl.glBufferData(GL2.GL_ARRAY_BUFFER, array.length * FLOAT_SIZE_BITS,Buffers.newDirectFloatBuffer(array), staticDraw?GL2.GL_STATIC_DRAW:GL2.GL_DYNAMIC_DRAW);
		gl.glBindBuffer(GL2.GL_ARRAY_BUFFER, 0);
	}

	public void updateData(float[] array) {
		gl.glBindBuffer(GL2.GL_ARRAY_BUFFER, vboId);
		gl.glBufferSubData(GL2.GL_ARRAY_BUFFER, 0,  array.length * FLOAT_SIZE_BITS, Buffers.newDirectFloatBuffer(array));
		gl.glBindBuffer(GL2.GL_ARRAY_BUFFER, 0);
	}

	public void updateDataFloat32Array(FloatBuffer array) {
		gl.glBindBuffer(GL2.GL_ARRAY_BUFFER, vboId);
		gl.glBufferSubData(GL2.GL_ARRAY_BUFFER, 0, array.capacity()* FLOAT_SIZE_BITS, array);
		gl.glBindBuffer(GL2.GL_ARRAY_BUFFER, 0);
	}

	public void setAttribs(OVertexAttribute[] attribs) {
		this.attribs = attribs;
		
		int num = attribs.length;
		offsets = new int[num];
		stride = 0;
		for (int i=0;i<num;i++) {
			offsets[i]=stride;
			stride += attribs[i].float32Count * FLOAT_SIZE_BITS; 
		}
	}

	public void loadAttribIndices(OShader program) {
		indices = program.getAttribIndices(attribs);
	}
	

	public void bindAttribs() {
		if (indices == null) {
			M.error( "indices are not loaded");
			return;
		}
		int num = attribs.length;
		gl.glBindBuffer(GL2.GL_ARRAY_BUFFER, vboId);
		for (int i=0;i<num;i++) {
			gl.glEnableVertexAttribArray(indices[i]);
			gl.glVertexAttribPointer(indices[i], attribs[i].float32Count, GL2.GL_FLOAT, false, stride, offsets[i]);
		}
		gl.glBindBuffer(GL2.GL_ARRAY_BUFFER, 0);
	}

}
