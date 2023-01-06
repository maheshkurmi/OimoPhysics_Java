package demo.gl2renderer;

import java.nio.FloatBuffer;

import com.jogamp.common.nio.Buffers;
import com.jogamp.opengl.GL2;
import com.jogamp.opengl.glu.GLU;

import demo.common.IDemoGraphics;
import oimo.common.DebugDraw;
import oimo.common.Mat4;
import oimo.common.Vec3;

public class GL2DebugDraw extends DebugDraw implements IDemoGraphics {
	GL2 gl;
	OVertexBuffer pointVBO;
	OIndexBuffer pointIBO;

	OVertexBuffer lineVBO;
	OIndexBuffer lineIBO;

	OVertexBuffer triVBO;
	OIndexBuffer triIBO;

	int pointBufferSize;
	FloatBuffer pointData;
	int numPointData;

	int lineBufferSize;
	FloatBuffer lineData;
	int numLineData;

	int triBufferSize;
	FloatBuffer triData;
	int numTriData;

	OShader shader;

	Vec3 color;
	Vec3 cameraPos;
	Vec3 lightDir;
	Mat4 viewMat;
	Mat4 projMat;

	Mat4 tmpMat;

	public GL2DebugDraw(GL2 gl) {
		this.setGL(gl);
		viewMat = new Mat4();
		projMat = new Mat4();
		cameraPos = new Vec3();
		lightDir = new Vec3(1,0,1);
		tmpMat = new Mat4();
		initShader();
		initBuffers();
	}

	void setGL(GL2 gl) {
		this.gl = gl;
		gl.glEnable(GL2.GL_DEPTH_TEST);

	}
	
	void initBuffers() {
		OVertexAttribute[] attribs  =new OVertexAttribute[] {new OVertexAttribute(3, "aPosition"), new OVertexAttribute(3, "aNormal"), new OVertexAttribute(3, "aColor")};
		pointVBO = new OVertexBuffer(gl);
		pointIBO = new OIndexBuffer(gl);
		pointVBO.setAttribs(attribs);
		pointVBO.loadAttribIndices(shader);
		lineVBO = new OVertexBuffer(gl);
		lineIBO = new OIndexBuffer(gl);
		lineVBO.setAttribs(attribs);
		lineVBO.loadAttribIndices(shader);
		triVBO = new OVertexBuffer(gl);
		triIBO = new OIndexBuffer(gl);
		triVBO.setAttribs(attribs);
		triVBO.loadAttribIndices(shader);

		pointBufferSize = 4096;
		lineBufferSize = 4096;
		triBufferSize = 4096;
		pointData = Buffers.newDirectFloatBuffer(pointBufferSize * 9);//(pointBufferSize * 9);
		lineData = Buffers.newDirectFloatBuffer(lineBufferSize * 9 * 2);
		triData =  Buffers.newDirectFloatBuffer(triBufferSize * 9 * 3);

		initFloatArray(pointData,pointBufferSize * 9);
		initFloatArray(lineData,lineBufferSize * 9 * 2);
		initFloatArray(triData,triBufferSize * 9 * 3);

		float[] vbo = new float[18*pointBufferSize];
		int[] ibo = new int[pointBufferSize];
		for (int i=0;i<pointBufferSize;i++) {
//			vbo.push(0); vbo.push(0); vbo.push(0); // pos1
//			vbo.push(0); vbo.push(0); vbo.push(0); // nml1
//			vbo.push(0); vbo.push(0); vbo.push(0); // rgb1
//			vbo.push(0); vbo.push(0); vbo.push(0); // pos2
//			vbo.push(0); vbo.push(0); vbo.push(0); // nml2
//			vbo.push(0); vbo.push(0); vbo.push(0); // rgb2
			ibo[i]=i;
		}
		pointVBO.setData(vbo, false);
		pointIBO.setData(ibo, false);

		vbo = new float[18*lineBufferSize];
		ibo = new int[2*lineBufferSize];
		for (int i=0;i<lineBufferSize;i++) {
//			vbo.push(0); vbo.push(0); vbo.push(0); // pos1
//			vbo.push(0); vbo.push(0); vbo.push(0); // nml1
//			vbo.push(0); vbo.push(0); vbo.push(0); // rgb1
//			vbo.push(0); vbo.push(0); vbo.push(0); // pos2
//			vbo.push(0); vbo.push(0); vbo.push(0); // nml2
//			vbo.push(0); vbo.push(0); vbo.push(0); // rgb2
			ibo[i * 2]=i * 2;
			ibo[i * 2 + 1]=i * 2 + 1;
		}
		lineVBO.setData(vbo, false);
		lineIBO.setData(ibo, false);

		vbo = new float[27*triBufferSize];
		ibo = new int[3*triBufferSize];
		for (int i=0;i<triBufferSize;i++) {
//			vbo.push(0); vbo.push(0); vbo.push(0); // pos1
//			vbo.push(0); vbo.push(0); vbo.push(0); // nml1
//			vbo.push(0); vbo.push(0); vbo.push(0); // rgb1
//			vbo.push(0); vbo.push(0); vbo.push(0); // pos2
//			vbo.push(0); vbo.push(0); vbo.push(0); // nml2
//			vbo.push(0); vbo.push(0); vbo.push(0); // rgb2
//			vbo.push(0); vbo.push(0); vbo.push(0); // pos3
//			vbo.push(0); vbo.push(0); vbo.push(0); // nml3
//			vbo.push(0); vbo.push(0); vbo.push(0); // rgb3
			ibo[i * 3]=i * 3;
			ibo[i * 3 + 1]=i * 3 + 1;
			ibo[i * 3 + 2]=i * 3 + 2;
		}
		triVBO.setData(vbo, false);
		triIBO.setData(ibo, false);
	}

	void initFloatArray(FloatBuffer buffer, int size) {
//		var num=buffer.capacity();
//		for (int i=0;i<num;i++) {
//			buffer.put(0);
//		}
		buffer.put(new float[size]);
		buffer.flip();
	}
	
	void initShader() {
		shader = new OShader(gl);
		shader.compile(
			
			"attribute vec3 aPosition;\n"+
			"attribute vec3 aColor;\n"+
			"attribute vec3 aNormal;\n"+
			"varying vec3 vPosition;\n"+
			"varying vec3 vNormal;\n"+
			"varying vec3 vColor;\n"+
			"uniform mat4 worldMat;\n"+
			"uniform mat4 viewMat;\n"+
			"uniform mat4 projMat;\n"+

			"void main() {\n"+
			"	vec4 worldPos = worldMat * vec4(aPosition, 1.0);\n"+
			"	vPosition = worldPos.xyz;\n"+
			"	vColor = aColor;\n"+
			"	vNormal = aNormal;\n"+
			"	gl_Position = projMat * (viewMat * worldPos);\n"+
			"	gl_PointSize = 6.0;\n"+
			"}\n"
		, 
			"//precision mediump float;\n"+
			"varying vec3 vPosition;\n"+
			"varying vec3 vNormal;\n"+
			"varying vec3 vColor;\n"+
			"uniform vec3 lightDir;\n"+
			"uniform vec3 lightCol;\n"+
			"uniform vec3 cameraPos;\n"+
			"uniform float ambient;\n"+
			"uniform float diffuse;\n"+
			"uniform float specular;\n"+
			"uniform float shininess;\n"+

			"void main() {\n"+
			"	vec3 normal = length(vNormal) > 0.0 ? normalize(vNormal) : vec3(0.0, 0.0, 0.0);\n"+
			"	vec3 dir = normalize(lightDir);\n"+
			"	float d = -dot(dir, normal);\n"+
			"	float brightness = max(0.0, d) * diffuse;\n"+
			"	vec3 eye = normalize(vPosition - cameraPos);\n"+
			"	vec3 pixColor = vColor * min(1.0, ambient + diffuse * brightness);\n"+
			"	if (d > 0.0) {\n"+
			"		d = -dot(dir, reflect(eye, normal));\n"+
			"		pixColor += lightCol * specular * pow(max(0.0, d), shininess);\n"+
			"	}\n"+
			"	gl_FragColor = vec4(pixColor, 1.0);\n"+
			"}\n"
		);
	}
	
	public void setViewMatrix(Mat4 matrix) {
	viewMat.copyFrom(matrix).inverseEq();
		cameraPos.set(
			viewMat.e03,
			viewMat.e13,
			viewMat.e23
		);
		lightDir.set(
			viewMat.e02,
			viewMat.e12,
			viewMat.e22
		).scaleEq(-1);
		viewMat.copyFrom(matrix);
	}

	public void setProjectionMatrix(Mat4 matrix) {
		projMat.copyFrom(matrix);
	}
	
	public void begin(Vec3 color) {
		gl.glClearColor((float)color.x, (float)color.y,(float) color.z,1);
		gl.glEnable(GL2.GL_CULL_FACE);
		//gl.glDepthFunc(GL2.GL_LEQUAL); // the type of depth test to do
		gl.glClearDepth(1);
		gl.glClear(GL2.GL_COLOR_BUFFER_BIT | GL2.GL_DEPTH_BUFFER_BIT);
		gl.glLineWidth(2.0f);
		gl.glDepthFunc(GL2.GL_LESS); 
		gl.glEnable(GL2.GL_DEPTH_TEST);  
		gl.glDepthMask(true);
		shader.use();
		setMat4("viewMat", viewMat);
		setMat4("projMat", projMat);
		setVec3("cameraPos", cameraPos);
		setVec3("lightDir", lightDir);
		setFloat3("lightCol", 1, 1, 1);
		tmpMat.identity();
		setMat4("worldMat", tmpMat);

		numPointData = 0;
		numLineData = 0;
		numTriData = 0;
	}
	
	public void end() {
		colorModeWireframe();

		if (numPointData > 0) {
			pointVBO.updateDataFloat32Array(pointData);
			pointVBO.bindAttribs();
			pointIBO.draw(GL2.GL_POINTS, numPointData);
			numPointData = 0;
		}
		if (numLineData > 0) {
			lineVBO.updateDataFloat32Array(lineData);
			lineVBO.bindAttribs();
			lineIBO.draw(GL2.GL_LINES, numLineData * 2);
			numLineData = 0;
		}

		colorModeSolid();

		if (numTriData > 0) {
			triVBO.updateDataFloat32Array(triData);
			triVBO.bindAttribs();
			triIBO.draw(GL2.GL_TRIANGLES, numTriData * 3);
			numTriData = 0;
		}

		gl.glFlush();
	}

	@Override 
	public void point(Vec3 v, Vec3 color) {
		int idx = numPointData * 9;
		FloatBuffer data = pointData;
		data.put(idx++, (float) v.x);
		data.put(idx++, (float) v.y);
		data.put(idx++,(float) v.z);
		data.put(idx++, 0);
		data.put(idx++, 0);
		data.put(idx++, 0);
		data.put(idx++, (float) color.x);
		data.put(idx++, (float) color.y);
		data.put(idx++, (float) color.z);
		numPointData++;

		if (numPointData == pointBufferSize) {
			colorModeWireframe();
			pointVBO.updateDataFloat32Array(pointData);
			pointVBO.bindAttribs();
			pointIBO.draw(GL2.GL_POINTS,-1);
			numPointData = 0;
		}
	}

	@Override 
	public void line(Vec3 v1, Vec3 v2, Vec3 color) {
		int idx = numLineData * 18;
		FloatBuffer data = lineData;
		data.put(idx++, (float)v1.x);
		data.put(idx++, (float)v1.y);
		data.put(idx++, (float)v1.z);
		data.put(idx++, (float)0);
		data.put(idx++, (float)0);
		data.put(idx++, (float)0);
		data.put(idx++, (float)color.x);
		data.put(idx++, (float)color.y);
		data.put(idx++, (float)color.z);
		data.put(idx++, (float)v2.x);
		data.put(idx++, (float)v2.y);
		data.put(idx++, (float)v2.z);
		data.put(idx++, (float)0);
		data.put(idx++, (float)0);
		data.put(idx++, (float)0);
		data.put(idx++, (float)color.x);
		data.put(idx++, (float)color.y);
		data.put(idx++, (float)color.z);
		numLineData++;

		if (numLineData == lineBufferSize) {
			colorModeWireframe();
			lineVBO.updateDataFloat32Array(lineData);
			lineVBO.bindAttribs();
			lineIBO.draw(GL2.GL_LINES,-1);
			numLineData = 0;
		}
	}

	@Override 
	public void triangle(Vec3 v1,Vec3 v2, Vec3 v3, Vec3 n1, Vec3 n2, Vec3 n3, Vec3 color) {
		int idx = numTriData * 27;
		FloatBuffer data = triData;
		data.put(idx++, (float)v1.x);
		data.put(idx++, (float)v1.y);
		data.put(idx++, (float)v1.z);
		data.put(idx++, (float)n1.x);
		data.put(idx++, (float)n1.y);
		data.put(idx++, (float)n1.z);
		data.put(idx++, (float)color.x);
		data.put(idx++, (float)color.y);
		data.put(idx++, (float)color.z);
		data.put(idx++, (float)v2.x);
		data.put(idx++, (float)v2.y);
		data.put(idx++, (float)v2.z);
		data.put(idx++, (float)n2.x);
		data.put(idx++, (float)n2.y);
		data.put(idx++, (float)n2.z);
		data.put(idx++, (float)color.x);
		data.put(idx++, (float)color.y);
		data.put(idx++, (float)color.z);
		data.put(idx++, (float)v3.x);
		data.put(idx++, (float)v3.y);
		data.put(idx++, (float)v3.z);
		data.put(idx++, (float)n3.x);
		data.put(idx++, (float)n3.y);
		data.put(idx++, (float)n3.z);
		data.put(idx++, (float)color.x);
		data.put(idx++, (float)color.y);
		data.put(idx++, (float)color.z);
		numTriData++;

		if (numTriData == triBufferSize) {
			colorModeSolid();
			triVBO.updateDataFloat32Array(triData);
			triVBO.bindAttribs();
			triIBO.draw(GL2.GL_TRIANGLES,-1);
			numTriData = 0;
		}
	}
	
	void colorModeSolid() {
		setFloat1("ambient", 0.2);
		setFloat1("diffuse", 0.8);
		setFloat1("specular", 0.8);
		setFloat1("shininess", 20);
	}

	void colorModeWireframe() {
		setFloat1("ambient", 1);
		setFloat1("diffuse", 0);
		setFloat1("specular", 0);
		setFloat1("shininess", 1);
	}

	
	void setMat4(String name, Mat4 matrix) {
		gl.glUniformMatrix4fv(shader.getUniformLocation(name), 1,false, matrix.toArray(true),0);
	}

	void setFloat3(String name, double x, double y, double z) {
		gl.glUniform3f(shader.getUniformLocation(name), (float)x, (float)y, (float)z);
	}

	void setVec3(String name, Vec3 v) {
		gl.glUniform3f(shader.getUniformLocation(name), (float)v.x, (float)v.y, (float)v.z);
	}

	void setFloat1(String name, double x) {
		gl.glUniform1f(shader.getUniformLocation(name), (float)x);
	}
	
	@Override
	public DebugDraw getDebugDraw() {
		return this;
	}


}
