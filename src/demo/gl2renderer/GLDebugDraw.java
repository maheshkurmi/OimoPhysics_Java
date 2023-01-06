package demo.gl2renderer;

import com.jogamp.opengl.GL2;
import com.jogamp.opengl.glu.GLU;

import demo.common.IDemoGraphics;
import oimo.common.DebugDraw;
import oimo.common.Mat4;
import oimo.common.Vec3;

public class GLDebugDraw extends DebugDraw implements IDemoGraphics {
	GL2 gl;
	Vec3 color;
	Vec3 cameraPos;
	Vec3 lightDir;
	Mat4 viewMat;
	Mat4 projMat;

	Mat4 tmpMat;
	private GLU glu; // for the GL Utility

	GLDebugDraw(GL2 gl) {
		this.setGL(gl);
		 viewMat=new Mat4();
		 projMat=new Mat4();
		  cameraPos=new Vec3();
	   lightDir =new Vec3();

	}

	void setGL(GL2 gl) {
		this.gl = gl;
		glu = new GLU(); // get GL Utilities

	}
	
	void setColor(Vec3 color){
		if(color!=null & color!=this.color) {
			gl.glColor3d(color.x, color.y, color.z);
			this.color=color;
		}
	}

	@Override
	public void point(Vec3 v, Vec3 color) {
		setColor(color);
		gl.glBegin(GL2.GL_POINT);
		gl.glVertex3d(v.x, v.y, v.z);
		gl.glEnd();
	}

	@Override
	public void triangle(Vec3 v1, Vec3 v2, Vec3 v3, Vec3 n1, Vec3 n2, Vec3 n3, Vec3 color) {
		setColor(color);
		gl.glBegin(GL2.GL_TRIANGLES);
		gl.glVertex3d(v1.x, v1.y, v1.z);
		gl.glVertex3d(v2.x, v2.y, v2.z);
		gl.glVertex3d(v3.x, v3.y, v3.z);
		gl.glEnd();
	}

	@Override
	public void line(Vec3 v1, Vec3 v2, Vec3 color) {
		setColor(color);
		gl.glBegin(GL2.GL_LINES);
		gl.glVertex3d(v1.x, v1.y, v1.z);
		gl.glVertex3d(v2.x, v2.y, v2.z);
		gl.glEnd();
	}

	@Override
	public void begin(Vec3 color) {
		gl.glClearColor((float) color.x, (float) color.y, (float) color.z, 1);
		gl.glEnable(GL2.GL_CULL_FACE);
		gl.glClearDepth(1);
		gl.glClear(GL2.GL_COLOR_BUFFER_BIT | GL2.GL_DEPTH_BUFFER_BIT);
		gl.glLineWidth(2.0f);
		// Setup perspective projection, with aspect ratio matches viewport
		gl.glMatrixMode(GL2.GL_PROJECTION); // choose projection matrix
		gl.glLoadIdentity();
		gl.glMultMatrixf(projMat.toArray(true), 0);
		// Enable the model-view transform
		gl.glMatrixMode(GL2.GL_MODELVIEW);
		gl.glLoadIdentity(); // reset
		gl.glMultMatrixf(viewMat.toArray(true), 0);

	}

	@Override
	public void end() {
		gl.glFlush();
	}

	@Override
	public DebugDraw getDebugDraw() {
		return this;
	}

	@Override
	public void setViewMatrix(Mat4 matrix) {
		viewMat.copyFrom(matrix).inverseEq();
		cameraPos.set(viewMat.e03, viewMat.e13, viewMat.e23);
		lightDir.set(viewMat.e02, viewMat.e12, viewMat.e22).scaleEq(-1);
		viewMat.copyFrom(matrix);
	}

	@Override
	public void setProjectionMatrix(Mat4 matrix) {
		projMat.copyFrom(matrix);
	}

}
