package demo.common;
import oimo.collision.broadphase.bvh.*;
import oimo.common.Mat4;
import oimo.common.Vec3;
import oimo.dynamics.*;
import oimo.dynamics.constraint.contact.*;
import oimo.dynamics.constraint.joint.*;
import oimo.dynamics.rigidbody.*;

/**
 * DemoRenderer renders a demo world using an instance of IDemoGraphics.
 */
public class DemoRenderer {
	IDemoGraphics g;
	World w;
	Vec3 bgColor;

	Mat4 viewMat;
	Mat4 projMat;
	double fov=Math.PI/3;
	public DemoRenderer(World world, IDemoGraphics graphics) {
		w = world;
		g = graphics;

		bgColor = new Vec3(0.2, 0.1, 0.1);
		viewMat = new Mat4();
		projMat = new Mat4();
	}

	// --- public ---

	public void setWorld(World world) {
		w = world;
	}

	public void setGraphics(IDemoGraphics graphics) {
		g = graphics;
	}

	public IDemoGraphics getGraphics() {
		return g;
	}

	public void draw(DemoBase test) {
		g.setViewMatrix(viewMat);
		g.setProjectionMatrix(projMat);

		g.begin(bgColor);

		w.setDebugDraw(g.getDebugDraw());
		w.debugDraw();

		test.drawAdditionalObjects(g.getDebugDraw());

		g.end();
	}

	public void camera(Vec3 eye,Vec3 at,Vec3 up) {
		viewMat.lookAt(eye.x, eye.y, eye.z, at.x, at.y, at.z, up.x, up.y, up.z);
	}

	public Mat4 getViewMatrix() {
		return viewMat.clone();
	}

	public Vec3 getCameraPosition() {
		return new Vec3(
			-(viewMat.e00 * viewMat.e03 + viewMat.e10 * viewMat.e13 + viewMat.e20 * viewMat.e23),
			-(viewMat.e01 * viewMat.e03 + viewMat.e11 * viewMat.e13 + viewMat.e21 * viewMat.e23),
			-(viewMat.e02 * viewMat.e03 + viewMat.e12 * viewMat.e13 + viewMat.e22 * viewMat.e23)
		);
	}

	/**
	 * 
	 * @param fovY in radians
	 * @param aspect
	 */
	public void perspective(double fovY, double aspect) {
		this.fov=fovY;
		projMat.perspective(fovY, aspect, 0.1, 1000);
	}

	/**
	 * returns fov in radians
	 * @return
	 */
	public double getFov() {
		return this.fov;
	}
	

	
	public Mat4 getProjectionMatrix() {
		return projMat.clone();
	}

}