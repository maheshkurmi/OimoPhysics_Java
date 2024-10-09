package demo.demoes;
import demo.common.*;
import oimo.collision.geometry.*;
import oimo.common.*;
import oimo.dynamics.*;
import oimo.dynamics.rigidbody.*;

/**
 * Convex hull demo
 */
public class ConvexHullDemo extends DemoBase {
	public ConvexHullDemo() {
		super("Convex Hull");
	}

	@Override 
	public void init(World world, DemoRenderer renderer, UserInput input, ViewInfo viewInfo) {
		super.init(world, renderer, input, viewInfo);
		renderer.camera(new Vec3(0, 7, 9), new Vec3(0, 2, 0), new Vec3(0, 1, 0));

		double thickness = 0.5;
		Utils.addBox(world, new Vec3(0, -thickness, 0), new Vec3(7, thickness, 7), true);

		int w = 1;
		int h = 1;
		double sp = 0.61;
		int n = 3;
		double wid = 0.6;
		double hei = 0.6;
		double dep = 0.6;
		for (int i=0;i<n;i++) {
			for (int j=-w;j<w + 1;j++) {
				for (int k=-h;k<h+1;k++) {
					Vec3 center = new Vec3(j * wid * 2, hei + i * hei * 3.0, k * dep * 2);
					RigidBodyConfig bc = new RigidBodyConfig();
					ShapeConfig sc = new ShapeConfig();
					bc.position = center;
					Vec3[] vertices=new Vec3[8];
					for (int m=0;m<8;m++) vertices[m]=new Vec3(rand() * wid, rand() * hei, rand() * dep);
					sc.geometry = new ConvexHullGeometry(vertices);
					RigidBody b = new RigidBody(bc);
					b.addShape(new Shape(sc));
					world.addRigidBody(b);
				}
			}
		}
	}

	double rand() {
		double x = Math.pow(Math.random(), 0.7);
		if (Math.random() < 0.5) x = -x;
		return x;
	}

	@Override
	public void update() {
		super.update();
		teleportRigidBodies(-20, 10, 5, 5);
	}
}
