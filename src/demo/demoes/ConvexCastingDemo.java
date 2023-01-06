package demo.demoes;
import demo.common.*;
import oimo.collision.geometry.*;
import oimo.common.*;
import oimo.dynamics.*;
import oimo.dynamics.callback.*;
import oimo.dynamics.constraint.joint.*;
import oimo.dynamics.rigidbody.*;

/**
 * Convex casting demo
 */
public class ConvexCastingDemo extends DemoBase {
	LaserPointer[] lps;

	public ConvexCastingDemo() {
		super("Convex Casting");
	}

	@Override 
	public void init(World world, DemoRenderer renderer, UserInput input, ViewInfo viewInfo) {
		super.init(world, renderer, input, viewInfo);
		renderer.camera(new Vec3(0, 7, 7), new Vec3(0, 0, 0), new Vec3(0, 1, 0));

		double thickness = 0.5;
		OimoUtil.addBox(world, new Vec3(0, -thickness, 0), new Vec3(5, thickness, 5), true);
		int w = 2;
		int h = 2;
		double sp = 0.8;
		int n = 3;
		double wid = 0.3;
		double hei = 0.3;
		double spH = hei * 2;
		for (int i=0;i<n;i++) {
			for (int j=-w;j<w+1;j++) {
				for (int k=-h;k<h+1;k++) {
					Setting.defaultDensity = i == n - 1 ? 1 : 1;

					wid = MathUtil.randIn(0.2, 0.4);
					hei = MathUtil.randIn(0.2, 0.4);
					final int randomShape=(int) (5 * Math.random());
					switch (randomShape) {
					case 0:
						OimoUtil.addCylinder(world, new Vec3(j * sp + MathUtil.randIn(-0.01, 0.01), spH + i * spH * 2 * 1.002, k * sp + MathUtil.randIn(-0.01, 0.01)), wid, hei, false);
					case 1:
						OimoUtil.addCone(world, new Vec3(j * sp + MathUtil.randIn(-0.01, 0.01), spH + i * spH * 2 * 1.002, k * sp + MathUtil.randIn(-0.01, 0.01)), wid, hei, false);
					case 2:
						OimoUtil.addCapsule(world, new Vec3(j * sp + MathUtil.randIn(-0.01, 0.01), spH + i * spH * 2 * 1.002, k * sp + MathUtil.randIn(-0.01, 0.01)), wid, hei, false);
					case 3:
						OimoUtil.addBox(world, new Vec3(j * sp + MathUtil.randIn(-0.01, 0.01), spH + i * spH * 2 * 0.9998, k * sp + MathUtil.randIn(-0.01, 0.01)), new Vec3(wid, hei, wid), false);
					case 4:
						OimoUtil.addSphere(world, new Vec3(j * sp + MathUtil.randIn(-0.01, 0.01), spH + i * spH * 2 * 0.9998, k * sp + MathUtil.randIn(-0.01, 0.01)), wid, false);
					}
				}
			}
		}

		lps = new LaserPointer[3];
		lps[0]=new LaserPointer(new Vec3(-2, 4, 0), world, new Vec3(0, 1, 0));
		lps[1]=new LaserPointer(new Vec3(0, 4, 0), world, new Vec3(0, 1, 0));
		lps[2]=new LaserPointer(new Vec3(2, 4, 0), world, new Vec3(0, 1, 0));
	}

	@Override
	public void update() {
		super.update();
		teleportRigidBodies(-20, 10, 5, 5);
	}

	@Override
	public void drawAdditionalObjects(DebugDraw debugDraw) {
		var w = debugDraw.wireframe;
		debugDraw.wireframe = true;

		for (LaserPointer lp:lps) lp.draw(debugDraw);

		debugDraw.wireframe = w;
	}
	
	private class LaserPointer {
		RigidBody rb;
		RayCastClosest cb;
		World world;
		Vec3 color;
		double length;

		public LaserPointer(Vec3 pos, World world, Vec3 color) {
			this.world = world;
			this.color = color;
			cb = new RayCastClosest();

			length = 0.4;

			var rc = new RigidBodyConfig();
			rc.autoSleep = false;
			rc.angularDamping = 4.0;

			rc.position.copyFrom(pos);
			rb = new RigidBody(rc);
			var sc = new ShapeConfig();
			sc.geometry = new CylinderGeometry(0.4, length);
			rb.addShape(new Shape(sc));
			world.addRigidBody(rb);

			rc.position.addEq(new Vec3(0, length, 0));
			rc.type = RigidBodyType.STATIC;
			RigidBody fix = new RigidBody(rc);
			sc.geometry = new SphereGeometry(0.1);
			fix.addShape(new Shape(sc));
			world.addRigidBody(fix);

			RagdollJointConfig jc = new RagdollJointConfig();
			jc.init(rb, fix, rb.getPosition().addEq(new Vec3(0, length, 0)), new Vec3(0, 1, 0), new Vec3(1, 0, 0));
			jc.twistLimitMotor.setLimits(0, 0);
			jc.maxSwingAngle1 = MathUtil.TO_RADIANS * 90;
			jc.maxSwingAngle2 = MathUtil.TO_RADIANS * 90;

			world.addJoint(new RagdollJoint(jc));
		}

		public void draw(DebugDraw debugDraw) {
			Transform tf = rb.getTransform();
			Vec3 begin = new Vec3(0, -length, 0).mulTransform(tf);
			Vec3 end = new Vec3(0, -length - 20, 0).mulTransform(tf);
			int depth = 3;
			Vec3 r = end.sub(begin);
			CylinderGeometry convex =  (CylinderGeometry) rb.getShapeList().getGeometry();
			cb.clear();

			Vec3 green = new Vec3(0, 1, 0);
			world.convexCast(convex, tf, r, cb);
			if (cb.hit) {
				Vec3 lineEnd = begin.addScaled(r, cb.fraction);
				debugDraw.line(begin, begin.addScaled(r, cb.fraction), green);
				tf.setPosition(tf.getPosition().addScaledEq(r, cb.fraction));
				debugDraw.cylinder(tf, convex.getRadius(), convex.getHalfHeight(), green);
				debugDraw.point(cb.position, green);
			}
		}
	}
}


