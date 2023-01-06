package demo.demoes;
import demo.common.*;
import oimo.collision.geometry.*;
import oimo.common.*;
import oimo.dynamics.*;
import oimo.dynamics.callback.*;
import oimo.dynamics.constraint.joint.*;
import oimo.dynamics.rigidbody.*;

/**
 * Ray casting demo
 */
public class RayCastingDemo extends DemoBase {
	LaserPointer[] lps;

	public RayCastingDemo() {
		super("Ray Casting");
	}

	@Override 
	public void init(World world, DemoRenderer renderer, UserInput input, ViewInfo viewInfo) {
		super.init(world, renderer, input, viewInfo);

		renderer.camera(new Vec3(0, 7, 7), new Vec3(0, 0, 0), new Vec3(0, 1, 0));

		double thickness = 0.5;
		OimoUtil.addBox(world, new Vec3(0, -thickness, 0), new Vec3(5, thickness, 5), true);

		int w = 2;
		int h = 1;
		double sp = 0.8;
		int n = 3;
		double wid = 0.4;
		double hei = 0.4;
		double spH = hei;
		for (int i=0;i<n;i++) {
			for (int j=-w;j<w+1;j++) {
				for (int k=-h;k<h+1;k++) {

					wid = MathUtil.randIn(0.3, 0.4);
					hei = MathUtil.randIn(0.3, 0.4);
					int shapeType=(int) (5 * Math.random());
					switch (shapeType) {
					case 0:
						OimoUtil.addCone(world, new Vec3(j * sp + MathUtil.randIn(-0.01, 0.01), spH + i * spH * 2 * 1.002, k * sp + MathUtil.randIn(-0.01, 0.01)), wid, hei, false);
					case 1:
						OimoUtil.addBox(world, new Vec3(j * sp + MathUtil.randIn(-0.01, 0.01), spH + i * spH * 2 * 0.9998, k * sp + MathUtil.randIn(-0.01, 0.01)), new Vec3(wid, hei, wid), false);
					case 2:
						OimoUtil.addCylinder(world, new Vec3(j * sp + MathUtil.randIn(-0.01, 0.01), spH + i * spH * 2 * 1.002, k * sp + MathUtil.randIn(-0.01, 0.01)), wid, hei, false);
					case 3:
						OimoUtil.addCapsule(world, new Vec3(j * sp + MathUtil.randIn(-0.01, 0.01), spH + i * spH * 2 * 1.002, k * sp + MathUtil.randIn(-0.01, 0.01)), wid, hei, false);
					case 4:
						OimoUtil.addSphere(world, new Vec3(j * sp + MathUtil.randIn(-0.01, 0.01), spH + i * spH * 2 * 0.9998, k * sp + MathUtil.randIn(-0.01, 0.01)), wid, false);
					}
				}
			}
		}

		lps = new LaserPointer[5];
		lps[0]=new LaserPointer(new Vec3(-2, 4, 0), world, new Vec3(0, 1, 0));
		lps[1]=new LaserPointer(new Vec3(-1, 4, 0), world, new Vec3(0, 1, 0));
		lps[2]=new LaserPointer(new Vec3(0, 4, 0), world, new Vec3(0, 1, 0));
		lps[3]=new LaserPointer(new Vec3(1, 4, 0), world, new Vec3(0, 1, 0));
		lps[4]=new LaserPointer(new Vec3(2, 4, 0), world, new Vec3(0, 1, 0));
	}

	@Override
	public void update() {
		super.update();
		teleportRigidBodies(-20, 10, 5, 5);
	}

	@Override
	public void drawAdditionalObjects(DebugDraw debugDraw) {
		for (LaserPointer lp:lps) {
			lp.draw(debugDraw);
		}
	}
	
	private class LaserPointer {
		RigidBody rb;
		RayCastClosest cb;
		World w;
		Vec3 color;
		double length;

		public LaserPointer(Vec3 pos, World world, Vec3 color) {
			w = world;
			this.color = color;
			cb = new RayCastClosest();

			length = 0.4;

			var rc = new RigidBodyConfig();
			rc.autoSleep = false;
			rc.angularDamping = 4.0;

			rc.position.copyFrom(pos);
			rb = new RigidBody(rc);
			var sc = new ShapeConfig();
			sc.geometry = new CylinderGeometry(0.2, length);
			rb.addShape(new Shape(sc));
			w.addRigidBody(rb);

			rc.position.addEq(new Vec3(0, length, 0));
			rc.type = RigidBodyType.STATIC;
			RigidBody fix = new RigidBody(rc);
			sc.geometry = new SphereGeometry(0.1);
			fix.addShape(new Shape(sc));
			w.addRigidBody(fix);

			RagdollJointConfig jc = new RagdollJointConfig();
			jc.init(rb, fix, rb.getPosition().addEq(new Vec3(0, length, 0)), new Vec3(0, 1, 0), new Vec3(1, 0, 0));
			jc.twistLimitMotor.setLimits(0, 0);
			jc.maxSwingAngle1 = MathUtil.TO_RADIANS * 90;
			jc.maxSwingAngle2 = MathUtil.TO_RADIANS * 90;

			world.addJoint(new RagdollJoint(jc));
		}

		public void draw(DebugDraw dd) {
			Transform tf = rb.getTransform();
			Vec3 begin = new Vec3(0, -length, 0).mulTransform(tf);
			Vec3 end = new Vec3(0, -length - 20, 0).mulTransform(tf);
			int depth = 3;

			for (int i=0;i<depth;i++) {
				cb.clear();
				w.rayCast(begin.clone(), end.clone(), cb);

				if (cb.hit) {
					dd.line(begin, cb.position, color);
					dd.point(cb.position, color);

					Vec3 dir = end.sub(begin).normalized();
					Vec3 refl = dir.addScaled(cb.normal, -2 * cb.normal.dot(dir));

					begin = cb.position.addScaled(cb.normal, 0.01);
					end = cb.position.addScaled(refl, 20);
				} else {
					dd.line(begin, end, color);
					break;
				}
			}
		}
	}

}

