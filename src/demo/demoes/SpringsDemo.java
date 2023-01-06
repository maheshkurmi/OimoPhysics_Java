package demo.demoes;
import demo.common.*;
import oimo.collision.geometry.*;
import oimo.common.*;
import oimo.dynamics.*;
import oimo.dynamics.rigidbody.*;
import oimo.dynamics.constraint.joint.RotationalLimitMotor;
import oimo.dynamics.constraint.joint.SpringDamper;
import oimo.dynamics.constraint.joint.TranslationalLimitMotor;

/**
 * Springs demo
 */
public class SpringsDemo extends DemoBase {
	public SpringsDemo() {
		super("Springs");
	}

	@Override 
	public void init(World world, DemoRenderer renderer, UserInput input, ViewInfo viewInfo) {
		super.init(world, renderer, input, viewInfo);
		renderer.camera(new Vec3(0, 7, 9), new Vec3(0, 2, 0), new Vec3(0, 1, 0));

		for (int i=0;i<5;i++) {
			OimoUtil.addSphere(world, MathUtil.randVec3In(-1, 1).scale3Eq(2, 2, 0.1).addEq(new Vec3(0, 8, 0)), 0.6, false);
		}

		addSpringyBoard(new Vec3(-3, 3, 0), 1, 8);
		addSpringyBoard(new Vec3(3, 3, 0), -1, 8);
		addSpringyBoard(new Vec3(-3, 4, 0), 1, 8);
		addSpringyBoard(new Vec3(3, 4, 0), -1, 8);

		for (int i=0;i<5;i++) {
			addSpringyFloor(new Vec3(i - 2, 0, 0));
		}
	}

	void addSpringyBoard(Vec3 at, int dir, int num) {
		RigidBody[] bodies = new RigidBody[num];
		for (int i=0;i<num;i++) {
			bodies[i]=OimoUtil.addBox(world, at.add(new Vec3(i * 0.4 * dir, 0, 0)), new Vec3(0.2, 0.1, 0.4), i == 0);
		}
		for (int i=1;i<num;i++) {
			RigidBody b1 = bodies[i - 1];
			RigidBody b2 = bodies[i];
			Vec3 anchor = b1.getPosition().addEq(b2.getPosition()).scaleEq(0.5);
			Vec3 axis = new Vec3(0, 0, 1);
			SpringDamper springDamper = new SpringDamper().setSpring(15, 0.4);
			RotationalLimitMotor limitMotor = new RotationalLimitMotor().setLimits(0, 0);
			OimoUtil.addRevoluteJoint(world, b1, b2, anchor, axis, springDamper, limitMotor);
		}
	}

	void addSpringyFloor(Vec3 at) {
		RigidBody base = OimoUtil.addBox(world, at.add(new Vec3(0, -2, 0)), new Vec3(0.5, 0.1, 0.5), true);
		RigidBody floor = OimoUtil.addBox(world, at, new Vec3(0.4, 0.1, 0.4), false);

		Vec3 anchor = floor.getPosition();
		Vec3 axis = new Vec3(0, 1, 0);
		SpringDamper springDamper = new SpringDamper().setSpring(3, 0.2);
		TranslationalLimitMotor limitMotor = new TranslationalLimitMotor().setLimits(0, 0);
		OimoUtil.addPrismaticJoint(world, base, floor, anchor, axis, springDamper, limitMotor);
	}

	@Override
	public void update() {
		super.update();
		teleportRigidBodies(-20, 10, 2, 0.1);
	}
}
