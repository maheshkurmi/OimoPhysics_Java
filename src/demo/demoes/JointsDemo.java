package demo.demoes;
import demo.common.DemoBase;
import demo.common.DemoRenderer;
import demo.common.OimoUtil;
import demo.common.UserInput;
import demo.common.ViewInfo;
import oimo.collision.geometry.BoxGeometry;
import oimo.collision.geometry.SphereGeometry;
import oimo.common.Mat3;
import oimo.common.MathUtil;
import oimo.common.Vec3;
import oimo.dynamics.World;
import oimo.dynamics.constraint.joint.RevoluteJoint;
import oimo.dynamics.constraint.joint.RevoluteJointConfig;
import oimo.dynamics.constraint.joint.RotationalLimitMotor;
import oimo.dynamics.constraint.joint.SphericalJoint;
import oimo.dynamics.constraint.joint.SphericalJointConfig;
import oimo.dynamics.constraint.joint.SpringDamper;
import oimo.dynamics.constraint.joint.TranslationalLimitMotor;
import oimo.dynamics.rigidbody.RigidBody;
import oimo.dynamics.rigidbody.RigidBodyConfig;
import oimo.dynamics.rigidbody.RigidBodyType;
import oimo.dynamics.rigidbody.Shape;
import oimo.dynamics.rigidbody.ShapeConfig;

/**
 * Gears demo
 */
public class JointsDemo extends DemoBase {
	public JointsDemo() {
		super("Joints Demo");
	}

	@Override 
	public void init(World world, DemoRenderer renderer, UserInput input, ViewInfo viewInfo) {
		super.init(world, renderer, input, viewInfo);

		renderer.camera(new Vec3(0, 7, 9), new Vec3(0, 2, 0), new Vec3(0, 1, 0));

		OimoUtil.addBox(world, new Vec3(0, -0.2, 0), new Vec3(6, 0.2, 6), true);

		renderer.getGraphics().getDebugDraw().drawJointLimits = true;

		createBallChain(new Vec3(-2, 5, -2), 0.4, 7);
		createHingeChain(new Vec3(2, 5, -2), 0.3, 7, new Vec3(0, 0, 1));

		createBoard(0, 4, 0, new RotationalLimitMotor().setLimits(-45 * MathUtil.TO_RADIANS, 45 * MathUtil.TO_RADIANS), new SpringDamper().setSpring(2, 0.3));
		createBoard(0, 6, 0, new RotationalLimitMotor().setMotor(MathUtil.TWO_PI, MathUtil.TWO_PI * 4), new SpringDamper());

		{
			double x = 2;
			double y = 5;
			double z = 1;
			RigidBody b1 = OimoUtil.addSphere(world, new Vec3(x, y, z), 0.1, true);
			RigidBody b2 = OimoUtil.addBox(world, new Vec3(x, y, z), new Vec3(0.3, 0.5, 0.5), false);
			OimoUtil.addPrismaticJoint(world, b1, b2, new Vec3(x, y, z), new Vec3(1, 1, 0), new SpringDamper(), new TranslationalLimitMotor().setLimits(-1, 1));
		}

		{
			double x = -2;
			double y = 5;
			double z = 1;
			RigidBody b1 = OimoUtil.addSphere(world, new Vec3(x, y, z), 0.1, true);
			RigidBody b2 = OimoUtil.addBox(world, new Vec3(x - 0.31, y, z), new Vec3(0.3, 0.5, 0.5), false);
			OimoUtil.addCylindricalJoint(world, b1, b2, new Vec3(x, y, z), new Vec3(1, 0, 0), new SpringDamper(), new RotationalLimitMotor().setLimits(-2, 2), new SpringDamper().setSpring(4, 0.7), new TranslationalLimitMotor().setLimits(-1, 1));
		}

		{
			double x = -2;
			double y = 3;
			double z = 3;
			double length = 1.0;

			RigidBody b1 = OimoUtil.addBox(world, new Vec3(x, y + length, z), new Vec3(0.2, 0.2, 0.2), true);
			b1.setType(RigidBodyType.KINEMATIC);
			b1.setAngularVelocity(new Vec3(0, 1.5, 0));
			RigidBody b2 = OimoUtil.addBox(world, new Vec3(x, y - length, z), new Vec3(0.2, 0.5, 0.2), false);
			OimoUtil.addRagdollJoint(world, b1, b2, new Vec3(x, y, z), new Vec3(0, 1, 0), new Vec3(0, 0, 1), new SpringDamper(), 40, 80, new SpringDamper(), new RotationalLimitMotor().setLimits(-MathUtil.HALF_PI, MathUtil.HALF_PI));
		}

		{
			double x = 2;
			double y = 3;
			double z = 3;
			double length = 1.0;
			RotationalLimitMotor hingeLimit1 = new RotationalLimitMotor().setLimits(-MathUtil.HALF_PI * 0.5, MathUtil.HALF_PI * 0.5);
			RotationalLimitMotor hingeLimit2 = new RotationalLimitMotor().setLimits(-MathUtil.HALF_PI * 0.8, MathUtil.HALF_PI * 0.8);

			RigidBody b1 = OimoUtil.addBox(world, new Vec3(x, y + length, z), new Vec3(0.2, 0.2, 0.2), true);
			b1.setType(RigidBodyType.KINEMATIC);
			b1.setAngularVelocity(new Vec3(0, 1.5, 0));
			RigidBody b2 = OimoUtil.addBox(world, new Vec3(x, y - length, z), new Vec3(0.2, 0.5, 0.2), false);
			OimoUtil.addUniversalJoint(world, b1, b2, new Vec3(x, y, z), new Vec3(1, 0, 0), new Vec3(0, 0, 1), new SpringDamper(), hingeLimit1, new SpringDamper(), hingeLimit2);
		}

		{
			double x = 0;
			double y = 3;
			double z = 3;
			double length = 1.0;
			RotationalLimitMotor rotXLimit = new RotationalLimitMotor().setLimits(-MathUtil.HALF_PI * 0.4, MathUtil.HALF_PI * 0.4);
			RotationalLimitMotor rotYLimit = new RotationalLimitMotor().setLimits(-MathUtil.HALF_PI * 0.2, MathUtil.HALF_PI * 0.2);
			RotationalLimitMotor rotZLimit = new RotationalLimitMotor().setLimits(-MathUtil.HALF_PI * 0.8, MathUtil.HALF_PI * 0.8);
			TranslationalLimitMotor translXLimit = new TranslationalLimitMotor().setLimits(-0.2, 0.2);
			TranslationalLimitMotor translYLimit = new TranslationalLimitMotor().setLimits(-0.3, 0);
			TranslationalLimitMotor translZLimit = new TranslationalLimitMotor().setLimits(-0.2, 0.8);

			RigidBody b1 = OimoUtil.addBox(world, new Vec3(x, y + length, z), new Vec3(0.2, 0.2, 0.2), true);
			b1.setType(RigidBodyType.KINEMATIC);
			b1.setAngularVelocity(new Vec3(0, 1.5, 0));
			RigidBody b2 = OimoUtil.addBox(world, new Vec3(x, y - length, z), new Vec3(0.2, 0.5, 0.2), false);
			OimoUtil.addGenericJoint(world, b1, b2, new Vec3(x, y, z), new Mat3(), new Mat3(), null, new TranslationalLimitMotor [] {translXLimit, translYLimit, translZLimit}, null,  new RotationalLimitMotor [] {rotXLimit, rotYLimit, rotZLimit});
		}
	}

	void createBoard(double x, double y, double z, RotationalLimitMotor lm, SpringDamper sd) {
		RigidBody b1 = OimoUtil.addBox(world, new Vec3(x, y, z), new Vec3(0.1, 0.1, 0.1), true);
		RigidBody b2 = OimoUtil.addBox(world, new Vec3(x + 0.5, y, z), new Vec3(0.5, 0.2, 0.4), false);
		OimoUtil.addRevoluteJoint(world, b1, b2, new Vec3(x, y, z), new Vec3(0, 0, 1), sd, lm);
	}

	void createBallChain(Vec3 from, double radius,int num) {
		RigidBodyConfig bc = new RigidBodyConfig();
		bc.position.copyFrom(from);
		bc.type = RigidBodyType.STATIC;

		ShapeConfig sc = new ShapeConfig();
		sc.geometry = new SphereGeometry(radius * 0.9);

		RigidBody b1;
		RigidBody b2;
		b1 = new RigidBody(bc);
		b1.addShape(new Shape(sc));
		world.addRigidBody(b1);

		SphericalJointConfig jc = new SphericalJointConfig();
		jc.localAnchor1.set(0, 0, 0);
		jc.localAnchor2.set(0, -radius * 2, 0);

		bc.type = RigidBodyType.DYNAMIC;
		for (int i=0;i<num;i++) {
			if (i == num - 1) {
				bc.position.x += MathUtil.randIn(-0.001, 0.001);
				bc.position.z += MathUtil.randIn(-0.001, 0.001);
			}
			bc.position.y += radius * 2;
			b2 = new RigidBody(bc);
			b2.addShape(new Shape(sc));
			world.addRigidBody(b2);

			jc.rigidBody1 = b1;
			jc.rigidBody2 = b2;
			world.addJoint(new SphericalJoint(jc));

			b1 = b2;
			jc.localAnchor1.set(0, radius, 0);
			jc.localAnchor2.set(0, -radius, 0);
		}
	}

	void createHingeChain(Vec3 from, double radius, int num, Vec3 axis) {
		RigidBodyConfig bc = new RigidBodyConfig();
		bc.position.copyFrom(from);
		bc.type = RigidBodyType.STATIC;

		ShapeConfig cc = new ShapeConfig();
		cc.geometry = new BoxGeometry(new Vec3(radius, radius, radius));

		RigidBody b1;
		RigidBody b2;
		b1 = new RigidBody(bc);
		b1.addShape(new Shape(cc));
		world.addRigidBody(b1);

		cc.geometry = new BoxGeometry(new Vec3(radius * 0.5, radius * 0.9, radius * 0.9));

		RevoluteJointConfig jc = new RevoluteJointConfig();
		jc.localAnchor1.set(0, 0, 0);
		jc.localAnchor2.set(0, -radius * 2, 0);

		bc.type = RigidBodyType.DYNAMIC;
		for (int i=0;i<num;i++) {
			jc.localAxis1 = axis;
			jc.localAxis2 = axis;
			if (i == num - 1) {
				bc.position.x += MathUtil.randIn(-0.001, 0.001);
				bc.position.z += MathUtil.randIn(-0.001, 0.001);
			}
			bc.position.y += radius * 2;
			b2 = new RigidBody(bc);
			b2.addShape(new Shape(cc));
			world.addRigidBody(b2);

			jc.rigidBody1 = b1;
			jc.rigidBody2 = b2;
			world.addJoint(new RevoluteJoint(jc));

			b1 = b2;
			jc.localAnchor1.set(0, radius, 0);
			jc.localAnchor2.set(0, -radius, 0);
		}
	}

	@Override 
	public void update() {
		super.update();
	}
}