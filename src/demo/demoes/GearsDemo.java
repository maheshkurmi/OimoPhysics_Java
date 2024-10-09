package demo.demoes;
import demo.common.DemoBase;
import demo.common.DemoRenderer;
import demo.common.Utils;
import demo.common.UserInput;
import demo.common.ViewInfo;
import oimo.collision.geometry.BoxGeometry;
import oimo.collision.geometry.ConeGeometry;
import oimo.collision.geometry.ConvexHullGeometry;
import oimo.collision.geometry.CylinderGeometry;
import oimo.collision.geometry.Geometry;
import oimo.common.Mat3;
import oimo.common.MathUtil;
import oimo.common.Vec3;
import oimo.dynamics.World;
import oimo.dynamics.constraint.joint.RotationalLimitMotor;
import oimo.dynamics.constraint.joint.SpringDamper;
import oimo.dynamics.rigidbody.RigidBody;
import oimo.dynamics.rigidbody.RigidBodyConfig;
import oimo.dynamics.rigidbody.Shape;
import oimo.dynamics.rigidbody.ShapeConfig;

/**
 * Gears demo
 */
public class GearsDemo extends DemoBase {
	public GearsDemo() {
		super("Compund Shapes Demo");
	}

	@Override 
	public void init(World world, DemoRenderer renderer, UserInput input, ViewInfo viewInfo) {
		super.init(world, renderer, input, viewInfo);
			renderer.camera(new Vec3(0, 6, 8), new Vec3(0, 2, 0), new Vec3(0, 1, 0));
		//	world.setGravity(new Vec3(0,0,0));
		double thickness = 0.2;
		Utils.addBox(world, new Vec3(0, -2-thickness, 0), new Vec3(4, thickness, 4), true);

		createGear(new Vec3(1, 3, 0.5), 1.0, 0.3,null);
		createGear(new Vec3(3, 3, 0.5), 1.0, 0.3,null);
		createGear(new Vec3(-0.5, 3, 0), 0.5, 1.6,null);
		createGear(new Vec3(1.5, 3, -0.5), 1.5, 0.3,null);
		createGear(new Vec3(-2, 3, 0), 1.0, 0.3, new RotationalLimitMotor().setMotor(MathUtil.PI, 50));
		createGear(new Vec3(-3.5, 3, 0), 0.5, 0.3,null);

		//createGear(new Vec3(0, 0, 0), 1.0, 0.2,null);
		for (int i=0;i<20;i++) {
			Utils.addBox(world, MathUtil.randVec3In(-1, 1).scale3Eq(3, 1, 1).addEq(new Vec3(0, 6, 0)), new Vec3(0.2, 0.2, 0.2), false);
		}
		for (int i=0;i<20;i++) {
			Utils.addSphere(world, MathUtil.randVec3In(-1, 1).scale3Eq(3, 1, 1).addEq(new Vec3(0, 6, 0)), 0.3, false);
		}
	}

	// note the gear is locally y-up
	void createGear(Vec3 center, double radius, double thickness, RotationalLimitMotor lm) {
		double toothInterval = 0.4;
		double toothLength = toothInterval / 1.5;
		int numTeeth = (int) (Math.round(MathUtil.TWO_PI * radius / toothInterval) + 1);
		if (numTeeth % 2 == 0) numTeeth--;
		if (numTeeth < 2) numTeeth = 2;

		Vec3 toothPos = new Vec3(radius - toothLength / 4, 0, 0);
		Mat3 toothRot = new Mat3();
		Mat3 dtoothRot = new Mat3().appendRotationEq(MathUtil.TWO_PI / numTeeth, 0, 1, 0);

		Geometry toothGeom = createGearTooth(toothLength / 2, thickness * 0.5, toothInterval / 3);
		ShapeConfig toothSc = new ShapeConfig();
		toothSc.restitution = 0;
		toothSc.geometry = toothGeom;

		RigidBody wheel = Utils.addCylinder(world, center, radius - toothLength / 2, thickness * 0.48, false);
		for (int i=0;i<numTeeth;i++) {
			toothSc.position = toothPos;
			toothSc.rotation = toothRot;
			wheel.addShape(new Shape(toothSc));

			toothPos.mulMat3Eq(dtoothRot);
			toothRot.mulEq(dtoothRot);
		}

		wheel.rotate(new Mat3().appendRotationEq(90 * MathUtil.TO_RADIANS, 1, 0, 0));

		RigidBody fixture = Utils.addCylinder(world, center, toothInterval / 4, thickness * 0.52, true);
		fixture.rotate(new Mat3().appendRotationEq(90 * MathUtil.TO_RADIANS, 1, 0, 0));
		//SpringDamper sd= new SpringDamper();
		//sd.setSpring(3, 0.5);
		Utils.addRevoluteJoint(world, wheel, fixture, center, new Vec3(0, 0, 1),null, lm);
	}

	Geometry createGearTooth(double hw, double hh, double hd) {
		double scale = 0.3;
		Vec3[] vertices = new Vec3[] {
			new Vec3(-hw, -hh, -hd),
			new Vec3(-hw, -hh, hd),
			new Vec3(-hw, hh, -hd),
			new Vec3(-hw, hh, hd),
			new Vec3(hw, -hh, -hd * scale),
			new Vec3(hw, -hh, hd * scale),
			new Vec3(hw, hh, -hd * scale),
			new Vec3(hw, hh, hd * scale),
		};
		ConvexHullGeometry geom = new ConvexHullGeometry(vertices);
		geom.setGjkMergin(0.0); // set external margin to 0 (not needed for other geoms)
		return geom;
	}

	@Override 
	public void update() {
		super.update();
		teleportRigidBodies(-20, 10, 3, 1);
	}
}
