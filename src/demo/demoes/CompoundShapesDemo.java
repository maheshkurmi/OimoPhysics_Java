package demo.demoes;
import demo.common.DemoBase;
import demo.common.DemoRenderer;
import demo.common.Utils;
import demo.common.UserInput;
import demo.common.ViewInfo;
import oimo.collision.geometry.BoxGeometry;
import oimo.collision.geometry.ConeGeometry;
import oimo.collision.geometry.CylinderGeometry;
import oimo.collision.geometry.Geometry;
import oimo.common.MathUtil;
import oimo.common.Vec3;
import oimo.dynamics.World;
import oimo.dynamics.rigidbody.RigidBody;
import oimo.dynamics.rigidbody.RigidBodyConfig;
import oimo.dynamics.rigidbody.Shape;
import oimo.dynamics.rigidbody.ShapeConfig;

/**
 * BridgeDemo demo
 */
public class CompoundShapesDemo extends DemoBase {
	public CompoundShapesDemo() {
		super("Compund Shapes Demo");
	}

	@Override 
	public void init(World world, DemoRenderer renderer, UserInput input, ViewInfo viewInfo) {
		super.init(world, renderer, input, viewInfo);
		renderer.camera(new Vec3(0, 7, 9), new Vec3(0, 2, 0), new Vec3(0, 1, 0));

		double thickness = 0.5;	
		Utils.addBox(world, new Vec3(0, -thickness, 0), new Vec3(7, thickness, 7), true);

		int n = 32;
		RigidBodyConfig rc = new RigidBodyConfig();

		{
			Geometry geom1 = new BoxGeometry(new Vec3(0.3, 0.1, 0.3));
			Geometry geom2 = new BoxGeometry(new Vec3(0.1, 0.3, 0.1));

			ShapeConfig sc1 = new ShapeConfig();
			ShapeConfig sc2 = new ShapeConfig();
			sc1.geometry = geom1;
			sc2.geometry = geom2;
			sc1.position.set(0, 0.2, 0);
			sc2.position.set(0, -0.2, 0);


			for (int i=0;i<n;i++) {
				rc.position.set(-2, 1 + i, 0).addEq(MathUtil.randVec3In(-0.01, 0.01));
				RigidBody rb = new RigidBody(rc);
				rb.addShape(new Shape(sc1));
				rb.addShape(new Shape(sc2));
				world.addRigidBody(rb);
			}
		}

		{
			Geometry geom1 = new ConeGeometry(0.275, 0.325);
			Geometry geom2 = new BoxGeometry(new Vec3(0.3, 0.075, 0.3));

			ShapeConfig sc1 = new ShapeConfig();
			ShapeConfig sc2 = new ShapeConfig();
			sc1.geometry = geom1;
			sc2.geometry = geom2;
			sc1.position.set(0, 0.2, 0);
			sc2.position.set(0, -0.2, 0);

			for (int i=0;i<n;i++) {
				rc.position.set(0, 1 + i, 0).addEq(MathUtil.randVec3In(-0.01, 0.01));
				RigidBody rb = new RigidBody(rc);
				rb.addShape(new Shape(sc1));
				rb.addShape(new Shape(sc2));
				world.addRigidBody(rb);
			}
		}

		{
			Geometry geom1 = new CylinderGeometry(0.25, 0.4);
			Geometry geom2 = new BoxGeometry(new Vec3(0.075, 0.4, 0.075));

			ShapeConfig sc1 = new ShapeConfig();
			ShapeConfig sc2 = new ShapeConfig();
			sc1.geometry = geom1;
			sc2.geometry = geom2;
			sc1.position.set(0, 0, 0);
			sc2.position.set(0, -0.1, 0);
			sc1.rotation.appendRotationEq(90 * MathUtil.TO_RADIANS, 0, 0, 1);

			for (int i=0;i<n;i++) {
				rc.position.set(2, 1 + i, 0).addEq(MathUtil.randVec3In(-0.01, 0.01));
				RigidBody rb = new RigidBody(rc);
				rb.addShape(new Shape(sc1));
				rb.addShape(new Shape(sc2));
				world.addRigidBody(rb);
			}
		}
	}

	@Override 
	public void update() {
		super.update();
		teleportRigidBodies(-20, 10, 5, 5);
	}
}