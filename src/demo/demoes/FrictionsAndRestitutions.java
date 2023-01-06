package demo.demoes;
import demo.common.DemoBase;
import demo.common.DemoRenderer;
import demo.common.OimoUtil;
import demo.common.UserInput;
import demo.common.ViewInfo;
import oimo.common.Mat3;
import oimo.common.MathUtil;
import oimo.common.Vec3;
import oimo.dynamics.World;
import oimo.dynamics.rigidbody.RigidBody;

/**
 * Friction and Restitution demo
 */
public class FrictionsAndRestitutions extends DemoBase {
	public FrictionsAndRestitutions() {
		super("Bridge Demo");
	}

	@Override 
	public void init(World world, DemoRenderer renderer, UserInput input, ViewInfo viewInfo) {
		super.init(world, renderer, input, viewInfo);
		super.init(world, renderer, input, viewInfo);
		renderer.camera(new Vec3(0, 7, 9), new Vec3(0, 2, 0), new Vec3(0, 1, 0));

		double thickness = 0.5;
		OimoUtil.addBox(world, new Vec3(0, -thickness, 0), new Vec3(7, thickness, 7), true);

		Mat3 rotMat = new Mat3().appendRotationEq(20 * MathUtil.TO_RADIANS, 0, 0, 1);
		RigidBody tiltedFloor = OimoUtil.addBox(world, new Vec3(0, 2, 0), new Vec3(3, 0.1, 1), true);
		tiltedFloor.rotate(rotMat);
		tiltedFloor.getShapeList().setFriction(0.5);

		for (int i=0;i<7;i++) {
			Vec3 pos = new Vec3((i - 3) * 0.8, 0, 0);
			pos.mulMat3Eq(rotMat);
			pos.y += 2.3;
			RigidBody box = OimoUtil.addBox(world, pos, new Vec3(0.2, 0.2, 0.2), false);
			box.getShapeList().setFriction(i / 16.0);
			box.rotate(rotMat);
		}

		RigidBody bouncyFloor = OimoUtil.addBox(world, new Vec3(0, 0.1, 2), new Vec3(3, 0.1, 1), true);
		bouncyFloor.getShapeList().setRestitution(1.0);

		for (int i=0;i<7;i++) {
			Vec3 pos = new Vec3((i - 3) * 0.8, 3, 2);
			OimoUtil.addSphere(world, pos, 0.25, false).getShapeList().setRestitution(i / 6.0);
		}
	}

	@Override 
	public void update() {
		super.update();
		teleportRigidBodies(-20, 10, 5, 5);
	}
}