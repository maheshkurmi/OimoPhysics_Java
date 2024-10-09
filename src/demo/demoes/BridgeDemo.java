package demo.demoes;
import demo.common.DemoBase;
import demo.common.DemoRenderer;
import demo.common.Utils;
import demo.common.UserInput;
import demo.common.ViewInfo;
import oimo.common.MathUtil;
import oimo.common.Vec3;
import oimo.dynamics.World;
import oimo.dynamics.rigidbody.RigidBody;

/**
 * BridgeDemo demo
 */
public class BridgeDemo extends DemoBase {
	public BridgeDemo() {
		super("Bridge Demo");
	}

	@Override 
	public void init(World world, DemoRenderer renderer, UserInput input, ViewInfo viewInfo) {
		super.init(world, renderer, input, viewInfo);

		renderer.camera(new Vec3(0, 8, 12), new Vec3(0, 2, 0), new Vec3(0, 1, 0));

		for (int i=0;i<5;i++) {
			Utils.addSphere(world, new Vec3(MathUtil.randIn(-4, 4), MathUtil.randIn(2, 3), MathUtil.randIn(-1, 1)), 0.8, false).getShapeList().setDensity(0.3);
			Utils.addBox(world, new Vec3(MathUtil.randIn(-4, 4), MathUtil.randIn(2, 3), MathUtil.randIn(-1, 1)), new Vec3(0.5, 0.5, 0.5), false).getShapeList().setDensity(0.3);
			Utils.addCone(world, new Vec3(MathUtil.randIn(-4, 4), MathUtil.randIn(2, 3), MathUtil.randIn(-1, 1)), 0.6, 0.6, false).getShapeList().setDensity(0.3);
		}

		int num = 20;
		double width = 3.0;
		double length = 0.7;
		double gap = 0.05;
		double height = 0.3;
		Vec3 dir = new Vec3(width, 0, 0);

		RigidBody[] bodies =new RigidBody[num];
		for (int i=0;i<num;i++) {
			double x = (i - (num - 1) * 0.5) * (length + gap);
			bodies[i]=Utils.addBox(world, new Vec3(x, 0, 0), new Vec3(length * 0.5, height * 0.5, width * 0.5), i == 0 || i == num - 1);
		}

		for (int i=0;i<num - 1;i++) {
			Utils.addRevoluteJoint(world, bodies[i], bodies[i + 1], bodies[i].getPosition().add(bodies[i + 1].getPosition()).scale(0.5), new Vec3(0, 0, 1),null,null);
		}

		for (int i=0;i<num;i++) {
			Vec3 newPos = bodies[i].getPosition();
			newPos.x *= 0.95;
			bodies[i].setPosition(newPos);
		}
	}

	@Override 
	public void update() {
		super.update();
		teleportRigidBodies(-20, 10, 5, 0);
	}
}