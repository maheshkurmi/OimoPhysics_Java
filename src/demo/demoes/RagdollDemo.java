package demo.demoes;
import demo.common.DemoRenderer;
import demo.common.DemoBase;
import demo.common.OimoUtil;
import demo.common.UserInput;
import demo.common.ViewInfo;
import oimo.collision.geometry.*;
import oimo.common.Mat3;
import oimo.common.MathUtil;
import oimo.common.Setting;
import oimo.common.Vec3;
import oimo.dynamics.*;
import oimo.dynamics.constraint.joint.*;
import oimo.dynamics.rigidbody.*;

/**
 * Ragdoll demo
 */
public class RagdollDemo extends DemoBase {

	public RagdollDemo() {
		super("Ragdolls");
	}

	@Override 
	public void init(World world, DemoRenderer renderer, UserInput input, ViewInfo viewInfo) {
		super.init(world, renderer, input, viewInfo);
		renderer.camera(new Vec3(0, 5, 6), new Vec3(0, 2, 0), new Vec3(0, 1, 0));

		renderer.getGraphics().getDebugDraw().drawJointLimits = false;

		OimoUtil.addBox(world, new Vec3(0, -0.2, 0), new Vec3(6, 0.2, 6), true);

		var tmp = Setting.defaultFriction;
		Setting.defaultFriction = 0.5;

		for (int i=0;i<10;i++) {
			OimoUtil.addRagdoll(world, new Vec3(0, 2 + i * 2, 0));
		}

		Setting.defaultFriction = tmp;
	}

	@Override 
	public void update() {
		super.update();
	}
}
