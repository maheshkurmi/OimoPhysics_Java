package demo.demoes;
import demo.common.*;
import oimo.collision.geometry.*;
import oimo.common.*;
import oimo.dynamics.*;
import oimo.dynamics.rigidbody.*;

/**
 * Falling ragdoll demo
 */
public class FallingRagdollDemo extends DemoBase {
	RigidBody ragdoll;

	public FallingRagdollDemo() {
		super("Falling Ragdoll");
	}

	@Override 
	public void init(World world, DemoRenderer renderer, UserInput input, ViewInfo viewInfo) {
		super.init(world, renderer, input, viewInfo);
		renderer.camera(new Vec3(2, 7, 9), new Vec3(0, 2, 0), new Vec3(0, 1, 0));

		renderer.getGraphics().getDebugDraw().drawJointLimits = false;

		double tmp1 = Setting.defaultRestitution;
		double tmp2 = Setting.defaultFriction;

		Setting.defaultRestitution = 0.3;
		Setting.defaultFriction = 0.2;

		double thickness = 0.5;
		Utils.addBox(world, new Vec3(0, -thickness, 0), new Vec3(7, thickness, 7), true);

		int n; // number of steps
		double height; // height of a step
		double length; // length of a step
		double width; // width of a step
		n = 30;
		width = 4.0;
		height = 0.5;
		length = 0.6;
		Vec3 stairCenter = new Vec3(0, 0, 0);

		for (int i=0;i<n;i++) {
			if (i == n - 1) {
				Utils.addBox(world, stairCenter.add(new Vec3(0, (i + 0.5) * height, -(i + 4 * 0.5) * length)), new Vec3(width * 0.5, height * 0.5, length * 4 * 0.5), true);
			} else {
				Utils.addBox(world, stairCenter.add(new Vec3(0, (i + 0.5) * height, -(i + 0.5) * length)), new Vec3(width * 0.5, height * 0.5, length * 0.5), true);
				Utils.addBox(world, stairCenter.add(new Vec3(-width * 0.5, (i + 2 * 0.5) * height, -(i + 0.5) * length)), new Vec3(0.1, height * 2 * 0.5, length * 0.5), true);
				Utils.addBox(world, stairCenter.add(new Vec3(width * 0.5, (i + 2 * 0.5) * height, -(i + 0.5) * length)), new Vec3(0.1, height * 2 * 0.5, length * 0.5), true);
			}
		}

		Vec3 ragdollPos = stairCenter.add(new Vec3(0, (n - 0.5) * height + 1.46, -(n - 0.5) * length));
		ragdoll = Utils.addRagdoll(world, ragdollPos);
		Utils.addBox(world, ragdollPos.add(new Vec3(0, 0, -2)), new Vec3(0.2, 0.2, 0.2), false).setLinearVelocity(new Vec3(0, 3.5, 4));

		Setting.defaultRestitution = tmp1;
		Setting.defaultFriction = tmp2;
	}

	@Override
	public void update() {
		super.update();

		if (ragdoll != null) {
			renderer.camera(ragdoll.getPosition().add(new Vec3(2, 5, 7)), ragdoll.getPosition(), new Vec3(0, 1, 0));
		}
	}
}
