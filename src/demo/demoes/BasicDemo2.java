package demo.demoes;

import demo.common.DemoBase;
import demo.common.DemoRenderer;
import demo.common.OimoUtil;
import demo.common.UserInput;
import demo.common.ViewInfo;
import oimo.common.MathUtil;
import oimo.common.Vec3;
import oimo.dynamics.World;
import oimo.dynamics.rigidbody.RigidBody;

/**
 * Basic demo
 */
public class BasicDemo2 extends DemoBase {
	public BasicDemo2() {
		super("Basic Demo");
	}

	@Override 
	public void init(World world, DemoRenderer renderer, UserInput input, ViewInfo viewInfo) {
		super.init(world, renderer, input, viewInfo);
		renderer.camera(new Vec3(0, 7, 9), new Vec3(0, 2, 0), new Vec3(0, 1, 0));

		double thickness = 0.5;
		OimoUtil.addBox(world, new Vec3(0, -thickness, 0), new Vec3(7, thickness, 7), true);

		int w = 2;
		int h = 2;
		double sp = 0.61;
		int n = 5;
		double size = 0.3;
		for (int i=0;i<n;i++) {
			for (int j=-w;j<w + 1;j++) {
				for (int k=-h;k<h+1;k++) {
					Vec3 pos = new Vec3(j * sp, size + i * size * 3, k * sp);
					RigidBody box = OimoUtil.addBox(world, pos, new Vec3(size, size, size), false);
					box.setAngularVelocity(MathUtil.randVec3In(-0.05, 0.05));
				}
			}
		}
	}

	@Override 
	public void update() {
		super.update();
		teleportRigidBodies(-20, 10, 5, 5);
	}
}