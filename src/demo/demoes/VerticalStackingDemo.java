package demo.demoes;
import demo.common.DemoBase;
import demo.common.DemoRenderer;
import demo.common.Utils;
import demo.common.UserInput;
import demo.common.ViewInfo;
import oimo.collision.geometry.*;
import oimo.common.*;
import oimo.dynamics.*;
import oimo.dynamics.rigidbody.*;

/**
 * Vertical stacking demo
 */
public class VerticalStackingDemo extends DemoBase {
	public VerticalStackingDemo() {
		super("Vertical Stacking");
	}

	@Override 
	public void init(World world, DemoRenderer renderer, UserInput input, ViewInfo viewInfo) {
		super.init(world, renderer, input, viewInfo);
		renderer.camera(new Vec3(0, 16, 16), new Vec3(0, 2, 0), new Vec3(0, 1, 0));

		double thickness = 0.5;
		Utils.addBox(world, new Vec3(0, -thickness, 0), new Vec3(10, thickness, 6), true);

		int w = 2;
		double sp = 3;
		int n = 6;
		int dn = 2;
		double size = 0.4;

		double tmp = Setting.defaultRestitution;
		Setting.defaultRestitution = 0;

		for (int i=-w;i<w+1;i++) {
			for (int j=0;j<n + dn * (i + w);j++) {
				Utils.addBox(world, new Vec3(i * sp + MathUtil.randIn(-0.01, 0.01), size + j * size * 2.2, MathUtil.randIn(-0.01, 0.01)), new Vec3(size, size, size), false);
			}
		}

		Setting.defaultRestitution = tmp;
	}

	@Override
	public void update() {
		super.update();
		teleportRigidBodies(-20, 10, 5, 5);
	}
}
