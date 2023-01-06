package demo.demoes;
import demo.common.*;
import oimo.collision.geometry.*;
import oimo.common.*;
import oimo.dynamics.*;
import oimo.dynamics.rigidbody.*;

/**
 * Rotation limit demo
 */
public class LimitRotationDemo extends DemoBase {
	RigidBody body;

	public LimitRotationDemo() {
		super("Rotation Limit");
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
			for (int j=-w;j<w+1;j++) {
				for (int k=-h;k<h+1;k++) {
					Vec3 pos = new Vec3(j * sp, size + i * size * 3, k * sp);
					RigidBody box = OimoUtil.addBox(world, pos, new Vec3(size, size, size), false);
				//	box.setRotationFactor(new Vec3(0, 0, 0));
				}
			}
		}

		RigidBody cylinder = OimoUtil.addCylinder(world, new Vec3(0, 8, 0), 1.0, 0.3, false);
		Shape cylinderShape = cylinder.getShapeList();

		// modify local transform
		Transform localTransform = cylinderShape.getLocalTransform();
		localTransform.rotateXyz(new Vec3(MathUtil.HALF_PI, 0, 0));
		cylinderShape.setLocalTransform(localTransform);

		// limit rotation
		cylinder.setRotationFactor(new Vec3(0, 0, 1));

		body = cylinder;
	}

	@Override 
	public void update() {
		super.update();
		teleportRigidBodies(-20, 10, 5, 5);
	}
}
