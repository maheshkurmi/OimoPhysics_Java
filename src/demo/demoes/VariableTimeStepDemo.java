package demo.demoes;
import demo.common.OimoUtil;
import oimo.collision.geometry.*;
import oimo.common.*;
import oimo.dynamics.*;
import oimo.dynamics.rigidbody.*;
import demo.common.Control;
import demo.common.DemoRenderer;
import demo.common.DemoBase;
import demo.common.UserInput;
import demo.common.ViewInfo;

/**
 * Variable time step demo
 */
public class VariableTimeStepDemo extends DemoBase {
	RigidBody bullet;

	public VariableTimeStepDemo() {
		super("Variable Time Step");
	}

	@Override 
	public void init(World world, DemoRenderer renderer, UserInput input, ViewInfo viewInfo) {
		super.init(world, renderer, input, viewInfo);
		renderer.camera(new Vec3(-5, 7, 9), new Vec3(0, 2, 0), new Vec3(0, 1, 0));

		double thickness = 0.5;
		OimoUtil.addBox(world, new Vec3(0, -thickness, 0), new Vec3(5, thickness, 5), true);

		int w = 5;
		int h = 1;
		int n = 5;
		double wid = 0.3;
		double hei = 0.3;
		double dep = 0.3;
		for (int i=0;i<n;i++) {
			for (int k=-h;k<h+1;k++) {
				for (int j=0;j<w;j++) {
					if ((j + k & 1) == 0) OimoUtil.addBox(world, new Vec3(j * wid * 2 + MathUtil.randIn(-0.01, 0.01), hei + i * hei * 2.2, k * dep * 2 + MathUtil.randIn(-0.01, 0.01)), new Vec3(wid, hei, dep), false);
					else OimoUtil.addCylinder(world, new Vec3(j * wid * 2 + MathUtil.randIn(-0.01, 0.01), hei + i * hei * 2.2, k * dep * 2 + MathUtil.randIn(-0.01, 0.01)), wid, hei, false);
				}
			}
		}

		{
			RigidBody b = OimoUtil.addBox(world, new Vec3(-4, 4, -4), new Vec3(0.5, 0.5, 0.5), false);
			b.setLinearVelocity(new Vec3(5, 0, 4));
			b.setAngularVelocity(new Vec3(3, 6, 8));
		}

		double bulletSize = hei;
		bullet = OimoUtil.addCone(world, new Vec3(-150, 3, 0), bulletSize * 1.4, bulletSize * 1.5, false);
		ShapeConfig sc = new ShapeConfig();
		sc.geometry = new BoxGeometry(new Vec3(0.4, 1, 0.4).scale(bulletSize));
		sc.position.y -= bulletSize * 1.5 + bulletSize;
		
		bullet.addShape(new Shape(sc));
		bullet.rotate(new Mat3().appendRotation(MathUtil.HALF_PI, 0, 0, -1));
		bullet.getShapeList().setDensity(50);
		bullet.setLinearVelocity(new Vec3(300, 0, 0));
		bullet.setAngularVelocity(new Vec3(MathUtil.TWO_PI * 100, 0, 0));
	}

	@Override
	public void update() {
		double timeStep = MathUtil.abs(0 - bullet.getPosition().x) / 8000;
		if (bullet.getPosition().x > 0) timeStep *= 10;

		double maxTimeStep = 1 / 60.0;
		if (bullet.getPosition().x < -10) maxTimeStep = 1 / 180.0;

		if (timeStep < 1 / 10000.0) timeStep = 1 / 10000.0;
		if (timeStep > maxTimeStep) timeStep = maxTimeStep;

		this.dt = timeStep;

		super.update();
	}
}
