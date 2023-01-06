package demo.demoes;
import demo.common.*;
import oimo.collision.geometry.*;
import oimo.common.*;
import oimo.dynamics.*;
import oimo.dynamics.rigidbody.*;
import oimo.dynamics.constraint.joint.Joint;

/**
 * Breakable joint demo
 */
public class BreakableJointDemo extends DemoBase {
	public BreakableJointDemo() {
		super("Breakable Joint");
	}

	@Override 
	public void init(World world, DemoRenderer renderer, UserInput input, ViewInfo viewInfo) {
		super.init(world, renderer, input, viewInfo);
		renderer.camera(new Vec3(0, 7, 9), new Vec3(0, 2, 0), new Vec3(0, 1, 0));

		double thickness = 0.5;
		OimoUtil.addBox(world, new Vec3(0, -thickness, 0), new Vec3(7, thickness, 7), true);

		RigidBody[] chain= new RigidBody[13];
		chain[0]=OimoUtil.addSphere(world, new Vec3(0, 6, 0), 0.3, true);
		for (int i=0;i<12;i++) {
			if (i % 2 == 0) {
				chain[i+1]=OimoUtil.addSphere(world, new Vec3((i + 1) * 0.4, 6, 0), 0.25, false);
			} else {
				chain[i+1]=OimoUtil.addBox(world, new Vec3((i + 1) * 0.4, 6, 0), new Vec3(0.25, 0.25, 0.25), false);
			}
			chain[i+1].setLinearVelocity(MathUtil.randVec3().scaleEq(0.05));
		}

		for (int i=1;i<chain.length;i++) {
			Vec3 center;

			if (i == 1) {
				center = chain[0].getPosition();
			} else {
				center = chain[i - 1].getPosition().addEq(chain[i].getPosition()).scaleEq(0.5);
			}
			Joint joint = OimoUtil.addSphericalJoint(world, chain[i - 1], chain[i], center);
			joint.setBreakForce(100);
		}
	}

	@Override
	public void update() {
		super.update();
		teleportRigidBodies(-20, 10, 5, 5);
	}
}
