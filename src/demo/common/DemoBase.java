package demo.common;
import oimo.collision.geometry.*;
import oimo.common.DebugDraw;
import oimo.common.Mat4;
import oimo.common.MathUtil;
import oimo.common.Vec3;
import oimo.dynamics.*;
import oimo.dynamics.callback.RayCastClosest;
import oimo.dynamics.constraint.joint.*;
import oimo.dynamics.rigidbody.*;

/**
 * Base class of demos.
 */
public class DemoBase {
	public String demoName;
	public double dt;
	protected World world;
	protected DemoRenderer renderer;
	UserInput input;
	int count;
	ViewInfo viewInfo;
	double grabbingDistance;
	RigidBody mouseJointDummyBody;
	SphericalJoint mouseJoint;

	public DemoBase(String demoName) {
		this.demoName = demoName;
		count = 0;
	}

	
	public void init(World world, DemoRenderer renderer, UserInput input, ViewInfo viewInfo) {
		this.world = world;
		this.renderer = renderer;
		this.input = input;
		this.viewInfo = viewInfo;
		renderer.camera(new Vec3(0, 0, 10), new Vec3(), new Vec3(0, 1, 0));
		RigidBodyConfig rigidBodyConfig = new RigidBodyConfig();
		rigidBodyConfig.type = RigidBodyType.STATIC;
		mouseJointDummyBody = new RigidBody(rigidBodyConfig);
		mouseJoint = null;
		dt = 1 / 60.0;
	}

	public void initControls(Control[] controls) {
	}

	public void teleportRigidBodies(double thresholdY, double toY, double rangeX, double rangeZ) {
		RigidBody rb = world.getRigidBodyList();
		Vec3 pos = new Vec3();
		Vec3 zero = new Vec3();
		while (rb != null) {
			rb.getPositionTo(pos);
			if (pos.y < thresholdY) {
				pos.y = toY;
				pos.x = MathUtil.randIn(-1, 1) * rangeX;
				pos.z = MathUtil.randIn(-1, 1) * rangeZ;
				rb.setPosition(pos);
				rb.setLinearVelocity(zero);
			}
			rb = rb._next;
		}
	}

	public void update() {
		count++;
		updateMouseJoint();
	}

	public void drawAdditionalObjects(DebugDraw debugDraw) {
	}

	void updateMouseJoint() {
		Vec3 cameraPos = renderer.getCameraPosition(); // camera

		double screenX = input.mouseX / viewInfo.width - 0.5;
		double screenY = 0.5 - input.mouseY / viewInfo.height;

		Vec3 screenPos = new Vec3(screenX * viewInfo.screenWidth, screenY * viewInfo.screenHeight, -viewInfo.screenDistance);

		Mat4 viewMat = renderer.getViewMatrix();
		viewMat.transposeEq();
		viewMat.e03 = 0; // remove translations
		viewMat.e13 = 0;
		viewMat.e23 = 0;
		viewMat.e33 = 0;
		screenPos.mulMat4Eq(viewMat).normalize();

		if (mouseJoint != null) {
			if (input.mouseL) {
				//var t:Float = grabbingDistance / screenPos.z;
				mouseJointDummyBody.setPosition(cameraPos.add(screenPos.scale(grabbingDistance)));
				mouseJoint.getRigidBody1().wakeUp();
				mouseJoint.getRigidBody2().wakeUp();
			} else {
				world.removeJoint(mouseJoint);
				mouseJoint = null;
			}
		} else {
			if (input.mouseL && !input.pmouseL) { // clicked
				// ray casting
				Vec3 end = cameraPos.add(screenPos.scale(500));

				RayCastClosest closest = new RayCastClosest();
				world.rayCast(cameraPos, end, closest);

				if (!closest.hit) return;

				RigidBody body = closest.shape.getRigidBody();
				Vec3 position = closest.position;

				if (body == null || body.getType() != RigidBodyType.DYNAMIC) return;

				SphericalJointConfig jc = new SphericalJointConfig();
				jc.springDamper.frequency = 4.0;
				jc.springDamper.dampingRatio = 1;
				jc.rigidBody1 = body;
				jc.rigidBody2 = mouseJointDummyBody;
				jc.allowCollision = false;
				jc.localAnchor1 = position.sub(body.getPosition());
				jc.localAnchor1.mulMat3Eq(body.getRotation().transposeEq());
				jc.localAnchor2.zero();
				mouseJointDummyBody.setPosition(position);
				mouseJoint = new SphericalJoint(jc);
				world.addJoint(mouseJoint);
				grabbingDistance = position.sub(cameraPos).length();
			}
		}
	}

	public void draw() {
		renderer.draw(this);
	}

	public String additionalInfo() {
		return "";
	}
}