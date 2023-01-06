package demo.demoes;
import java.util.function.Function;

import demo.common.*;
import oimo.collision.broadphase.BroadPhase;
import oimo.collision.broadphase.bvh.*;
import oimo.collision.geometry.*;
import oimo.common.*;
import oimo.dynamics.*;
import oimo.dynamics.rigidbody.*;

/**
 * Broad-phase stress demo
 */
public class BroadPhaseStressDemo extends DemoBase {
	static int RIGID_BODIES_STEP= 100;
	static int FIELD_W = 50;
	static int  FIELD_H = 50;
	static int FIELD_D = 50;

	int pairTestCount;
	int treeBalance;
	int speed = 4;

	public BroadPhaseStressDemo() {
		super("Broad-Phase Stress Test");
	}
	
	@Override 
	public void initControls(Control[] controls) {
		if(true) return;
		int n=controls.length;
		
		controls[n]=new Control("↑", 
		new Function(){
			@Override
			public Object apply(Object t) {
				return "increase shapes";
			}
			
		},
		UserInput.KEYCODE_UP, 
		new Function(){
			@Override
			public Object apply(Object t) {
				increaseRigidBodies();
				return null;
			}
			
		});
		
		controls[n+1]=new Control("↓",
		new Function(){
			@Override
			public Object apply(Object t) {
				return "decrease shapes";
			}
			
		},
		UserInput.KEYCODE_DOWN, 
		new Function(){
			@Override
			public Object apply(Object t) {
				decreaseRigidBodies();
				return null;
			}
			
		});
		
		
		controls[n+2]=new Control("→", 
		new Function(){
			@Override
			public Object apply(Object t) {
				return "increase speed";
			}
			
		},
		
		UserInput.KEYCODE_RIGHT, 
		new Function(){
			@Override
			public Object apply(Object t) {
				speed += 2;
				if (speed > 20) speed = 20;
				return null;
			}
			
		});
		
		controls[n+3]=new Control("←", 
		new Function(){
			@Override
			public Object apply(Object t) {
				return "decrease speed";
			}
			
		},
		UserInput.KEYCODE_LEFT, 
		new Function(){
			@Override
			public Object apply(Object t) {
				speed -= 2;
				if (speed < 0) speed = 0;
				return null;
			}
			
		});
		
	}

	double max(double a, double b) {
		return a > b ? a : b;
	}

	@Override 
	public void init(World world, DemoRenderer renderer,UserInput input, ViewInfo viewInfo) {
		super.init(world, renderer, input, viewInfo);
		renderer.camera(new Vec3(0, 0, max(FIELD_W, max(FIELD_H, FIELD_D)) * 3), new Vec3(), new Vec3(0, 1, 0));
		world.setGravity(new Vec3());
		for (int i=0;i<5;i++) {
			increaseRigidBodies();
		}
	}

	@Override
	public void update() {
		super.update();
		BroadPhase bp = world.getBroadPhase();
		pairTestCount = bp.getTestCount();
		if (bp instanceof BvhBroadPhase) {
			treeBalance = ((BvhBroadPhase)bp).getTreeBalance();
		} else {
			treeBalance = 0;
		}
		reflectRigidBodies();
	}

	@Override
	public String additionalInfo() {
		return
			"  broad-phase test count: $pairTestCount\n" +
			"  tree balance          : $treeBalance\n"
		;
	}

	public void increaseRigidBodies() {
		double scale = 0.7;
		int num = RIGID_BODIES_STEP;
		Geometry[] shapes = new Geometry[] {
			new BoxGeometry(new Vec3(0.4, 0.5, 0.6).scaleEq(scale)),
			new SphereGeometry(0.5 * scale)
		};
		ShapeConfig compc = new ShapeConfig();
		RigidBodyConfig rigidc = new RigidBodyConfig();
		for (int i=0;i<num;i++) {
			compc.geometry = shapes[(int) Math.round(Math.random()*1)];
			compc.position.set(MathUtil.randIn(-1, 1), MathUtil.randIn(-1, 1), MathUtil.randIn(-1, 1));
			compc.position.scaleEq(0);
			rigidc.position.set(MathUtil.randIn(-1, 1), MathUtil.randIn(-1, 1), MathUtil.randIn(-1, 1));
			rigidc.position.x *= FIELD_W;
			rigidc.position.y *= FIELD_H;
			rigidc.position.z *= FIELD_D;
			var body = new RigidBody(rigidc);
			body.addShape(new Shape(compc));
			moveRigidBody(body);
			world.addRigidBody(body);
		}
	}

	void moveRigidBody(RigidBody b) {
		double speed = this.speed;
		Vec3 v = new Vec3().set(MathUtil.randIn(-1, 1), MathUtil.randIn(-1, 1), MathUtil.randIn(-1, 1)).normalize().scaleEq(speed);
		Vec3 av = new Vec3().set(MathUtil.randIn(-1, 1), MathUtil.randIn(-1, 1), MathUtil.randIn(-1, 1)).scaleEq(5);
		b.setLinearVelocity(v);
		b.setAngularVelocity(av);
	}

	void decreaseRigidBodies() {
		int num = RIGID_BODIES_STEP;
		if (num > world.getNumRigidBodies()) {
			return;
		}
		var rb = world.getRigidBodyList();
		while (rb._next != null) {
			rb = rb._next;
		}
		for (int i=0;i<num;i++) {
			var prev = rb._prev;
			world.removeRigidBody(rb);
			rb = prev;
		}
	}

	void reflectRigidBodies() {
		RigidBody rb = world.getRigidBodyList();
		Vec3 pos = new Vec3();
		while (rb != null) {
			rb.getPositionTo(pos);
			var lv = rb.getLinearVelocity();
			if (pos.x < -FIELD_W || pos.x > FIELD_W) lv.x *= -1;
			if (pos.y < -FIELD_H || pos.y > FIELD_H) lv.y *= -1;
			if (pos.z < -FIELD_D || pos.z > FIELD_D) lv.z *= -1;
			rb.setLinearVelocity(lv);
			rb = rb._next;
		}
	}

}
