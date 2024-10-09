package demo.demoes;
import demo.common.*;
import oimo.common.*;
import oimo.dynamics.*;
import oimo.dynamics.rigidbody.*;

/**
 * Collision filtering demo
 */
public class CollisionFilteringDemo extends DemoBase {
	// collision groups
	static int G_FLOOR = 1;
	static  int G_WALL = 2;
	static int G_BALL = 4;
	static int G_BOX = 8;

	public CollisionFilteringDemo() {
		super("Collision Filtering");
	}

	@Override 
	public void init(World world, DemoRenderer renderer, UserInput input, ViewInfo viewInfo) {
		super.init(world, renderer, input, viewInfo);
		renderer.camera(new Vec3(0, 7, 9), new Vec3(0, 2, 0), new Vec3(0, 1, 0));

		double thickness = 0.5;
		Shape floorShape = Utils.addBox(world, new Vec3(0, -thickness, 0), new Vec3(7, thickness, 7), true).getShapeList();
		floorShape.setCollisionGroup(G_FLOOR); // belongs to group FLOOR
		floorShape.setCollisionMask(G_BOX);    // collides to group BOX

		Shape wallShape = Utils.addBox(world, new Vec3(0, 2, 0), new Vec3(3, 0.2, 3), true).getShapeList();
		wallShape.setCollisionGroup(G_WALL); // belongs to group WALL
		wallShape.setCollisionMask(G_BALL);  // collides to group BALL

		int w = 2;
		int h = 2;
		int n = 2;
		double size = 0.3;
		for (int i=0;i<n;i++) {
			for (int j=-w;j<w+1;j++) {
				for (int k=-h;k<h+1;k++) {
					Vec3 pos = new Vec3(j * size * 3, 3 + i * size * 3, k * size * 3);
					pos.addEq(MathUtil.randVec3In(-0.01, 0.01));
					Shape shape;
					if (i == 0) {
						shape = Utils.addSphere(world, pos, size, false).getShapeList();
						shape.setCollisionGroup(G_BALL);                 // belongs to group BALL
						shape.setCollisionMask(G_WALL | G_BALL | G_BOX); // collides to group WALL, BALL and BOX
					} else {
						shape = Utils.addBox(world, pos, new Vec3(size, size, size), false).getShapeList();
						shape.setCollisionGroup(G_BOX);                   // belongs to group BOX
						shape.setCollisionMask(G_FLOOR | G_BALL | G_BOX); // collides to group FLOOR, BALL and BOX
					}
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
