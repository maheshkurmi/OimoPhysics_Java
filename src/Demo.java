import java.util.ArrayList;

import demo.common.OimoUtil;
import oimo.common.MathUtil;
import oimo.common.Vec3;
import oimo.dynamics.World;
import oimo.dynamics.rigidbody.RigidBody;

public class Demo {
	World world;
	ArrayList<RigidBody> bodies=new ArrayList<RigidBody>() ;
	public Demo() {
		world = new World(0, new Vec3(0,-10,0));
		double thickness = 0.5f;
		OimoUtil.addBox(world, new Vec3(0, -thickness, 0), new Vec3(7, thickness, 7), true);

		int w = 2;
		int h = 2;
		double sp = 0.61f;
		int n = 0;
		double size = 0.3f;
		//RigidBody box = OimoUtil.addBox(world, new Vec3(0,5f,0), new Vec3(size, size, size), false);
		//box.setAngularVelocity(MathUtil.randVec3In(-0.05f, 0.05f));
		//box.userData="Box";
		//bodies.add(box);
		
		RigidBody sphere = OimoUtil.addSphere(world, new Vec3(-1,4f,0), size/2, false);
		sphere.userData="sphere";
		bodies.add(sphere);
		
		for (int i=0;i<n;i++) {
			for (int j=-w;j<w;j++) {
				for (int k=-h;k<h;k++) {
					Vec3 pos = new Vec3(j * sp, size + i * size * 3, k * sp);
					RigidBody box = OimoUtil.addBox(world, pos, new Vec3(size, size, size), false);
					box.setAngularVelocity(MathUtil.randVec3In(-0.05f, 0.05f));
					bodies.add(box);
				}
			}
		}
	}
	
	
	public void update() throws InterruptedException {
		
		while (true) {
			Thread.sleep(200);
			world.step(0.02f);
			for(RigidBody b:bodies) {
				System.out.println(b.userData+","+b.getLinearVelocity()+","+b.getPosition()+"");
				if(b.getPosition().y<-3)return;
			}
        }
		
		
	}
	
	public static void main(String[] args) {
		Demo demo= new Demo();
		try {
			demo.update();
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

}
