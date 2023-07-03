package demo.common;
import java.util.function.Function;

import demo.demoes.BasicDemo;
import demo.demoes.BreakableJointDemo;
import demo.demoes.BridgeDemo;
import demo.demoes.BroadPhaseStressDemo;
import demo.demoes.CollisionFilteringDemo;
import demo.demoes.CompoundShapesDemo;
import demo.demoes.ConvexCastingDemo;
import demo.demoes.ConvexHullDemo;
import demo.demoes.FallingRagdollDemo;
import demo.demoes.FrictionsAndRestitutions;
import demo.demoes.GearsDemo;
import demo.demoes.JointsDemo;
import demo.demoes.LimitRotationDemo;
import demo.demoes.RagdollDemo;
import demo.demoes.RayCastingDemo;
import demo.demoes.SpringsDemo;
import demo.demoes.VariableTimeStepDemo;
import demo.demoes.VerticalStackingDemo;
import oimo.common.DebugDraw;
import oimo.common.DebugDrawStyle;
import oimo.common.Mat3;
import oimo.common.MathUtil;
import oimo.common.Performance;
import oimo.common.Quat;
import oimo.common.Setting;
import oimo.common.Vec3;
import oimo.dynamics.*;

/**
 * OimoPhysics demo box.
 */
public class DemoMain {
	/**
	 * Information text to be drawn.
	 */
	public String text;

	double nmouseX;
	double nmouseY;
	boolean nmouseL;
	boolean nmouseR;
	int nmouseScroll;

	boolean[] nkeyboard;

	DemoBase currentDemo;
	int currentDemoIndex;

	World world;
	DemoRenderer renderer;
	IDemoGraphics g;

	UserInput input;
	ViewInfo viewInfo;

	int width;
	int height;

	DemoBase[] demos;

	int count;

	boolean pause;
	boolean singleStep;

	Control[] controls;

	int fpsCount;
	double fpsTime;
	int fps;
	Performance performance;
	public DemoMain(int width, int height) {
			input = new UserInput();
		viewInfo = new ViewInfo();

		nmouseX = 0;
		nmouseY = 0;
		fpsTime = System.currentTimeMillis()/1000.0;
		fpsCount = 0;
		fps = 0;
		pause = false;
		singleStep = false;
		nkeyboard = new boolean[UserInput.KEYBOARD_LENGTH];
		for (int i=0;i<UserInput.KEYBOARD_LENGTH;i++) {
			nkeyboard[i] = false;
		}

		text = "";
		this.width = width;
		this.height = height;
		createDemos();
	}

	public void setSize(int width,int height) {
		this.width = width;
		this.height = height;
		double aspectRatio = (double)width / height;

		double fov = 60 * MathUtil.TO_RADIANS;
		double fovY = Math.max(fov, Math.atan(Math.tan(fov / 2) / aspectRatio) * 2);

		renderer.perspective(fovY, aspectRatio);

		// set viewport infomation for ray casting
		viewInfo.width = width;
		viewInfo.height = height;
		viewInfo.screenDistance = 1;
		viewInfo.screenHeight = 2 * MathUtil.tan(fovY * 0.5);
		viewInfo.screenWidth = viewInfo.screenHeight * aspectRatio;
	}
	
	void createDemos() {
		demos = new DemoBase[] {
			new BasicDemo(),
			new CompoundShapesDemo(),
			new FrictionsAndRestitutions(),
			//new GearsDemo(),
			//new JointsDemo(),
			new CollisionFilteringDemo(),
			new ConvexHullDemo(),
			new VerticalStackingDemo(),
			new LimitRotationDemo(),
			new BroadPhaseStressDemo(),
			new BridgeDemo(),
			new SpringsDemo(),
			new JointsDemo(),
			new BreakableJointDemo(),
			new RagdollDemo(),
			new FallingRagdollDemo(),
			new VariableTimeStepDemo(),
			new GearsDemo(),
			new RayCastingDemo(),
			new ConvexCastingDemo(),
			
		};
		currentDemoIndex = 0;
		currentDemo = demos[0];
	}

	public void init(IDemoGraphics graphics) {
		g = graphics;
		renderer = new DemoRenderer(null, g);
		setSize(width,height);
		initDemo();
	}

	void initDemo() {
		world = new World(2,new Vec3(0,-10,0));
		performance=world.performance;
		renderer.setWorld(world);
		renderer.getGraphics().getDebugDraw().style = new DebugDrawStyle(); // reset style
		initControls();
		currentDemo.init(world, renderer, input, viewInfo);
		count = 0;

	}

	void initControls() {
		DebugDraw dd = g.getDebugDraw();

		//controls = new Control[11];
		controls = new Control[]{
			new Control(
				"Q",
				new Function(){
					@Override
					public Object apply(Object t) {
						return "previous demo";
					}
					
				},
				"Q".charAt(0),
				new Function() {
					@Override
					public Object apply(Object t) {
						changeDemo(-1);
						return null;
					}
				}
			),
			new Control(
				"E",
				new Function(){
					@Override
					public Object apply(Object t) {
						return "next demo";
					}
					
				},
				"E".charAt(0),
				new Function() {
					@Override
					public Object apply(Object t) {
						changeDemo(1);
						return null;
					}
				}
			),
			new Control(
				"ENTER",
				new Function(){
					@Override
					public Object apply(Object t) {
						return pause ? "resume" : "pause";
					}
					
				},
				UserInput.KEYCODE_ENTER,
				new Function() {
					@Override
					public Object apply(Object t) {
						pause = !pause;
						return null;
					}
				}
			),
			new Control(
				"P",
				new Function(){
					@Override
					public Object apply(Object t) {
						return pause ? "compute single step" : "---";
					}
					
				},
				"P".charAt(0),
				new Function() {
					@Override
					public Object apply(Object t) {
						if (pause) singleStep = true;
						return null;
					}
				}
			),
			new Control(
				"W",
				new Function(){
					@Override
					public Object apply(Object t) {
						return (dd.wireframe ? "solid" : "wireframe") + " mode";
					}
					
				},
				"W".charAt(0),
				new Function() {
					@Override
					public Object apply(Object t) {
						dd.wireframe = !dd.wireframe;
						return null;
					}
				}
			),
			new Control(
				"L",
				new Function(){
					@Override
					public Object apply(Object t) {
						return (Setting.disableSleeping ? "enable" : "disable") + " sleeping";
					}
					
				},
				"L".charAt(0),
				new Function() {
					@Override
					public Object apply(Object t) {
						Setting.disableSleeping = !Setting.disableSleeping;
						return null;
					}
				}
			),
			new Control(
				"R",
				new Function(){
					@Override
					public Object apply(Object t) {
						return (dd.drawPairs ? "hide" : "draw") + " pairs";
					}
					
				},
				"R".charAt(0),
				new Function() {
					@Override
					public Object apply(Object t) {
						dd.drawPairs = !dd.drawPairs;
						return null;
					}
				}
			),
			new Control(
				"V",
				new Function(){
					@Override
					public Object apply(Object t) {
						return (dd.drawAabbs ? "hide" : "draw") + " AABBs";
					}
					
				},
				"V".charAt(0),
				new Function() {
					@Override
					public Object apply(Object t) {
						dd.drawAabbs = !dd.drawAabbs;
						return null;
					}
				}
			),
			new Control(
				"B",
				new Function(){
					@Override
					public Object apply(Object t) {
						return (dd.drawBases ? "hide" : "draw") + " bases";
					}
					
				},
				"B".charAt(0),
				new Function() {
					@Override
					public Object apply(Object t) {
						dd.drawBases = !dd.drawBases;
						return null;
					}
				}
			),
			new Control(
				"C",
				new Function(){
					@Override
					public Object apply(Object t) {
						return (dd.drawContacts ? "hide" : "draw") + " contacts";
					}
					
				},
				"C".charAt(0),
				new Function() {
					@Override
					public Object apply(Object t) {
						dd.drawContacts = !dd.drawContacts;
						return null;
					}
				}
			),
			new Control(
				"J",
				new Function(){
					@Override
					public Object apply(Object t) {
						return (dd.drawJoints ? "hide" : "draw") + " joints";
					}
					
				},
				"J".charAt(0),
				new Function() {
					@Override
					public Object apply(Object t) {
						dd.drawJoints = !dd.drawJoints;
						return null;
					}
				}
			),
			new Control(
				"K",
				new Function(){
					@Override
					public Object apply(Object t) {
						return (dd.drawJointLimits ? "hide" : "draw") + " joint limits";
					}
					
				},
				"K".charAt(0),
				new Function() {
					@Override
					public Object apply(Object t) {
						dd.drawJointLimits = !dd.drawJointLimits;
						return null;
					}
				}
			),
			new Control(
				"A",
				new Function(){
					@Override
					public Object apply(Object t) {
						return (dd.drawContactBases ? "hide" : "draw") + " contact bases";
					}
					
				},
				"A".charAt(0),
				new Function() {
					@Override
					public Object apply(Object t) {
						dd.drawContactBases = !dd.drawContactBases;
						return null;
					}
				}
			),
			new Control(
				"T",
				new Function(){
					@Override
					public Object apply(Object t) {
						return (dd.drawBvh ? "hide" : "draw") + " AABB tree";
					}
					
				},
				"T".charAt(0),
				new Function() {
					@Override
					public Object apply(Object t) {
						dd.drawBvh = !dd.drawBvh;
						return null;
					}
				}
			),
			new Control(
				"S",
				new Function(){
					@Override
					public Object apply(Object t) {
						return (dd.drawShapes ? "hide" : "draw") + " shapes";
								
					}
					
				},
				"S".charAt(0),
				new Function() {
					@Override
					public Object apply(Object t) {
						dd.drawShapes = !dd.drawShapes;
						return null;
					}
				}
			)
		};
		input.mouseL = false;
		input.mouseR = false;
		input.mouseScroll = 0;
		currentDemo.initControls(controls);
	}

	/**
	 * Main loop, steps simulation.
	 */
	public void loop() {
		count++;
		input.pmouseX = input.mouseX;
		input.pmouseY = input.mouseY;
		input.pmouseL = input.mouseL;
		input.pmouseR = input.mouseR;
		input.mouseX = nmouseX;
		input.mouseY = nmouseY;
		input.mouseL = nmouseL;
		input.mouseR = nmouseR;
		input.mouseScroll = nmouseScroll;
		for (int i=0;i<UserInput.KEYBOARD_LENGTH;i++) {
			input.pkeyboard[i] = input.keyboard[i];
			input.keyboard[i] = nkeyboard[i];
		}

		control();
		currentDemo.update();
		if (!pause || singleStep) {
			world.step(currentDemo.dt);
			singleStep = false;
		}
		double drawBegin = System.currentTimeMillis()/1000.0f;
		currentDemo.draw();
		double drawEnd = System.currentTimeMillis()/1000.0f;
		double drawTime = (drawEnd - drawBegin) * 1000.0f;

		double currentTime = System.currentTimeMillis()/1000.0f;
		fpsCount++;
		if (fpsTime + 1 < currentTime) {
			fps = fpsCount;
			fpsTime += 1;
			fpsCount = 0;
		}

		
	}
	
	public String getDemoInfo() {
		text ="<h2>"+currentDemo.demoName+ (pause ? " [Paused]" : " [Running]")+"</h2>"+
				"<h3>Profile</h3>"+
				"  FPS			 : "+fps+
				"  Rigid Bodies  : "+world.getNumRigidBodies()+"\n" +
				"  Joints        : "+world.getNumJoints()+"\n" +
				"  Shapes        : "+world.getNumShapes()+"\n" +
				"  Pairs         : "+world.getContactManager().getNumContacts()+"" +
				"<h3>Performance</h3>" +
				"  Broad Phase  : "+Math.round(performance.broadPhaseCollisionTime)+"\n" +
				"  Narrow Phase : "+Math.round(performance.narrowPhaseCollisionTime)+"\n" +
				"  Dynamics     : "+Math.round(performance.dynamicsTime)+"\n" +
				"  Physics Total: "+Math.round(performance.totalTime)+"" +
				//"  Rendering    : "+Math.round(drawTime)+"\n" +
				//"  Actual FPS   : "+fps+"\n" +
				"<h3>Control</h3>" ;
			text+=createKeyDescriptions(" ");
			
			text+=	"------------\n" +
				"<h3>Misc. Info</h3>" +
				additionalInfo() +
				'\n'
			;
			Vec3.numCreations = 0;
			Mat3.numCreations = 0;
			Quat.numCreations = 0;
			return text;
	}

	String createKeyDescriptions(String prefix) {
		String res = "";
		for (Control control :controls) {
			res += prefix+"{"+control.keyText+"}: {"+control.description.apply(null)+"}\n";
		}
		return res;
	}

	String additionalInfo() {
		return currentDemo.additionalInfo();
	}

	void control() {
		for (Control control:controls) {
			if (input.isKeyPressed(control.keyCode)) {
				control.onAction.apply(null);
			}
		}
	}

	public void changeDemo(int offset) {
		int num = demos.length;
		currentDemoIndex = ((currentDemoIndex + offset) % num + num) % num;
		currentDemo = demos[currentDemoIndex];
		initDemo();
	}

	public void mouseScrolled(int delta) {
		nmouseScroll = delta;
	}

	public void mouseMoved(double x, double  y) {
		nmouseX = x;
		nmouseY = y;
	}

	public void mouseLPressed() {
		nmouseL = true;
	}

	public void mouseRPressed() {
		nmouseR = true;
	}

	public void mouseLReleased() {
		nmouseL = false;
	}

	public void mouseRReleased() {
		nmouseR = false;
	}

	public void keyPressed(int keyCode) {
		if (keyCode < 0 || keyCode >= UserInput.KEYBOARD_LENGTH) return;
		nkeyboard[keyCode] = true;
	}

	public void keyReleased(int keyCode) {
		if (keyCode < 0 || keyCode >= UserInput.KEYBOARD_LENGTH) return;
		nkeyboard[keyCode] = false;
	}

	public void action(int index) {
		if (index >= 0 && index < controls.length) {
			controls[index].onAction.apply(null);
		}
	}
}