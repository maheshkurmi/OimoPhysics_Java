package oimo.common;
/**
 * Style settings of the debug draw.
 */
public class DebugDrawStyle {
	public Vec3 shapeColor1 = new Vec3(0.7f, 0.2f, 0.4f);
	public Vec3 shapeColor2 = new Vec3(1.0f, 0.8f, 0.1f);
	public Vec3 sleepyShapeColor1 = new Vec3(0.5f, 0.25f, 0.6f);
	public Vec3 sleepyShapeColor2 = new Vec3(0.6f, 0.8f, 0.3f);
	public Vec3 sleepingShapeColor1 = new Vec3(0.3f, 0.3f, 0.8f);
	public Vec3 sleepingShapeColor2 = new Vec3(0.2f, 0.8f, 0.5f);
	public Vec3 staticShapeColor = new Vec3(0.7f, 0.7f, 0.7f);
	public Vec3 kinematicShapeColor = new Vec3(1.0f, 0.5f, 0.1f);

	public Vec3 aabbColor = new Vec3(1.0f, 0.1f, 0.1f);
	public Vec3 bvhNodeColor = new Vec3(0.4f, 0.4f, 0.4f);

	public Vec3 pairColor = new Vec3(1.0f, 1.0f, 0.1f);

	public Vec3 contactColor = new Vec3(1.0f, 0.1f, 0.1f);
	public Vec3 contactColor2 = new Vec3(1.0f, 0.6f, 0.1f);
	public Vec3 contactColor3 = new Vec3(0.1f, 0.8f, 0.6f);
	public Vec3 contactColor4 = new Vec3(0.8f, 0.1f, 1.0f);
	public Vec3 newContactColor = new Vec3(1.0f, 1.0f, 0.1f);
	public Vec3 disabledContactColor = new Vec3(0.5f, 0.1f, 0.1f);

	public Vec3 contactNormalColor = new Vec3(1.0f, 0.1f, 0.1f);
	public Vec3 contactTangentColor = new Vec3(0.1f, 0.8f, 0.1f);
	public Vec3 contactBinormalColor = new Vec3(0.2f, 0.2f, 1.0f);
	public float contactNormalLength = 0.5f;
	public float contactTangentLength = 0.5f;
	public float contactBinormalLength = 0.5f;

	public Vec3 jointLineColor = new Vec3(0.8f, 0.8f, 0.8f);
	public Vec3 jointErrorColor = new Vec3(1.0f, 0.1f, 0.1f);
	public float jointRotationalConstraintRadius = 0.3f;

	public float basisLength = 0.5f;
	public Vec3 basisColorX = new Vec3(1.0f, 0.0f, 0.0f);
	public Vec3 basisColorY = new Vec3(0.0f, 1.0f, 0.0f);
	public Vec3 basisColorZ = new Vec3(0.0f, 0.0f, 1.0f);

	/**
	 * Default constructor.
	 */
	public DebugDrawStyle() {
	}

}
