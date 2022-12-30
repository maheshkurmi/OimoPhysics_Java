package oimo.common;

import oimo.dynamics.constraint.PositionCorrectionAlgorithm;
import oimo.dynamics.constraint.solver.ConstraintSolverType;

public class Setting {
	// default shape parameters
	public static float defaultFriction = 0.2f;
	public static float defaultRestitution = 0.2f;
	public static float defaultDensity = 1;
	public static int defaultCollisionGroup = 1;
	public static int defaultCollisionMask = 1;

	// velocity limitations
	public static float maxTranslationPerStep = 20;
	public static float maxRotationPerStep = MathUtil.PI;

	// dynamic BVH
	public static float bvhProxyPadding = 0.1f;
	public static float bvhIncrementalCollisionThreshold = 0.45f;

	// GJK/EPA
	public static float defaultGJKMargin = 0.05f;
	public static boolean enableGJKCaching = true;
	public static int maxEPAVertices = 128;
	public static int maxEPAPolyhedronFaces = 128;

	// general constraints
	public static float contactEnableBounceThreshold = 0.5f;
	public static float velocityBaumgarte = 0.2f;
	public static float positionSplitImpulseBaumgarte = 0.4f;
	public static float positionNgsBaumgarte = 1.0f;

	// contacts
	public static float contactUseAlternativePositionCorrectionAlgorithmDepthThreshold = 0.05f;
	public static int defaultContactPositionCorrectionAlgorithm = PositionCorrectionAlgorithm._BAUMGARTE;
	public static int alternativeContactPositionCorrectionAlgorithm = PositionCorrectionAlgorithm._SPLIT_IMPULSE;
	public static float contactPersistenceThreshold = 0.05f;
	public static int maxManifoldPoints = 4;

	// joints
	public static int defaultJointConstraintSolverType = ConstraintSolverType._ITERATIVE;
	public static int defaultJointPositionCorrectionAlgorithm = PositionCorrectionAlgorithm._BAUMGARTE;
	public static float jointWarmStartingFactorForBaungarte = 0.8f;
	public static float jointWarmStartingFactor = 0.95f;
	public static float minSpringDamperDampingRatio = 1e-6f;
	public static float minRagdollMaxSwingAngle = 1e-6f;
	public static int maxJacobianRows = 6;

	// direct MLCP solver
	public static float directMlcpSolverEps = 1e-9f;

	// islands
	public static int islandInitialRigidBodyArraySize = 128;
	public static int islandInitialConstraintArraySize = 128;

	// sleeping
	public static float sleepingVelocityThreshold = 0.2f;
	public static float sleepingAngularVelocityThreshold = 0.5f;
	public static float sleepingTimeThreshold = 1.0f;
	public static boolean disableSleeping = false;

	// slops
	public static float linearSlop = 0.005f;
	public static float angularSlop = 1 * MathUtil.TO_RADIANS;

}
