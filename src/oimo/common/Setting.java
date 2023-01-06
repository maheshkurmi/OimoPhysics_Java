package oimo.common;

import oimo.dynamics.constraint.PositionCorrectionAlgorithm;
import oimo.dynamics.constraint.solver.ConstraintSolverType;

public class Setting {
	// default shape parameters
	public static double defaultFriction = 0.2f;
	public static double defaultRestitution = 0.2f;
	public static double defaultDensity = 1;
	public static int defaultCollisionGroup = 1;
	public static int defaultCollisionMask = 1;

	// velocity limitations
	public static double maxTranslationPerStep = 20;
	public static double maxRotationPerStep = MathUtil.PI;

	// dynamic BVH
	public static double bvhProxyPadding = 0.1f;
	public static double bvhIncrementalCollisionThreshold = 0.45f;

	// GJK/EPA
	public static double defaultGJKMargin = 0.05f;
	public static boolean enableGJKCaching = true;
	public static int maxEPAVertices = 128;
	public static int maxEPAPolyhedronFaces = 128;

	// general constraints
	public static double contactEnableBounceThreshold = 0.5f;
	public static double velocityBaumgarte = 0.2f;
	public static double positionSplitImpulseBaumgarte = 0.4f;
	public static double positionNgsBaumgarte = 1.0f;

	// contacts
	public static double contactUseAlternativePositionCorrectionAlgorithmDepthThreshold = 0.05f;
	public static int defaultContactPositionCorrectionAlgorithm = PositionCorrectionAlgorithm._BAUMGARTE;
	public static int alternativeContactPositionCorrectionAlgorithm = PositionCorrectionAlgorithm._SPLIT_IMPULSE;
	public static double contactPersistenceThreshold = 0.05f;
	public static int maxManifoldPoints = 4;

	// joints
	public static int defaultJointConstraintSolverType = ConstraintSolverType._ITERATIVE;
	public static int defaultJointPositionCorrectionAlgorithm = PositionCorrectionAlgorithm._BAUMGARTE;
	public static double jointWarmStartingFactorForBaungarte = 0.8f;
	public static double jointWarmStartingFactor = 0.95f;
	public static double minSpringDamperDampingRatio = 1e-6f;
	public static double minRagdollMaxSwingAngle = 1e-6f;
	public static int maxJacobianRows = 6;

	// direct MLCP solver
	public static double directMlcpSolverEps = 1e-9f;

	// islands
	public static int islandInitialRigidBodyArraySize = 128;
	public static int islandInitialConstraintArraySize = 128;

	// sleeping
	public static double sleepingVelocityThreshold = 0.2f;
	public static double sleepingAngularVelocityThreshold = 0.5f;
	public static double sleepingTimeThreshold = 1.0f;
	public static boolean disableSleeping = false;

	// slops
	public static double linearSlop = 0.005f;
	public static double angularSlop = 1 * MathUtil.TO_RADIANS;

}
