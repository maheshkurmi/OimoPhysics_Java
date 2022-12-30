package oimo.dynamics.constraint.joint;

/**
 * The list of the types of the joints.
 */
public class JointType {
	public static final int _SPHERICAL     = 0;
	public static final int _REVOLUTE      = 1;
	public static final int _CYLINDRICAL   = 2;
	public static final int _PRISMATIC     = 3;
	public static final int _UNIVERSAL     = 4;
	public static final int _RAGDOLL       = 5;
	public static final int _GENERIC       = 6;

	/**
	 * Represents a spherical joint.
	 *
	 * See `SphericalJoint` for details.
	 */
	public static final int  SPHERICAL = _SPHERICAL;

	/**
	 * Represents a revolute joint.
	 *
	 * See `RevoluteJoint` for details.
	 */
	public static final int  REVOLUTE = _REVOLUTE;

	/**
	 * Represents a cylindrical joint.
	 *
	 * See `CylindricalJoint` for details.
	 */
	public static final int  CYLINDRICAL = _CYLINDRICAL;

	/**
	 * Represents a prismatic joint.
	 *
	 * See `PrismaticJoint` for details.
	 */
	public static final int  PRISMATIC = _PRISMATIC;

	/**
	 * Represents a universal joint.
	 *
	 * See `UniversalJoint` for details.
	 */
	public static final int  UNIVERSAL = _UNIVERSAL;

	/**
	 * Represents a ragdoll joint.
	 *
	 * See `RagdollJoint` for details.
	 */
	public static final int  RAGDOLL = _RAGDOLL;

	/**
	 * Represents a generic joint.
	 *
	 * See `GenericJoint` for details.
	 */
	public static final int  GENERIC = _GENERIC;
}
