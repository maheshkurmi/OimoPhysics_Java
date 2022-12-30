package oimo.dynamics.constraint.joint;

import oimo.common.M;
import oimo.common.Mat3;
import oimo.common.Transform;
import oimo.common.Vec3;
import oimo.dynamics.rigidbody.RigidBody;

/**
 * A generic joint config is used for constructions of generic joints.
 */
public class GenericJointConfig extends JointConfig {
	/**
	 * The first body's local constraint basis.
	 */
	public Mat3 localBasis1;

	/**
	 * The second body's local constraint basis.
	 */
	public Mat3 localBasis2;

	/**
	 * The translational limits and motors along the first body's the constraint basis.
	 */
	public TranslationalLimitMotor[] translationalLimitMotors;

	/**
	 * The translational springs and dampers along the first body's constraint basis.
	 */
	public SpringDamper[] translationalSpringDampers;

	/**
	 * The rotational limits and motors along the rotation axes of the relative x-y-z Euler angles.
	 */
	public RotationalLimitMotor[] rotationalLimitMotors;

	/**
	 * The rotational springs and dampers along the rotation axes of the relative x-y-z Euler angles.
	 */
	public SpringDamper[] rotationalSpringDampers;

	/**
	 * Default constructor.
	 */
	public GenericJointConfig() {
		super();
		localBasis1 = new Mat3();
		localBasis2 = new Mat3();
		
		translationalLimitMotors =new TranslationalLimitMotor[3];
		for(int i=0;i<3;i++)translationalLimitMotors[i]=new TranslationalLimitMotor().setLimits(0, 0);
		
		rotationalLimitMotors =new RotationalLimitMotor[3];
		for(int i=0;i<3;i++)rotationalLimitMotors[i]=new RotationalLimitMotor().setLimits(0, 0);
		
		translationalSpringDampers =new SpringDamper[3];
		for(int i=0;i<3;i++)translationalSpringDampers[i]=new SpringDamper();
		
		rotationalSpringDampers =new SpringDamper[3];
		for(int i=0;i<3;i++)rotationalSpringDampers[i]=new SpringDamper();
		
	}

	/**
	 * Sets rigid bodies, local anchors from the world anchor `worldAnchor`, local bases
	 * from the world bases `worldBasis1` and `worldBasis2`, and returns `this`.
	 */
	public GenericJointConfig init(RigidBody rigidBody1,RigidBody rigidBody2, Vec3 worldAnchor, Mat3 worldBasis1, Mat3 worldBasis2) {
		_init(rigidBody1, rigidBody2, worldAnchor);
		Transform tf1 = rigidBody1._transform;
		Transform tf2 = rigidBody2._transform;
		Mat3 wb1=new Mat3();
		Mat3 wb2=new Mat3();
		Mat3 lb1=new Mat3();
		Mat3 lb2=new Mat3();
		M.mat3_fromMat3(wb1, worldBasis1);
		M.mat3_fromMat3(wb2, worldBasis2);
		M.mat3_mulLhsTransposed(lb1, tf1._rotation, wb1);
		M.mat3_mulLhsTransposed(lb2, tf2._rotation, wb2);
		M.mat3_toMat3(localBasis1, lb1);
		M.mat3_toMat3(localBasis2, lb2);
		return this;
	}

}