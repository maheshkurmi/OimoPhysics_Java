package demo.common;

import oimo.collision.geometry.BoxGeometry;
import oimo.collision.geometry.CapsuleGeometry;
import oimo.collision.geometry.ConeGeometry;
import oimo.collision.geometry.CylinderGeometry;
import oimo.collision.geometry.Geometry;
import oimo.collision.geometry.SphereGeometry;
import oimo.common.Mat3;
import oimo.common.MathUtil;
import oimo.common.Vec3;
import oimo.dynamics.World;
import oimo.dynamics.constraint.joint.CylindricalJoint;
import oimo.dynamics.constraint.joint.CylindricalJointConfig;
import oimo.dynamics.constraint.joint.GenericJoint;
import oimo.dynamics.constraint.joint.GenericJointConfig;
import oimo.dynamics.constraint.joint.PrismaticJoint;
import oimo.dynamics.constraint.joint.PrismaticJointConfig;
import oimo.dynamics.constraint.joint.RagdollJoint;
import oimo.dynamics.constraint.joint.RagdollJointConfig;
import oimo.dynamics.constraint.joint.RevoluteJoint;
import oimo.dynamics.constraint.joint.RevoluteJointConfig;
import oimo.dynamics.constraint.joint.RotationalLimitMotor;
import oimo.dynamics.constraint.joint.SphericalJoint;
import oimo.dynamics.constraint.joint.SphericalJointConfig;
import oimo.dynamics.constraint.joint.SpringDamper;
import oimo.dynamics.constraint.joint.TranslationalLimitMotor;
import oimo.dynamics.constraint.joint.UniversalJoint;
import oimo.dynamics.constraint.joint.UniversalJointConfig;
import oimo.dynamics.rigidbody.RigidBody;
import oimo.dynamics.rigidbody.RigidBodyConfig;
import oimo.dynamics.rigidbody.RigidBodyType;
import oimo.dynamics.rigidbody.Shape;
import oimo.dynamics.rigidbody.ShapeConfig;

/**
 * Defines some shortcuts to creating and adding objects to a world.
 */
public class OimoUtil {
	public static RagdollJoint addRagdollJoint(World w, RigidBody rb1, RigidBody rb2, Vec3 anchor, Vec3 twistAxis, Vec3 swingAxis) {
		return addRagdollJoint(w,rb1,rb2,anchor,twistAxis,swingAxis,null,180,180,null,null);
	}
	
	public static RagdollJoint addRagdollJoint(World w, RigidBody rb1, RigidBody rb2, Vec3 anchor, Vec3 twistAxis, Vec3 swingAxis,SpringDamper swingSd ,double maxSwing1Deg, double maxSwing2Deg , SpringDamper twistSd , RotationalLimitMotor twistLm) {
		RagdollJointConfig jc = new RagdollJointConfig();
		jc.init(rb1, rb2, anchor, twistAxis, swingAxis);
		if (twistSd != null) jc.twistSpringDamper = twistSd;
		if (twistLm != null) jc.twistLimitMotor = twistLm;
		if (swingSd != null) jc.swingSpringDamper = swingSd;
		jc.maxSwingAngle1 = maxSwing1Deg * MathUtil.TO_RADIANS;
		jc.maxSwingAngle2 = maxSwing2Deg * MathUtil.TO_RADIANS;
		RagdollJoint j = new RagdollJoint(jc);
		w.addJoint(j);
		return j;
	}
	

	public static UniversalJoint addUniversalJoint(World w,RigidBody rb1, RigidBody rb2, Vec3 anchor, Vec3 axis1,Vec3 axis2, SpringDamper sd1 , RotationalLimitMotor lm1 , SpringDamper sd2, RotationalLimitMotor lm2 ) {
		UniversalJointConfig jc = new UniversalJointConfig();
		jc.init(rb1, rb2, anchor, axis1, axis2);
		if (sd1 != null) jc.springDamper1 = sd1;
		if (lm1 != null) jc.limitMotor1 = lm1;
		if (sd2 != null) jc.springDamper2 = sd2;
		if (lm2 != null) jc.limitMotor2 = lm2;
		UniversalJoint j = new UniversalJoint(jc);
		w.addJoint(j);
		return j;
	}

	public static GenericJoint addGenericJoint(World w, RigidBody rb1, RigidBody rb2, Vec3 anchor, Mat3 basis1, Mat3 basis2, SpringDamper[] translSds, TranslationalLimitMotor[] translLms, SpringDamper[] rotSds, RotationalLimitMotor[] rotLms) {
		GenericJointConfig jc = new GenericJointConfig();
		jc.init(rb1, rb2, anchor, basis1, basis2);
		for (int i=0;i<3;i++) {
			if (translSds != null && translSds[i] != null) jc.translationalSpringDampers[i] = translSds[i];
			if (translLms != null && translLms[i] != null) jc.translationalLimitMotors[i]   = translLms[i];
			if (rotSds != null    && rotSds[i] != null)    jc.rotationalSpringDampers[i]    = rotSds[i];
			if (rotLms != null    && rotLms[i] != null)    jc.rotationalLimitMotors[i]      = rotLms[i];
		}
		GenericJoint j = new GenericJoint(jc);
		w.addJoint(j);
		return j;
	}

	public static PrismaticJoint addPrismaticJoint(World w, RigidBody rb1, RigidBody rb2,Vec3 anchor, Vec3 axis, SpringDamper sd , TranslationalLimitMotor lm) {
		PrismaticJointConfig jc = new PrismaticJointConfig();
		jc.init(rb1, rb2, anchor, axis);
		if (sd != null) jc.springDamper = sd;
		if (lm != null) jc.limitMotor = lm;
		PrismaticJoint j = new PrismaticJoint(jc);
		w.addJoint(j);
		return j;
	}

	public static RevoluteJoint addRevoluteJoint(World w, RigidBody rb1, RigidBody rb2, Vec3 anchor, Vec3 axis,SpringDamper sd, RotationalLimitMotor lm) {
		RevoluteJointConfig jc = new RevoluteJointConfig();
		jc.init(rb1, rb2, anchor, axis);
		if (sd != null) jc.springDamper = sd;
		if (lm != null) jc.limitMotor = lm;
		RevoluteJoint j = new RevoluteJoint(jc);
		w.addJoint(j);
		return j;
	}

	public static CylindricalJoint addCylindricalJoint(World w, RigidBody rb1, RigidBody rb2, Vec3 anchor, Vec3 axis, SpringDamper rotSd, RotationalLimitMotor rotLm, SpringDamper traSd, TranslationalLimitMotor traLm ) {
		CylindricalJointConfig jc = new CylindricalJointConfig();
		jc.init(rb1, rb2, anchor, axis);
		if (rotSd != null) jc.rotationalSpringDamper = rotSd;
		if (rotLm != null) jc.rotationalLimitMotor = rotLm;
		if (traSd != null) jc.translationalSpringDamper = traSd;
		if (traLm != null) jc.translationalLimitMotor = traLm;
		CylindricalJoint j = new CylindricalJoint(jc);
		w.addJoint(j);
		return j;
	}

	public static SphericalJoint addSphericalJoint(World w, RigidBody rb1, RigidBody rb2, Vec3 anchor) {
		var jc = new SphericalJointConfig();
		jc.init(rb1, rb2, anchor);
		SphericalJoint j = new SphericalJoint(jc);
		w.addJoint(j);
		return j;
	}

	public static RigidBody addSphere(World w, Vec3 center, double radius, boolean wall) {
		return addRigidBody(w, center, new SphereGeometry(radius), wall);
	}

	public static RigidBody addBox(World w,  Vec3 center, Vec3 halfExtents, boolean wall) {
		return addRigidBody(w, center, new BoxGeometry(halfExtents), wall);
	}

	public static RigidBody addCylinder(World w, Vec3 center, double radius, double halfHeight, boolean wall) {
		return addRigidBody(w, center, new CylinderGeometry(radius, halfHeight), wall);
	}

	public static RigidBody addCone(World w, Vec3 center,double radius,double halfHeight, boolean wall) {
		return addRigidBody(w, center, new ConeGeometry(radius, halfHeight), wall);
	}

	public static RigidBody addCapsule(World w, Vec3 center, double radius, double halfHeight, boolean wall) {
		return addRigidBody(w, center, new CapsuleGeometry(radius, halfHeight), wall);
	}

	public static RigidBody addRigidBody(World w, Vec3 center, Geometry geom, boolean wall) {
		ShapeConfig shapec = new ShapeConfig();
		shapec.geometry = geom;
		//shapec.restitution=0.5f;
		//shapec.friction=0.2f;
		RigidBodyConfig bodyc = new RigidBodyConfig();
		bodyc.type = wall ? RigidBodyType.STATIC : RigidBodyType.DYNAMIC;
		bodyc.position = center;
		
		RigidBody body = new RigidBody(bodyc);
		body.addShape(new Shape(shapec));
		w.addRigidBody(body);
		return body;
	}

	// ---------------------------------------------------------------------------

	public static RigidBody addRagdoll(World w,Vec3 pos) {
		RigidBody head;
		RigidBody body1;
		RigidBody body2;
		RigidBody armL1;
		RigidBody armL2;
		RigidBody armR1;
		RigidBody armR2;
		RigidBody legL1;
		RigidBody legL2;
		RigidBody legR1;
		RigidBody legR2;

		double headHeight = 0.3f;
		double upperBody = 0.35f;
		double lowerBody = 0.35f;
		double bodyRadius = 0.2f;
		double legRadius = 0.1f;
		double legInterval = 0.15f;
		double upperLeg = 0.5f;
		double lowerLeg = 0.5f;
		double armRadius = 0.075f;
		double upperArm = 0.35f;
		double lowerArm = 0.35f;

		head = addCapsule(w, pos.add(new Vec3(0, lowerBody + upperBody + bodyRadius + headHeight / 2, 0)), headHeight / 2 * 0.8f, headHeight / 2 * 0.2f, false);
		body1 = addCapsule(w, pos.add(new Vec3(0, lowerBody + upperBody / 2, 0)), bodyRadius, upperBody / 2, false);
		body2 = addCapsule(w, pos.add(new Vec3(0, lowerBody / 2, 0)), bodyRadius, lowerBody / 2, false);

		legL1 = addCapsule(w, pos.add(new Vec3(-legInterval, -upperLeg / 2 - legInterval, 0)), legRadius, upperLeg / 2, false);
		legL2 = addCapsule(w, pos.add(new Vec3(-legInterval, -upperLeg - lowerLeg / 2 - legInterval, 0)), legRadius, lowerLeg / 2, false);

		legR1 = addCapsule(w, pos.add(new Vec3(legInterval, -upperLeg / 2 - legInterval, 0)), legRadius, upperLeg / 2, false);
		legR2 = addCapsule(w, pos.add(new Vec3(legInterval, -upperLeg - lowerLeg / 2 - legInterval, 0)), legRadius, lowerLeg / 2, false);

		armL1 = addCapsule(w, pos.add(new Vec3(-bodyRadius - upperArm / 2, lowerBody + upperBody, 0)), armRadius, upperArm / 2, false);
		armL2 = addCapsule(w, pos.add(new Vec3(-bodyRadius - upperArm - lowerArm / 2, lowerBody + upperBody, 0)), armRadius, lowerArm / 2, false);

		armR1 = addCapsule(w, pos.add(new Vec3(bodyRadius + upperArm / 2, lowerBody + upperBody, 0)), armRadius, upperArm / 2, false);
		armR2 = addCapsule(w, pos.add(new Vec3(bodyRadius + upperArm + lowerArm / 2, lowerBody + upperBody, 0)), armRadius, lowerArm / 2, false);

		Mat3 rotZ90 = new Mat3().appendRotationEq(90 * MathUtil.TO_RADIANS, 0, 0, 1);
		armL1.setRotation(rotZ90);
		armL2.setRotation(rotZ90);
		armR1.setRotation(rotZ90);
		armR2.setRotation(rotZ90);

		Vec3 x = new Vec3(1, 0, 0);
		Vec3 y = new Vec3(0, 1, 0);
		Vec3 z = new Vec3(0, 0, 1);

		SpringDamper sd = new SpringDamper();
		sd.setSpring(10, 1);
		RotationalLimitMotor lm180 = new RotationalLimitMotor().setLimits(-90 * MathUtil.TO_RADIANS, 90 * MathUtil.TO_RADIANS);
		RotationalLimitMotor lm90 = new RotationalLimitMotor().setLimits(-45 * MathUtil.TO_RADIANS, 45 * MathUtil.TO_RADIANS);
		RotationalLimitMotor lmElbow = new RotationalLimitMotor().setLimits(0 * MathUtil.TO_RADIANS, 160 * MathUtil.TO_RADIANS);
		RotationalLimitMotor lmKnee = new RotationalLimitMotor().setLimits(0 * MathUtil.TO_RADIANS, 160 * MathUtil.TO_RADIANS);

		addRagdollJoint(w, body1, head, pos.add(new Vec3(0, lowerBody + upperBody + bodyRadius, 0)), y, x, sd, 90, 70, sd, lm180);
		addRagdollJoint(w, body1, body2, pos.add(new Vec3(0, lowerBody, 0)), y, x, sd, 60, 45, sd, lm90);

		addRagdollJoint(w, body1, armL1, pos.add(new Vec3(-bodyRadius, lowerBody + upperBody, 0)), x, z, sd, 90, 90, sd, lm180);
		addRagdollJoint(w, body1, armR1, pos.add(new Vec3(bodyRadius, lowerBody + upperBody, 0)), x.negate(), z, sd, 90, 90, sd, lm180);

		addRevoluteJoint(w, armL1, armL2, pos.add(new Vec3(-bodyRadius - upperArm, lowerBody + upperBody, 0)), y, sd, lmElbow);
		addRevoluteJoint(w, armR1, armR2, pos.add(new Vec3(bodyRadius + upperArm, lowerBody + upperBody, 0)), y.negate(), sd, lmElbow);

		RagdollJointConfig jc = new RagdollJointConfig();
		jc.swingSpringDamper = sd;
		jc.maxSwingAngle1 = 90 * MathUtil.TO_RADIANS;
		jc.maxSwingAngle2 = 70 * MathUtil.TO_RADIANS;
		jc.twistSpringDamper = sd;
		jc.twistLimitMotor = lm180;

		jc.init(body2, legL1, pos.add(new Vec3(-legInterval, -legInterval, 0)), y, x);
		jc.localTwistAxis1 = z.negate();
		w.addJoint(new RagdollJoint(jc));

		jc.init(body2, legR1, pos.add(new Vec3(legInterval, -legInterval, 0)), y, x);
		jc.localTwistAxis1 = z.negate();
		w.addJoint(new RagdollJoint(jc));

		addRevoluteJoint(w, legL1, legL2, pos.add(new Vec3(-legInterval, -legInterval - upperLeg, 0)), x, sd, lmKnee);
		addRevoluteJoint(w, legR1, legR2, pos.add(new Vec3(legInterval, -legInterval - upperLeg, 0)), x, sd, lmKnee);

		return body1;
	}
}
