package oimo.dynamics.constraint.contact;
import oimo.common.Vec3;
import oimo.dynamics.constraint.contact.ContactImpulse;

/**
 * Internal class.
 */
public class ContactImpulse {
	// normal impulse
	public double impulseN;

	// tangent impulse
	public double impulseT;

	// binomal impulse
	public double impulseB;

	// position impulse
	public double impulseP;

	// lateral impulse
	public Vec3 impulseL=new Vec3();

	public ContactImpulse() {
		clear();
	}

   public void clear() {
		impulseN = 0;
		impulseT = 0;
		impulseB = 0;
		impulseP = 0;
		impulseL.zero();
	}

	public void copyFrom(ContactImpulse imp) {
		impulseN = imp.impulseN;
		impulseT = imp.impulseT;
		impulseB = imp.impulseB;
		impulseL.set(imp.impulseL);
	}

}
