package oimo.dynamics.constraint.solver.common;
import oimo.common.M;
import oimo.common.Vec3;

/**
 * Internal class.
 */
public class ContactSolverMassDataRow {
	// normal impulse -> linear/angular velocity change
	public Vec3 invMLinN1=new Vec3();
	public Vec3 invMLinN2=new Vec3();
	public Vec3 invMAngN1=new Vec3();
	public Vec3 invMAngN2=new Vec3();

	// tangent impulse -> linear/angular velocity change
	public Vec3 invMLinT1=new Vec3();
	public Vec3 invMLinT2=new Vec3();
	public Vec3 invMAngT1=new Vec3();
	public Vec3 invMAngT2=new Vec3();

	// binormal impulse -> linear/angular velocity change
	public Vec3 invMLinB1=new Vec3();
	public Vec3 invMLinB2=new Vec3();
	public Vec3 invMAngB1=new Vec3();
	public Vec3 invMAngB2=new Vec3();

	// normal mass
	public float massN;

	// tangent/binormal mass matrix for cone friction
	public float massTB00;
	public float massTB01;
	public float massTB10;
	public float massTB11;

	public ContactSolverMassDataRow() {
	

		massN = 0;

		massTB00 = 0;
		massTB01 = 0;
		massTB10 = 0;
		massTB11 = 0;
	}
}
