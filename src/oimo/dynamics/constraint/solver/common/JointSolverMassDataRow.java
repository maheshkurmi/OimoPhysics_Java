package oimo.dynamics.constraint.solver.common;
import oimo.common.Vec3;

/**
 * Internal class.
 */
public class JointSolverMassDataRow {
	// impulse -> linear/angular velocity change
	public Vec3 invMLin1=new Vec3();
	public Vec3 invMLin2=new Vec3();
	public Vec3 invMAng1=new Vec3();
	public Vec3 invMAng2=new Vec3();

	// mass
	public double mass;
	public double massWithoutCfm;

	public JointSolverMassDataRow() {

		mass = 0;
		massWithoutCfm = 0;
	}
}
