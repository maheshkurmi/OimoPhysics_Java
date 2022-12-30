package oimo.common;

import oimo.dynamics.World;

/**
 * A class that records the time the world takes for physics calculations.
 * Unless otherwise specified, time is in milliseconds.
 * 
 * @author saharan
 */
public class Performance {
	/**
	 * Time required for wide area collision detection.
	 */
	public long broadPhaseTime;

	/**
	 * Time required for detailed collision detection.
	 */
	public long narrowPhaseTime;

	/**
	 * Time spent calculating constraints and integrals.
	 */
	public long solvingTime;

	/**
	 * Time taken for other updates.
	 */
	public long updateTime;

	private String broadPhase;
	/**
	 * Total time spent calculating the step.
	 */
	public long totalTime;

	public long MaxBroadPhaseTime = 0;
	public long MaxNarrowPhaseTime = 0;
	public long MaxSolvingTime = 0;
	public long MaxTotalTime = 0;
	public long MaxUpdateTime = 0;

	public long[] infos = new long[13];

	private long[] times = new long[] { 0, 0, 0, 0 };

	private long[] f = new long[] { 0, 0, 0 };
	private float fps = 0;

	private long tt = 0;

	private World parent;

	public long broadPhaseCollisionTime;

	public long narrowPhaseCollisionTime;

	public long dynamicsTime;

	/**
	 * Create a new Performance object.
	 */
	public Performance(World world) {
		this.parent = world;
	}

	public void setTime(int n) {
		this.times[n] = System.currentTimeMillis();
	}

	public void resetMax() {

		this.MaxBroadPhaseTime = 0;
		this.MaxNarrowPhaseTime = 0;
		this.MaxSolvingTime = 0;
		this.MaxTotalTime = 0;
		this.MaxUpdateTime = 0;
		//this.broadPhase = this.parent.broadPhaseType;
	}

	public void calcBroadPhase() {

		this.setTime(2);
		this.broadPhaseTime = this.times[2] - this.times[1];

	}

	public void calcNarrowPhase() {

		this.setTime(3);
		this.narrowPhaseTime = this.times[3] - this.times[2];

	}

	public void calcEnd() {
		this.setTime(2);
		this.solvingTime = this.times[2] - this.times[1];
		this.totalTime = this.times[2] - this.times[0];
		this.updateTime = this.totalTime - (this.broadPhaseTime + this.narrowPhaseTime + this.solvingTime);

		if (this.tt == 100)
			this.resetMax();

		if (this.tt > 100) {
			if (this.broadPhaseTime > this.MaxBroadPhaseTime)
				this.MaxBroadPhaseTime = this.broadPhaseTime;
			if (this.narrowPhaseTime > this.MaxNarrowPhaseTime)
				this.MaxNarrowPhaseTime = this.narrowPhaseTime;
			if (this.solvingTime > this.MaxSolvingTime)
				this.MaxSolvingTime = this.solvingTime;
			if (this.totalTime > this.MaxTotalTime)
				this.MaxTotalTime = this.totalTime;
			if (this.updateTime > this.MaxUpdateTime)
				this.MaxUpdateTime = this.updateTime;
		}

		this.upfps();

		this.tt++;
		if (this.tt > 500)
			this.tt = 0;

	}

	public void upfps() {
		this.f[1] = System.currentTimeMillis();
		if (this.f[1] - 1000 > this.f[0]) {
			this.f[0] = this.f[1];
			this.fps = this.f[2];
			this.f[2] = 0;
		}
		this.f[2]++;
	}

	public String[] show() {
		String[] info = new String[] { "Oimo.js "  + "<br>", this.broadPhase + "<br><br>",
				"FPS: " + this.fps + " fps<br><br>", "rigidbody " + this.parent._numRigidBodies + "<br>",
				"ct-point &nbsp;" + this.parent._contactManager._numContacts+ "<br>",
				"paircheck " + this.parent._broadPhase._numProxies + "<br>",
				"island &nbsp;&nbsp;&nbsp;" + this.parent._numIslands + "<br><br>", "Time in milliseconds<br><br>",
				"broadphase &nbsp;" + _fix(this.broadPhaseTime) + " | " + _fix(this.MaxBroadPhaseTime) + "<br>",
				"narrowphase " + _fix(this.narrowPhaseTime) + " | " + _fix(this.MaxNarrowPhaseTime) + "<br>",
				"solving &nbsp;&nbsp;&nbsp;&nbsp;" + _fix(this.solvingTime) + " | " + _fix(this.MaxSolvingTime)
						+ "<br>",
				"total &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;" + _fix(this.totalTime) + " | " + _fix(this.MaxTotalTime)
						+ "<br>",
				"updating &nbsp;&nbsp;&nbsp;" + _fix(this.updateTime) + " | " + _fix(this.MaxUpdateTime) + "<br>" };
		return info;
	}

	private long _fix(long time) {
		return time;
	}

	public long[] toArray() {
		this.infos[0] = this.parent._broadPhase._type;
		this.infos[1] = this.parent._numRigidBodies;
		this.infos[2] = this.parent._contactManager._numContacts;
		this.infos[3] = this.parent._broadPhase._numProxies;
		this.infos[4] = this.parent._contactManager._numContacts;
		this.infos[5] = this.parent._numIslands;
		this.infos[6] = this.broadPhaseTime;
		this.infos[7] = this.narrowPhaseTime;
		this.infos[8] = this.solvingTime;
		this.infos[9] = this.updateTime;
		this.infos[10] = this.totalTime;
		this.infos[11] = (long) this.fps;
		return this.infos;
	}
}
