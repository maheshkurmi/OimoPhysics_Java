package oimo.collision.geometry;

import oimo.common.M;
import oimo.common.Vec3;

public class Aabb {
	public Vec3 _min;
	public Vec3 _max;

	/**
	 * Creates an empty AABB. Minimum and maximum points are set to zero.
	 */
	public Aabb() {
		_min=new Vec3();
		_max=new Vec3();
	}
		

	/**
	 * Sets the minimum and maximum point and returns `this`.
	 *
	 * Equivallent to `setMin(min).setMax(max)`.
	 */
	public Aabb set(Vec3 min, Vec3 max) {
		_min.copyFrom(min);
		_max.copyFrom(max);
		return this;
	}
		
	/**
	 * Returns new vector as the minimum point of the axis-aligned bounding box.
	 */
	public Vec3 getMin() {
		Vec3 v= new Vec3();
		return v.copyFrom(_min);
	}

	/**
	 * Sets the minimum point of the axis-aligned bounding box to `min`.
	 *
	 * This does not create a new instance of `Vec3`.
	 */
	public void getMinTo(Vec3 min) {
		min.copyFrom(_min);
	}

	/**
	 * Sets the minimum point of the axis-aligned bounding box to `min` and returns `this`.
	 */
	public Aabb setMin(Vec3 min) {
		_min.copyFrom(min);
		return this;
	}

	/**
	 * Returns new vector as the maximum point of the axis-aligned bounding box.
	 */
	public Vec3  getMax() {
		Vec3 v= new Vec3();
		return v.copyFrom(_max);
	}

	/**
	 * Sets the maximum point of the axis-aligned bounding box to `max`.
	 *
	 * This does not create a new instance of `Vec3`.
	 */
	public void getMaxTo(Vec3 max) {
		max.copyFrom(_max);
	}

	/**
	 * Sets the maximum point of the axis-aligned bounding box to `max` and returns `this`.
	 */
	public Aabb setMax(Vec3 max) {
		_max.copyFrom(max);
		return this;
	}

	/**
	 * Returns new vector as the center of the AABB.
	 */
	public Vec3 getCenter() {
		return new Vec3((_min.x+_max.x)*0.5f,(_min.y+_max.y)*0.5f,(_min.z+_max.z)*0.5f);
	}

	/**
	 * Sets `center` to the center of the AABB.
	 *
	 * This does not create a new instance of `Vec3`.
	 */
	public void getCenterTo(Vec3 center) {
		center.set((_min.x+_max.x)*0.5f,(_min.y+_max.y)*0.5f,(_min.z+_max.z)*0.5f);
	}

	/**
	 * Returns new vector represeting half extents of the AABB.
	 */
	public Vec3 getExtents() {
		return new Vec3((_max.x-_min.x)*0.5f,(_max.y-_min.y)*0.5f,(_max.z+_min.z)*0.5f);
	}

	/**
	 * Sets `halfExtents` to the half extents of the AABB.
	 *
	 * This does not create a new instance of `Vec3`.
	 */
	public void getExtentsTo(Vec3 halfExtents) {
		halfExtents.set((_max.x-_min.x)*0.5f,(_max.y-_min.y)*0.5f,(_max.z+_min.z)*0.5f);
	}

	/**
	 * Combines `other` into this AABB and returns `this`.
	 */
	public Aabb combine(Aabb other) {
		this._min.x =  this._min.x < other._min.x  ? this._min.x : other._min.x;
		this._min.y =  this._min.y < other._min.y  ? this._min.y : other._min.y;
		this._min.z =  this._min.z < other._min.z  ? this._min.z : other._min.z;
		
		this._max.x =  this._max.x > other._max.x  ? this._max.x : other._max.x;
		this._max.y =  this._max.y > other._max.y  ? this._max.y : other._max.y;
		this._max.z =  this._max.z > other._max.z  ? this._max.z : other._max.z;
		
		return this;
	}
	
	

	/**
	 * Returns the new combined aabb of `this` and `other`.
	 */
	public Aabb combined(Aabb other) {
		Aabb aabb = new Aabb();
		aabb.combine(this);
		return aabb;
	}

	/**
	 * Returns whether `this` and `other` intersect.
	 */
	public boolean overlap(Aabb other) {
		return 	(this._min.x < other._max.x) && (this._max.x > other._min.x) && 
				(this._min.y < other._max.y) && (this._max.y > other._min.y) && 
				(this._min.z < other._max.z) && (this._max.z > other._min.z);
	}
	

	/**
	 * Returns the new aabb as intersection of `this` and `other`.
	 */
	public Aabb getIntersection(Aabb other) {
		Aabb aabb= new Aabb();
		M.vec3_max(aabb._min, this._min, other._min);
		M.vec3_min(aabb._max, this._max, other._max);
		return aabb;
	}

	/**
	 * Sets `intersection` to the intersection of `this` and `other`.
	 *
	 * This does not create a new instance of `Aabb`.
	 */
	public void  getIntersectionTo(Aabb other, Aabb intersection) {
		M.vec3_max(intersection._min, this._min, other._min);
		M.vec3_min(intersection._max, this._max, other._max);
	}

	/**
	 * Copies AABB from `aabb` to and returns `this`.
	 */
	public Aabb copyFrom(Aabb aabb) {
		this._min.copyFrom(aabb._min);
		this._max.copyFrom(aabb._max);
		return this;
	}
		
	/**
	 * Returns a clone of the AABB.
	 */
	public Aabb clone() {
		Aabb aabb= new Aabb();
		aabb.set(_min, _max);
		return aabb;
	}
	
	@Override
	public String toString() {
		return "Aabb[min:"+_min+" , max:"+_max+"]";
	}
}