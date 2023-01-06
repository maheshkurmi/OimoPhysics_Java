package oimo.dynamics;
import oimo.collision.*;
import oimo.collision.broadphase.*;
import oimo.collision.broadphase.bruteforce.*;
import oimo.collision.broadphase.bvh.BvhBroadPhase;
import oimo.collision.broadphase.bvh.BvhNode;
import oimo.collision.broadphase.bvh.BvhTree;
import oimo.collision.geometry.*;
import oimo.collision.narrowphase.detector.gjkepa.*;
import oimo.common.*;
import oimo.dynamics.callback.*;
import oimo.dynamics.constraint.*;
import oimo.dynamics.constraint.contact.*;
import oimo.dynamics.constraint.joint.*;
import oimo.dynamics.rigidbody.*;

/**
 * The physics simulation world. This manages entire the dynamic simulation. You can add
 * rigid bodies and joints to the world to simulate them.
 */
public class World {
	public RigidBody _rigidBodyList;
	public RigidBody _rigidBodyListLast;

	public Joint _jointList;
	public Joint _jointListLast;

	public BroadPhase _broadPhase;
	public ContactManager _contactManager;

	public int _numRigidBodies;
	public int _numJoints;
	public int _numShapes;
	public int _numIslands;

	public int _numVelocityIterations;
	public int _numPositionIterations;

	public Vec3 _gravity;

	TimeStep _timeStep;
	Island _island;
	RigidBody[] _rigidBodyStack;
	ConstraintSolver[] _solversInIslands;
	int _numSolversInIslands;

	DebugDraw _debugDraw;
	public Performance performance;
	RayCastWrapper _rayCastWrapper;
	RayCastWrapper.ConvexCastWrapper _convexCastWrapper;
	RayCastWrapper.ConvexCastWrapper.AabbTestWrapper _aabbTestWrapper;

	Pool _pool;

	int  _shapeIdCount;

	/**
	 * Creates a new physics world, with broad-phase collision detection algorithm `broadPhaseType` and
	 * gravitational acceleration `gravity`.
	 * @param broadPhaseType 1=Bruteforce 2=BVM
	 * @param gravity
	 */
	public World(int broadPhaseType ,Vec3 gravity ) {
		broadPhaseType=1;
		switch(broadPhaseType) {
		case BroadPhaseType.BRUTE_FORCE:
			_broadPhase = new BruteForceBroadPhase();
			break;
		case BroadPhaseType.BVH:
		 default:
			_broadPhase = new BvhBroadPhase();

		}
		_contactManager = new ContactManager(_broadPhase);

		if (gravity == null) gravity = new Vec3(0, -9.80665f, 0);
		_gravity = gravity.clone();

		_rigidBodyList = null;
		_rigidBodyListLast = null;

		_jointList = null;
		_jointListLast = null;

		_numRigidBodies = 0;
		_numShapes = 0;
		_numJoints = 0;
		_numIslands = 0;

		_numVelocityIterations = 10;
		_numPositionIterations = 5;

		_rayCastWrapper = new RayCastWrapper();
		_convexCastWrapper = new oimo.dynamics.World.RayCastWrapper.ConvexCastWrapper();
		_aabbTestWrapper = new oimo.dynamics.World.RayCastWrapper.ConvexCastWrapper.AabbTestWrapper();

		_island = new Island();
		_solversInIslands = new ConstraintSolver[Setting.islandInitialConstraintArraySize];
		_rigidBodyStack = new RigidBody[Setting.islandInitialRigidBodyArraySize];

		_timeStep = new TimeStep();

		_pool = new Pool();


		_shapeIdCount = 0;
		performance=new Performance(this);
	}

	void _updateContacts() {
		long st = System.currentTimeMillis();
			// update contacts (broad phase)
		_contactManager._updateContacts();
		long st1=System.currentTimeMillis();
		performance.broadPhaseCollisionTime=st1-st;
		// update manifolds (narrow phase)
		_contactManager._updateManifolds();
		performance.narrowPhaseCollisionTime=System.currentTimeMillis()-st1;
		
	}
	

	void _solveIslands() {
		
		long st = System.currentTimeMillis() ;/// 1000;
		// wake up all rigid bodies if sleeping is disabled
		if(Setting.disableSleeping) {
			RigidBody b = this._rigidBodyList;
			while(b != null) {
				b._sleeping = false;
				b._sleepTime = 0;
				b = b._next;
			}
		}
		// expand array size if needed
		if(this._rigidBodyStack.length < this._numRigidBodies) {
			int newStackSize = this._rigidBodyStack.length << 1;
			while(newStackSize < this._numRigidBodies) newStackSize <<= 1;
			this._rigidBodyStack = new RigidBody[newStackSize];
		}
		// build and solve islands
		_numIslands = 0;
		_island._setGravity(_gravity);
		RigidBody b = _rigidBodyList;
		_numSolversInIslands = 0;
		while(b != null) {
			RigidBody n = b._next;
			while(!(b._addedToIsland || b._sleeping || b._type == 1)) {
				if(b._numContactLinks == 0 && b._numJointLinks == 0) {
					// never be the base of an island
					this._island._stepSingleRigidBody(this._timeStep,b);
					this._numIslands++;
					break;
				}
				this.buildIsland(b);
				this._island._step(this._timeStep,this._numVelocityIterations,this._numPositionIterations);
				this._island._clear();
				this._numIslands++;
				break;
			}
			b = n;
		}
		
		this._contactManager._postSolve();
		
		// clear island flags
		b = this._rigidBodyList;
		while(b != null) {
			b._addedToIsland = false;
			b = b._next;
		}
		
		// clear forces and torques
		b = this._rigidBodyList;
		while(b != null) {
			b._force.zero();
			b._torque.zero();
			b = b._next;
		}
		
		while(this._numSolversInIslands > 0) {
			this._solversInIslands[--this._numSolversInIslands]._addedToIsland = false;
			this._solversInIslands[this._numSolversInIslands] = null;
		}
		performance.dynamicsTime =System.currentTimeMillis()- st;
	
	}

	void buildIsland(RigidBody base) {
		// begin DFS
		int stackCount = 1;
		_island._addRigidBody(base);
		_rigidBodyStack[0] = base;

		while (stackCount > 0) {
			// pop a rigid body
			RigidBody rb = _rigidBodyStack[--stackCount];
			_rigidBodyStack[stackCount] = null;

			// stop searching deeper
			if (rb._type == RigidBodyType._STATIC) {
				continue;
			}

			// searching contacts
			ContactLink cl = rb._contactLinkList;
			while(cl != null) {
				ContactLink n = cl._next;
				ContactConstraint cc = cl._contact._contactConstraint;
				ConstraintSolver ccs = cl._contact._contactConstraint._solver;
				// ignore if not touching
				if(cc.isTouching() && !ccs._addedToIsland) {
					//expand capacity if needed
					if(this._solversInIslands.length == this._numSolversInIslands) {
						ConstraintSolver[] newArray = new ConstraintSolver[this._numSolversInIslands << 1];
						for(int  i=0;i<this._numSolversInIslands;i++) {
							newArray[i] = this._solversInIslands[i];
							this._solversInIslands[i] = null;
						}
						this._solversInIslands = newArray;
					}
					// add to constraint array (to clear island flag later)
					this._solversInIslands[this._numSolversInIslands++] = ccs;
					// add to island
					this._island._addConstraintSolver(ccs,cc._positionCorrectionAlgorithm);
					// push the other rigid body if not added
					RigidBody other = cl._other;
					if(!other._addedToIsland) {
						this._island._addRigidBody(other);
						this._rigidBodyStack[stackCount++] = other;
					}
				}
				cl = n;
			}
			

			// searching joints
			JointLink jl = rb._jointLinkList;
			while(jl != null) {
				JointLink n = jl._next;
				Joint j = jl._joint;
				ConstraintSolver js = j._solver;
				if (!js._addedToIsland) {
					//expand capacity if needed
					if(this._solversInIslands.length == this._numSolversInIslands) {
						ConstraintSolver[] newArray = new ConstraintSolver[this._numSolversInIslands << 1];
						for(int  i=0;i<this._numSolversInIslands;i++) {
							newArray[i] = this._solversInIslands[i];
							this._solversInIslands[i] = null;
						}
						this._solversInIslands = newArray;
					}
					
					// add to constraint array (to clear island flag later)
					_solversInIslands[_numSolversInIslands++] = js;

					// add to island
					_island._addConstraintSolver(js, j._positionCorrectionAlgorithm);

					// push the other rigid body if not added
					RigidBody other = jl._other;
					if (!other._addedToIsland) {
						_island._addRigidBody(other);
						_rigidBodyStack[stackCount++] = other;
					}
				}
				
				jl = n;
			}
			
		}
	}

	public void _addShape(Shape shape) {
		shape._proxy = _broadPhase.createProxy(shape, shape._aabb);
		shape._id = _shapeIdCount++;

		_numShapes++;
	}

	public void _removeShape(Shape shape) {
		_broadPhase.destroyProxy(shape._proxy);
	
		// destroy linked contacts
		ContactLink cl = shape._rigidBody._contactLinkList;
		while(cl != null) {
			Contact c = cl._contact;
			if (c._s1 == shape || c._s2 == shape) {
				cl._other.wakeUp();
				_contactManager._destroyContact(c);
			}
			cl=cl._next;
		}
		shape._proxy = null;
		shape._id = -1;

		_numShapes--;
	}

	void _drawBvh(DebugDraw d,BvhTree tree) {
		if (d.drawBvh) {
			_drawBvhNode(d, tree._root, 0, d.style.bvhNodeColor);
		}
	}

	void _drawBvhNode(DebugDraw d, BvhNode node, int level, Vec3 color) {
		if (node == null) return;
		if (level >= d.drawBvhMinLevel && level <= d.drawBvhMaxLevel) {
			Vec3 min = _pool.vec3();
			Vec3 max = _pool.vec3();
			M.vec3_toVec3(min, node._aabbMin);
			M.vec3_toVec3(max, node._aabbMax);
			d.aabb(min, max, color);
			_pool.disposeVec3(min);
			_pool.disposeVec3(max);
		}
		_drawBvhNode(d, node._children[0], level + 1, color);
		_drawBvhNode(d, node._children[1], level + 1, color);
	}

	void _drawRigidBodies(DebugDraw d) {
		DebugDrawStyle style = d.style;

		RigidBody r = _rigidBodyList;
		while(r != null) {
			RigidBody n = r._next;
			if (d.drawBases) {
				_drawBasis(d, r._transform);
			}
			Vec3 shapeColor = null;
			boolean isDynamic = r._type == RigidBodyType._DYNAMIC;
			if (!isDynamic) {
				shapeColor = r._type == RigidBodyType._KINEMATIC ? style.kinematicShapeColor : style.staticShapeColor;
			}
			Shape s = r._shapeList;
			while(s != null) {
				Shape ns = s._next;
				if (isDynamic) {
					if ((s._id & 1) == 0) {
						shapeColor = r._sleeping ? style.sleepingShapeColor1 : r._sleepTime > oimo.common.Setting.sleepingTimeThreshold ? style.sleepyShapeColor1 : style.shapeColor1;
					} else {
						shapeColor = r._sleeping ? style.sleepingShapeColor2 : r._sleepTime > oimo.common.Setting.sleepingTimeThreshold ? style.sleepyShapeColor2 : style.shapeColor2;
					}
				}
				if (d.drawShapes) {
					_drawShape(d, s._geom, s._transform, shapeColor);
				}
				if (d.drawAabbs) {
					_drawAabb(d, s._aabb, style.aabbColor);
				}
				s=ns;
			}
			r=n;
		}
			
	}

	void   _drawBasis(DebugDraw d, Transform tf) {
		DebugDrawStyle style = d.style;
		d.basis(tf, style.basisLength, style.basisColorX, style.basisColorY, style.basisColorZ);
	}

	void _drawShape(DebugDraw d, Geometry geom, Transform tf, Vec3 color) {
		switch (geom._type) {
		case GeometryType.SPHERE:
			_drawSphere(d,  (SphereGeometry) geom, tf, color);
			break;
		case GeometryType.BOX:
			_drawBox(d,  (BoxGeometry) geom, tf, color);
			break;
		case GeometryType.CYLINDER:
			_drawCylinder(d,  (CylinderGeometry) geom, tf, color);
			break;
		case GeometryType.CONE:
			_drawCone(d,  (ConeGeometry) geom, tf, color);
			break;
		case GeometryType.CAPSULE:
			_drawCapsule(d,  (CapsuleGeometry) geom, tf, color);
			break;
		case GeometryType.CONVEX_HULL:
			_drawConvexHull(d,  (ConvexHullGeometry) geom, tf, color);
			break;
		}
	}

	void _drawSphere(DebugDraw d, SphereGeometry g, Transform tf, Vec3 color) {
		d.sphere(tf, g._radius, color);
	}

	void _drawBox(DebugDraw d, BoxGeometry g, Transform tf, Vec3 color) {
		Vec3 hx = _pool.vec3();
		M.vec3_toVec3(hx, g._halfExtents);
		d.box(tf, hx, color);
		_pool.disposeVec3(hx);
	}

	void _drawCylinder(DebugDraw d, CylinderGeometry g, Transform tf, Vec3 color) {
		d.cylinder(tf, g._radius, g._halfHeight, color);
	}

	void _drawCone(DebugDraw d, ConeGeometry g,Transform tf, Vec3 color) {
		d.cone(tf, g._radius, g._halfHeight, color);
	}

	void _drawCapsule(DebugDraw d, CapsuleGeometry g, Transform tf, Vec3 color) {
		d.capsule(tf, g._radius, g._halfHeight, color);
	}

	void _drawConvexHull(DebugDraw d, ConvexHullGeometry g,Transform tf, Vec3 color) {
		int n = g._numVertices;
		Vec3 v1 = _pool.vec3();
		Vec3 v2 = _pool.vec3();
		Vec3 v3 = _pool.vec3();
		Vec3 v12 = _pool.vec3();
		Vec3 v13 = _pool.vec3();
		Vec3 normal = _pool.vec3();
		Mat3 m = _pool.mat3();
		Vec3 o = _pool.vec3();
		tf.getRotationTo(m);
		tf.getPositionTo(o);

		for (int i=0;i<n;i++) {
			g._tmpVertices[i].copyFrom(g._vertices[i]).mulMat3Eq(m).addEq(o);
		}

		if (n > 30) {
			// O(n)
			for (int i=0;i<n;i++)  {
				v1.copyFrom(g._tmpVertices[i]);
				v2.copyFrom(g._tmpVertices[(i + 1) % n]);
				d.line(v1, v2, color);
			}
		} else if (_debugDraw.wireframe || n > 10) {
			// O(n^2)
			for (int i=0;i<n;i++)  {
				v1.copyFrom(g._tmpVertices[i]);
				for (int j=0;j<i;j++) {
					v2.copyFrom(g._tmpVertices[j]);
					d.line(v1, v2, color);
				}
			}
		} else {
			// O(n^3)
			for (int i=0;i<n;i++)  {
				v1.copyFrom(g._tmpVertices[i]);
				for (int j=0;j<i;j++) {
					v2.copyFrom(g._tmpVertices[j]);
					for (int k=0;k<j;k++) {
						v3.copyFrom(g._tmpVertices[k]);
						v12.copyFrom(v2).subEq(v1);
						v13.copyFrom(v3).subEq(v1);
						normal.copyFrom(v12).crossEq(v13).normalize();
						d.triangle(v1, v2, v3, normal, normal, normal, color);
						normal.negateEq();
						d.triangle(v1, v3, v2, normal, normal, normal, color);
					}
				}
			}
		}
		_pool.disposeVec3(v1);
		_pool.disposeVec3(v2);
		_pool.disposeVec3(v3);
		_pool.disposeVec3(v12);
		_pool.disposeVec3(v13);
		_pool.disposeVec3(normal);
		_pool.disposeMat3(m);
		_pool.disposeVec3(o);
	}

	void _drawAabb(DebugDraw d, Aabb aabb,Vec3 color) {
		Vec3 min = _pool.vec3();
		Vec3 max = _pool.vec3();
		M.vec3_toVec3(min, aabb._min);
		M.vec3_toVec3(max, aabb._max);
		d.aabb(min, max, color);
		_pool.disposeVec3(min);
		_pool.disposeVec3(max);
	}

	void _drawConstraints(DebugDraw d) {
		DebugDrawStyle style = d.style;

		if (d.drawPairs || d.drawContacts) {
			Contact c = _contactManager._contactList;
			while(c != null) {
				Contact n = c._next;
				if (d.drawPairs) {
					_drawPair(d, c, style.pairColor);
				}
				if (d.drawContacts) {
					ContactConstraint cc = c._contactConstraint;
					ManifoldPoint[] ps = c._contactConstraint._manifold._points;
					for (int i=0;i<c._contactConstraint._manifold._numPoints;i++) {
						_drawContactPoint(d, cc, ps[i]);
					}
				}
				c=n;
			};
		}
		if (d.drawJoints) {
			Joint j = _jointList;
			while(j != null) {
				Joint n = j._next;
				_drawJoint(d, j);
				j=n;
			}
		}
	}

	void _drawContactPoint(DebugDraw d, ContactConstraint c, ManifoldPoint p) {
		DebugDrawStyle style = d.style;
		Transform tf1 = c._s1._transform;
		Transform tf2 = c._s2._transform;
		Vec3 pos1 = _pool.vec3();
		Vec3 pos2 = _pool.vec3();
		Vec3 normal = _pool.vec3();
		Vec3 tangent = _pool.vec3();
		Vec3 binormal = _pool.vec3();

		M.vec3_toVec3(pos1, p._pos1);
		M.vec3_toVec3(pos2, p._pos2);
		M.vec3_toVec3(normal, c._manifold._normal);
		M.vec3_toVec3(tangent, c._manifold._tangent);
		M.vec3_toVec3(binormal, c._manifold._binormal);

		if (p._disabled) {
			d.point(pos1, style.disabledContactColor);
			d.point(pos2, style.disabledContactColor);
			d.line(pos1, pos2, style.disabledContactColor);
		} else if (p._warmStarted) {
			Vec3 color;
			switch (p._id & 3) {
			case 0:
				color = style.contactColor;
				break;
			case 1:
				color = style.contactColor2;
				break;
			case 2:
				color = style.contactColor3;
				break;
			default:
				color = style.contactColor4;
			}
			d.point(pos1, color);
			d.point(pos2, color);
			d.line(pos1, pos2, style.contactColor);
		} else {
			d.point(pos1, style.newContactColor);
			d.point(pos2, style.newContactColor);
			d.line(pos1, pos2, style.newContactColor);
		}

		pos2.copyFrom(pos1).addScaledEq(normal, style.contactNormalLength);
		d.line(pos1, pos2, style.contactNormalColor);

		if (d.drawContactBases) {
			pos2.copyFrom(pos1).addScaledEq(tangent, style.contactTangentLength);
			d.line(pos1, pos2, style.contactTangentColor);
			pos2.copyFrom(pos1).addScaledEq(binormal, style.contactBinormalLength);
			d.line(pos1, pos2, style.contactBinormalColor);
		}

		_pool.disposeVec3(pos1);
		_pool.disposeVec3(pos2);
		_pool.disposeVec3(normal);
		_pool.disposeVec3(tangent);
		_pool.disposeVec3(binormal);
	}

	void _drawPair(DebugDraw d, Contact c, Vec3 color) {
		Vec3 v1 = _pool.vec3();
		Vec3 v2 = _pool.vec3();
		M.vec3_toVec3(v1, c._s1._transform._position);
		M.vec3_toVec3(v2, c._s2._transform._position);
		d.line(v1, v2, color);
		_pool.disposeVec3(v1);
		_pool.disposeVec3(v2);
	}

	void _drawJoint(DebugDraw d, Joint j) {
		Vec3 p1 = _pool.vec3();
		Vec3 p2 = _pool.vec3();
		M.vec3_toVec3(p1, j._b1._transform._position);
		M.vec3_toVec3(p2, j._b2._transform._position);

		Vec3 anchor1 = _pool.vec3();
		Vec3 anchor2 = _pool.vec3();
		Vec3 basisX1 = _pool.vec3();
		Vec3 basisY1 = _pool.vec3();
		Vec3 basisZ1 = _pool.vec3();
		Vec3 basisX2 = _pool.vec3();
		Vec3 basisY2 = _pool.vec3();
		Vec3 basisZ2 = _pool.vec3();
		M.vec3_toVec3(anchor1, j._anchor1);
		M.vec3_toVec3(anchor2, j._anchor2);
		M.vec3_toVec3(basisX1, j._basisX1);
		M.vec3_toVec3(basisY1, j._basisY1);
		M.vec3_toVec3(basisZ1, j._basisZ1);
		M.vec3_toVec3(basisX2, j._basisX2);
		M.vec3_toVec3(basisY2, j._basisY2);
		M.vec3_toVec3(basisZ2, j._basisZ2);

		d.line(p1, anchor1, d.style.jointLineColor);
		d.line(p2, anchor2, d.style.jointLineColor);

		if (d.drawJointLimits) {
			switch (j._type) {
			case JointType.SPHERICAL:
				break;
				// draw nothing here
			case JointType._REVOLUTE:
				_drawRevolute(d,  (RevoluteJoint) j, anchor1, anchor2, basisX1, basisY1, basisZ1, basisX2, basisY2, basisZ2);
				break;
			case JointType.CYLINDRICAL:
				_drawCylindrical(d,  (CylindricalJoint) j, anchor1, anchor2, basisX1, basisY1, basisZ1, basisX2, basisY2, basisZ2);
				break;
			case JointType.PRISMATIC:
				_drawPrismatic(d,  (PrismaticJoint) j, anchor1, anchor2, basisX1, basisY1, basisZ1, basisX2, basisY2, basisZ2);
				break;
			case JointType.UNIVERSAL:
				_drawUniversal(d,  (UniversalJoint) j, anchor1, anchor2, basisX1, basisY1, basisZ1, basisX2, basisY2, basisZ2);
				break;
			case JointType.RAGDOLL:
				_drawRagdoll(d,  (RagdollJoint) j, anchor1, anchor2, basisX1, basisY1, basisZ1, basisX2, basisY2, basisZ2);
				break;
			case JointType.GENERIC:
				_drawGeneric(d,  (GenericJoint) j, anchor1, anchor2, basisX1, basisY1, basisZ1, basisX2, basisY2, basisZ2);
				break;
			}
		}

		d.line(anchor1, anchor2, d.style.jointErrorColor);

		_pool.disposeVec3(p1);
		_pool.disposeVec3(p2);
		_pool.disposeVec3(anchor1);
		_pool.disposeVec3(anchor2);
		_pool.disposeVec3(basisX1);
		_pool.disposeVec3(basisY1);
		_pool.disposeVec3(basisZ1);
		_pool.disposeVec3(basisX2);
		_pool.disposeVec3(basisY2);
		_pool.disposeVec3(basisZ2);
	}

	void _drawRevolute(DebugDraw d, RevoluteJoint j, Vec3 anchor1, Vec3 anchor2, Vec3 basisX1, Vec3 basisY1, Vec3 basisZ1, Vec3 basisX2, Vec3 basisY2, Vec3 basisZ2) {
		double radius = d.style.jointRotationalConstraintRadius;
		Vec3 color = d.style.jointLineColor;
		RotationalLimitMotor lm = j._lm;
		_drawRotationalLimit(d, anchor1, basisY1, basisZ1, basisY2, radius, lm.lowerLimit, lm.upperLimit, color);
	}

	void _drawCylindrical(DebugDraw d, CylindricalJoint j, Vec3 anchor1, Vec3 anchor2, Vec3 basisX1, Vec3 basisY1, Vec3 basisZ1, Vec3 basisX2, Vec3 basisY2, Vec3 basisZ2) {
		double radius = d.style.jointRotationalConstraintRadius;
		Vec3 color = d.style.jointLineColor;
		RotationalLimitMotor rlm = j._rotLm;
		TranslationalLimitMotor tlm = j._translLm;

		_drawRotationalLimit(d, anchor2, basisY1, basisZ1, basisY2, radius, rlm.lowerLimit, rlm.upperLimit, color);
		_drawTranslationalLimit(d, anchor1, basisX1, tlm.lowerLimit, tlm.upperLimit, color);
	}

	void _drawPrismatic(DebugDraw d, PrismaticJoint j, Vec3 anchor1, Vec3 anchor2, Vec3 basisX1, Vec3 basisY1, Vec3 basisZ1, Vec3 basisX2, Vec3 basisY2, Vec3 basisZ2) {
		double radius = d.style.jointRotationalConstraintRadius;
		Vec3 color = d.style.jointLineColor;
		TranslationalLimitMotor lm = j._lm;

		_drawTranslationalLimit(d, anchor1, basisX1, lm.lowerLimit, lm.upperLimit, color);
	}

	void  _drawUniversal(DebugDraw d, UniversalJoint j, Vec3 anchor1, Vec3 anchor2, Vec3 basisX1, Vec3 basisY1, Vec3 basisZ1, Vec3 basisX2, Vec3 basisY2, Vec3 basisZ2) {
		double radius = d.style.jointRotationalConstraintRadius;
		Vec3 color = d.style.jointLineColor;
		RotationalLimitMotor lm1 = j._lm1;
		RotationalLimitMotor lm2 = j._lm2;

		_drawRotationalLimit(d, anchor1, basisY1, basisZ1, basisY1, radius, j._angleX - lm1.upperLimit, j._angleX - lm1.lowerLimit, color);
		_drawRotationalLimit(d, anchor2, basisX2, basisY2, basisX2, radius, lm2.lowerLimit - j._angleZ, lm2.upperLimit - j._angleZ, color);
	}

	void  _drawRagdoll(DebugDraw d, RagdollJoint j, Vec3 anchor1, Vec3 anchor2, Vec3 basisX1, Vec3 basisY1, Vec3 basisZ1, Vec3 basisX2, Vec3 basisY2, Vec3 basisZ2) {
		double radius = d.style.jointRotationalConstraintRadius;
		Vec3 color = d.style.jointLineColor;
		RotationalLimitMotor lm = j._twistLm;

		_drawRotationalLimit(d, anchor2, basisY2, basisZ2, basisY2, radius, lm.lowerLimit - j._twistAngle, lm.upperLimit - j._twistAngle, color);
		_drawEllipseOnSphere(d, anchor1, basisX1, basisY1, basisZ1, j._maxSwingAngle1, j._maxSwingAngle2, radius, color);

		Vec3 to = _pool.vec3().copyFrom(anchor2).addScaledEq(basisX2, radius);
		d.line(anchor2, to, color);
		_pool.disposeVec3(to);
	}

	void  _drawGeneric(DebugDraw d, GenericJoint j,Vec3 anchor1, Vec3 anchor2, Vec3 basisX1, Vec3 basisY1, Vec3 basisZ1, Vec3 basisX2, Vec3 basisY2, Vec3 basisZ2) {
		double radius = d.style.jointRotationalConstraintRadius;
		Vec3 color = d.style.jointLineColor;
		TranslationalLimitMotor txlm = j._translLms[0];
		TranslationalLimitMotor tylm = j._translLms[1];
		TranslationalLimitMotor tzlm = j._translLms[2];
		RotationalLimitMotor rxlm = j._rotLms[0];
		RotationalLimitMotor rylm = j._rotLms[1];
		RotationalLimitMotor rzlm = j._rotLms[2];
		_drawTranslationalLimit3D(d, anchor1, basisX1, basisY1, basisZ1, txlm, tylm, tzlm, color);
		
		Vec3 rotYAxis = _pool.vec3();
		M.vec3_toVec3(rotYAxis, j._axisY);
		Vec3 rotYBasisX = _pool.vec3().copyFrom(basisX1);
		Vec3 rotYBasisY = _pool.vec3().copyFrom(basisX1).crossEq(rotYAxis);
		
		_drawRotationalLimit(d, anchor2, basisY1, basisZ1, basisY1, radius, j._angleX - rxlm.upperLimit, j._angleX - rxlm.lowerLimit, color);
		_drawRotationalLimit(d, anchor2, rotYBasisX, rotYBasisY, rotYBasisX, radius, rylm.lowerLimit - j._angleY, rylm.upperLimit - j._angleY, color);
		_drawRotationalLimit(d, anchor2, basisX2, basisY2, basisX2, radius, rzlm.lowerLimit - j._angleZ, rzlm.upperLimit - j._angleZ, color);
	}

	void _drawRotationalLimit(DebugDraw d, Vec3 center, Vec3 ex, Vec3 ey, Vec3 needle, double radius, double min, double max, Vec3 color) {
		if (min != max) {
			Vec3 to = _pool.vec3().copyFrom(center).addScaledEq(needle, radius);
			d.line(center, to, color);
			_pool.disposeVec3(to);
			if (min > max) {
				d.ellipse(center, ex, ey, radius, radius, color);
			} else {
				d.arc(center, ex, ey, radius, radius, min, max, true, color);
			}
		}
	}

	void _drawTranslationalLimit(DebugDraw d, Vec3 center, Vec3 ex, double min, double max, Vec3 color) {
		if (min < max) {
			Vec3 lower = _pool.vec3().copyFrom(center).addScaledEq(ex, min);
			Vec3 upper = _pool.vec3().copyFrom(center).addScaledEq(ex, max);
			d.line(lower, upper, color);
			_pool.disposeVec3(lower);
			_pool.disposeVec3(upper);
		}
	}

	void _drawTranslationalLimit3D(DebugDraw d, Vec3 center, Vec3 ex, Vec3 ey, Vec3 ez, TranslationalLimitMotor xlm, TranslationalLimitMotor ylm, TranslationalLimitMotor zlm, Vec3 color) {
		double minx = xlm.lowerLimit;
		double maxx = xlm.upperLimit;
		double miny = ylm.lowerLimit;
		double maxy = ylm.upperLimit;
		double minz = zlm.lowerLimit;
		double maxz = zlm.upperLimit;
		Vec3 lower = _pool.vec3();
		Vec3 upper = _pool.vec3();
		Vec3 xyz = _pool.vec3().copyFrom(center).addScaledEq(ex, minx).addScaledEq(ey, miny).addScaledEq(ez, minz);
		Vec3 xyZ = _pool.vec3().copyFrom(center).addScaledEq(ex, minx).addScaledEq(ey, miny).addScaledEq(ez, maxz);
		Vec3 xYz = _pool.vec3().copyFrom(center).addScaledEq(ex, minx).addScaledEq(ey, maxy).addScaledEq(ez, minz);
		Vec3 xYZ = _pool.vec3().copyFrom(center).addScaledEq(ex, minx).addScaledEq(ey, maxy).addScaledEq(ez, maxz);
		Vec3 Xyz = _pool.vec3().copyFrom(center).addScaledEq(ex, maxx).addScaledEq(ey, miny).addScaledEq(ez, minz);
		Vec3 XyZ = _pool.vec3().copyFrom(center).addScaledEq(ex, maxx).addScaledEq(ey, miny).addScaledEq(ez, maxz);
		Vec3 XYz = _pool.vec3().copyFrom(center).addScaledEq(ex, maxx).addScaledEq(ey, maxy).addScaledEq(ez, minz);
		Vec3 XYZ = _pool.vec3().copyFrom(center).addScaledEq(ex, maxx).addScaledEq(ey, maxy).addScaledEq(ez, maxz);
		// x
		d.line(xyz, Xyz, color);
		d.line(xYz, XYz, color);
		d.line(xyZ, XyZ, color);
		d.line(xYZ, XYZ, color);
		// y
		d.line(xyz, xYz, color);
		d.line(Xyz, XYz, color);
		d.line(xyZ, xYZ, color);
		d.line(XyZ, XYZ, color);
		// z
		d.line(xyz, xyZ, color);
		d.line(Xyz, XyZ, color);
		d.line(xYz, xYZ, color);
		d.line(XYz, XYZ, color);
		_pool.disposeVec3(xyz);
		_pool.disposeVec3(xyZ);
		_pool.disposeVec3(xYz);
		_pool.disposeVec3(xYZ);
		_pool.disposeVec3(Xyz);
		_pool.disposeVec3(XyZ);
		_pool.disposeVec3(XYz);
		_pool.disposeVec3(XYZ);
	}

	void _drawEllipseOnSphere(DebugDraw d,Vec3 center,Vec3 normal, Vec3 x,Vec3 y, double radiansX, double radiansY, double radius, Vec3 color) {
		int n = 16;
		double theta = 0;
		double dTheta = MathUtil.TWO_PI / n;

		Vec3 rotVec = _pool.vec3();
		Quat rotQ = _pool.quat();
		Mat3 rotM = _pool.mat3();
		Vec3 prevV = _pool.vec3();

		for (int i=0; i<n + 1;i++) {
			double rx = MathUtil.cos(theta) * radiansX;
			double ry = MathUtil.sin(theta) * radiansY;

			double halfRotAng = MathUtil.sqrt(rx * rx + ry * ry);
			double rotSin = MathUtil.sin(halfRotAng * 0.5f);
			double rotCos = MathUtil.cos(halfRotAng * 0.5f);
			rotVec.zero().addScaledEq(x, rx).addScaledEq(y, ry);
			rotVec.scaleEq(1 / halfRotAng * rotSin);
			rotQ.init(rotVec.x, rotVec.y, rotVec.z, rotCos);
			rotM.fromQuat(rotQ);

			Vec3 v = _pool.vec3().addScaledEq(normal, radius);
			v.mulMat3Eq(rotM).addEq(center);

			if (i >= 1) {
				d.line(prevV, v, color);
			}

			_pool.disposeVec3(prevV);
			prevV = v;
			theta += dTheta;
		}

		_pool.disposeVec3(rotVec);
		_pool.disposeQuat(rotQ);
		_pool.disposeMat3(rotM);
		_pool.disposeVec3(prevV);
	}

	// --- public ---

	/**
	 * Advances the simulation by the time step `timeStep`.
	 */
	public void step(double timeStep) {
		if (_timeStep.dt > 0) {
			_timeStep.dtRatio = timeStep / _timeStep.dt;
		}
		_timeStep.dt = timeStep;
		_timeStep.invDt = 1 / timeStep;
		long st =System.currentTimeMillis() ;
		this._updateContacts();
		this._solveIslands();
		performance.totalTime =System.currentTimeMillis()-st;
	
	}

	/**
	 * Adds the rigid body `rigidBody` to the simulation world.
	 */
	public void addRigidBody(RigidBody rigidBody) {
		if (rigidBody._world != null) {
			 M.error("A rigid body cannot belong to multiple worlds.");
			 return;
		}

		// first, add the rigid body to the world
		//M.list_push(_rigidBodyList, _rigidBodyListLast, _prev, _next, rigidBody);
		if(this._rigidBodyList == null) {
			this._rigidBodyList = rigidBody;
			this._rigidBodyListLast = rigidBody;
		} else {
			this._rigidBodyListLast._next = rigidBody;
			rigidBody._prev = this._rigidBodyListLast;
			this._rigidBodyListLast = rigidBody;
		}
		rigidBody._world = this;

		// then add the shapes to the world
		//		M.list_foreach(s, _next, {
		//			_addShape(s);
		//		});
		Shape s = rigidBody._shapeList;
		while(s != null) {
			_addShape(s);
			s = s._next;;
		}
		_numRigidBodies++;
	}

	/**
	 * Removes the rigid body `rigidBody` from the simulation world.
	 */
	public void removeRigidBody(RigidBody rigidBody) {
		if (rigidBody._world != this) {
			 M.error("The rigid body doesn't belong to the world.");
			 return;
		}
		// first, remove the rigid body from the world
		//M.list_remove(_rigidBodyList, _rigidBodyListLast, _prev, _next, rigidBody);
		RigidBody prev = rigidBody._prev;
		RigidBody next = rigidBody._next;
		if(prev != null) {
			prev._next = next;
		}
		if(next != null) {
			next._prev = prev;
		}
		if(rigidBody == this._rigidBodyList) {
			this._rigidBodyList = this._rigidBodyList._next;
		}
		if(rigidBody == this._rigidBodyListLast) {
			this._rigidBodyListLast = this._rigidBodyListLast._prev;
		}
		rigidBody._next = null;
		rigidBody._prev = null;
		rigidBody._world = null;

		// then remove the shapes from the world
		//Shape s = rigidBody._shapeList;
		//		M.list_foreach(s, _next, {
		//			_removeShape(s);
		//		});
		Shape s = rigidBody._shapeList;
		
		while(s != null) {
			_removeShape(s);
			s=s._next;
		}
		_numRigidBodies--;
	}

	/**
	 * Adds the joint `joint` to the simulation world.
	 */
	public void addJoint(Joint joint) {
		if (joint._world != null) {
			 M.error("A joint cannot belong to multiple worlds.");
			 return;
		}
		//M.list_push(_jointList, _jointListLast, _prev, _next, joint);

		if(this._jointList == null) {
			this._jointList = joint;
			this._jointListLast = joint;
		} else {
			this._jointListLast._next = joint;
			joint._prev = this._jointListLast;
			this._jointListLast = joint;
		}
		joint._world = this;
		joint._attachLinks();
		joint._syncAnchors();

		_numJoints++;
	}

	/**
	 * Removes the joint `joint` from the simulation world.
	 */
	public void removeJoint(Joint joint) {
		if (joint._world != this) {
			 M.error("The joint doesn't belong to the world.");
			 return;
		}
		//M.list_remove(_jointList, _jointListLast, _prev, _next, joint);
		Joint prev = joint._prev;
		Joint next = joint._next;
		if(prev != null) {
			prev._next = next;
		}
		if(next != null) {
			next._prev = prev;
		}
		if(joint == this._jointList) {
			this._jointList = this._jointList._next;
		}
		if(joint == this._jointListLast) {
			this._jointListLast = this._jointListLast._prev;
		}
		joint._world = null;
		joint._detachLinks();

		_numJoints--;
	}

	/**
	 * Sets the debug draw interface to `debugDraw`. Call `World.debugDraw` to draw the simulation world.
	 */
	public void setDebugDraw(DebugDraw debugDraw) {
		_debugDraw = debugDraw;
	}

	/**
	 * Returns the debug draw interface.
	 */
	public DebugDraw getDebugDraw() {
		return _debugDraw;
	}

	/**
	 * Draws the simulation world for debugging. Call `World.setDebugDraw` to set the debug draw interface.
	 */
	public void debugDraw() {
		if (_debugDraw != null) {
			if (_broadPhase._type == BroadPhaseType.BVH) {
				BvhBroadPhase bvhBroadPhase =  (BvhBroadPhase) _broadPhase;
				_drawBvh(_debugDraw, bvhBroadPhase._tree);
			}
			_drawRigidBodies(_debugDraw);
			_drawConstraints(_debugDraw);
		}
	}

	/**
	 * Performs a ray casting. `callback.process` is called for all shapes the ray
	 * from `begin` to `end` hits.
	 */
	public void rayCast(Vec3 begin, Vec3 end, RayCastCallback callback) {
		_rayCastWrapper.begin.copyFrom(begin);
		_rayCastWrapper.end.copyFrom(end);
		_rayCastWrapper.callback = callback;

		_broadPhase.rayCast(begin, end, _rayCastWrapper);
	}

	/**
	 * Performs a convex casting. `callback.process` is called for all shapes the convex geometry
	 * `convex` hits. The convex geometry translates by `translation` starting from the beginning
	 * transform `begin`.
	 */
	public void convexCast(ConvexGeometry convex, Transform begin, Vec3 translation,RayCastCallback callback) {
		_convexCastWrapper.convex = convex;
		_convexCastWrapper.begin.copyFrom(begin);
		_convexCastWrapper.translation.copyFrom(translation);
		_convexCastWrapper.callback = callback;

		_broadPhase.convexCast(convex, begin, translation, _convexCastWrapper);
	}

	/**
	 * Performs an AABB query. `callback.process` is called for all shapes that their
	 * AABB and `aabb` intersect.
	 */
	public void aabbTest(Aabb aabb,AabbTestCallback callback) {
		_aabbTestWrapper._aabb.copyFrom(aabb);
		_aabbTestWrapper._callback = callback;

		_broadPhase.aabbTest(aabb, _aabbTestWrapper);
	}

	/**
	 * Returns the list of the rigid bodies added to the world.
	 */
	public RigidBody getRigidBodyList() {
		return _rigidBodyList;
	}

	/**
	 * Returns the list of the joints added to the world.
	 */
	public Joint getJointList() {
		return _jointList;
	}

	/**
	 * Returns the broad-phase collision detection algorithm.
	 */
	public BroadPhase getBroadPhase() {
		return _broadPhase;
	}

	/**
	 * Returns the contact manager.
	 */
	public ContactManager getContactManager() {
		return _contactManager;
	}

	/**
	 * Returns the number of the rigid bodies added to the world.
	 */
	public int getNumRigidBodies() {
		return _numRigidBodies;
	}

	/**
	 * Returns the number of the joints added to the world.
	 */
	public int getNumJoints() {
		return _numJoints;
	}

	/**
	 * Returns the number of the shapes added to the world.
	 */
	public int getNumShapes() {
		return _numShapes;
	}

	/**
	 * Returns the number of simulation islands.
	 */
	public int getNumIslands() {
		return _numIslands;
	}

	/**
	 * Returns the number of velocity iterations of constraint solvers.
	 */
	public int getNumVelocityIterations() {
		return _numVelocityIterations;
	}

	/**
	 * Sets the number of velocity iterations of constraint solvers to `numVelocityIterations`.
	 */
	public void setNumVelocityIterations(int numVelocityIterations) {
		_numVelocityIterations = numVelocityIterations;
	}

	/**
	 * Returns the number of position iterations of constraint solvers.
	 */
	public int getNumPositionIterations() {
		return _numPositionIterations;
	}

	/**
	 * Sets the number of position iterations of constraint solvers to `numPositionIterations`.
	 */
	public void setNumPositionIterations(int numPositionIterations) {
		_numPositionIterations = numPositionIterations;
	}

	/**
	 * Returns the gravitational acceleration of the simulation world.
	 */
	public Vec3 getGravity() {
		return _gravity;
	}

	/**
	 * Sets the gravitational acceleration of the simulation world to `gravity`.
	 */
	public void setGravity(Vec3 gravity) {
		_gravity.copyFrom(gravity);
	}

	// ray cast wrapper (broadphase -> world)
	protected static class RayCastWrapper extends BroadPhaseProxyCallback {
		public RayCastCallback callback;
		public Vec3 begin;
		public Vec3 end;

		RayCastHit rayCastHit;

		public RayCastWrapper() {
			super();
			rayCastHit = new RayCastHit();

			begin = new Vec3();
			end = new Vec3();
			callback = null;
		}

		@Override
		public void process(Proxy proxy) {
			Shape shape = (Shape) proxy.userData;

			if (shape._geom.rayCast(begin, end, shape._transform, rayCastHit)) {
				callback.process(shape, rayCastHit);
			}
		}

		// convex cast wrapper (broadphase -> world)
		protected static class ConvexCastWrapper extends BroadPhaseProxyCallback {
			public RayCastCallback callback;
			public Transform begin;
			public Vec3 translation;
			public ConvexGeometry convex;

			RayCastHit rayCastHit;
			Vec3 zero;

			public ConvexCastWrapper() {
				super();
				rayCastHit = new RayCastHit();

				begin = new Transform();
				translation = new Vec3();
				zero = new Vec3();
				callback = null;
				convex = null;
			}

			@Override
			public void process(Proxy proxy) {
				Shape shape = (Shape) proxy.userData;
				int type = shape._geom._type;
				if (type < GeometryType._CONVEX_MIN || type > GeometryType._CONVEX_MAX)
					return;

				ConvexGeometry geom = (ConvexGeometry) shape._geom;
				if (GjkEpa.getInstance().convexCast(convex, geom, begin, shape._transform, translation, zero,
						rayCastHit)) {
					callback.process(shape, rayCastHit);
				}
			}

			// aabb test wrapper (broadphase -> world)
			protected static class AabbTestWrapper extends BroadPhaseProxyCallback {
				public AabbTestCallback _callback;
				public Aabb _aabb;

				public AabbTestWrapper() {
					super();
					_aabb = new Aabb();
					_callback = null;
				}

				@Override
				public void process(Proxy proxy) {
					Shape shape = (Shape) proxy.userData;
					Aabb shapeAabb = shape._aabb;

					// check if aabbs overlap again as proxies can be fattened by broadphase
					if (M.aabb_overlap(shapeAabb._min, shapeAabb._max, _aabb._min, _aabb._max)) {
						_callback.process(shape);
					}
				}
			}
		}
	}
}


