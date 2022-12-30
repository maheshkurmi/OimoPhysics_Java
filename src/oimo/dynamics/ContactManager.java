package oimo.dynamics;
import oimo.collision.broadphase.*;
import oimo.collision.geometry.*;
import oimo.collision.narrowphase.*;
import oimo.collision.narrowphase.detector.Detector;
import oimo.common.M;
import oimo.dynamics.constraint.contact.ContactConstraint;
import oimo.dynamics.constraint.joint.*;
import oimo.dynamics.rigidbody.*;

/**
 * The manager of the contacts in the physics world. A contact of two
 * shapes is created when the AABBs of them begin overlapping, and
 * is destroyed when they end overlapping.
 */
public class ContactManager {
	public int _numContacts;
	public Contact _contactList;
	public Contact _contactListLast;
	public Contact _contactPool;

	public BroadPhase _broadPhase;
	public CollisionMatrix _collisionMatrix;

	public ContactManager(BroadPhase broadPhase) {
		_broadPhase = broadPhase;
		_collisionMatrix = new CollisionMatrix();
		_numContacts = 0;
	}

	// --- private ---

	private void createContacts() {
		ProxyPair pp = _broadPhase._proxyPairList;
		while(pp != null) {
			ProxyPair n = pp._next;
			while(true) {
				Shape s1;
				Shape s2;
				//M.assert(pp._p1._id != pp._p2._id);
				if (pp._p1._id < pp._p2._id) {
					s1 =  (Shape) pp._p1.userData;
					s2 =  (Shape) pp._p2.userData;
				} else {
					s1 =  (Shape) pp._p2.userData;
					s2 =  (Shape) pp._p1.userData;
				}
				// collision filtering
				if (!shouldCollide(s1, s2)) {
					break;
				}

				// search for the same contact
				RigidBody b1 = s1._rigidBody;
				RigidBody b2 = s2._rigidBody;
				int n1 = b1._numContactLinks;
				int n2 = b2._numContactLinks;
				ContactLink l;
				// select shorter linked list
				if (n1 < n2) {
					l = b1._contactLinkList;
				} else {
					l = b2._contactLinkList;
				}
				int id1 = s1._id;
				int id2 = s2._id;
				boolean found = false;
				while(l != null) {
					Contact c = l._contact;
					if(c._s1._id == id1 && c._s2._id == id2) {
						c._latest = true;
						found = true;
						break;
					}
					l = l._next;
				}

				// if not found, create a new contact
				if (!found) {
					// trying to pick an object up from the pool
					Contact first = this._contactPool;
					if(first != null) {
						this._contactPool = first._next;
						first._next = null;
					} else {
						first = new oimo.dynamics.Contact();
					}
					Contact c = first;
					if(this._contactList == null) {
						this._contactList = c;
						this._contactListLast = c;
					} else {
						this._contactListLast._next = c;
						c._prev = this._contactListLast;
						this._contactListLast = c;
					}
					c._latest = true;
					Detector detector = this._collisionMatrix.detectors[s1._geom._type][s2._geom._type];
					c._s1 = s1;
					c._s2 = s2;
					c._b1 = s1._rigidBody;
					c._b2 = s2._rigidBody;
					c._touching = false;
					if(c._b1._contactLinkList == null) {
						c._b1._contactLinkList = c._link1;
						c._b1._contactLinkListLast = c._link1;
					} else {
						c._b1._contactLinkListLast._next = c._link1;
						c._link1._prev = c._b1._contactLinkListLast;
						c._b1._contactLinkListLast = c._link1;
					}
					if(c._b2._contactLinkList == null) {
						c._b2._contactLinkList = c._link2;
						c._b2._contactLinkListLast = c._link2;
					} else {
						c._b2._contactLinkListLast._next = c._link2;
						c._link2._prev = c._b2._contactLinkListLast;
						c._b2._contactLinkListLast = c._link2;
					}
					c._b1._numContactLinks++;
					c._b2._numContactLinks++;
					c._link1._other = c._b2;
					c._link2._other = c._b1;
					c._link1._contact = c;
					c._link2._contact = c;
					c._detector = detector;
					ContactConstraint _this = c._contactConstraint;
					_this._s1 = s1;
					_this._s2 = s2;
					_this._b1 = _this._s1._rigidBody;
					_this._b2 = _this._s2._rigidBody;
					_this._tf1 = _this._b1._transform;
					_this._tf2 = _this._b2._transform;
					this._numContacts++;
//					var c:Contact = M.singleList_pick(_contactPool, _next, new Contact());
//					M.list_push(_contactList, _contactListLast, _prev, _next, c);
//					c._latest = true;
//					c._attach(s1, s2, _collisionMatrix.getDetector(s1._geom._type, s2._geom._type));
//					_numContacts++;
				}
				break;
			} 
			pp = n;
		}
	}

	private void destroyOutdatedContacts() {
		// whether the broadphase returns only new overlapping pairs
		boolean incremental = _broadPhase._incremental;

		Contact c = _contactList;
		while(c != null) {
			Contact n = c._next;
			while(true) {
				if (c._latest) {
					// the contact is overlapping, make it old for the next step
					c._latest = false;
					c._shouldBeSkipped = false;
					break;
				}
				if (!incremental) {
					// the pair is separated, because the broad-phase algorithm collects
					// all the overlapping pairs and they are marked as latest
					_destroyContact(c);
					break;
				}

				Shape s1 = c._s1;
				Shape s2 = c._s2;
				RigidBody r1 = s1._rigidBody;
				RigidBody r2 = s2._rigidBody;
				boolean active1 = !r1._sleeping && r1._type != RigidBodyType._STATIC;
				boolean active2 = !r2._sleeping && r2._type != RigidBodyType._STATIC;
				if (!active1 && !active2) {
					// skip the pair if both rigid bodies are inactive
					c._shouldBeSkipped = true;
					break;
				}
				Aabb aabb1 = s1._aabb;
				Aabb aabb2 = s2._aabb;
				if (!_broadPhase.isOverlapping(s1._proxy, s2._proxy) || !shouldCollide(s1, s2)) {
					// the proxy pair is separated or shouldn't collide
					_destroyContact(c);
					break;
				}
				
				// the proxies are overlapping, but AABBs might be separated
				boolean aabbOverlapping = M.aabb_overlap(aabb1._min, aabb1._max, aabb2._min, aabb2._max);
				// needs narrow-phase collision detection if AABBs are overlapping
				c._shouldBeSkipped = !aabbOverlapping;
				break;
			} 
			c = n;
		};
	}

	private boolean shouldCollide(Shape s1, Shape s2) {
		RigidBody r1 = s1._rigidBody;
		RigidBody r2 = s2._rigidBody;

		if (r1 == r2) {
			// they have the same parent
			return false;
		}

		if (r1._type != RigidBodyType._DYNAMIC && r2._type != RigidBodyType._DYNAMIC) {
			// none of the two bodies are dynamic
			return false;
		}

		// collision filtering
		if (
			(s1._collisionGroup & s2._collisionMask) == 0 ||
			(s2._collisionGroup & s1._collisionMask) == 0
		) {
			return false;
		}

		// search for joints the two bodies are connected to
		JointLink jl;
		RigidBody other;
		if (r1._numJointLinks < r2._numJointLinks) {
			jl = r1._jointLinkList;
			other = r2;
		} else {
			jl = r2._jointLinkList;
			other = r1;
		}
		while(jl != null) {
			if(jl._other == other && !jl._joint._allowCollision) {
				// collisions between the two bodies are disabled
				return false;
			}
			jl = jl._next;
		}
		
//		M.list_foreach(jl, _next, {
//			if (jl._other == other && !jl._joint._allowCollision) {
//				// collisions between the two bodies are disabled
//				return false;
//			}
//		});

		return true;
	}

	// --- internal ---

	void _updateContacts() {
		_broadPhase.collectPairs();
		createContacts();
		destroyOutdatedContacts();
	}

	// send postSolve events
	 void _postSolve() {
		Contact c = _contactList;
		while(c != null) {
			Contact n = c._next;
			if(c._touching) {
				c._postSolve();
			}
			c = n;
		}
		
//		M.list_foreach(c, _next, {
//			if (c._touching) {
//				c._postSolve();
//			}
//		});
	}

	public void _updateManifolds() {
		Contact c = _contactList;
		while(c != null) {
			Contact n = c._next;
			if (!c._shouldBeSkipped) {
				c._updateManifold();
			}
			c = n;
		}
//		M.list_foreach(c, _next, {
//			if (!c._shouldBeSkipped) {
//				c._updateManifold();
//			}
//		});
	}

	public void _destroyContact(Contact c) {
		Contact prev = c._prev;
		Contact next = c._next;
		if(prev != null) {
			prev._next = next;
		}
		if(next != null) {
			next._prev = prev;
		}
		if(c == this._contactList) {
			this._contactList = this._contactList._next;
		}
		if(c == this._contactListLast) {
			this._contactListLast = this._contactListLast._prev;
		}
		c._next = null;
		c._prev = null;
		
		
		//M.list_remove(_contactList, _contactListLast, _prev, _next, contact);
		c._detach();

		// put it into the pool
		c._next = this._contactPool;
		this._contactPool = c;
		//M.singleList_pool(_contactPool, _next, contact);
		this._numContacts--;
		
	}

	// --- public ---

	/**
	 * Returns the number of the contacts in the world.
	 */
	public int getNumContacts() {
		return _numContacts;
	}

	/**
	 * Returns the linked list of the contacts in the world.
	 */
	public Contact getContactList() {
		return _contactList;
	}

}
