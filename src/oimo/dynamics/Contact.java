package oimo.dynamics;
import oimo.collision.narrowphase.*;
import oimo.collision.narrowphase.detector.*;
import oimo.common.Setting;
import oimo.dynamics.callback.ContactCallback;
import oimo.dynamics.constraint.contact.*;
import oimo.dynamics.rigidbody.*;

/**
 * A contact is a cached pair of overlapping shapes in the physics world. contacts
 * are created by `ContactManager` when two AABBs of shapes begin overlapping.
 *
 * As AABBs are larger than its shapes, shapes of a contact don't always
 * touching or colliding though their AABBs are overlapping.
 */
public class Contact {
	public Contact _next;
	public Contact _prev;

	public ContactLink _link1;
	public ContactLink _link2;

	public Shape _s1;
	public Shape _s2;
	public RigidBody _b1;
	public RigidBody _b2;

	// detector data
	public Detector _detector;
	public CachedDetectorData _cachedDetectorData;
	public DetectorResult _detectorResult;

	// tmp data
	public boolean _latest;
	public boolean _shouldBeSkipped;

	// constraint/manifold data
	public Manifold _manifold;
	public ManifoldUpdater _updater;
	public ContactConstraint _contactConstraint;
	public boolean _touching;

	public Contact() {
		_next = null;
		_prev = null;

		_link1 = new ContactLink();
		_link2 = new ContactLink();

		_s1 = null;
		_s2 = null;
		_b1 = null;
		_b2 = null;

		_detector = null;
		_cachedDetectorData = new CachedDetectorData();
		_detectorResult = new DetectorResult();

		_latest = false;
		_shouldBeSkipped = false;

		_manifold = new Manifold();
		_updater = new ManifoldUpdater(_manifold);
		_contactConstraint = new ContactConstraint(_manifold);
		_touching = false;
	}

	// --- private ---

	public void attachLinks() {
		
		//M.list_push(_b1._contactLinkList, _b1._contactLinkListLast, _prev, _next, _link1);
		if (_b1._contactLinkList == null) {
			_b1._contactLinkList = _link1;
			_b1._contactLinkListLast = _link1;
		} else {
			_b1._contactLinkListLast._next = _link1;
			_link1._prev = _b1._contactLinkListLast;
			_b1._contactLinkListLast = _link1;
		}
    
		//M.list_push(_b2._contactLinkList, _b2._contactLinkListLast, _prev, _next, _link2);
		if (_b2._contactLinkList == null) {
			_b2._contactLinkList = _link2;
			_b2._contactLinkListLast = _link2;
		} else {
			_b2._contactLinkListLast._next = _link2;
			_link2._prev = _b2._contactLinkListLast;
			_b2._contactLinkListLast = _link2;
		}

		_b1._numContactLinks++;
		_b2._numContactLinks++;
		_link1._other = _b2;
		_link2._other = _b1;
		_link1._contact = this;
		_link2._contact = this;
	}

	public void detachLinks() {
		//M.list_remove(_b1._contactLinkList, _b1._contactLinkListLast, _prev, _next, _link1);

		{
			ContactLink prev = _link1._prev;
			ContactLink next = _link1._next;
			if (( prev != null )) 
			{
				prev._next = next;
			}
			
			if (( next != null )) 
			{
				next._prev = prev;
			}
			
			if (( _link1 == _b1._contactLinkList )) 
			{
				_b1._contactLinkList = _b1._contactLinkList._next;
			}
			
			if (( _link1 == _b1._contactLinkListLast )) 
			{
				_b1._contactLinkListLast = _b1._contactLinkListLast._prev;
			}
			
			_link1._next = null;
			_link1._prev = null;
		}
		
		//M.list_remove(_b2._contactLinkList, _b2._contactLinkListLast, _prev, _next, _link2);
		{
			ContactLink prev = _link2._prev;
			ContactLink next = _link2._next;
			if (( prev != null )) 
			{
				prev._next = next;
			}
			
			if (( next != null )) 
			{
				next._prev = prev;
			}
			
			if (( _link2 == _b2._contactLinkList )) 
			{
				_b2._contactLinkList = _b2._contactLinkList._next;
			}
			
			if (( _link2 == _b2._contactLinkListLast )) 
			{
				_b2._contactLinkListLast = _b2._contactLinkListLast._prev;
			}
			
			_link2._next = null;
			_link2._prev = null;
		}
		
		_b1._numContactLinks--;
		_b2._numContactLinks--;
		_link1._other = null;
		_link2._other = null;
		_link1._contact = null;
		_link2._contact = null;
	}

	private void sendBeginContact() {
		ContactCallback cc1 = _s1._contactCallback;
		ContactCallback cc2 = _s2._contactCallback;
		if (cc1 == cc2) {
			cc2 = null; // avoid calling twice
		}
		if (cc1 != null) cc1.beginContact(this);
		if (cc2 != null) cc2.beginContact(this);
	}

	private void  sendEndContact() {
		ContactCallback cc1 = _s1._contactCallback;
		ContactCallback cc2 = _s2._contactCallback;
		if (cc1 == cc2) {
			cc2 = null; // avoid calling twice
		}
		if (cc1 != null) cc1.endContact(this);
		if (cc2 != null) cc2.endContact(this);
	}

	private void  sendPreSolve() {
		ContactCallback cc1 = _s1._contactCallback;
		ContactCallback cc2 = _s2._contactCallback;
		if (cc1 == cc2) {
			cc2 = null; // avoid calling twice
		}
		if (cc1 != null) cc1.preSolve(this);
		if (cc2 != null) cc2.preSolve(this);
	}

	private void  sendPostSolve() {
		ContactCallback cc1 = _s1._contactCallback;
		ContactCallback cc2 = _s2._contactCallback;
		if (cc1 == cc2) {
			cc2 = null; // avoid calling twice
		}
		if (cc1 != null) cc1.postSolve(this);
		if (cc2 != null) cc2.postSolve(this);
	}

	// --- internal ---

	 void _attach(Shape s1, Shape s2, Detector detector) {
		_s1 = s1;
		_s2 = s2;
		_b1 = s1._rigidBody;
		_b2 = s2._rigidBody;
		_touching = false;
		attachLinks();

		_detector = detector;

		_contactConstraint._attach(s1, s2);
	}

	void _detach() {
		if (_touching) {
			// touching in the last frame
			sendEndContact();
		}

		detachLinks();
		_s1 = null;
		_s2 = null;
		_b1 = null;
		_b2 = null;
		_touching = false;

		_cachedDetectorData._clear();
		_manifold._clear();

		_detector = null;

		_contactConstraint._detach();
	}

	public void _updateManifold() {
		if (_detector == null) return;

		boolean ptouching = _touching;

		DetectorResult result = _detectorResult;
		_detector.detect(result, _s1._geom, _s2._geom, _s1._transform, _s2._transform, _cachedDetectorData);

		int num = result.numPoints;
		_touching = num > 0;

		if (_touching) {
			// update manifold basis
			_manifold._buildBasis(result.normal);

			// determine position correction algorithm
			if (result.getMaxDepth() > Setting.contactUseAlternativePositionCorrectionAlgorithmDepthThreshold) {
				// use alternative position correction method (split impulse by default) for deeply overlapped contacts
				_contactConstraint._positionCorrectionAlgorithm = Setting.alternativeContactPositionCorrectionAlgorithm;
			} else {
				// use default position correction algorithm for slightly overlapped contacts
				_contactConstraint._positionCorrectionAlgorithm = Setting.defaultContactPositionCorrectionAlgorithm;
			}

			// update contact manifold
			if (result.incremental) {
				// incremental manifold
				_updater.incrementalUpdate(result, _b1._transform, _b2._transform);
			} else {
				// one-shot manifold
				_updater.totalUpdate(result, _b1._transform, _b2._transform);
			}
		} else {
			_manifold._clear();
		}

		if (_touching && !ptouching) {
			sendBeginContact();
		}
		if (!_touching && ptouching) {
			sendEndContact();
		}
		if (_touching) {
			sendPreSolve();
		}
	}

	// called from the contact manager
	public void _postSolve() {
		sendPostSolve();
	}

	// --- public ---

	/**
	 * Returns the first shape of the contact.
	 */
	public Shape getShape1() {
		return _s1;
	}

	/**
	 * Returns the second shape of the contact.
	 */
	public Shape  getShape2() {
		return _s2;
	}

	/**
	 * Returns whether the shapes are touching.
	 */
	public boolean isTouching() {
		return _touching;
	}

	/**
	 * Returns the contact manifold.
	 */
	public Manifold getManifold() {
		return _manifold;
	}

	/**
	 * Returns the contact constraint.
	 */
	public ContactConstraint getContactConstraint() {
		return _contactConstraint;
	}

	/**
	 * Returns the previous contact in the world.
	 *
	 * If the previous contact does not exist, `null` will be returned.
	 */
	public Contact getPrev() {
		return _prev;
	}

	/**
	 * Returns the next contact in the world.
	 *
	 * If the next contact does not exist, `null` will be returned.
	 */
	public Contact getNext() {
		return _next;
	}

}
