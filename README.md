# OimoPhysics_Java
Pure java port (jdk 9+) of [OimoPhysics 1.2.4 3d Engine](https://saharan.github.io/OimoPhysics/), using jogl to render demoes.
Since engine is ported manually, it includes several java specific optimisations and some bug fixes, but the the code structure almost remains same, therefore original API documentation is till applicable. 
This engine will serve as the base of 3D physics engine for [SimPHY](https://www.simphy.com/).

---

## [API Documentation](https://saharan.github.io/OimoPhysics/)

## Demos
<a href="https://el-ement.com/etc/oimo/demos/"><img src="https://el-ement.com/etc/oimo/demos/thumbnail.png"></a><br>
To run demoes in Eclipse follow following steps
* Import the project in eclipse
* Run the eclipse project
* Press `E` or `Q` to change demos
* Press Keys specified in info on right side to adjust settings

## Platforms
* Written in core Java, so compatible with windows/mac/linux/android

## Features
* Rigid body with motion types
	* Dynamic
	* Static
	* Kinematic
* Fast collision detection with bounding volume hierarchy (BVH)
* Contacts with friction and restitution
* Collision geometries
	* Sphere
	* Box
	* Cylinder
	* Cone
	* Capsule
	* Convex hull
* Joints with springs, limits and motors
	* Spherical (a.k.a. ball and socket, point to point)
	* Revolute (a.k.a. hinge)
	* Cylindrical
	* Prismatic (a.k.a. slider)
	* Universal
	* Ragdoll (a.k.a. cone twist, character)
	* Generic (a.k.a. 6-DoF joint)
* Breakable joints
* Constraint solvers
	* Direct block MLCP solver
	* Projected Gauss-Seidel solver
* Sleepings with island splittings
* Rotation limits
* Collision event callbacks
* Collision filterings
* Collision queries
	* AABB query
	* Ray casting
	* Convex casting

## License
The MIT License

---
