
jiglib = {};

jiglib.extend = function(dest, source)
{
	for (proto in source.prototype)
	{
		dest.prototype[proto] = source.prototype[proto];
	}
};

var trace = function(message) {};


(function(jiglib) {

	var JWheel = jiglib.JWheel;
	var JChassis = jiglib.JChassis;
	var Vector3D = jiglib.Vector3D;
	var JNumber3D = jiglib.JNumber3D;
	var PhysicsSystem = jiglib.PhysicsSystem;

	var JCar = function(skin)
	{
		this._maxSteerAngle = null; // Number
		this._steerRate = null; // Number
		this._driveTorque = null; // Number
		this._destSteering = null; // Number
		this._destAccelerate = null; // Number
		this._steering = null; // Number
		this._accelerate = null; // Number
		this._HBrake = null; // Number
		this._chassis = null; // JChassis
		this._wheels = null; // Array
		this._steerWheels = null; // Array

		this._chassis = new JChassis(this, skin);
		this._wheels = [];
		this._steerWheels = [];
		this._destSteering = this._destAccelerate = this._steering = this._accelerate = this._HBrake = 0;
		this.setCar();
		
	}

	JCar.prototype.setCar = function(maxSteerAngle, steerRate, driveTorque)
	{
		if (maxSteerAngle == null) maxSteerAngle = 45;
		if (steerRate == null) steerRate = 1;
		if (driveTorque == null) driveTorque = 500;

		this._maxSteerAngle = maxSteerAngle;
		this._steerRate = steerRate;
		this._driveTorque = driveTorque;
		
	}

	JCar.prototype.setupWheel = function(_name, pos, wheelSideFriction, wheelFwdFriction, wheelTravel, wheelRadius, wheelRestingFrac, wheelDampingFrac, wheelNumRays)
	{
		if (wheelSideFriction == null) wheelSideFriction = 2;
		if (wheelFwdFriction == null) wheelFwdFriction = 2;
		if (wheelTravel == null) wheelTravel = 3;
		if (wheelRadius == null) wheelRadius = 10;
		if (wheelRestingFrac == null) wheelRestingFrac = 0.5;
		if (wheelDampingFrac == null) wheelDampingFrac = 0.5;
		if (wheelNumRays == null) wheelNumRays = 1;

		var mass = this._chassis.get_mass();
		var mass4 = 0.25 * mass;
		
		var gravity = PhysicsSystem.getInstance().get_gravity().clone();
		var gravityLen = PhysicsSystem.getInstance().get_gravity().get_length();
		gravity.normalize();
		var axis = JNumber3D.getScaleVector(gravity, -1);
		var spring = mass4 * gravityLen / (wheelRestingFrac * wheelTravel);
		var inertia = 0.015 * wheelRadius * wheelRadius * mass;
		var damping = 2 * Math.sqrt(spring * mass);
		damping *= (0.25 * wheelDampingFrac);

		this._wheels[_name] = new JWheel(this);
		this._wheels[_name].setup(pos, axis, spring, wheelTravel, inertia,
			wheelRadius, wheelSideFriction, wheelFwdFriction,
			damping, wheelNumRays);
		
	}

	JCar.prototype.get_chassis = function()
	{

		return this._chassis;
		
	}

	JCar.prototype.get_wheels = function()
	{

		return this._wheels;
		
	}

	JCar.prototype.setAccelerate = function(val)
	{

		this._destAccelerate = val;
		
	}

	JCar.prototype.setSteer = function(wheels, val)
	{

		this._destSteering = val;
		this._steerWheels = [];
		for (var i in wheels)
		{
			if (this.findWheel(wheels[i]))
			{
				this._steerWheels[wheels[i]] = this._wheels[wheels[i]];
			}
		}
		
	}

	JCar.prototype.findWheel = function(_name)
	{

		for (var i in this._wheels)
		{
			if (i == _name)
			{
				return true;
			}
		}
		return false;
		
	}

	JCar.prototype.setHBrake = function(val)
	{

		this._HBrake = val;
		
	}

	JCar.prototype.addExternalForces = function(dt)
	{

		for (var wheels_i = 0, wheels_l = this.get_wheels().length, wheel; (wheels_i < wheels_l) && (wheel = this.get_wheels()[wheels_i]); wheels_i++)
		{
			wheel.addForcesToCar(dt);
		}
		
	}

	JCar.prototype.postPhysics = function(dt)
	{

		var wheel;
		for (var wheels_i = 0, wheels_l = this.get_wheels().length, wheel; (wheels_i < wheels_l) && (wheel = this.get_wheels()[wheels_i]); wheels_i++)
		{
			wheel.update(dt);
		}

		var deltaAccelerate, deltaSteering, dAccelerate, dSteering, alpha, angleSgn;
		deltaAccelerate = dt;
		deltaSteering = dt * this._steerRate;
		dAccelerate = this._destAccelerate - this._accelerate;
		if (dAccelerate < -deltaAccelerate)
		{
			dAccelerate = -deltaAccelerate;
		}
		else if (dAccelerate > deltaAccelerate)
		{
			dAccelerate = deltaAccelerate;
		}
		this._accelerate += dAccelerate;

		dSteering = this._destSteering - this._steering;
		if (dSteering < -deltaSteering)
		{
			dSteering = -deltaSteering;
		}
		else if (dSteering > deltaSteering)
		{
			dSteering = deltaSteering;
		}
		this._steering += dSteering;

		for (var wheels_i = 0, wheels_l = this.get_wheels().length, wheel; (wheels_i < wheels_l) && (wheel = this.get_wheels()[wheels_i]); wheels_i++)
		{
			wheel.addTorque(this._driveTorque * this._accelerate);
			wheel.setLock(this._HBrake > 0.5);
		}

		alpha = Math.abs(this._maxSteerAngle * this._steering);
		angleSgn = (this._steering > 0) ? 1 : -1;
		for (var _steerWheels_i = 0, _steerWheels_l = this._steerWheels.length, _steerWheel; (_steerWheels_i < _steerWheels_l) && (_steerWheel = this._steerWheels[_steerWheels_i]); _steerWheels_i++)
		{
			_steerWheel.setSteerAngle(angleSgn * alpha);
		}
		
	}

	JCar.prototype.getNumWheelsOnFloor = function()
	{

		var count = 0;
		for (var wheels_i = 0, wheels_l = this.get_wheels().length, wheel; (wheels_i < wheels_l) && (wheel = this.get_wheels()[wheels_i]); wheels_i++)
		{
			if (wheel.getOnFloor())
			{
				count++;
			}
		}
		return count;
		
	}



	jiglib.JCar = JCar; 

})(jiglib);


(function(jiglib) {

	var JCar = jiglib.JCar;
	var JChassis = jiglib.JChassis;
	var Vector3D = jiglib.Vector3D;
	var CollisionSystemAbstract = jiglib.CollisionSystemAbstract;
	var CollOutBodyData = jiglib.CollOutBodyData;
	var JSegment = jiglib.JSegment;
	var JMatrix3D = jiglib.JMatrix3D;
	var Matrix3D = jiglib.Matrix3D;
	var JMath3D = jiglib.JMath3D;
	var JNumber3D = jiglib.JNumber3D;
	var PhysicsSystem = jiglib.PhysicsSystem;
	var RigidBody = jiglib.RigidBody;

	var JWheel = function(car)
	{
		this.noslipVel =  0.2; // Number
		this.slipVel =  0.4; // Number
		this.slipFactor =  0.7; // Number
		this.smallVel =  3; // Number
		this._car = null; // JCar
		this._pos = null; // Vector3D
		this._axisUp = null; // Vector3D
		this._spring = null; // Number
		this._travel = null; // Number
		this._inertia = null; // Number
		this._radius = null; // Number
		this._sideFriction = null; // Number
		this._fwdFriction = null; // Number
		this._damping = null; // Number
		this._numRays = null; // int
		this._angVel = null; // Number
		this._steerAngle = null; // Number
		this._torque = null; // Number
		this._driveTorque = null; // Number
		this._axisAngle = null; // Number
		this._displacement = null; // Number
		this._upSpeed = null; // Number
		this._rotDamping = null; // Number
		this._locked = null; // Boolean
		this._lastDisplacement = null; // Number
		this._lastOnFloor = null; // Boolean
		this._angVelForGrip = null; // Number
		this.worldPos = null; // Vector3D
		this.worldAxis = null; // Vector3D
		this.wheelFwd = null; // Vector3D
		this.wheelUp = null; // Vector3D
		this.wheelLeft = null; // Vector3D
		this.wheelRayEnd = null; // Vector3D
		this.wheelRay = null; // JSegment
		this.groundUp = null; // Vector3D
		this.groundLeft = null; // Vector3D
		this.groundFwd = null; // Vector3D
		this.wheelPointVel = null; // Vector3D
		this.rimVel = null; // Vector3D
		this.worldVel = null; // Vector3D
		this.wheelCentreVel = null; // Vector3D
		this._collisionSystem = null; // CollisionSystemAbstract

		this._car = car;
		
	}

	JWheel.prototype.setup = function(pos, axisUp, spring, travel, inertia, radius, sideFriction, fwdFriction, damping, numRays)
	{

		this._pos = pos;
		this._axisUp = axisUp;
		this._spring = spring;
		this._travel = travel;
		this._inertia = inertia;
		this._radius = radius;
		this._sideFriction = sideFriction;
		this._fwdFriction = fwdFriction;
		this._damping = damping;
		this._numRays = numRays;
		this.reset();
		
	}

	JWheel.prototype.addTorque = function(torque)
	{

		this._driveTorque += torque;
		
	}

	JWheel.prototype.setLock = function(lock)
	{

		this._locked = lock;
		
	}

	JWheel.prototype.setSteerAngle = function(steer)
	{

		this._steerAngle = steer;
		
	}

	JWheel.prototype.getSteerAngle = function()
	{

		return this._steerAngle;
		
	}

	JWheel.prototype.getPos = function()
	{

		return this._pos;
		
	}

	JWheel.prototype.getLocalAxisUp = function()
	{

		return this._axisUp;
		
	}

	JWheel.prototype.getActualPos = function()
	{

		return this._pos.add(JNumber3D.getScaleVector(this._axisUp, this._displacement));
		
	}

	JWheel.prototype.getRadius = function()
	{

		return this._radius;
		
	}

	JWheel.prototype.getDisplacement = function()
	{

		return this._displacement;
		
	}

	JWheel.prototype.getAxisAngle = function()
	{

		return this._axisAngle;
		
	}

	JWheel.prototype.getRollAngle = function()
	{

		return 0.1 * this._angVel * 180 / Math.PI;
		
	}

	JWheel.prototype.setRotationDamping = function(vel)
	{

		this._rotDamping = vel;
		
	}

	JWheel.prototype.getRotationDamping = function()
	{

		return this._rotDamping;
		
	}

	JWheel.prototype.getOnFloor = function()
	{

		return this._lastOnFloor;
		
	}

	JWheel.prototype.addForcesToCar = function(dt)
	{

		var force = new Vector3D();
		this._lastDisplacement = this._displacement;
		this._displacement = 0;

		var carBody = this._car.get_chassis();
		this.worldPos = carBody.get_currentState().orientation.transformVector(this._pos);
		this.worldPos = carBody.get_currentState().position.add(this.worldPos);
		this.worldAxis = carBody.get_currentState().orientation.transformVector(this._axisUp);

		this.wheelFwd = JMatrix3D.getRotationMatrix(this.worldAxis.x, this.worldAxis.y, this.worldAxis.z, this._steerAngle).transformVector(carBody.get_currentState().getOrientationCols()[2]);
		this.wheelUp = this.worldAxis;
		this.wheelLeft = this.wheelUp.crossProduct(this.wheelFwd);
		this.wheelLeft.normalize();

		var rayLen = 2 * this._radius + this._travel;
		this.wheelRayEnd = this.worldPos.subtract(JNumber3D.getScaleVector(this.worldAxis, this._radius));
		this.wheelRay = new JSegment(this.wheelRayEnd.add(JNumber3D.getScaleVector(this.worldAxis, rayLen)), JNumber3D.getScaleVector(this.worldAxis, -rayLen));

		if(!this._collisionSystem)
			this._collisionSystem = PhysicsSystem.getInstance().getCollisionSystem();

		var maxNumRays = 10;
		var numRays = Math.min(this._numRays, maxNumRays);

		var objArr = [];
		var segments = [];

		var deltaFwd = (2 * this._radius) / (numRays + 1);
		var deltaFwdStart = deltaFwd;

		this._lastOnFloor = false;

		var distFwd;
		var yOffset;
		var bestIRay = 0;
		var iRay = 0;
		var collOutBodyData;
		var segment;
		for (iRay = 0; iRay < numRays; iRay++)
		{
			collOutBodyData = objArr[iRay] = new CollOutBodyData();
			distFwd = (deltaFwdStart + iRay * deltaFwd) - this._radius;
			yOffset = this._radius * (1 - Math.cos(90 * (distFwd / this._radius) * Math.PI / 180));
			segment = segments[iRay] = this.wheelRay.clone();
			segment.origin = segment.origin.add(JNumber3D.getScaleVector(this.wheelFwd, distFwd).add(JNumber3D.getScaleVector(this.wheelUp, yOffset)));
			if (this._collisionSystem.segmentIntersect(collOutBodyData, segment, carBody))
			{
				this._lastOnFloor = true;
				if (collOutBodyData.frac < objArr[bestIRay].frac)
				{
				bestIRay = iRay;
				}
			}
		}

		if (!this._lastOnFloor)
		{
			return false;
		}

		var frac = objArr[bestIRay].frac;
		var groundPos = objArr[bestIRay].position;
		var otherBody = objArr[bestIRay].rigidBody;

		var groundNormal = this.worldAxis.clone();
		if (numRays > 1)
		{
			for (iRay = 0; iRay < numRays; iRay++)
			{
				collOutBodyData = objArr[iRay];
				if (collOutBodyData.frac <= 1)
				groundNormal = groundNormal.add(JNumber3D.getScaleVector(this.worldPos.subtract(segments[iRay].getEnd()), 1 - collOutBodyData.frac));
			}
			
			groundNormal.normalize();
		}
		else
		{
			groundNormal = objArr[bestIRay].normal;
		}

		this._displacement = rayLen * (1 - frac);
		
		if (this._displacement < 0)
			this._displacement = 0;
		else if (this._displacement > this._travel)
			this._displacement = this._travel;

		var displacementForceMag = this._displacement * this._spring;
		displacementForceMag *= groundNormal.dotProduct(this.worldAxis);

		var dampingForceMag = this._upSpeed * this._damping;
		var totalForceMag = displacementForceMag + dampingForceMag;
		if (totalForceMag < 0)
			totalForceMag = 0;
		
		var extraForce = JNumber3D.getScaleVector(this.worldAxis, totalForceMag);
		force = force.add(extraForce);

		this.groundUp = groundNormal;
		this.groundLeft = groundNormal.crossProduct(this.wheelFwd);
		this.groundLeft.normalize();
		this.groundFwd = this.groundLeft.crossProduct(this.groundUp);

		var tempv = carBody.get_currentState().orientation.transformVector(this._pos);
		this.wheelPointVel = carBody.get_currentState().linVelocity.add(carBody.get_currentState().rotVelocity.crossProduct(tempv));

		this.rimVel = JNumber3D.getScaleVector(this.wheelLeft.crossProduct(groundPos.subtract(this.worldPos)), this._angVel);
		this.wheelPointVel = this.wheelPointVel.add(this.rimVel);

		if (otherBody.get_movable())
		{
			this.worldVel = otherBody.get_currentState().linVelocity.add(otherBody.get_currentState().rotVelocity.crossProduct(groundPos.subtract(otherBody.get_currentState().position)));
			this.wheelPointVel = this.wheelPointVel.subtract(this.worldVel);
		}

		var friction = this._sideFriction;
		var sideVel = this.wheelPointVel.dotProduct(this.groundLeft);
		
		if ((sideVel > this.slipVel) || (sideVel < -this.slipVel))
			friction *= this.slipFactor;
		else if ((sideVel > this.noslipVel) || (sideVel < -this.noslipVel))
			friction *= (1 - (1 - this.slipFactor) * (Math.abs(sideVel) - this.noslipVel) / (this.slipVel - this.noslipVel));
		
		if (sideVel < 0)
		{
			friction *= -1;
		}
		if (Math.abs(sideVel) < this.smallVel)
		{
			friction *= Math.abs(sideVel) / this.smallVel;
		}

		var sideForce = -friction * totalForceMag;
		extraForce = JNumber3D.getScaleVector(this.groundLeft, sideForce);
		force = force.add(extraForce);

		friction = this._fwdFriction;
		var fwdVel = this.wheelPointVel.dotProduct(this.groundFwd);
		if ((fwdVel > this.slipVel) || (fwdVel < -this.slipVel))
		{
			friction *= this.slipFactor;
		}
		else if ((fwdVel > this.noslipVel) || (fwdVel < -this.noslipVel))
		{
			friction *= (1 - (1 - this.slipFactor) * (Math.abs(fwdVel) - this.noslipVel) / (this.slipVel - this.noslipVel));
		}
		if (fwdVel < 0)
		{
			friction *= -1;
		}
		if (Math.abs(fwdVel) < this.smallVel)
		{
			friction *= (Math.abs(fwdVel) / this.smallVel);
		}
		var fwdForce = -friction * totalForceMag;
		extraForce = JNumber3D.getScaleVector(this.groundFwd, fwdForce);
		force = force.add(extraForce);

		this.wheelCentreVel = carBody.get_currentState().linVelocity.add(carBody.get_currentState().rotVelocity.crossProduct(tempv));
		this._angVelForGrip = this.wheelCentreVel.dotProduct(this.groundFwd) / this._radius;
		this._torque += (-fwdForce * this._radius);

		carBody.addWorldForce(force, groundPos, false);
		if (otherBody.get_movable())
		{
			var maxOtherBodyAcc = 500;
			var maxOtherBodyForce = maxOtherBodyAcc * otherBody.get_mass();
			if (force.get_lengthSquared() > maxOtherBodyForce * maxOtherBodyForce)
			{
				force = JNumber3D.getScaleVector(force, maxOtherBodyForce / force.get_length());
			}
			otherBody.addWorldForce(JNumber3D.getScaleVector(force, -1), groundPos, false);
		}
		return true;
		
	}

	JWheel.prototype.update = function(dt)
	{

		if (dt <= 0)
		{
			return;
		}
		var origAngVel = this._angVel;
		this._upSpeed = (this._displacement - this._lastDisplacement) / Math.max(dt, JMath3D.NUM_TINY);

		if (this._locked)
		{
			this._angVel = 0;
			this._torque = 0;
		}
		else
		{
			this._angVel += (this._torque * dt / this._inertia);
			this._torque = 0;

			if (((origAngVel > this._angVelForGrip) && (this._angVel < this._angVelForGrip)) || ((origAngVel < this._angVelForGrip) && (this._angVel > this._angVelForGrip)))
			{
				this._angVel = this._angVelForGrip;
			}

			this._angVel += this._driveTorque * dt / this._inertia;
			this._driveTorque = 0;

			if (this._angVel < -100)
			{
				this._angVel = -100;
			}
			else if (this._angVel > 100)
			{
				this._angVel = 100;
			}
			this._angVel *= this._rotDamping;
			this._axisAngle += (this._angVel * dt * 180 / Math.PI);
		}

		
	}

	JWheel.prototype.reset = function()
	{

		this._angVel = 0;
		this._steerAngle = 0;
		this._torque = 0;
		this._driveTorque = 0;
		this._axisAngle = 0;
		this._displacement = 0;
		this._upSpeed = 0;
		this._locked = false;
		this._lastDisplacement = 0;
		this._lastOnFloor = false;
		this._angVelForGrip = 0;
		this._rotDamping = 0.99;
		
	}



	jiglib.JWheel = JWheel; 

})(jiglib);


(function(jiglib) {

	var CollDetectBoxPlane = jiglib.CollDetectBoxPlane;
	var CollDetectBoxMesh = jiglib.CollDetectBoxMesh;
	var CollDetectBoxBox = jiglib.CollDetectBoxBox;
	var CollDetectSphereTerrain = jiglib.CollDetectSphereTerrain;
	var CollDetectSphereBox = jiglib.CollDetectSphereBox;
	var CollDetectCapsuleTerrain = jiglib.CollDetectCapsuleTerrain;
	var CollDetectSphereCapsule = jiglib.CollDetectSphereCapsule;
	var CollisionSystemBrute = jiglib.CollisionSystemBrute;
	var CollDetectCapsuleBox = jiglib.CollDetectCapsuleBox;
	var CollDetectSphereMesh = jiglib.CollDetectSphereMesh;
	var CollDetectBoxTerrain = jiglib.CollDetectBoxTerrain;
	var CollisionSystemGrid = jiglib.CollisionSystemGrid;
	var CollDetectCapsuleCapsule = jiglib.CollDetectCapsuleCapsule;
	var CollPointInfo = jiglib.CollPointInfo;
	var CollisionInfo = jiglib.CollisionInfo;
	var CollisionSystemAbstract = jiglib.CollisionSystemAbstract;
	var CollDetectCapsulePlane = jiglib.CollDetectCapsulePlane;
	var CollDetectInfo = jiglib.CollDetectInfo;
	var CollDetectSphereSphere = jiglib.CollDetectSphereSphere;
	var CollisionSystemGridEntry = jiglib.CollisionSystemGridEntry;
	var CollDetectSpherePlane = jiglib.CollDetectSpherePlane;

	var CollDetectFunctor = function()
	{
		this.name = null; // String
		this.type0 = null; // String
		this.type1 = null; // String
	}

	CollDetectFunctor.prototype.collDetect = function(info, collArr)
	{

		
	}



	jiglib.CollDetectFunctor = CollDetectFunctor; 

})(jiglib);


(function(jiglib) {

	var CollDetectBoxPlane = jiglib.CollDetectBoxPlane;
	var CollDetectBoxMesh = jiglib.CollDetectBoxMesh;
	var CollDetectBoxBox = jiglib.CollDetectBoxBox;
	var CollDetectSphereTerrain = jiglib.CollDetectSphereTerrain;
	var CollDetectSphereBox = jiglib.CollDetectSphereBox;
	var CollDetectCapsuleTerrain = jiglib.CollDetectCapsuleTerrain;
	var CollDetectSphereCapsule = jiglib.CollDetectSphereCapsule;
	var CollisionSystemBrute = jiglib.CollisionSystemBrute;
	var CollDetectCapsuleBox = jiglib.CollDetectCapsuleBox;
	var CollDetectSphereMesh = jiglib.CollDetectSphereMesh;
	var CollisionSystemGrid = jiglib.CollisionSystemGrid;
	var CollDetectCapsuleCapsule = jiglib.CollDetectCapsuleCapsule;
	var CollPointInfo = jiglib.CollPointInfo;
	var CollisionInfo = jiglib.CollisionInfo;
	var CollisionSystemAbstract = jiglib.CollisionSystemAbstract;
	var CollDetectCapsulePlane = jiglib.CollDetectCapsulePlane;
	var CollDetectInfo = jiglib.CollDetectInfo;
	var CollDetectSphereSphere = jiglib.CollDetectSphereSphere;
	var CollisionSystemGridEntry = jiglib.CollisionSystemGridEntry;
	var CollDetectSpherePlane = jiglib.CollDetectSpherePlane;
	var CollDetectFunctor = jiglib.CollDetectFunctor;
	var Vector3D = jiglib.Vector3D;
	var JConfig = jiglib.JConfig;
	var TerrainData = jiglib.TerrainData;
	var JBox = jiglib.JBox;
	var JTerrain = jiglib.JTerrain;
	var JNumber3D = jiglib.JNumber3D;
	var MaterialProperties = jiglib.MaterialProperties;
	var RigidBody = jiglib.RigidBody;

	var CollDetectBoxTerrain = function()
	{

		this.name = "BoxTerrain";
		this.type0 = "BOX";
		this.type1 = "TERRAIN";
		
	}

	jiglib.extend(CollDetectBoxTerrain, CollDetectFunctor);

	CollDetectBoxTerrain.prototype.collDetect = function(info, collArr)
	{

		var tempBody;
		if (info.body0.get_type() == "TERRAIN")
		{
			tempBody = info.body0;
			info.body0 = info.body1;
			info.body1 = tempBody;
		}
		
		var box = info.body0;
		var terrain = info.body1;
				
		var oldPts = box.getCornerPoints(box.get_oldState());
		var newPts = box.getCornerPoints(box.get_currentState());
		var collNormal = new Vector3D();
		
		var obj;
		var dist;
		var newPt;
		var oldPt;
		
		var collPts = [];
		var cpInfo;
		
		for (var i = 0; i < 8; i++ ) {
			newPt = newPts[i];
			obj = terrain.getHeightAndNormalByPoint(newPt);
			
			if (obj.height < JConfig.collToll) {
				oldPt = oldPts[i];
				dist = terrain.getHeightByPoint(oldPt);
				collNormal = collNormal.add(obj.normal);
				cpInfo = new CollPointInfo();
				cpInfo.r0 = oldPt.subtract(box.get_oldState().position);
				cpInfo.r1 = oldPt.subtract(terrain.get_oldState().position);
				cpInfo.initialPenetration = -dist;
				collPts.push(cpInfo);
			}
		}
		
		if (collPts.length > 0) {
			collNormal.normalize();
			
			var collInfo = new CollisionInfo();
			collInfo.objInfo = info;
			collInfo.dirToBody = collNormal;
			collInfo.pointInfo = collPts;
			
			var mat = new MaterialProperties();
			mat.restitution = 0.5*(box.get_material().restitution + terrain.get_material().restitution);
			mat.friction = 0.5*(box.get_material().friction + terrain.get_material().friction);
			collInfo.mat = mat;
			collArr.push(collInfo);
			info.body0.collisions.push(collInfo);
			info.body1.collisions.push(collInfo);
			info.body0.addCollideBody(info.body1);
			info.body1.addCollideBody(info.body0);
		}else {
			info.body0.removeCollideBodies(info.body1);
			info.body1.removeCollideBodies(info.body0);
		}
		
	}



	jiglib.CollDetectBoxTerrain = CollDetectBoxTerrain; 

})(jiglib);


(function(jiglib) {

	var CollDetectBoxPlane = jiglib.CollDetectBoxPlane;
	var CollDetectBoxMesh = jiglib.CollDetectBoxMesh;
	var CollDetectBoxBox = jiglib.CollDetectBoxBox;
	var CollDetectSphereTerrain = jiglib.CollDetectSphereTerrain;
	var CollDetectSphereBox = jiglib.CollDetectSphereBox;
	var CollDetectCapsuleTerrain = jiglib.CollDetectCapsuleTerrain;
	var CollDetectSphereCapsule = jiglib.CollDetectSphereCapsule;
	var CollisionSystemBrute = jiglib.CollisionSystemBrute;
	var CollDetectCapsuleBox = jiglib.CollDetectCapsuleBox;
	var CollDetectBoxTerrain = jiglib.CollDetectBoxTerrain;
	var CollisionSystemGrid = jiglib.CollisionSystemGrid;
	var CollDetectCapsuleCapsule = jiglib.CollDetectCapsuleCapsule;
	var CollPointInfo = jiglib.CollPointInfo;
	var CollisionInfo = jiglib.CollisionInfo;
	var CollisionSystemAbstract = jiglib.CollisionSystemAbstract;
	var CollDetectCapsulePlane = jiglib.CollDetectCapsulePlane;
	var CollDetectInfo = jiglib.CollDetectInfo;
	var CollDetectSphereSphere = jiglib.CollDetectSphereSphere;
	var CollisionSystemGridEntry = jiglib.CollisionSystemGridEntry;
	var CollDetectSpherePlane = jiglib.CollDetectSpherePlane;
	var CollDetectFunctor = jiglib.CollDetectFunctor;
	var Vector3D = jiglib.Vector3D;
	var JConfig = jiglib.JConfig;
	var JIndexedTriangle = jiglib.JIndexedTriangle;
	var JSphere = jiglib.JSphere;
	var JTriangle = jiglib.JTriangle;
	var JTriangleMesh = jiglib.JTriangleMesh;
	var JNumber3D = jiglib.JNumber3D;
	var JMath3D = jiglib.JMath3D;
	var MaterialProperties = jiglib.MaterialProperties;
	var RigidBody = jiglib.RigidBody;

	var CollDetectSphereMesh = function()
	{

		this.name = "SphereMesh";
		this.type0 = "SPHERE";
		this.type1 = "TRIANGLEMESH";
		
	}

	jiglib.extend(CollDetectSphereMesh, CollDetectFunctor);

	CollDetectSphereMesh.prototype.collDetectSphereStaticMeshOverlap = function(sphere, mesh, info, collTolerance, collArr)
	{

		var body0Pos = info.body0.get_oldState().position;
		var body1Pos = info.body1.get_oldState().position;
		
		var sphereTolR = collTolerance + sphere.get_radius();
		var sphereTolR2 = sphereTolR * sphereTolR;
		
		var collNormal = new Vector3D();
		var collPts = [];
		
		var potentialTriangles = [];
		var numTriangles = mesh.get_octree().getTrianglesIntersectingtAABox(potentialTriangles, sphere.get_boundingBox());
		
		var newD2, distToCentre, oldD2, dist, depth, tiny=JMath3D.NUM_TINY;
		var meshTriangle;
		var vertexIndices;
		var arr;
		var triangle;
		for (var iTriangle = 0 ; iTriangle < numTriangles ; ++iTriangle) {
			meshTriangle = mesh.get_octree().getTriangle(potentialTriangles[iTriangle]);
			distToCentre = meshTriangle.get_plane().pointPlaneDistance(sphere.get_currentState().position);
			if (distToCentre <= 0) continue;
		    if (distToCentre >= sphereTolR) continue;
			
			vertexIndices = meshTriangle.get_vertexIndices();
			triangle = new JTriangle(mesh.get_octree().getVertex(vertexIndices[0]), mesh.get_octree().getVertex(vertexIndices[1]), mesh.get_octree().getVertex(vertexIndices[2]));
			arr = [];
			newD2 = triangle.pointTriangleDistanceSq(arr, sphere.get_currentState().position);
			
			if (newD2 < sphereTolR2) {
				// have overlap - but actually report the old intersection
			    oldD2 = triangle.pointTriangleDistanceSq(arr, sphere.get_oldState().position);
			    dist = Math.sqrt(oldD2);
			    depth = sphere.get_radius() - dist;
			    var collisionN = (dist > tiny) ? (sphere.get_oldState().position.subtract(triangle.getPoint(arr[0], arr[1]))) : triangle.get_normal().clone();
				collisionN.normalize();
			    // since impulse get applied at the old position
			    var pt = sphere.get_oldState().position.subtract(JNumber3D.getScaleVector(collisionN, sphere.get_radius()));
				
				var cpInfo = new CollPointInfo();
				cpInfo.r0 = pt.subtract(body0Pos);
				cpInfo.r1 = pt.subtract(body1Pos);
				cpInfo.initialPenetration = depth;
				collPts.push(cpInfo);
				collNormal = collNormal.add(collisionN);
				collNormal.normalize();
			}
		}
		if(collPts.length>0){
			var collInfo = new CollisionInfo();
			collInfo.objInfo = info;
			collInfo.dirToBody = collNormal;
			collInfo.pointInfo = collPts;
			
			var mat = new MaterialProperties();
			mat.restitution = 0.5*(sphere.get_material().restitution + mesh.get_material().restitution);
			mat.friction = 0.5*(sphere.get_material().friction + mesh.get_material().friction);
			collInfo.mat = mat;
			collArr.push(collInfo);
			info.body0.collisions.push(collInfo);
			info.body1.collisions.push(collInfo);
			info.body0.addCollideBody(info.body1);
			info.body1.addCollideBody(info.body0);
		}else {
			info.body0.removeCollideBodies(info.body1);
			info.body1.removeCollideBodies(info.body0);
		}
		
	}

	CollDetectSphereMesh.prototype.collDetect = function(info, collArr)
	{

		var tempBody;
		if (info.body0.get_type() == "TRIANGLEMESH")
		{
			tempBody = info.body0;
			info.body0 = info.body1;
			info.body1 = tempBody;
		}
		
		var sphere = info.body0;
		var mesh = info.body1;
		
		this.collDetectSphereStaticMeshOverlap(sphere, mesh, info, JConfig.collToll, collArr);
		
	}



	jiglib.CollDetectSphereMesh = CollDetectSphereMesh; 

})(jiglib);


(function(jiglib) {

	var CollDetectBoxPlane = jiglib.CollDetectBoxPlane;
	var CollDetectBoxMesh = jiglib.CollDetectBoxMesh;
	var CollDetectBoxBox = jiglib.CollDetectBoxBox;
	var CollDetectSphereTerrain = jiglib.CollDetectSphereTerrain;
	var CollDetectSphereBox = jiglib.CollDetectSphereBox;
	var CollDetectCapsuleTerrain = jiglib.CollDetectCapsuleTerrain;
	var CollDetectSphereCapsule = jiglib.CollDetectSphereCapsule;
	var CollisionSystemBrute = jiglib.CollisionSystemBrute;
	var CollDetectSphereMesh = jiglib.CollDetectSphereMesh;
	var CollDetectBoxTerrain = jiglib.CollDetectBoxTerrain;
	var CollisionSystemGrid = jiglib.CollisionSystemGrid;
	var CollDetectCapsuleCapsule = jiglib.CollDetectCapsuleCapsule;
	var CollPointInfo = jiglib.CollPointInfo;
	var CollisionInfo = jiglib.CollisionInfo;
	var CollisionSystemAbstract = jiglib.CollisionSystemAbstract;
	var CollDetectCapsulePlane = jiglib.CollDetectCapsulePlane;
	var CollDetectInfo = jiglib.CollDetectInfo;
	var CollDetectSphereSphere = jiglib.CollDetectSphereSphere;
	var CollisionSystemGridEntry = jiglib.CollisionSystemGridEntry;
	var CollDetectSpherePlane = jiglib.CollDetectSpherePlane;
	var CollDetectFunctor = jiglib.CollDetectFunctor;
	var Vector3D = jiglib.Vector3D;
	var JConfig = jiglib.JConfig;
	var JIndexedTriangle = jiglib.JIndexedTriangle;
	var JOctree = jiglib.JOctree;
	var JCapsule = jiglib.JCapsule;
	var JBox = jiglib.JBox;
	var JRay = jiglib.JRay;
	var JAABox = jiglib.JAABox;
	var JTerrain = jiglib.JTerrain;
	var JPlane = jiglib.JPlane;
	var JTriangleMesh = jiglib.JTriangleMesh;
	var JTriangle = jiglib.JTriangle;
	var JSphere = jiglib.JSphere;
	var JSegment = jiglib.JSegment;
	var JMatrix3D = jiglib.JMatrix3D;
	var Matrix3D = jiglib.Matrix3D;
	var JMath3D = jiglib.JMath3D;
	var JNumber3D = jiglib.JNumber3D;
	var MaterialProperties = jiglib.MaterialProperties;
	var RigidBody = jiglib.RigidBody;

	var CollDetectCapsuleBox = function()
	{

		this.name = "CapsuleBox";
		this.type0 = "CAPSULE";
		this.type1 = "BOX";
		
	}

	jiglib.extend(CollDetectCapsuleBox, CollDetectFunctor);

	CollDetectCapsuleBox.prototype.collDetect = function(info, collArr)
	{

	}



	jiglib.CollDetectCapsuleBox = CollDetectCapsuleBox; 

})(jiglib);


(function(jiglib) {

	var CollDetectBoxPlane = jiglib.CollDetectBoxPlane;
	var CollDetectBoxMesh = jiglib.CollDetectBoxMesh;
	var CollDetectBoxBox = jiglib.CollDetectBoxBox;
	var CollDetectSphereTerrain = jiglib.CollDetectSphereTerrain;
	var CollDetectSphereBox = jiglib.CollDetectSphereBox;
	var CollDetectCapsuleTerrain = jiglib.CollDetectCapsuleTerrain;
	var CollisionSystemBrute = jiglib.CollisionSystemBrute;
	var CollDetectCapsuleBox = jiglib.CollDetectCapsuleBox;
	var CollDetectSphereMesh = jiglib.CollDetectSphereMesh;
	var CollDetectBoxTerrain = jiglib.CollDetectBoxTerrain;
	var CollisionSystemGrid = jiglib.CollisionSystemGrid;
	var CollDetectCapsuleCapsule = jiglib.CollDetectCapsuleCapsule;
	var CollPointInfo = jiglib.CollPointInfo;
	var CollisionInfo = jiglib.CollisionInfo;
	var CollisionSystemAbstract = jiglib.CollisionSystemAbstract;
	var CollDetectCapsulePlane = jiglib.CollDetectCapsulePlane;
	var CollDetectInfo = jiglib.CollDetectInfo;
	var CollDetectSphereSphere = jiglib.CollDetectSphereSphere;
	var CollisionSystemGridEntry = jiglib.CollisionSystemGridEntry;
	var CollDetectSpherePlane = jiglib.CollDetectSpherePlane;
	var CollDetectFunctor = jiglib.CollDetectFunctor;
	var Vector3D = jiglib.Vector3D;
	var JConfig = jiglib.JConfig;
	var JIndexedTriangle = jiglib.JIndexedTriangle;
	var JOctree = jiglib.JOctree;
	var JCapsule = jiglib.JCapsule;
	var JBox = jiglib.JBox;
	var JRay = jiglib.JRay;
	var JAABox = jiglib.JAABox;
	var JTerrain = jiglib.JTerrain;
	var JPlane = jiglib.JPlane;
	var JTriangleMesh = jiglib.JTriangleMesh;
	var JTriangle = jiglib.JTriangle;
	var JSphere = jiglib.JSphere;
	var JSegment = jiglib.JSegment;
	var JMatrix3D = jiglib.JMatrix3D;
	var Matrix3D = jiglib.Matrix3D;
	var JMath3D = jiglib.JMath3D;
	var JNumber3D = jiglib.JNumber3D;
	var MaterialProperties = jiglib.MaterialProperties;
	var RigidBody = jiglib.RigidBody;

	var CollDetectSphereCapsule = function()
	{

		this.name = "SphereCapsule";
		this.type0 = "SPHERE";
		this.type1 = "CAPSULE";
		
	}

	jiglib.extend(CollDetectSphereCapsule, CollDetectFunctor);

	CollDetectSphereCapsule.prototype.collDetect = function(info, collArr)
	{

		var tempBody;
		if (info.body0.get_type() == "CAPSULE")
		{
			tempBody = info.body0;
			info.body0 = info.body1;
			info.body1 = tempBody;
		}

		var sphere = info.body0;
		var capsule = info.body1;

		if (!sphere.hitTestObject3D(capsule))
		{
			return;
		}
		
		if (!sphere.get_boundingBox().overlapTest(capsule.get_boundingBox())) {
			return;
		}

		var oldSeg = new JSegment(capsule.getBottomPos(capsule.get_oldState()), JNumber3D.getScaleVector(capsule.get_oldState().getOrientationCols()[1], capsule.get_length()));
		var newSeg = new JSegment(capsule.getBottomPos(capsule.get_currentState()), JNumber3D.getScaleVector(capsule.get_currentState().getOrientationCols()[1], capsule.get_length()));
		var radSum = sphere.get_radius() + capsule.get_radius();

		var oldObj = [];
		var oldDistSq = oldSeg.pointSegmentDistanceSq(oldObj, sphere.get_oldState().position);
		var newObj = [];
		var newDistSq = newSeg.pointSegmentDistanceSq(newObj, sphere.get_currentState().position);

		if (Math.min(oldDistSq, newDistSq) < Math.pow(radSum + JConfig.collToll, 2))
		{
			var segPos = oldSeg.getPoint(oldObj[0]);
			var delta = sphere.get_oldState().position.subtract(segPos);

			var dist = Math.sqrt(oldDistSq);
			var depth = radSum - dist;

			if (dist > JMath3D.NUM_TINY)
			{
				delta = JNumber3D.getDivideVector(delta, dist);
			}
			else
			{
				delta = JMatrix3D.getRotationMatrix(0, 0, 1, 360 * Math.random()).transformVector(Vector3D.Y_AXIS);
			}

			var worldPos = segPos.add(JNumber3D.getScaleVector(delta, capsule.get_radius() - 0.5 * depth));

			var collPts = [];
			var cpInfo = new CollPointInfo();
			cpInfo.r0 = worldPos.subtract(sphere.get_oldState().position);
			cpInfo.r1 = worldPos.subtract(capsule.get_oldState().position);
			cpInfo.initialPenetration = depth;
			collPts[0]=cpInfo;
			
			var collInfo = new CollisionInfo();
			collInfo.objInfo = info;
			collInfo.dirToBody = delta;
			collInfo.pointInfo = collPts;
			
			var mat = new MaterialProperties();
			mat.restitution = 0.5*(sphere.get_material().restitution + capsule.get_material().restitution);
			mat.friction = 0.5*(sphere.get_material().friction + capsule.get_material().friction);
			collInfo.mat = mat;
			collArr.push(collInfo);
			info.body0.collisions.push(collInfo);
			info.body1.collisions.push(collInfo);
			info.body0.addCollideBody(info.body1);
			info.body1.addCollideBody(info.body0);
		}else {
			info.body0.removeCollideBodies(info.body1);
			info.body1.removeCollideBodies(info.body0);
		}
		
	}



	jiglib.CollDetectSphereCapsule = CollDetectSphereCapsule; 

})(jiglib);


(function(jiglib) {

	var CollDetectBoxPlane = jiglib.CollDetectBoxPlane;
	var CollDetectBoxMesh = jiglib.CollDetectBoxMesh;
	var CollDetectBoxBox = jiglib.CollDetectBoxBox;
	var CollDetectSphereTerrain = jiglib.CollDetectSphereTerrain;
	var CollDetectSphereBox = jiglib.CollDetectSphereBox;
	var CollDetectSphereCapsule = jiglib.CollDetectSphereCapsule;
	var CollisionSystemBrute = jiglib.CollisionSystemBrute;
	var CollDetectCapsuleBox = jiglib.CollDetectCapsuleBox;
	var CollDetectSphereMesh = jiglib.CollDetectSphereMesh;
	var CollDetectBoxTerrain = jiglib.CollDetectBoxTerrain;
	var CollisionSystemGrid = jiglib.CollisionSystemGrid;
	var CollDetectCapsuleCapsule = jiglib.CollDetectCapsuleCapsule;
	var CollPointInfo = jiglib.CollPointInfo;
	var CollisionInfo = jiglib.CollisionInfo;
	var CollisionSystemAbstract = jiglib.CollisionSystemAbstract;
	var CollDetectCapsulePlane = jiglib.CollDetectCapsulePlane;
	var CollDetectInfo = jiglib.CollDetectInfo;
	var CollDetectSphereSphere = jiglib.CollDetectSphereSphere;
	var CollisionSystemGridEntry = jiglib.CollisionSystemGridEntry;
	var CollDetectSpherePlane = jiglib.CollDetectSpherePlane;
	var CollDetectFunctor = jiglib.CollDetectFunctor;
	var Vector3D = jiglib.Vector3D;
	var JConfig = jiglib.JConfig;
	var TerrainData = jiglib.TerrainData;
	var JCapsule = jiglib.JCapsule;
	var JTerrain = jiglib.JTerrain;
	var JNumber3D = jiglib.JNumber3D;
	var MaterialProperties = jiglib.MaterialProperties;
	var RigidBody = jiglib.RigidBody;

	var CollDetectCapsuleTerrain = function()
	{

		this.name = "BoxTerrain";
		this.type0 = "CAPSULE";
		this.type1 = "TERRAIN";
		
	}

	jiglib.extend(CollDetectCapsuleTerrain, CollDetectFunctor);

	CollDetectCapsuleTerrain.prototype.collDetect = function(info, collArr)
	{

		var tempBody;
		if (info.body0.get_type() == "TERRAIN")
		{
			tempBody = info.body0;
			info.body0 = info.body1;
			info.body1 = tempBody;
		}
		
		var capsule = info.body0;
		var terrain = info.body1;
				
		var collPts = [];
		var cpInfo;
		
		var averageNormal = new Vector3D();
		var pos1 = capsule.getBottomPos(capsule.get_oldState());
		var pos2 = capsule.getBottomPos(capsule.get_currentState());
		var obj1 = terrain.getHeightAndNormalByPoint(pos1);
		var obj2 = terrain.getHeightAndNormalByPoint(pos2);
		if (Math.min(obj1.height, obj2.height) < JConfig.collToll + capsule.get_radius()) {
			var oldDepth = capsule.get_radius() - obj1.height;
			var worldPos = pos1.subtract(JNumber3D.getScaleVector(obj2.normal, capsule.get_radius()));
			cpInfo = new CollPointInfo();
			cpInfo.r0 = worldPos.subtract(capsule.get_oldState().position);
			cpInfo.r1 = worldPos.subtract(terrain.get_oldState().position);
			cpInfo.initialPenetration = oldDepth;
			collPts.push(cpInfo);
			averageNormal = averageNormal.add(obj2.normal);
		}
		
		pos1 = capsule.getEndPos(capsule.get_oldState());
		pos2 = capsule.getEndPos(capsule.get_currentState());
		obj1 = terrain.getHeightAndNormalByPoint(pos1);
		obj2 = terrain.getHeightAndNormalByPoint(pos2);
		if (Math.min(obj1.height, obj2.height) < JConfig.collToll + capsule.get_radius()) {
			oldDepth = capsule.get_radius() - obj1.height;
			worldPos = pos1.subtract(JNumber3D.getScaleVector(obj2.normal, capsule.get_radius()));
			cpInfo = new CollPointInfo();
			cpInfo.r0 = worldPos.subtract(capsule.get_oldState().position);
			cpInfo.r1 = worldPos.subtract(terrain.get_oldState().position);
			cpInfo.initialPenetration = oldDepth;
			collPts.push(cpInfo);
			averageNormal = averageNormal.add(obj2.normal);
		}
		
		if (collPts.length > 0)
		{
			averageNormal.normalize();
			
			var collInfo = new CollisionInfo();
			collInfo.objInfo = info;
			collInfo.dirToBody = averageNormal;
			collInfo.pointInfo = collPts;
			
			var mat = new MaterialProperties();
			mat.restitution = 0.5*(capsule.get_material().restitution + terrain.get_material().restitution);
			mat.friction = 0.5*(capsule.get_material().friction + terrain.get_material().friction);
			collInfo.mat = mat;
			collArr.push(collInfo);
			info.body0.collisions.push(collInfo);
			info.body1.collisions.push(collInfo);
			info.body0.addCollideBody(info.body1);
			info.body1.addCollideBody(info.body0);
		}else {
			info.body0.removeCollideBodies(info.body1);
			info.body1.removeCollideBodies(info.body0);
		}
		
	}



	jiglib.CollDetectCapsuleTerrain = CollDetectCapsuleTerrain; 

})(jiglib);


(function(jiglib) {

	var CollDetectBoxPlane = jiglib.CollDetectBoxPlane;
	var CollDetectBoxMesh = jiglib.CollDetectBoxMesh;
	var CollDetectBoxBox = jiglib.CollDetectBoxBox;
	var CollDetectSphereTerrain = jiglib.CollDetectSphereTerrain;
	var CollDetectCapsuleTerrain = jiglib.CollDetectCapsuleTerrain;
	var CollDetectSphereCapsule = jiglib.CollDetectSphereCapsule;
	var CollisionSystemBrute = jiglib.CollisionSystemBrute;
	var CollDetectCapsuleBox = jiglib.CollDetectCapsuleBox;
	var CollDetectSphereMesh = jiglib.CollDetectSphereMesh;
	var CollDetectBoxTerrain = jiglib.CollDetectBoxTerrain;
	var CollisionSystemGrid = jiglib.CollisionSystemGrid;
	var CollDetectCapsuleCapsule = jiglib.CollDetectCapsuleCapsule;
	var CollPointInfo = jiglib.CollPointInfo;
	var CollisionInfo = jiglib.CollisionInfo;
	var CollisionSystemAbstract = jiglib.CollisionSystemAbstract;
	var CollDetectCapsulePlane = jiglib.CollDetectCapsulePlane;
	var CollDetectInfo = jiglib.CollDetectInfo;
	var CollDetectSphereSphere = jiglib.CollDetectSphereSphere;
	var CollisionSystemGridEntry = jiglib.CollisionSystemGridEntry;
	var CollDetectSpherePlane = jiglib.CollDetectSpherePlane;
	var CollDetectFunctor = jiglib.CollDetectFunctor;
	var Vector3D = jiglib.Vector3D;
	var JConfig = jiglib.JConfig;
	var JIndexedTriangle = jiglib.JIndexedTriangle;
	var JOctree = jiglib.JOctree;
	var JCapsule = jiglib.JCapsule;
	var JBox = jiglib.JBox;
	var JRay = jiglib.JRay;
	var JAABox = jiglib.JAABox;
	var JTerrain = jiglib.JTerrain;
	var JPlane = jiglib.JPlane;
	var JTriangleMesh = jiglib.JTriangleMesh;
	var JTriangle = jiglib.JTriangle;
	var JSphere = jiglib.JSphere;
	var JSegment = jiglib.JSegment;
	var JMatrix3D = jiglib.JMatrix3D;
	var Matrix3D = jiglib.Matrix3D;
	var JMath3D = jiglib.JMath3D;
	var JNumber3D = jiglib.JNumber3D;
	var MaterialProperties = jiglib.MaterialProperties;
	var RigidBody = jiglib.RigidBody;

	var CollDetectSphereBox = function()
	{

		this.name = "SphereBox";
		this.type0 = "SPHERE";
		this.type1 = "BOX";
		
	}

	jiglib.extend(CollDetectSphereBox, CollDetectFunctor);

	CollDetectSphereBox.prototype.collDetect = function(info, collArr)
	{

		var tempBody;
		if (info.body0.get_type() == "BOX")
		{
			tempBody = info.body0;
			info.body0 = info.body1;
			info.body1 = tempBody;
		}
		
		var sphere = info.body0;
		var box = info.body1;
		
		if (!sphere.hitTestObject3D(box))
		{
			return;
		}
		if (!sphere.get_boundingBox().overlapTest(box.get_boundingBox()))
		{
			return;
		}
		
		//var spherePos = sphere.get_oldState().position;
		//var boxPos = box.get_oldState().position;
		
		var oldBoxPoint = [new Vector3D()];
		var newBoxPoint = [new Vector3D()];
		
		var oldDist, newDist, oldDepth, newDepth, tiny=JMath3D.NUM_TINY;
		oldDist = box.getDistanceToPoint(box.get_oldState(), oldBoxPoint, sphere.get_oldState().position);
		newDist = box.getDistanceToPoint(box.get_currentState(), newBoxPoint, sphere.get_currentState().position);
		
		var _oldBoxPosition = oldBoxPoint[0];
		
		oldDepth = sphere.get_radius() - oldDist;
		newDepth = sphere.get_radius() - newDist;
		if (Math.max(oldDepth, newDepth) > -JConfig.collToll)
		{
			var dir;
			var collPts = [];
			if (oldDist < -tiny)
			{
				dir = _oldBoxPosition.subtract(sphere.get_oldState().position).subtract(_oldBoxPosition);
				dir.normalize();
			}
			else if (oldDist > tiny)
			{
				dir = sphere.get_oldState().position.subtract(_oldBoxPosition);
				dir.normalize();
			}
			else
			{
				dir = sphere.get_oldState().position.subtract(box.get_oldState().position);
				dir.normalize();
			}
			
			var cpInfo = new CollPointInfo();
			cpInfo.r0 = _oldBoxPosition.subtract(sphere.get_oldState().position);
			cpInfo.r1 = _oldBoxPosition.subtract(box.get_oldState().position);
			cpInfo.initialPenetration = oldDepth;
			collPts[0]=cpInfo;
			
			var collInfo = new CollisionInfo();
			collInfo.objInfo = info;
			collInfo.dirToBody = dir;
			collInfo.pointInfo = collPts;
			
			var mat = new MaterialProperties();
			mat.restitution = 0.5*(sphere.get_material().restitution + box.get_material().restitution);
			mat.friction = 0.5*(sphere.get_material().friction + box.get_material().friction);
			collInfo.mat = mat;
			collArr.push(collInfo);
			info.body0.collisions.push(collInfo);
			info.body1.collisions.push(collInfo);
			info.body0.addCollideBody(info.body1);
			info.body1.addCollideBody(info.body0);
		}else {
			info.body0.removeCollideBodies(info.body1);
			info.body1.removeCollideBodies(info.body0);
		}
		
	}



	jiglib.CollDetectSphereBox = CollDetectSphereBox; 

})(jiglib);


(function(jiglib) {

	var CollDetectBoxPlane = jiglib.CollDetectBoxPlane;
	var CollDetectBoxMesh = jiglib.CollDetectBoxMesh;
	var CollDetectBoxBox = jiglib.CollDetectBoxBox;
	var CollDetectSphereBox = jiglib.CollDetectSphereBox;
	var CollDetectCapsuleTerrain = jiglib.CollDetectCapsuleTerrain;
	var CollDetectSphereCapsule = jiglib.CollDetectSphereCapsule;
	var CollisionSystemBrute = jiglib.CollisionSystemBrute;
	var CollDetectCapsuleBox = jiglib.CollDetectCapsuleBox;
	var CollDetectSphereMesh = jiglib.CollDetectSphereMesh;
	var CollDetectBoxTerrain = jiglib.CollDetectBoxTerrain;
	var CollisionSystemGrid = jiglib.CollisionSystemGrid;
	var CollDetectCapsuleCapsule = jiglib.CollDetectCapsuleCapsule;
	var CollPointInfo = jiglib.CollPointInfo;
	var CollisionInfo = jiglib.CollisionInfo;
	var CollisionSystemAbstract = jiglib.CollisionSystemAbstract;
	var CollDetectCapsulePlane = jiglib.CollDetectCapsulePlane;
	var CollDetectInfo = jiglib.CollDetectInfo;
	var CollDetectSphereSphere = jiglib.CollDetectSphereSphere;
	var CollisionSystemGridEntry = jiglib.CollisionSystemGridEntry;
	var CollDetectSpherePlane = jiglib.CollDetectSpherePlane;
	var CollDetectFunctor = jiglib.CollDetectFunctor;
	var Vector3D = jiglib.Vector3D;
	var JConfig = jiglib.JConfig;
	var TerrainData = jiglib.TerrainData;
	var JSphere = jiglib.JSphere;
	var JTerrain = jiglib.JTerrain;
	var JNumber3D = jiglib.JNumber3D;
	var MaterialProperties = jiglib.MaterialProperties;
	var RigidBody = jiglib.RigidBody;

	var CollDetectSphereTerrain = function()
	{

		this.name = "SphereTerrain";
		this.type0 = "SPHERE";
		this.type1 = "TERRAIN";
		
	}

	jiglib.extend(CollDetectSphereTerrain, CollDetectFunctor);

	CollDetectSphereTerrain.prototype.collDetect = function(info, collArr)
	{

		var tempBody;
		if (info.body0.get_type() == "TERRAIN")
		{
			tempBody = info.body0;
			info.body0 = info.body1;
			info.body1 = tempBody;
		}

		var sphere = info.body0;
		var terrain = info.body1;
				
		var obj = terrain.getHeightAndNormalByPoint(sphere.get_currentState().position);
		if (obj.height < JConfig.collToll + sphere.get_radius()) {
			var dist = terrain.getHeightByPoint(sphere.get_oldState().position);
			var depth = sphere.get_radius() - dist;
			
			var Pt = sphere.get_oldState().position.subtract(JNumber3D.getScaleVector(obj.normal, sphere.get_radius()));
			
			var collPts = [];
			var cpInfo = new CollPointInfo();
			cpInfo.r0 = Pt.subtract(sphere.get_oldState().position);
			cpInfo.r1 = Pt.subtract(terrain.get_oldState().position);
			cpInfo.initialPenetration = depth;
			collPts[0]=cpInfo;
			
			var collInfo = new CollisionInfo();
			collInfo.objInfo = info;
			collInfo.dirToBody = obj.normal;
			collInfo.pointInfo = collPts;
			
			var mat = new MaterialProperties();
			mat.restitution = 0.5*(sphere.get_material().restitution + terrain.get_material().restitution);
			mat.friction = 0.5*(sphere.get_material().friction + terrain.get_material().friction);
			collInfo.mat = mat;
			collArr.push(collInfo);
			info.body0.collisions.push(collInfo);
			info.body1.collisions.push(collInfo);
			info.body0.addCollideBody(info.body1);
			info.body1.addCollideBody(info.body0);
		}else {
			info.body0.removeCollideBodies(info.body1);
			info.body1.removeCollideBodies(info.body0);
		}
		
	}



	jiglib.CollDetectSphereTerrain = CollDetectSphereTerrain; 

})(jiglib);


(function(jiglib) {

	var CollDetectBoxPlane = jiglib.CollDetectBoxPlane;
	var CollDetectBoxMesh = jiglib.CollDetectBoxMesh;
	var CollDetectSphereTerrain = jiglib.CollDetectSphereTerrain;
	var CollDetectSphereBox = jiglib.CollDetectSphereBox;
	var CollDetectCapsuleTerrain = jiglib.CollDetectCapsuleTerrain;
	var CollDetectSphereCapsule = jiglib.CollDetectSphereCapsule;
	var CollisionSystemBrute = jiglib.CollisionSystemBrute;
	var CollDetectCapsuleBox = jiglib.CollDetectCapsuleBox;
	var CollDetectSphereMesh = jiglib.CollDetectSphereMesh;
	var CollDetectBoxTerrain = jiglib.CollDetectBoxTerrain;
	var CollisionSystemGrid = jiglib.CollisionSystemGrid;
	var CollDetectCapsuleCapsule = jiglib.CollDetectCapsuleCapsule;
	var CollPointInfo = jiglib.CollPointInfo;
	var CollisionInfo = jiglib.CollisionInfo;
	var CollisionSystemAbstract = jiglib.CollisionSystemAbstract;
	var CollDetectCapsulePlane = jiglib.CollDetectCapsulePlane;
	var CollDetectInfo = jiglib.CollDetectInfo;
	var CollDetectSphereSphere = jiglib.CollDetectSphereSphere;
	var CollisionSystemGridEntry = jiglib.CollisionSystemGridEntry;
	var CollDetectSpherePlane = jiglib.CollDetectSpherePlane;
	var CollDetectFunctor = jiglib.CollDetectFunctor;
	var Vector3D = jiglib.Vector3D;
	var JConfig = jiglib.JConfig;
	var EdgeData = jiglib.EdgeData;
	var SpanData = jiglib.SpanData;
	var JIndexedTriangle = jiglib.JIndexedTriangle;
	var JOctree = jiglib.JOctree;
	var JCapsule = jiglib.JCapsule;
	var JBox = jiglib.JBox;
	var JRay = jiglib.JRay;
	var JAABox = jiglib.JAABox;
	var JTerrain = jiglib.JTerrain;
	var JPlane = jiglib.JPlane;
	var JTriangleMesh = jiglib.JTriangleMesh;
	var JTriangle = jiglib.JTriangle;
	var JSphere = jiglib.JSphere;
	var JSegment = jiglib.JSegment;
	var JMatrix3D = jiglib.JMatrix3D;
	var Matrix3D = jiglib.Matrix3D;
	var JMath3D = jiglib.JMath3D;
	var JNumber3D = jiglib.JNumber3D;
	var MaterialProperties = jiglib.MaterialProperties;
	var PhysicsState = jiglib.PhysicsState;

	var CollDetectBoxBox = function()
	{
		this.MAX_SUPPORT_VERTS =  10; // Number
		this.combinationDist = null; // Number

		this.name = "BoxBox";
		this.type0 = "BOX";
		this.type1 = "BOX";
		
	}

	jiglib.extend(CollDetectBoxBox, CollDetectFunctor);

	CollDetectBoxBox.prototype.disjoint = function(out, axis, box0, box1)
	{

		var obj0 = box0.getSpan(axis);
		var obj1 = box1.getSpan(axis);
		var obj0Min=obj0.min, obj0Max=obj0.max, obj1Min=obj1.min, obj1Max=obj1.max, tiny=JMath3D.NUM_TINY;

		if (obj0Min > (obj1Max + JConfig.collToll + tiny) || obj1Min > (obj0Max + JConfig.collToll + tiny))
		{
			out.flag = true;
			return true;
		}
		if ((obj0Max > obj1Max) && (obj1Min > obj0Min))
		{
			out.depth = Math.min(obj0Max - obj1Min, obj1Max - obj0Min);
		}
		else if ((obj1Max > obj0Max) && (obj0Min > obj1Min))
		{
			out.depth = Math.min(obj1Max - obj0Min, obj0Max - obj1Min);
		}
		else
		{
			out.depth = Math.min(obj0Max, obj1Max);
			out.depth -= Math.max(obj0Min, obj1Min);
		}
		out.flag = false;
		return false;
		
	}

	CollDetectBoxBox.prototype.addPoint = function(contactPoints, pt, combinationDistanceSq)
	{

		for (var contactPoints_i = 0, contactPoints_l = contactPoints.length, contactPoint; (contactPoints_i < contactPoints_l) && (contactPoint = contactPoints[contactPoints_i]); contactPoints_i++)
		{
			if (contactPoint.subtract(pt).get_lengthSquared() < combinationDistanceSq)
			{
				contactPoint = JNumber3D.getScaleVector(contactPoint.add(pt), 0.5);
				return false;
			}
		}
		contactPoints.push(pt);
		return true;
		
	}

	CollDetectBoxBox.prototype.getSupportPoint = function(box, axis)
	{

		var orientationCol = box.get_currentState().getOrientationCols();
		var _as=axis.dotProduct(orientationCol[0]), _au=axis.dotProduct(orientationCol[1]), _ad=axis.dotProduct(orientationCol[2]), tiny=JMath3D.NUM_TINY;
		
		var p = box.get_currentState().position.clone();
  
		if (_as < -tiny) {
			p = p.add(JNumber3D.getScaleVector(orientationCol[0], 0.5 * box.get_sideLengths().x));
		}else if (_as >= tiny) {
			p = p.subtract(JNumber3D.getScaleVector(orientationCol[0], 0.5 * box.get_sideLengths().x));
		}
  
		if (_au < -tiny) {
			p = p.add(JNumber3D.getScaleVector(orientationCol[1], 0.5 * box.get_sideLengths().y));
		}else if (_au > tiny) {
			p = p.subtract(JNumber3D.getScaleVector(orientationCol[1], 0.5 * box.get_sideLengths().y));
		}
  
		if (_ad < -tiny) {
			p = p.add(JNumber3D.getScaleVector(orientationCol[2], 0.5 * box.get_sideLengths().z));
		}else if (_ad > tiny) {
			p = p.subtract(JNumber3D.getScaleVector(orientationCol[2], 0.5 * box.get_sideLengths().z));
		}
		return p;
		
	}

	CollDetectBoxBox.prototype.getAABox2EdgeIntersectionPoints = function(contactPoint, origBoxSides, origBoxState, edgePt0, edgePt1)
	{

		var jDir, kDir, num=0, iDir, iFace;
		var dist0, dist1, frac, tiny=JMath3D.NUM_TINY;
		var pt, edgeDir;
		
		edgeDir = edgePt1.subtract(edgePt0);
		edgeDir.normalize();
		var ptArr, faceOffsets, edgePt0Arr, edgePt1Arr, edgeDirArr, sidesArr;
		edgePt0Arr = JNumber3D.toArray(edgePt0);
		edgePt1Arr = JNumber3D.toArray(edgePt1);
		edgeDirArr = JNumber3D.toArray(edgeDir);
		sidesArr = JNumber3D.toArray(JNumber3D.getScaleVector(origBoxSides, 0.5));
		for (iDir = 2; iDir >= 0; iDir--) {
			if (Math.abs(edgeDirArr[iDir]) < 0.1) {
				continue;
			}
			jDir = (iDir + 1) % 3;
			kDir = (iDir + 2) % 3;
			faceOffsets = [[ -sidesArr[iDir], sidesArr[iDir]]];
			for (iFace = 1; iFace >= 0; iFace-- ) {
				dist0 = edgePt0Arr[iDir] - faceOffsets[iFace];
				dist1 = edgePt1Arr[iDir] - faceOffsets[iFace];
				frac = -1;
				if (dist0 * dist1 < -tiny) {
				frac = -dist0 / (dist1 - dist0);
				}else if (Math.abs(dist0) < tiny) {
				frac = 0;
				}else if (Math.abs(dist1) < tiny) {
				frac = 1;
				}
				if (frac >= 0) {
				pt = JNumber3D.getScaleVector(edgePt0, 1 - frac).add(JNumber3D.getScaleVector(edgePt1, frac));
				ptArr = JNumber3D.toArray(pt);
				if ((ptArr[jDir] > -sidesArr[jDir] - tiny) && (ptArr[jDir] < sidesArr[jDir] + tiny) && (ptArr[kDir] > -sidesArr[kDir] - tiny) && (ptArr[kDir] < sidesArr[kDir] + tiny) ) {
					pt = origBoxState.orientation.transformVector(pt);
					pt = pt.add(origBoxState.position);
					this.addPoint(contactPoint, pt, this.combinationDist);
					if (++num == 2) {
						return num;
					}
				}
				}
			}
		}
		return num;
		
	}

	CollDetectBoxBox.prototype.getBox2BoxEdgesIntersectionPoints = function(contactPoint, box0, box1, newState)
	{

		var num = 0;
		var seg;
		var box0State = (newState) ? box0.get_currentState() : box0.get_oldState();
		var box1State = (newState) ? box1.get_currentState() : box1.get_oldState();
		var boxPts = box1.getCornerPointsInBoxSpace(box1State, box0State);
		
		var boxEdges = box1.get_edges();
		var edgePt0, edgePt1;
		for (var boxEdges_i = 0, boxEdges_l = boxEdges.length, boxEdge; (boxEdges_i < boxEdges_l) && (boxEdge = boxEdges[boxEdges_i]); boxEdges_i++)
		{
			edgePt0 = boxPts[boxEdge.ind0];
			edgePt1 = boxPts[boxEdge.ind1];
			num += this.getAABox2EdgeIntersectionPoints(contactPoint, box0.get_sideLengths(), box0State, edgePt0, edgePt1);
			if (num >= 8) {
				return num;
			}
		}
		return num;
		
	}

	CollDetectBoxBox.prototype.getBoxBoxIntersectionPoints = function(contactPoint, box0, box1, newState)
	{

		this.getBox2BoxEdgesIntersectionPoints(contactPoint, box0, box1, newState);
		this.getBox2BoxEdgesIntersectionPoints(contactPoint, box1, box0, newState);
		return contactPoint.length;
		
	}

	CollDetectBoxBox.prototype.collDetect = function(info, collArr)
	{

		var box0 = info.body0;
		var box1 = info.body1;

		if (!box0.hitTestObject3D(box1))
			return;

		if (!box0.get_boundingBox().overlapTest(box1.get_boundingBox()))
			return;

		var numTiny = JMath3D.NUM_TINY, numHuge = JMath3D.NUM_HUGE;

		var dirs0Arr = box0.get_currentState().getOrientationCols();
		var dirs1Arr = box1.get_currentState().getOrientationCols();

		// the 15 potential separating axes
		var axes = [dirs0Arr[0], dirs0Arr[1], dirs0Arr[2],
			dirs1Arr[0], dirs1Arr[1], dirs1Arr[2],
			dirs0Arr[0].crossProduct(dirs1Arr[0]),
			dirs0Arr[1].crossProduct(dirs1Arr[0]),
			dirs0Arr[2].crossProduct(dirs1Arr[0]),
			dirs0Arr[0].crossProduct(dirs1Arr[1]),
			dirs0Arr[1].crossProduct(dirs1Arr[1]),
			dirs0Arr[2].crossProduct(dirs1Arr[1]),
			dirs0Arr[0].crossProduct(dirs1Arr[2]),
			dirs0Arr[1].crossProduct(dirs1Arr[2]),
			dirs0Arr[2].crossProduct(dirs1Arr[2])];

		var l2;
		// the overlap depths along each axis
		var overlapDepths = [];
		var i = 0;
		var axesLength = axes.length;

		// see if the boxes are separate along any axis, and if not keep a 
		// record of the depths along each axis
		var ax;
		for (i = 0; i < axesLength; i++)
		{
			overlapDepths[i] = new SpanData();

			l2 = axes[i].get_lengthSquared();
			if (l2 < numTiny)
				continue;
			
			ax = axes[i].clone();
			ax.normalize();
			if (this.disjoint(overlapDepths[i], ax, box0, box1)) {
				info.body0.removeCollideBodies(info.body1);
				info.body1.removeCollideBodies(info.body0);
				return;
			}
		}

		// The box overlap, find the separation depth closest to 0.
		var minDepth = numHuge;
		var minAxis = -1;
		axesLength = axes.length;
		for (i = 0; i < axesLength; i++)
		{
			l2 = axes[i].get_lengthSquared();
			if (l2 < numTiny)
				continue;

			// If this axis is the minimum, select it
			if (overlapDepths[i].depth < minDepth)
			{
				minDepth = overlapDepths[i].depth;
				minAxis = i;
			}
		}
		
		if (minAxis == -1) {
			info.body0.removeCollideBodies(info.body1);
			info.body1.removeCollideBodies(info.body0);
			return;
		}
		
		// Make sure the axis is facing towards the box0. if not, invert it
		var N = axes[minAxis].clone();
		if (box1.get_currentState().position.subtract(box0.get_currentState().position).dotProduct(N) > 0)
			N.negate();
		
		var contactPointsFromOld = true;
		var contactPoints = [];
		this.combinationDist = 0.05 * Math.min(Math.min(box0.get_sideLengths().x, box0.get_sideLengths().y, box0.get_sideLengths().z), Math.min(box1.get_sideLengths().x, box1.get_sideLengths().y, box1.get_sideLengths().z));
		this.combinationDist += (JConfig.collToll * 3.464);
		this.combinationDist *= this.combinationDist;

		if (minDepth > -numTiny)
			this.getBoxBoxIntersectionPoints(contactPoints, box0, box1, false);
		
		if (contactPoints.length == 0)
		{
			contactPointsFromOld = false;
			this.getBoxBoxIntersectionPoints(contactPoints, box0, box1, true);
		}
		
		var bodyDelta = box0.get_currentState().position.subtract(box0.get_oldState().position).subtract(box1.get_currentState().position.subtract(box1.get_oldState().position));
		var bodyDeltaLen = bodyDelta.dotProduct(N);
		var oldDepth = minDepth + bodyDeltaLen;
		
		var SATPoint = new Vector3D();
		switch(minAxis){
			//-----------------------------------------------------------------
			// Box0 face, Box1 Corner collision
			//-----------------------------------------------------------------
		case 0:
		case 1:
		case 2:
		{
			//-----------------------------------------------------------------
			// Get the lowest point on the box1 along box1 normal
			//-----------------------------------------------------------------
			SATPoint = this.getSupportPoint(box1, JNumber3D.getScaleVector(N, -1));
			break;
		}
		//-----------------------------------------------------------------
		// We have a Box2 corner/Box1 face collision
		//-----------------------------------------------------------------
		case 3:
		case 4:
		case 5:
		{
			//-----------------------------------------------------------------
			// Find with vertex on the triangle collided
			//-----------------------------------------------------------------
			SATPoint = this.getSupportPoint(box0, N);
			break;
		}
		//-----------------------------------------------------------------
		// We have an edge/edge colliiosn
		//-----------------------------------------------------------------
		case 6:
		case 7:
		case 8:
		case 9:
		case 10:
		case 11:
		case 12:
		case 13:
		case 14:
		{ 
			//-----------------------------------------------------------------
			// Retrieve which edges collided.
			//-----------------------------------------------------------------
			i = minAxis - 6;
			var ia = Math.floor(i / 3);
			var ib = Math.floor(i - ia * 3);
			//-----------------------------------------------------------------
			// find two P0, P1 point on both edges. 
			//-----------------------------------------------------------------
			var P0 = this.getSupportPoint(box0, N);
			var P1 = this.getSupportPoint(box1, JNumber3D.getScaleVector(N, -1));
      
			//-----------------------------------------------------------------
			// Find the edge intersection. 
			//-----------------------------------------------------------------
     
			//-----------------------------------------------------------------
			// plane along N and F, and passing through PB
			//-----------------------------------------------------------------
			var planeNormal = N.crossProduct(dirs1Arr[ib]);
			var planeD = planeNormal.dotProduct(P1);
      
			//-----------------------------------------------------------------
			// find the intersection t, where Pintersection = P0 + t*box edge dir
			//-----------------------------------------------------------------
			var div = dirs0Arr[ia].dotProduct(planeNormal);
      
			//-----------------------------------------------------------------
			// plane and ray colinear, skip the intersection.
			//-----------------------------------------------------------------
			if (Math.abs(div) < numTiny)
				return;
      
			var t = (planeD - P0.dotProduct(planeNormal)) / div;
      
			//-----------------------------------------------------------------
			// point on edge of box0
			//-----------------------------------------------------------------
			P0 = P0.add(JNumber3D.getScaleVector(dirs0Arr[ia], t));
			SATPoint = P0.add(JNumber3D.getScaleVector(N, 0.5 * minDepth));
			break;
		}
		}

		var collPts;
		if (contactPoints.length > 0)
		{
			collPts = [];

			var minDist = numHuge, maxDist = -numHuge, dist, depth, depthScale;
			
			var cpInfo;
			var contactPoint;

			for (var contactPoints_i = 0, contactPoints_l = contactPoints.length, contactPoint; (contactPoints_i < contactPoints_l) && (contactPoint = contactPoints[contactPoints_i]); contactPoints_i++)
			{
				dist = contactPoint.subtract(SATPoint).length;
				
				if (dist < minDist)
				minDist = dist;

				if (dist > maxDist)
				maxDist = dist;
			}

			if (maxDist < minDist + numTiny)
				maxDist = minDist + numTiny;

			i = 0;
			for (var contactPoints_i = 0, contactPoints_l = contactPoints.length, contactPoint; (contactPoints_i < contactPoints_l) && (contactPoint = contactPoints[contactPoints_i]); contactPoints_i++)
			{
				dist = contactPoint.subtract(SATPoint).length;
				depthScale = (dist - minDist) / (maxDist - minDist);
				depth = (1 - depthScale) * oldDepth;
				cpInfo = new CollPointInfo();
				
				if (contactPointsFromOld)
				{
				cpInfo.r0 = contactPoint.subtract(box0.get_oldState().position);
				cpInfo.r1 = contactPoint.subtract(box1.get_oldState().position);
				}
				else
				{
				cpInfo.r0 = contactPoint.subtract(box0.get_currentState().position);
				cpInfo.r1 = contactPoint.subtract(box1.get_currentState().position);
				}
				
				cpInfo.initialPenetration = depth;
				collPts[i++] = cpInfo;
			}
		}
		else
		{
			cpInfo = new CollPointInfo();
			cpInfo.r0 = SATPoint.subtract(box0.get_currentState().position);
			cpInfo.r1 = SATPoint.subtract(box1.get_currentState().position);
			cpInfo.initialPenetration = oldDepth;
			
			collPts = [];
			collPts[0] = cpInfo;
		}

		var collInfo = new CollisionInfo();
		collInfo.objInfo = info;
		collInfo.dirToBody = N;
		collInfo.pointInfo = collPts;
		
		var mat = new MaterialProperties();
		mat.restitution = 0.5*(box0.get_material().restitution + box1.get_material().restitution);
		mat.friction = 0.5*(box0.get_material().friction + box1.get_material().friction);
		collInfo.mat = mat;
		collArr.push(collInfo);
		info.body0.collisions.push(collInfo);
		info.body1.collisions.push(collInfo);
		info.body0.addCollideBody(info.body1);
		info.body1.addCollideBody(info.body0);
		
	}



	jiglib.CollDetectBoxBox = CollDetectBoxBox; 

})(jiglib);


(function(jiglib) {

	var CollDetectBoxPlane = jiglib.CollDetectBoxPlane;
	var CollDetectBoxBox = jiglib.CollDetectBoxBox;
	var CollDetectSphereTerrain = jiglib.CollDetectSphereTerrain;
	var CollDetectSphereBox = jiglib.CollDetectSphereBox;
	var CollDetectCapsuleTerrain = jiglib.CollDetectCapsuleTerrain;
	var CollDetectSphereCapsule = jiglib.CollDetectSphereCapsule;
	var CollisionSystemBrute = jiglib.CollisionSystemBrute;
	var CollDetectCapsuleBox = jiglib.CollDetectCapsuleBox;
	var CollDetectSphereMesh = jiglib.CollDetectSphereMesh;
	var CollDetectBoxTerrain = jiglib.CollDetectBoxTerrain;
	var CollisionSystemGrid = jiglib.CollisionSystemGrid;
	var CollDetectCapsuleCapsule = jiglib.CollDetectCapsuleCapsule;
	var CollPointInfo = jiglib.CollPointInfo;
	var CollisionInfo = jiglib.CollisionInfo;
	var CollisionSystemAbstract = jiglib.CollisionSystemAbstract;
	var CollDetectCapsulePlane = jiglib.CollDetectCapsulePlane;
	var CollDetectInfo = jiglib.CollDetectInfo;
	var CollDetectSphereSphere = jiglib.CollDetectSphereSphere;
	var CollisionSystemGridEntry = jiglib.CollisionSystemGridEntry;
	var CollDetectSpherePlane = jiglib.CollDetectSpherePlane;
	var CollDetectFunctor = jiglib.CollDetectFunctor;
	var Matrix3D = jiglib.Matrix3D;
	var Vector3D = jiglib.Vector3D;
	var JConfig = jiglib.JConfig;
	var CollOutData = jiglib.CollOutData;
	var EdgeData = jiglib.EdgeData;
	var SpanData = jiglib.SpanData;
	var JBox = jiglib.JBox;
	var JIndexedTriangle = jiglib.JIndexedTriangle;
	var JSegment = jiglib.JSegment;
	var JTriangle = jiglib.JTriangle;
	var JTriangleMesh = jiglib.JTriangleMesh;
	var JNumber3D = jiglib.JNumber3D;
	var JMath3D = jiglib.JMath3D;
	var MaterialProperties = jiglib.MaterialProperties;
	var RigidBody = jiglib.RigidBody;

	var CollDetectBoxMesh = function()
	{

		this.name = "BoxMesh";
		this.type0 = "BOX";
		this.type1 = "TRIANGLEMESH";
		
	}

	jiglib.extend(CollDetectBoxMesh, CollDetectFunctor);

	CollDetectBoxMesh.prototype.disjoint = function(out, axis, box, triangle)
	{

		var obj0 = box.getSpan(axis);
		var obj1 = triangle.getSpan(axis);
		var obj0Min=obj0.min, obj0Max=obj0.max, obj1Min=obj1.min, obj1Max=obj1.max, tiny=JMath3D.NUM_TINY;
		
		if (obj0Min > (obj1Max + JConfig.collToll + tiny) || obj1Min > (obj0Max + JConfig.collToll + tiny))
		{
			out.flag = true;
			return true;
		}
		if ((obj0Max > obj1Max) && (obj1Min > obj0Min))
		{
			out.depth = Math.min(obj0Max - obj1Min, obj1Max - obj0Min);
		}
		else if ((obj1Max > obj0Max) && (obj0Min > obj1Min))
		{
			out.depth = Math.min(obj1Max - obj0Min, obj0Max - obj1Min);
		}
		else
		{
			out.depth = Math.min(obj0Max, obj1Max);
			out.depth -= Math.max(obj0Min, obj1Min);
		}
		out.flag = false;
		return false;
		
	}

	CollDetectBoxMesh.prototype.addPoint = function(contactPoints, pt, combinationDistanceSq)
	{

		for (var contactPoints_i = 0, contactPoints_l = contactPoints.length, contactPoint; (contactPoints_i < contactPoints_l) && (contactPoint = contactPoints[contactPoints_i]); contactPoints_i++)
		{
			if (contactPoint.subtract(pt).get_lengthSquared() < combinationDistanceSq)
			{
				contactPoint = JNumber3D.getScaleVector(contactPoint.add(pt), 0.5);
				return false;
			}
		}
		contactPoints.push(pt);
		return true;
		
	}

	CollDetectBoxMesh.prototype.getBoxTriangleIntersectionPoints = function(pts, box, triangle, combinationDistanceSq)
	{

		var edges=box.get_edges();
		var boxPts=box.getCornerPoints(box.get_currentState());
		
		var data;
		var edge;
		var seg;
		for(var i=0;i<12;i++){
			edge=edges[i];
			data=new CollOutData();
			seg=new JSegment(boxPts[edge.ind0],boxPts[edge.ind1].subtract(boxPts[edge.ind0]));
			if(triangle.segmentTriangleIntersection(data,seg)){
				this.addPoint(pts,seg.getPoint(data.frac),combinationDistanceSq);
				if(pts.length>8) return pts.length;
			}
		}
		
		var pt0, pt1;
		for(i=0;i<3;i++){
			pt0=triangle.getVertex(i);
			pt1=triangle.getVertex((i+1)%3);
			data=new CollOutData();
			if(box.segmentIntersect(data,new JSegment(pt0,pt1.subtract(pt0)),box.get_currentState())){
				this.addPoint(pts,data.position,combinationDistanceSq);
				if(pts.length>8) return pts.length;
			}
			if(box.segmentIntersect(data,new JSegment(pt1,pt0.subtract(pt1)),box.get_currentState())){
				this.addPoint(pts,data.position,combinationDistanceSq);
				if(pts.length>8) return pts.length;
			}
		}
		return pts.length;
		
	}

	CollDetectBoxMesh.prototype.doOverlapBoxTriangleTest = function(box, triangle, mesh, info, collArr)
	{

		
		var triEdge0, triEdge1, triEdge2, triNormal, D, N, boxOldPos, boxNewPos, meshPos, delta;
		var dirs0=box.get_currentState().getOrientationCols();
		var tri=new JTriangle(mesh.get_octree().getVertex(triangle.getVertexIndex(0)),mesh.get_octree().getVertex(triangle.getVertexIndex(1)),mesh.get_octree().getVertex(triangle.getVertexIndex(2)));
		triEdge0=tri.getVertex(1).subtract(tri.getVertex(0));
		triEdge0.normalize();
		triEdge1=tri.getVertex(2).subtract(tri.getVertex(1));
		triEdge1.normalize();
		triEdge2=tri.getVertex(0).subtract(tri.getVertex(2));
		triEdge2.normalize();
		triNormal=triangle.get_plane().get_normal().clone();
		
		var numAxes=13;
		var axes = [triNormal,dirs0[0],dirs0[1],dirs0[2],
										dirs0[0].crossProduct(triEdge0),
										dirs0[0].crossProduct(triEdge1),
										dirs0[0].crossProduct(triEdge2),
										dirs0[1].crossProduct(triEdge0),
										dirs0[1].crossProduct(triEdge1),
										dirs0[1].crossProduct(triEdge2),
										dirs0[2].crossProduct(triEdge0),
										dirs0[2].crossProduct(triEdge1),
										dirs0[2].crossProduct(triEdge2)];
		
		var overlapDepths=[];
		for(var i=0;i<numAxes;i++){
			overlapDepths[i]=new SpanData();
			if(this.disjoint(overlapDepths[i],axes[i],box,tri)){
				return false;
			}
		}
		
		var minAxis=-1;
		var tiny=JMath3D.NUM_TINY, minDepth=JMath3D.NUM_HUGE, l2, invl, depth, combinationDist, oldDepth;

		for(i = 0; i < numAxes; i++){
			l2=axes[i].get_lengthSquared();
			if (l2 < tiny){
				continue;
			}
			
			invl=1/Math.sqrt(l2);
			axes[i].scaleBy(invl);
			overlapDepths[i].depth*=invl;
			
			if (overlapDepths[i].depth < minDepth){
				minDepth = overlapDepths[i].depth;
				minAxis=i;
			}
		}
		
		if (minAxis == -1) return false;
		
		D=box.get_currentState().position.subtract(tri.getCentre());
		N=axes[minAxis];
		depth=overlapDepths[minAxis].depth;
		
		if(D.dotProduct(N)<0){
			N.negate();
		}
		
		boxOldPos=box.get_oldState().position;
		boxNewPos=box.get_currentState().position;
		meshPos=mesh.get_currentState().position;
		
		var pts=[];
		combinationDist=depth+0.05;
		this.getBoxTriangleIntersectionPoints(pts,box,tri,combinationDist*combinationDist);
		
		delta=boxNewPos.subtract(boxOldPos);
		oldDepth=depth+delta.dotProduct(N);
		
		var numPts = pts.length;
		var collPts = [];
		if(numPts>0){
			var cpInfo;
			for (i=0; i<numPts; i++){
				cpInfo = new CollPointInfo();
				cpInfo.r0=pts[i].subtract(boxNewPos);
				cpInfo.r1=pts[i].subtract(meshPos);
				cpInfo.initialPenetration=oldDepth;
				collPts[i]=cpInfo;
			}
			
			var collInfo = new CollisionInfo();
			collInfo.objInfo = info;
			collInfo.dirToBody = N;
			collInfo.pointInfo = collPts;
			
			var mat = new MaterialProperties();
			mat.restitution = 0.5*(box.get_material().restitution + mesh.get_material().restitution);
			mat.friction = 0.5*(box.get_material().friction + mesh.get_material().friction);
			collInfo.mat = mat;
			collArr.push(collInfo);
			info.body0.collisions.push(collInfo);
			info.body1.collisions.push(collInfo);
			info.body0.addCollideBody(info.body1);
			info.body1.addCollideBody(info.body0);
			return true;
		}else {
			info.body0.removeCollideBodies(info.body1);
			info.body1.removeCollideBodies(info.body0);
			return false;
		}
		
	}

	CollDetectBoxMesh.prototype.collDetectBoxStaticMeshOverlap = function(box, mesh, info, collArr)
	{

		var boxRadius=box.get_boundingSphere();
		var boxCentre=box.get_currentState().position;
		
		var potentialTriangles = [];
		var numTriangles=mesh.get_octree().getTrianglesIntersectingtAABox(potentialTriangles,box.get_boundingBox());
		
		var collision=false;
		var dist;
		var meshTriangle;
		for (var iTriangle = 0 ; iTriangle < numTriangles ; ++iTriangle) {
			meshTriangle=mesh.get_octree().getTriangle(potentialTriangles[iTriangle]);
			
			dist=meshTriangle.get_plane().pointPlaneDistance(boxCentre);
			if (dist > boxRadius || dist < 0){
				continue;
			}
			
			if(this.doOverlapBoxTriangleTest(box,meshTriangle,mesh,info,collArr)){
				collision = true;
			}
		}
		
		return collision;
		
	}

	CollDetectBoxMesh.prototype.collDetect = function(info, collArr)
	{

		var tempBody;
		if (info.body0.get_type() == "TRIANGLEMESH")
		{
			tempBody = info.body0;
			info.body0 = info.body1;
			info.body1 = tempBody;
		}
		var box = info.body0;
		var mesh = info.body1;
		
		this.collDetectBoxStaticMeshOverlap(box,mesh,info,collArr);
		
	}



	jiglib.CollDetectBoxMesh = CollDetectBoxMesh; 

})(jiglib);


(function(jiglib) {

	var CollDetectBoxMesh = jiglib.CollDetectBoxMesh;
	var CollDetectBoxBox = jiglib.CollDetectBoxBox;
	var CollDetectSphereTerrain = jiglib.CollDetectSphereTerrain;
	var CollDetectSphereBox = jiglib.CollDetectSphereBox;
	var CollDetectCapsuleTerrain = jiglib.CollDetectCapsuleTerrain;
	var CollDetectSphereCapsule = jiglib.CollDetectSphereCapsule;
	var CollisionSystemBrute = jiglib.CollisionSystemBrute;
	var CollDetectCapsuleBox = jiglib.CollDetectCapsuleBox;
	var CollDetectSphereMesh = jiglib.CollDetectSphereMesh;
	var CollDetectBoxTerrain = jiglib.CollDetectBoxTerrain;
	var CollisionSystemGrid = jiglib.CollisionSystemGrid;
	var CollDetectCapsuleCapsule = jiglib.CollDetectCapsuleCapsule;
	var CollPointInfo = jiglib.CollPointInfo;
	var CollisionInfo = jiglib.CollisionInfo;
	var CollisionSystemAbstract = jiglib.CollisionSystemAbstract;
	var CollDetectCapsulePlane = jiglib.CollDetectCapsulePlane;
	var CollDetectInfo = jiglib.CollDetectInfo;
	var CollDetectSphereSphere = jiglib.CollDetectSphereSphere;
	var CollisionSystemGridEntry = jiglib.CollisionSystemGridEntry;
	var CollDetectSpherePlane = jiglib.CollDetectSpherePlane;
	var CollDetectFunctor = jiglib.CollDetectFunctor;
	var Vector3D = jiglib.Vector3D;
	var JConfig = jiglib.JConfig;
	var JIndexedTriangle = jiglib.JIndexedTriangle;
	var JOctree = jiglib.JOctree;
	var JCapsule = jiglib.JCapsule;
	var JBox = jiglib.JBox;
	var JRay = jiglib.JRay;
	var JAABox = jiglib.JAABox;
	var JTerrain = jiglib.JTerrain;
	var JPlane = jiglib.JPlane;
	var JTriangleMesh = jiglib.JTriangleMesh;
	var JTriangle = jiglib.JTriangle;
	var JSphere = jiglib.JSphere;
	var JSegment = jiglib.JSegment;
	var JMatrix3D = jiglib.JMatrix3D;
	var Matrix3D = jiglib.Matrix3D;
	var JMath3D = jiglib.JMath3D;
	var JNumber3D = jiglib.JNumber3D;
	var MaterialProperties = jiglib.MaterialProperties;
	var RigidBody = jiglib.RigidBody;

	var CollDetectBoxPlane = function()
	{

		this.name = "BoxPlane";
		this.type0 = "BOX";
		this.type1 = "PLANE";
		
	}

	jiglib.extend(CollDetectBoxPlane, CollDetectFunctor);

	CollDetectBoxPlane.prototype.collDetect = function(info, collArr)
	{

		var tempBody;
		if (info.body0.get_type() == "PLANE")
		{
			tempBody = info.body0;
			info.body0 = info.body1;
			info.body1 = tempBody;
		}

		var box = info.body0;
		var plane = info.body1;

		var centreDist = plane.pointPlaneDistance(box.get_currentState().position);
		if (centreDist > box.get_boundingSphere() + JConfig.collToll)
			return;

		var newPts = box.getCornerPoints(box.get_currentState());
		var oldPts = box.getCornerPoints(box.get_oldState());
		var collPts = [];
		var cpInfo;
		var newPt, oldPt;
		var newDepth, oldDepth;
		var newPts_length = newPts.length;
		
		for (var i = 0; i < newPts_length; i++)
		{
			newPt = newPts[i];
			oldPt = oldPts[i];
			newDepth = -1 * plane.pointPlaneDistance(newPt);
			oldDepth = -1 * plane.pointPlaneDistance(oldPt);
			
			if (Math.max(newDepth, oldDepth) > -JConfig.collToll)
			{
				cpInfo = new CollPointInfo();
				cpInfo.r0 = oldPt.subtract(box.get_oldState().position);
				cpInfo.r1 = oldPt.subtract(plane.get_oldState().position);
				cpInfo.initialPenetration = oldDepth;
				collPts.push(cpInfo);
			}
		}
		
		if (collPts.length > 0)
		{
			var collInfo = new CollisionInfo();
			collInfo.objInfo = info;
			collInfo.dirToBody = plane.get_normal().clone();
			collInfo.pointInfo = collPts;
			
			var mat = new MaterialProperties();
			mat.restitution = 0.5*(box.get_material().restitution + plane.get_material().restitution);
			mat.friction = 0.5*(box.get_material().friction + plane.get_material().friction);
			collInfo.mat = mat;
			collArr.push(collInfo);
			info.body0.collisions.push(collInfo);
			info.body1.collisions.push(collInfo);
			info.body0.addCollideBody(info.body1);
			info.body1.addCollideBody(info.body0);
		}else {
			info.body0.removeCollideBodies(info.body1);
			info.body1.removeCollideBodies(info.body0);
		}
		
	}



	jiglib.CollDetectBoxPlane = CollDetectBoxPlane; 

})(jiglib);


(function(jiglib) {

	var CollDetectBoxPlane = jiglib.CollDetectBoxPlane;
	var CollDetectBoxMesh = jiglib.CollDetectBoxMesh;
	var CollDetectBoxBox = jiglib.CollDetectBoxBox;
	var CollDetectSphereTerrain = jiglib.CollDetectSphereTerrain;
	var CollDetectSphereBox = jiglib.CollDetectSphereBox;
	var CollDetectCapsuleTerrain = jiglib.CollDetectCapsuleTerrain;
	var CollDetectSphereCapsule = jiglib.CollDetectSphereCapsule;
	var CollisionSystemBrute = jiglib.CollisionSystemBrute;
	var CollDetectCapsuleBox = jiglib.CollDetectCapsuleBox;
	var CollDetectSphereMesh = jiglib.CollDetectSphereMesh;
	var CollDetectBoxTerrain = jiglib.CollDetectBoxTerrain;
	var CollisionSystemGrid = jiglib.CollisionSystemGrid;
	var CollPointInfo = jiglib.CollPointInfo;
	var CollisionInfo = jiglib.CollisionInfo;
	var CollisionSystemAbstract = jiglib.CollisionSystemAbstract;
	var CollDetectCapsulePlane = jiglib.CollDetectCapsulePlane;
	var CollDetectInfo = jiglib.CollDetectInfo;
	var CollDetectSphereSphere = jiglib.CollDetectSphereSphere;
	var CollisionSystemGridEntry = jiglib.CollisionSystemGridEntry;
	var CollDetectSpherePlane = jiglib.CollDetectSpherePlane;
	var CollDetectFunctor = jiglib.CollDetectFunctor;
	var Vector3D = jiglib.Vector3D;
	var JConfig = jiglib.JConfig;
	var JIndexedTriangle = jiglib.JIndexedTriangle;
	var JOctree = jiglib.JOctree;
	var JCapsule = jiglib.JCapsule;
	var JBox = jiglib.JBox;
	var JRay = jiglib.JRay;
	var JAABox = jiglib.JAABox;
	var JTerrain = jiglib.JTerrain;
	var JPlane = jiglib.JPlane;
	var JTriangleMesh = jiglib.JTriangleMesh;
	var JTriangle = jiglib.JTriangle;
	var JSphere = jiglib.JSphere;
	var JSegment = jiglib.JSegment;
	var JMatrix3D = jiglib.JMatrix3D;
	var Matrix3D = jiglib.Matrix3D;
	var JMath3D = jiglib.JMath3D;
	var JNumber3D = jiglib.JNumber3D;
	var MaterialProperties = jiglib.MaterialProperties;

	var CollDetectCapsuleCapsule = function()
	{

		this.name = "CapsuleCapsule";
		this.type0 = "CAPSULE";
		this.type1 = "CAPSULE";
		
	}

	jiglib.extend(CollDetectCapsuleCapsule, CollDetectFunctor);

	CollDetectCapsuleCapsule.prototype.collDetect = function(info, collArr)
	{

		var capsule0 = info.body0;
		var capsule1 = info.body1;

		if (!capsule0.hitTestObject3D(capsule1))
		{
			return;
		}
		
		if (!capsule0.get_boundingBox().overlapTest(capsule1.get_boundingBox())) {
			return;
		}

		var collPts = [];
		var cpInfo;

		var averageNormal = new Vector3D();
		var oldSeg0, newSeg0, oldSeg1, newSeg1;
		oldSeg0 = new JSegment(capsule0.getEndPos(capsule0.get_oldState()), JNumber3D.getScaleVector(capsule0.get_oldState().getOrientationCols()[1], -capsule0.get_length()));
		newSeg0 = new JSegment(capsule0.getEndPos(capsule0.get_currentState()), JNumber3D.getScaleVector(capsule0.get_currentState().getOrientationCols()[1], -capsule0.get_length()));
		oldSeg1 = new JSegment(capsule1.getEndPos(capsule1.get_oldState()), JNumber3D.getScaleVector(capsule1.get_oldState().getOrientationCols()[1], -capsule1.get_length()));
		newSeg1 = new JSegment(capsule1.getEndPos(capsule1.get_currentState()), JNumber3D.getScaleVector(capsule1.get_currentState().getOrientationCols()[1], -capsule1.get_length()));

		var radSum = capsule0.get_radius() + capsule1.get_radius();

		var oldObj = [];
		var oldDistSq = oldSeg0.segmentSegmentDistanceSq(oldObj, oldSeg1);
		var newObj = [];
		var newDistSq = newSeg0.segmentSegmentDistanceSq(oldObj, newSeg1);

		if (Math.min(oldDistSq, newDistSq) < Math.pow(radSum + JConfig.collToll, 2))
		{
			var pos0 = oldSeg0.getPoint(oldObj[0]);
			var pos1 = oldSeg1.getPoint(oldObj[1]);

			var delta = pos0.subtract(pos1);
			var dist = Math.sqrt(oldDistSq);
			var depth = radSum - dist;

			if (dist > JMath3D.NUM_TINY)
			{
				delta = JNumber3D.getDivideVector(delta, dist);
			}
			else
			{
				delta = JMatrix3D.getRotationMatrix(0, 0, 1, 360 * Math.random()).transformVector(Vector3D.Y_AXIS);
			}

			var worldPos = pos1.add(JNumber3D.getScaleVector(delta, capsule1.get_radius() - 0.5 * depth));
			averageNormal = averageNormal.add(delta);

			cpInfo = new CollPointInfo();
			cpInfo.r0 = worldPos.subtract(capsule0.get_oldState().position);
			cpInfo.r1 = worldPos.subtract(capsule1.get_oldState().position);
			cpInfo.initialPenetration = depth;
			collPts[0]=cpInfo;
		}

		if (collPts.length > 0)
		{
			var collInfo = new CollisionInfo();
			collInfo.objInfo = info;
			collInfo.dirToBody = averageNormal;
			collInfo.pointInfo = collPts;
			
			var mat = new MaterialProperties();
			mat.restitution = 0.5*(capsule0.get_material().restitution + capsule1.get_material().restitution);
			mat.friction = 0.5*(capsule0.get_material().friction + capsule1.get_material().friction);
			collInfo.mat = mat;
			collArr.push(collInfo);
			info.body0.collisions.push(collInfo);
			info.body1.collisions.push(collInfo);
			info.body0.addCollideBody(info.body1);
			info.body1.addCollideBody(info.body0);
		}else {
			info.body0.removeCollideBodies(info.body1);
			info.body1.removeCollideBodies(info.body0);
		}
		
	}



	jiglib.CollDetectCapsuleCapsule = CollDetectCapsuleCapsule; 

})(jiglib);


(function(jiglib) {

	var CollDetectBoxPlane = jiglib.CollDetectBoxPlane;
	var CollDetectBoxMesh = jiglib.CollDetectBoxMesh;
	var CollDetectBoxBox = jiglib.CollDetectBoxBox;
	var CollDetectSphereTerrain = jiglib.CollDetectSphereTerrain;
	var CollDetectSphereBox = jiglib.CollDetectSphereBox;
	var CollDetectCapsuleTerrain = jiglib.CollDetectCapsuleTerrain;
	var CollDetectSphereCapsule = jiglib.CollDetectSphereCapsule;
	var CollisionSystemBrute = jiglib.CollisionSystemBrute;
	var CollDetectCapsuleBox = jiglib.CollDetectCapsuleBox;
	var CollDetectSphereMesh = jiglib.CollDetectSphereMesh;
	var CollDetectBoxTerrain = jiglib.CollDetectBoxTerrain;
	var CollDetectFunctor = jiglib.CollDetectFunctor;
	var CollisionSystemGrid = jiglib.CollisionSystemGrid;
	var CollDetectCapsuleCapsule = jiglib.CollDetectCapsuleCapsule;
	var CollisionInfo = jiglib.CollisionInfo;
	var CollisionSystemAbstract = jiglib.CollisionSystemAbstract;
	var CollDetectCapsulePlane = jiglib.CollDetectCapsulePlane;
	var CollDetectInfo = jiglib.CollDetectInfo;
	var CollDetectSphereSphere = jiglib.CollDetectSphereSphere;
	var CollisionSystemGridEntry = jiglib.CollisionSystemGridEntry;
	var CollDetectSpherePlane = jiglib.CollDetectSpherePlane;
	var Vector3D = jiglib.Vector3D;
	var JMatrix3D = jiglib.JMatrix3D;
	var Matrix3D = jiglib.Matrix3D;
	var JMath3D = jiglib.JMath3D;
	var JNumber3D = jiglib.JNumber3D;

	var CollPointInfo = function()
	{
		this.initialPenetration = null; // Number
		this.r0 = null; // Vector3D
		this.r1 = null; // Vector3D
		this.position = null; // Vector3D
		this.minSeparationVel =  0; // Number
		this.denominator =  0; // Number
		this.accumulatedNormalImpulse =  0; // Number
		this.accumulatedNormalImpulseAux =  0; // Number
		this.accumulatedFrictionImpulse =  new Vector3D(); // Vector3D
	}



	jiglib.CollPointInfo = CollPointInfo; 

})(jiglib);


(function(jiglib) {

	var CollDetectBoxPlane = jiglib.CollDetectBoxPlane;
	var CollDetectBoxMesh = jiglib.CollDetectBoxMesh;
	var CollDetectBoxBox = jiglib.CollDetectBoxBox;
	var CollDetectSphereTerrain = jiglib.CollDetectSphereTerrain;
	var CollDetectSphereBox = jiglib.CollDetectSphereBox;
	var CollDetectCapsuleTerrain = jiglib.CollDetectCapsuleTerrain;
	var CollDetectSphereCapsule = jiglib.CollDetectSphereCapsule;
	var CollisionSystemBrute = jiglib.CollisionSystemBrute;
	var CollDetectCapsuleBox = jiglib.CollDetectCapsuleBox;
	var CollDetectSphereMesh = jiglib.CollDetectSphereMesh;
	var CollDetectBoxTerrain = jiglib.CollDetectBoxTerrain;
	var CollDetectFunctor = jiglib.CollDetectFunctor;
	var CollisionSystemGrid = jiglib.CollisionSystemGrid;
	var CollDetectCapsuleCapsule = jiglib.CollDetectCapsuleCapsule;
	var CollPointInfo = jiglib.CollPointInfo;
	var CollisionSystemAbstract = jiglib.CollisionSystemAbstract;
	var CollDetectCapsulePlane = jiglib.CollDetectCapsulePlane;
	var CollDetectInfo = jiglib.CollDetectInfo;
	var CollDetectSphereSphere = jiglib.CollDetectSphereSphere;
	var CollisionSystemGridEntry = jiglib.CollisionSystemGridEntry;
	var CollDetectSpherePlane = jiglib.CollDetectSpherePlane;
	var Vector3D = jiglib.Vector3D;
	var MaterialProperties = jiglib.MaterialProperties;

	var CollisionInfo = function()
	{
		this.mat =  new MaterialProperties(); // MaterialProperties
		this.objInfo = null; // CollDetectInfo
		this.dirToBody = null; // Vector3D
		this.pointInfo = null; // CollPointInfo
		this.satisfied = null; // Boolean
	}



	jiglib.CollisionInfo = CollisionInfo; 

})(jiglib);


(function(jiglib) {

	var CollDetectBoxPlane = jiglib.CollDetectBoxPlane;
	var CollDetectBoxMesh = jiglib.CollDetectBoxMesh;
	var CollDetectBoxBox = jiglib.CollDetectBoxBox;
	var CollDetectSphereTerrain = jiglib.CollDetectSphereTerrain;
	var CollDetectSphereBox = jiglib.CollDetectSphereBox;
	var CollDetectCapsuleTerrain = jiglib.CollDetectCapsuleTerrain;
	var CollDetectSphereCapsule = jiglib.CollDetectSphereCapsule;
	var CollisionSystemBrute = jiglib.CollisionSystemBrute;
	var CollDetectCapsuleBox = jiglib.CollDetectCapsuleBox;
	var CollDetectSphereMesh = jiglib.CollDetectSphereMesh;
	var CollDetectBoxTerrain = jiglib.CollDetectBoxTerrain;
	var CollDetectFunctor = jiglib.CollDetectFunctor;
	var CollisionSystemGrid = jiglib.CollisionSystemGrid;
	var CollDetectCapsuleCapsule = jiglib.CollDetectCapsuleCapsule;
	var CollPointInfo = jiglib.CollPointInfo;
	var CollisionInfo = jiglib.CollisionInfo;
	var CollDetectCapsulePlane = jiglib.CollDetectCapsulePlane;
	var CollDetectInfo = jiglib.CollDetectInfo;
	var CollDetectSphereSphere = jiglib.CollDetectSphereSphere;
	var CollisionSystemGridEntry = jiglib.CollisionSystemGridEntry;
	var CollDetectSpherePlane = jiglib.CollDetectSpherePlane;
	var Vector3D = jiglib.Vector3D;
	var CollOutBodyData = jiglib.CollOutBodyData;
	var JSegment = jiglib.JSegment;
	var JNumber3D = jiglib.JNumber3D;
	var JMath3D = jiglib.JMath3D;
	var RigidBody = jiglib.RigidBody;

	var CollisionSystemAbstract = function()
	{
		this.detectionFunctors = null; // Dictionary
		this.collBody = null; // RigidBody
		this._numCollisionsChecks =  0; // uint
		this.startPoint = null; // Vector3D

		this.collBody = [];
		this.detectionFunctors = [];
		this.detectionFunctors["BOX_BOX"] = new CollDetectBoxBox();
		this.detectionFunctors["BOX_SPHERE"] = new CollDetectSphereBox();
		this.detectionFunctors["BOX_CAPSULE"] = new CollDetectCapsuleBox();
		this.detectionFunctors["BOX_PLANE"] = new CollDetectBoxPlane();
		this.detectionFunctors["BOX_TERRAIN"] = new CollDetectBoxTerrain();
		this.detectionFunctors["BOX_TRIANGLEMESH"] = new CollDetectBoxMesh();
		this.detectionFunctors["SPHERE_BOX"] = new CollDetectSphereBox();
		this.detectionFunctors["SPHERE_SPHERE"] = new CollDetectSphereSphere();
		this.detectionFunctors["SPHERE_CAPSULE"] = new CollDetectSphereCapsule();
		this.detectionFunctors["SPHERE_PLANE"] = new CollDetectSpherePlane();
		this.detectionFunctors["SPHERE_TERRAIN"] = new CollDetectSphereTerrain();
		this.detectionFunctors["SPHERE_TRIANGLEMESH"] = new CollDetectSphereMesh();
		this.detectionFunctors["CAPSULE_CAPSULE"] = new CollDetectCapsuleCapsule();
		this.detectionFunctors["CAPSULE_BOX"] = new CollDetectCapsuleBox();
		this.detectionFunctors["CAPSULE_SPHERE"] = new CollDetectSphereCapsule();
		this.detectionFunctors["CAPSULE_PLANE"] = new CollDetectCapsulePlane();
		this.detectionFunctors["CAPSULE_TERRAIN"] = new CollDetectCapsuleTerrain();
		this.detectionFunctors["PLANE_BOX"] = new CollDetectBoxPlane();
		this.detectionFunctors["PLANE_SPHERE"] = new CollDetectSpherePlane();
		this.detectionFunctors["PLANE_CAPSULE"] = new CollDetectCapsulePlane();
		this.detectionFunctors["TERRAIN_SPHERE"] = new CollDetectSphereTerrain();
		this.detectionFunctors["TERRAIN_BOX"] = new CollDetectBoxTerrain();
		this.detectionFunctors["TERRAIN_CAPSULE"] = new CollDetectCapsuleTerrain();
		this.detectionFunctors["TRIANGLEMESH_SPHERE"] = new CollDetectSphereMesh();
		this.detectionFunctors["TRIANGLEMESH_BOX"] = new CollDetectBoxMesh();
		
	}

	CollisionSystemAbstract.prototype.addCollisionBody = function(body)
	{

		if (this.collBody.indexOf(body) < 0)
			this.collBody.push(body);
		
	}

	CollisionSystemAbstract.prototype.removeCollisionBody = function(body)
	{

		if (this.collBody.indexOf(body) >= 0)
			this.collBody.splice(this.collBody.indexOf(body), 1);
		
	}

	CollisionSystemAbstract.prototype.removeAllCollisionBodies = function()
	{

		this.collBody.length=0;
		
	}

	CollisionSystemAbstract.prototype.detectCollisions = function(body, collArr)
	{

		if (!body.isActive)
			return;
		
		var info;
		var fu;
		
		for (var collBody_i = 0, collBody_l = this.collBody.length, _collBody; (collBody_i < collBody_l) && (_collBody = this.collBody[collBody_i]); collBody_i++)
		{
			if (body == _collBody)
			{
				continue;
			}
			if (this.checkCollidables(body, _collBody) && this.detectionFunctors[body.get_type() + "_" + _collBody.get_type()] != undefined)
			{
				info = new CollDetectInfo();
				info.body0 = body;
				info.body1 = _collBody;
				fu = this.detectionFunctors[info.body0.get_type() + "_" + info.body1.get_type()];
				fu.collDetect(info, collArr);
			}
		}
		
	}

	CollisionSystemAbstract.prototype.detectAllCollisions = function(bodies, collArr)
	{

		
	}

	CollisionSystemAbstract.prototype.collisionSkinMoved = function(colBody)
	{

		// used for grid
		
	}

	CollisionSystemAbstract.prototype.segmentIntersect = function(out, seg, ownerBody)
	{

		out.frac = JMath3D.NUM_HUGE;
		out.position = new Vector3D();
		out.normal = new Vector3D();
		
		var obj = new CollOutBodyData();
		for (var collBody_i = 0, collBody_l = this.collBody.length, _collBody; (collBody_i < collBody_l) && (_collBody = this.collBody[collBody_i]); collBody_i++)
		{
			if (_collBody != ownerBody && this.segmentBounding(seg, _collBody))
			{
				if (_collBody.segmentIntersect(obj, seg, _collBody.get_currentState()))
				{
				if (obj.frac < out.frac)
				{
					out.position = obj.position;
					out.normal = obj.normal;
					out.frac = obj.frac;
					out.rigidBody = _collBody;
				}
				}
			}
		}
		
		if (out.frac > 1)
			return false;
		
		if (out.frac < 0)
		{
			out.frac = 0;
		}
		else if (out.frac > 1)
		{
			out.frac = 1;
		}
		
		return true;
		
	}

	CollisionSystemAbstract.prototype.segmentBounding = function(seg, obj)
	{

		var pos = seg.getPoint(0.5);
		var r = seg.delta.get_length() / 2;
		
		var num1 = pos.subtract(obj.get_currentState().position).get_length();
		var num2 = r + obj.get_boundingSphere();
		
		if (num1 <= num2)
			return true;
		else
			return false;
		
	}

	CollisionSystemAbstract.prototype.get_numCollisionsChecks = function()
	{

		return this._numCollisionsChecks;	
		
	}

	CollisionSystemAbstract.prototype.checkCollidables = function(body0, body1)
	{

		if (body0.get_nonCollidables().length == 0 && body1.get_nonCollidables().length == 0)
			return true;
		
		if(body0.get_nonCollidables().indexOf(body1) > -1)
			return false;
		
		if(body1.get_nonCollidables().indexOf(body0) > -1)
			return false;
		
		return true;
		
	}



	jiglib.CollisionSystemAbstract = CollisionSystemAbstract; 

})(jiglib);


(function(jiglib) {

	var CollDetectBoxPlane = jiglib.CollDetectBoxPlane;
	var CollDetectBoxMesh = jiglib.CollDetectBoxMesh;
	var CollDetectBoxBox = jiglib.CollDetectBoxBox;
	var CollDetectSphereTerrain = jiglib.CollDetectSphereTerrain;
	var CollDetectSphereBox = jiglib.CollDetectSphereBox;
	var CollDetectCapsuleTerrain = jiglib.CollDetectCapsuleTerrain;
	var CollDetectSphereCapsule = jiglib.CollDetectSphereCapsule;
	var CollisionSystemBrute = jiglib.CollisionSystemBrute;
	var CollDetectCapsuleBox = jiglib.CollDetectCapsuleBox;
	var CollDetectSphereMesh = jiglib.CollDetectSphereMesh;
	var CollDetectBoxTerrain = jiglib.CollDetectBoxTerrain;
	var CollDetectFunctor = jiglib.CollDetectFunctor;
	var CollDetectCapsuleCapsule = jiglib.CollDetectCapsuleCapsule;
	var CollPointInfo = jiglib.CollPointInfo;
	var CollisionInfo = jiglib.CollisionInfo;
	var CollDetectCapsulePlane = jiglib.CollDetectCapsulePlane;
	var CollDetectInfo = jiglib.CollDetectInfo;
	var CollDetectSphereSphere = jiglib.CollDetectSphereSphere;
	var CollisionSystemGridEntry = jiglib.CollisionSystemGridEntry;
	var CollDetectSpherePlane = jiglib.CollDetectSpherePlane;
	var CollisionSystemAbstract = jiglib.CollisionSystemAbstract;
	var Vector3D = jiglib.Vector3D;
	var CollOutBodyData = jiglib.CollOutBodyData;
	var JAABox = jiglib.JAABox;
	var JSegment = jiglib.JSegment;
	var JMath3D = jiglib.JMath3D;
	var JNumber3D = jiglib.JNumber3D;
	var RigidBody = jiglib.RigidBody;

	var CollisionSystemGrid = function(sx, sy, sz, nx, ny, nz, dx, dy, dz)
	{
		this.gridEntries = null; // CollisionSystemGridEntry
		this.overflowEntries = null; // CollisionSystemGridEntry
		this.nx = null; // int
		this.ny = null; // int
		this.nz = null; // int
		this.dx = null; // Number
		this.dy = null; // Number
		this.dz = null; // Number
		this.sizeX = null; // Number
		this.sizeY = null; // Number
		this.sizeZ = null; // Number
		this.minDelta = null; // Number

		jiglib.CollisionSystemAbstract.apply(this, [  ]);
		
		this.nx = nx; this.ny = ny; this.nz = nz;
		this.dx = dx; this.dy = dy; this.dz = dz;
		this.sizeX = nx * dx;
		this.sizeY = ny * dy;
		this.sizeZ = nz * dz;
		this.minDelta = Math.min(dx, dy, dz);
		
		this.startPoint = new Vector3D(sx, sy, sz);
		
		this.gridEntries = [];
		
		var len=this.gridEntries.length;
		for (var j = 0; j < len; ++j)
		{
			var gridEntry = new CollisionSystemGridEntry(null);
			gridEntry.gridIndex = j;
			this.gridEntries[j]=gridEntry;
		}
		
		this.overflowEntries = new CollisionSystemGridEntry(null);
		this.overflowEntries.gridIndex = -1;
		
	}

	jiglib.extend(CollisionSystemGrid, CollisionSystemAbstract);

	CollisionSystemGrid.prototype.calcIndex = function(i, j, k)
	{

		var _i = i % this.nx;
		var _j = j % this.ny;
		var _k = k % this.nz;
		
		return (_i + this.nx * _j + (this.nx + this.ny) * _k);
		
	}

	CollisionSystemGrid.prototype.calcGridForSkin3 = function(colBody)
	{

		var i;var j;var k;
		var sides = colBody.get_boundingBox().get_sideLengths();
		
		if ((sides.x > this.dx) || (sides.y > this.dy) || (sides.z > this.dz))
		{
			i = j = k = -1;
			return new Vector3D(i,j,k);
		}
		
		var min = colBody.get_boundingBox().minPos.clone();
		min.x = JMath3D.getLimiteNumber(min.x, this.startPoint.x, this.startPoint.x + this.sizeX);
		min.y = JMath3D.getLimiteNumber(min.y, this.startPoint.y, this.startPoint.y + this.sizeY);
		min.z = JMath3D.getLimiteNumber(min.z, this.startPoint.z, this.startPoint.z + this.sizeZ);
		
		i =  ((min.x - this.startPoint.x) / this.dx) % this.nx;
		j =  ((min.y - this.startPoint.y) / this.dy) % this.ny;
		k =  ((min.z - this.startPoint.z) / this.dz) % this.nz;
		
		return new Vector3D(i,j,k);
		
	}

	CollisionSystemGrid.prototype.calcGridForSkin6 = function(colBody)
	{

		var tempStoreObject = new Object;
		var i;var j;var k;
		var fi;var fj;var fk;
		
		var sides = colBody.get_boundingBox().get_sideLengths();
		
		if ((sides.x > this.dx) || (sides.y > this.dy) || (sides.z > this.dz))
		{
			//trace("this.calcGridForSkin6 -- Rigidbody to big for gridsystem - putting it into overflow list (lengths,type,id):", sides.x,sides.y,sides.z,colBody.get_type(),colBody.get_id(),colBody.get_boundingBox().minPos,colBody.get_boundingBox().maxPos);
			i = j = k = -1;
			fi = fj = fk = 0.0;
			tempStoreObject.i = i; tempStoreObject.j = j; tempStoreObject.k = k; tempStoreObject.fi = fi; tempStoreObject.fj = fj; tempStoreObject.fk = fk;
			return tempStoreObject;
		}
		
		var min = colBody.get_boundingBox().minPos.clone();

		min.x = JMath3D.getLimiteNumber(min.x, this.startPoint.x, this.startPoint.x + this.sizeX);
		min.y = JMath3D.getLimiteNumber(min.y, this.startPoint.y, this.startPoint.y + this.sizeY);
		min.z = JMath3D.getLimiteNumber(min.z, this.startPoint.z, this.startPoint.z + this.sizeZ);
		
		fi = (min.x - this.startPoint.x) / this.dx;
		fj = (min.y - this.startPoint.y) / this.dy;
		fk = (min.z - this.startPoint.z) / this.dz;
		
		i = fi;
		j = fj;
		k = fk;
		
		if (i < 0) { i = 0; fi = 0.0; }
		else if (i >= this.nx) { i = 0; fi = 0.0; }
		else fi -= Number(i);
		
		if (j < 0) { j = 0; fj = 0.0; }
		else if (j >= this.ny) { j = 0; fj = 0.0; }
		else fj -= Number(j);
		
		if (k < 0) { k = 0; fk = 0.0; }
		else if (k >= this.nz) { k = 0; fk = 0.0; }
		else fk -= Number(k);
		
		tempStoreObject.i = i; tempStoreObject.j = j; tempStoreObject.k = k; tempStoreObject.fi = fi; tempStoreObject.fj = fj; tempStoreObject.fk = fk;
		//trace(i,j,k,fi,fj,fk);
		//trace(colBody.get_x(),colBody.get_y(),colBody.get_z());
		return tempStoreObject;
		
	}

	CollisionSystemGrid.prototype.calcGridIndexForBody = function(colBody)
	{

		var tempStoreVector = this.calcGridForSkin3(colBody);
		
		if (tempStoreVector.x == -1) return -1;
		return this.calcIndex(tempStoreVector.x, tempStoreVector.y, tempStoreVector.z);
		
	}

	CollisionSystemGrid.prototype.addCollisionBody = function(body)
	{

		if (this.collBody.indexOf(body) < 0)
			this.collBody.push(body);
		
		body.collisionSystem = this;

		// also do the grid stuff - for now put it on the overflow list
		var entry = new CollisionSystemGridEntry(body);
		body.externalData = entry;
		
		// add entry to the start of the list
		CollisionSystemGridEntry.insertGridEntryAfter(entry, this.overflowEntries);
		this.collisionSkinMoved(body);
		
	}

	CollisionSystemGrid.prototype.removeCollisionBody = function(body)
	{

		if (body.externalData != null)
		{
			body.externalData.collisionBody = null;
			CollisionSystemGridEntry.removeGridEntry(body.externalData);
			body.externalData = null;
		}

		if (this.collBody.indexOf(body) >= 0)
			this.collBody.splice(this.collBody.indexOf(body), 1);
		
	}

	CollisionSystemGrid.prototype.removeAllCollisionBodies = function()
	{

		for (var collBody_i = 0, collBody_l = this.collBody.length, body; (collBody_i < collBody_l) && (body = this.collBody[collBody_i]); collBody_i++){
			if (body.externalData != null)
			{
				body.externalData.collisionBody = null;
				CollisionSystemGridEntry.removeGridEntry(body.externalData);
			}
		}
		this.collBody.length=0;
		
	}

	CollisionSystemGrid.prototype.collisionSkinMoved = function(colBody)
	{

		var entry = colBody.externalData;
		if (entry == null)
		{
			//trace("Warning rigidbody has grid entry null!");
			return;
		}
		
		var gridIndex = this.calcGridIndexForBody(colBody);
				
		// see if it's moved grid
		if (gridIndex == entry.gridIndex)
			return;

		//trace(gridIndex);
		var start;
		//if (gridIndex >= 0**)
		if (this.gridEntries.length-1 > gridIndex && gridIndex >=0) // check if it's outside the gridspace, if so add to overflow
			start = this.gridEntries[gridIndex];
		else
			start = this.overflowEntries;
		
		CollisionSystemGridEntry.removeGridEntry(entry);
		CollisionSystemGridEntry.insertGridEntryAfter(entry, start);
		
	}

	CollisionSystemGrid.prototype.getListsToCheck = function(colBody)
	{

		var entries = []; 
		
		var entry = colBody.externalData;
		if (entry == null)
		{
			//trace("Warning skin has grid entry null!");
			return null;
		}
		
		// todo - work back from the mGridIndex rather than calculating it again...
		var i, j, k;
		var fi, fj, fk;
		var tempStoreObject = this.calcGridForSkin6(colBody);
		i = tempStoreObject.i; j = tempStoreObject.j; k = tempStoreObject.k; fi = tempStoreObject.fi; fj = tempStoreObject.fj; fk = tempStoreObject.fk;
		
		if (i == -1)
		{
			//trace("ADD ALL!");
			entries=this.gridEntries.concat();
			entries.push(this.overflowEntries);
			return entries;
		}
		
		// always add the overflow
		entries.push(this.overflowEntries);
		
		var delta = colBody.get_boundingBox().get_sideLengths(); // skin.WorldBoundingBox.Max - skin.WorldBoundingBox.Min;
		var maxI = 1, maxJ = 1, maxK = 1;
		if (fi + (delta.x / this.dx) < 1)
			maxI = 0;
		if (fj + (delta.y / this.dy) < 1)
			maxJ = 0;
		if (fk + (delta.z / this.dz) < 1)
			maxK = 0;
		
		// now add the contents of all grid boxes - their contents may extend beyond the bounds
		for (var di = -1; di <= maxI; ++di)
		{
			for (var dj = -1; dj <= maxJ; ++dj)
			{
				for (var dk = -1; dk <= maxK; ++dk)
				{
				var thisIndex = this.calcIndex(i + di, j + dj, k + dk); // + ((this.nx*this.ny*this.nz)*0.5);
				//trace("ge", this.gridEntries.length);
				if (this.gridEntries.length-1 > thisIndex && thisIndex >=0) {
					var start = this.gridEntries[thisIndex];
				 
					//trace(thisIndex,this.gridEntries.length);
					if (start != null && start.next != null)
					{
						entries.push(start);
					}
				}
				}
			}
		}
		return entries;
		
	}

	CollisionSystemGrid.prototype.detectAllCollisions = function(bodies, collArr)
	{

		var info;
		var fu;
		var bodyID;
		var bodyType;
		this._numCollisionsChecks = 0;
		
		for (var bodies_i = 0, bodies_l = bodies.length, body; (bodies_i < bodies_l) && (body = bodies[bodies_i]); bodies_i++)
		{
			if (!body.isActive)
				continue;

			bodyID = body.get_id();
			bodyType = body.get_type();
			
			var lists=this.getListsToCheck(body);
			
			for (var lists_i = 0, lists_l = lists.length, entry; (lists_i < lists_l) && (entry = lists[lists_i]); lists_i++)
			{
				
				for (entry = entry.next; entry != null; entry = entry.next)
				{
				if (body == entry.collisionBody)
					continue;
				
				if (entry.collisionBody.isActive && bodyID > entry.collisionBody.get_id())
					continue;
				
				if (this.checkCollidables(body, entry.collisionBody) && this.detectionFunctors[bodyType + "_" + entry.collisionBody.get_type()] != undefined)
				{
					info = new CollDetectInfo();
					info.body0 = body;
					info.body1 = entry.collisionBody;
					fu = this.detectionFunctors[info.body0.get_type() + "_" + info.body1.get_type()];
					fu.collDetect(info, collArr);
					this._numCollisionsChecks += 1;
				} //check collidables
 				}// loop over entries
			} // loop over lists
		} // loop over bodies
		
	}



	jiglib.CollisionSystemGrid = CollisionSystemGrid; 

})(jiglib);


(function(jiglib) {

	var CollDetectBoxPlane = jiglib.CollDetectBoxPlane;
	var CollDetectBoxMesh = jiglib.CollDetectBoxMesh;
	var CollDetectBoxBox = jiglib.CollDetectBoxBox;
	var CollDetectSphereTerrain = jiglib.CollDetectSphereTerrain;
	var CollDetectSphereBox = jiglib.CollDetectSphereBox;
	var CollDetectCapsuleTerrain = jiglib.CollDetectCapsuleTerrain;
	var CollDetectSphereCapsule = jiglib.CollDetectSphereCapsule;
	var CollDetectCapsuleBox = jiglib.CollDetectCapsuleBox;
	var CollDetectSphereMesh = jiglib.CollDetectSphereMesh;
	var CollDetectBoxTerrain = jiglib.CollDetectBoxTerrain;
	var CollDetectFunctor = jiglib.CollDetectFunctor;
	var CollisionSystemGrid = jiglib.CollisionSystemGrid;
	var CollDetectCapsuleCapsule = jiglib.CollDetectCapsuleCapsule;
	var CollPointInfo = jiglib.CollPointInfo;
	var CollisionInfo = jiglib.CollisionInfo;
	var CollDetectCapsulePlane = jiglib.CollDetectCapsulePlane;
	var CollDetectInfo = jiglib.CollDetectInfo;
	var CollDetectSphereSphere = jiglib.CollDetectSphereSphere;
	var CollisionSystemGridEntry = jiglib.CollisionSystemGridEntry;
	var CollDetectSpherePlane = jiglib.CollDetectSpherePlane;
	var CollisionSystemAbstract = jiglib.CollisionSystemAbstract;
	var RigidBody = jiglib.RigidBody;

	var CollisionSystemBrute = function()
	{

		jiglib.CollisionSystemAbstract.apply(this, [  ]);
		
	}

	jiglib.extend(CollisionSystemBrute, CollisionSystemAbstract);

	CollisionSystemBrute.prototype.detectAllCollisions = function(bodies, collArr)
	{

		var info;
		var fu;
		var bodyID;
		var bodyType;
		this._numCollisionsChecks = 0;
		for (var bodies_i = 0, bodies_l = bodies.length, _body; (bodies_i < bodies_l) && (_body = bodies[bodies_i]); bodies_i++)
		{
			if(!_body.isActive)continue;
			
			bodyID = _body.get_id();
			bodyType = _body.get_type();
			for (var collBody_i = 0, collBody_l = this.collBody.length, _collBody; (collBody_i < collBody_l) && (_collBody = this.collBody[collBody_i]); collBody_i++)
			{
				if (_body == _collBody)
				{
				continue;
				}
				
				if (_collBody.isActive && bodyID > _collBody.get_id())
				{
				continue;
				}
				
				if (this.checkCollidables(_body, _collBody) && this.detectionFunctors[bodyType + "_" + _collBody.get_type()] != undefined)
				{
				info = new CollDetectInfo();
				info.body0 = _body;
				info.body1 = _collBody;
				fu = this.detectionFunctors[info.body0.get_type() + "_" + info.body1.get_type()];
				fu.collDetect(info, collArr);
				this._numCollisionsChecks += 1;
				}
			}
		}
		
	}



	jiglib.CollisionSystemBrute = CollisionSystemBrute; 

})(jiglib);


(function(jiglib) {

	var CollDetectBoxPlane = jiglib.CollDetectBoxPlane;
	var CollDetectBoxMesh = jiglib.CollDetectBoxMesh;
	var CollDetectBoxBox = jiglib.CollDetectBoxBox;
	var CollDetectSphereTerrain = jiglib.CollDetectSphereTerrain;
	var CollDetectSphereBox = jiglib.CollDetectSphereBox;
	var CollDetectCapsuleTerrain = jiglib.CollDetectCapsuleTerrain;
	var CollDetectSphereCapsule = jiglib.CollDetectSphereCapsule;
	var CollisionSystemBrute = jiglib.CollisionSystemBrute;
	var CollDetectCapsuleBox = jiglib.CollDetectCapsuleBox;
	var CollDetectSphereMesh = jiglib.CollDetectSphereMesh;
	var CollDetectBoxTerrain = jiglib.CollDetectBoxTerrain;
	var CollisionSystemGrid = jiglib.CollisionSystemGrid;
	var CollDetectCapsuleCapsule = jiglib.CollDetectCapsuleCapsule;
	var CollPointInfo = jiglib.CollPointInfo;
	var CollisionInfo = jiglib.CollisionInfo;
	var CollisionSystemAbstract = jiglib.CollisionSystemAbstract;
	var CollDetectInfo = jiglib.CollDetectInfo;
	var CollDetectSphereSphere = jiglib.CollDetectSphereSphere;
	var CollisionSystemGridEntry = jiglib.CollisionSystemGridEntry;
	var CollDetectSpherePlane = jiglib.CollDetectSpherePlane;
	var CollDetectFunctor = jiglib.CollDetectFunctor;
	var Vector3D = jiglib.Vector3D;
	var JConfig = jiglib.JConfig;
	var JIndexedTriangle = jiglib.JIndexedTriangle;
	var JOctree = jiglib.JOctree;
	var JCapsule = jiglib.JCapsule;
	var JBox = jiglib.JBox;
	var JRay = jiglib.JRay;
	var JAABox = jiglib.JAABox;
	var JTerrain = jiglib.JTerrain;
	var JPlane = jiglib.JPlane;
	var JTriangleMesh = jiglib.JTriangleMesh;
	var JTriangle = jiglib.JTriangle;
	var JSphere = jiglib.JSphere;
	var JSegment = jiglib.JSegment;
	var JMatrix3D = jiglib.JMatrix3D;
	var Matrix3D = jiglib.Matrix3D;
	var JMath3D = jiglib.JMath3D;
	var JNumber3D = jiglib.JNumber3D;
	var MaterialProperties = jiglib.MaterialProperties;
	var RigidBody = jiglib.RigidBody;

	var CollDetectCapsulePlane = function()
	{

		this.name = "CapsulePlane";
		this.type0 = "CAPSULE";
		this.type1 = "PLANE";
		
	}

	jiglib.extend(CollDetectCapsulePlane, CollDetectFunctor);

	CollDetectCapsulePlane.prototype.collDetect = function(info, collArr)
	{

		var tempBody;
		if (info.body0.get_type() == "PLANE")
		{
			tempBody = info.body0;
			info.body0 = info.body1;
			info.body1 = tempBody;
		}

		var capsule = info.body0;
		var plane = info.body1;

		var collPts = [];
		var cpInfo;

		var oldPos = capsule.getBottomPos(capsule.get_oldState());
		var oldDist = plane.pointPlaneDistance(oldPos);
		var newPos = capsule.getBottomPos(capsule.get_currentState());
		var newDist = plane.pointPlaneDistance(newPos);

		if (Math.min(oldDist, newDist) < capsule.get_radius() + JConfig.collToll)
		{
			var oldDepth = capsule.get_radius() - oldDist;
			var worldPos = oldPos.subtract(JNumber3D.getScaleVector(plane.get_normal(), capsule.get_radius()));

			cpInfo = new CollPointInfo();
			cpInfo.r0 = worldPos.subtract(capsule.get_oldState().position);
			cpInfo.r1 = worldPos.subtract(plane.get_oldState().position);
			cpInfo.initialPenetration = oldDepth;
			collPts.push(cpInfo);
		}

		oldPos = capsule.getEndPos(capsule.get_oldState());
		newPos = capsule.getEndPos(capsule.get_currentState());
		oldDist = plane.pointPlaneDistance(oldPos);
		newDist = plane.pointPlaneDistance(newPos);
		if (Math.min(oldDist, newDist) < capsule.get_radius() + JConfig.collToll)
		{
			oldDepth = capsule.get_radius() - oldDist;
			worldPos = oldPos.subtract(JNumber3D.getScaleVector(plane.get_normal(), capsule.get_radius()));

			cpInfo = new CollPointInfo();
			cpInfo.r0 = worldPos.subtract(capsule.get_oldState().position);
			cpInfo.r1 = worldPos.subtract(plane.get_oldState().position);
			cpInfo.initialPenetration = oldDepth;
			collPts.push(cpInfo);
		}

		if (collPts.length > 0)
		{
			var collInfo = new CollisionInfo();
			collInfo.objInfo = info;
			collInfo.dirToBody = plane.get_normal().clone();
			collInfo.pointInfo = collPts;
			
			var mat = new MaterialProperties();
			mat.restitution = 0.5*(capsule.get_material().restitution + plane.get_material().restitution);
			mat.friction = 0.5*(capsule.get_material().friction + plane.get_material().friction);
			collInfo.mat = mat;
			collArr.push(collInfo);
			info.body0.collisions.push(collInfo);
			info.body1.collisions.push(collInfo);
			info.body0.addCollideBody(info.body1);
			info.body1.addCollideBody(info.body0);
		}else {
			info.body0.removeCollideBodies(info.body1);
			info.body1.removeCollideBodies(info.body0);
		}
		
	}



	jiglib.CollDetectCapsulePlane = CollDetectCapsulePlane; 

})(jiglib);


(function(jiglib) {

	var CollDetectBoxPlane = jiglib.CollDetectBoxPlane;
	var CollDetectBoxMesh = jiglib.CollDetectBoxMesh;
	var CollDetectBoxBox = jiglib.CollDetectBoxBox;
	var CollDetectSphereTerrain = jiglib.CollDetectSphereTerrain;
	var CollDetectSphereBox = jiglib.CollDetectSphereBox;
	var CollDetectCapsuleTerrain = jiglib.CollDetectCapsuleTerrain;
	var CollDetectSphereCapsule = jiglib.CollDetectSphereCapsule;
	var CollisionSystemBrute = jiglib.CollisionSystemBrute;
	var CollDetectCapsuleBox = jiglib.CollDetectCapsuleBox;
	var CollDetectSphereMesh = jiglib.CollDetectSphereMesh;
	var CollDetectBoxTerrain = jiglib.CollDetectBoxTerrain;
	var CollDetectFunctor = jiglib.CollDetectFunctor;
	var CollisionSystemGrid = jiglib.CollisionSystemGrid;
	var CollDetectCapsuleCapsule = jiglib.CollDetectCapsuleCapsule;
	var CollPointInfo = jiglib.CollPointInfo;
	var CollisionInfo = jiglib.CollisionInfo;
	var CollisionSystemAbstract = jiglib.CollisionSystemAbstract;
	var CollDetectCapsulePlane = jiglib.CollDetectCapsulePlane;
	var CollDetectSphereSphere = jiglib.CollDetectSphereSphere;
	var CollisionSystemGridEntry = jiglib.CollisionSystemGridEntry;
	var CollDetectSpherePlane = jiglib.CollDetectSpherePlane;
	var RigidBody = jiglib.RigidBody;

	var CollDetectInfo = function()
	{
		this.body0 = null; // RigidBody
		this.body1 = null; // RigidBody
	}



	jiglib.CollDetectInfo = CollDetectInfo; 

})(jiglib);


(function(jiglib) {

	var CollDetectBoxPlane = jiglib.CollDetectBoxPlane;
	var CollDetectBoxMesh = jiglib.CollDetectBoxMesh;
	var CollDetectBoxBox = jiglib.CollDetectBoxBox;
	var CollDetectSphereTerrain = jiglib.CollDetectSphereTerrain;
	var CollDetectSphereBox = jiglib.CollDetectSphereBox;
	var CollDetectCapsuleTerrain = jiglib.CollDetectCapsuleTerrain;
	var CollDetectSphereCapsule = jiglib.CollDetectSphereCapsule;
	var CollisionSystemBrute = jiglib.CollisionSystemBrute;
	var CollDetectCapsuleBox = jiglib.CollDetectCapsuleBox;
	var CollDetectSphereMesh = jiglib.CollDetectSphereMesh;
	var CollDetectBoxTerrain = jiglib.CollDetectBoxTerrain;
	var CollisionSystemGrid = jiglib.CollisionSystemGrid;
	var CollDetectCapsuleCapsule = jiglib.CollDetectCapsuleCapsule;
	var CollPointInfo = jiglib.CollPointInfo;
	var CollisionInfo = jiglib.CollisionInfo;
	var CollisionSystemAbstract = jiglib.CollisionSystemAbstract;
	var CollDetectCapsulePlane = jiglib.CollDetectCapsulePlane;
	var CollDetectInfo = jiglib.CollDetectInfo;
	var CollisionSystemGridEntry = jiglib.CollisionSystemGridEntry;
	var CollDetectSpherePlane = jiglib.CollDetectSpherePlane;
	var CollDetectFunctor = jiglib.CollDetectFunctor;
	var Matrix3D = jiglib.Matrix3D;
	var Vector3D = jiglib.Vector3D;
	var JConfig = jiglib.JConfig;
	var JIndexedTriangle = jiglib.JIndexedTriangle;
	var JOctree = jiglib.JOctree;
	var JCapsule = jiglib.JCapsule;
	var JBox = jiglib.JBox;
	var JRay = jiglib.JRay;
	var JAABox = jiglib.JAABox;
	var JTerrain = jiglib.JTerrain;
	var JPlane = jiglib.JPlane;
	var JTriangleMesh = jiglib.JTriangleMesh;
	var JTriangle = jiglib.JTriangle;
	var JSphere = jiglib.JSphere;
	var JSegment = jiglib.JSegment;
	var JMatrix3D = jiglib.JMatrix3D;
	var JMath3D = jiglib.JMath3D;
	var JNumber3D = jiglib.JNumber3D;
	var MaterialProperties = jiglib.MaterialProperties;

	var CollDetectSphereSphere = function()
	{

		this.name = "SphereSphere";
		this.type0 = "SPHERE";
		this.type1 = "SPHERE";
		
	}

	jiglib.extend(CollDetectSphereSphere, CollDetectFunctor);

	CollDetectSphereSphere.prototype.collDetect = function(info, collArr)
	{

		var sphere0 = info.body0;
		var sphere1 = info.body1;

		var oldDelta = sphere0.get_oldState().position.subtract(sphere1.get_oldState().position);
		var newDelta = sphere0.get_currentState().position.subtract(sphere1.get_currentState().position);

		var oldDistSq, newDistSq, radSum, oldDist, depth;
		oldDistSq = oldDelta.get_lengthSquared();
		newDistSq = newDelta.get_lengthSquared();
		radSum = sphere0.get_radius() + sphere1.get_radius();

		if (Math.min(oldDistSq, newDistSq) < Math.pow(radSum + JConfig.collToll, 2))
		{
			oldDist = Math.sqrt(oldDistSq);
			depth = radSum - oldDist;
			if (oldDist > JMath3D.NUM_TINY)
			{
				oldDelta = JNumber3D.getDivideVector(oldDelta, oldDist);
			}
			else
			{
				oldDelta = JMatrix3D.getRotationMatrix(0, 0, 1, 360 * Math.random()).transformVector(Vector3D.Y_AXIS);
			}
			
			var worldPos = sphere1.get_oldState().position.add(JNumber3D.getScaleVector(oldDelta, sphere1.get_radius() - 0.5 * depth));

			var collPts = [];
			var cpInfo = new CollPointInfo();
			cpInfo.r0 = worldPos.subtract(sphere0.get_oldState().position);
			cpInfo.r1 = worldPos.subtract(sphere1.get_oldState().position);
			cpInfo.initialPenetration = depth;
			collPts[0]=cpInfo;
			
			var collInfo = new CollisionInfo();
			collInfo.objInfo = info;
			collInfo.dirToBody = oldDelta;
			collInfo.pointInfo = collPts;
			
			var mat = new MaterialProperties();
			mat.restitution = 0.5*(sphere0.get_material().restitution + sphere1.get_material().restitution);
			mat.friction = 0.5*(sphere0.get_material().friction + sphere1.get_material().friction);
			collInfo.mat = mat;
			collArr.push(collInfo);
			info.body0.collisions.push(collInfo);
			info.body1.collisions.push(collInfo);
			info.body0.addCollideBody(info.body1);
			info.body1.addCollideBody(info.body0);
		}else {
			info.body0.removeCollideBodies(info.body1);
			info.body1.removeCollideBodies(info.body0);
		}
		
	}



	jiglib.CollDetectSphereSphere = CollDetectSphereSphere; 

})(jiglib);


(function(jiglib) {

	var CollDetectBoxPlane = jiglib.CollDetectBoxPlane;
	var CollDetectBoxMesh = jiglib.CollDetectBoxMesh;
	var CollDetectBoxBox = jiglib.CollDetectBoxBox;
	var CollDetectSphereTerrain = jiglib.CollDetectSphereTerrain;
	var CollDetectSphereBox = jiglib.CollDetectSphereBox;
	var CollDetectCapsuleTerrain = jiglib.CollDetectCapsuleTerrain;
	var CollDetectSphereCapsule = jiglib.CollDetectSphereCapsule;
	var CollisionSystemBrute = jiglib.CollisionSystemBrute;
	var CollDetectCapsuleBox = jiglib.CollDetectCapsuleBox;
	var CollDetectSphereMesh = jiglib.CollDetectSphereMesh;
	var CollDetectBoxTerrain = jiglib.CollDetectBoxTerrain;
	var CollDetectFunctor = jiglib.CollDetectFunctor;
	var CollisionSystemGrid = jiglib.CollisionSystemGrid;
	var CollDetectCapsuleCapsule = jiglib.CollDetectCapsuleCapsule;
	var CollPointInfo = jiglib.CollPointInfo;
	var CollisionInfo = jiglib.CollisionInfo;
	var CollisionSystemAbstract = jiglib.CollisionSystemAbstract;
	var CollDetectCapsulePlane = jiglib.CollDetectCapsulePlane;
	var CollDetectInfo = jiglib.CollDetectInfo;
	var CollDetectSphereSphere = jiglib.CollDetectSphereSphere;
	var CollDetectSpherePlane = jiglib.CollDetectSpherePlane;
	var RigidBody = jiglib.RigidBody;

	var CollisionSystemGridEntry = function(collisionBody)
	{
		this.collisionBody = null; // RigidBody
		this.previous = null; // CollisionSystemGridEntry
		this.next = null; // CollisionSystemGridEntry
		this.gridIndex = null; // int

		this.collisionBody = collisionBody;
		this.previous = this.next = null;
		
	}


	CollisionSystemGridEntry.removeGridEntry = function(entry)
	{

			// link the CollisionSystemGridEntry.previous to the CollisionSystemGridEntry.next (may be 0)
			entry.previous.next = entry.next;
			// link the CollisionSystemGridEntry.next (if it exists) to the CollisionSystemGridEntry.previous.
			if (entry.next != null)
				entry.next.previous = entry.previous;
			// tidy up this entry
			entry.previous = entry.next = null;
			entry.gridIndex = -2;
		
	}

	CollisionSystemGridEntry.insertGridEntryAfter = function(entry, prev)
	{

			var next = prev.next;
			prev.next = entry;
			entry.previous = prev;
			entry.next = next;
			if (next != null)
				next.previous = entry;
			entry.gridIndex = prev.gridIndex;
		
	}


	jiglib.CollisionSystemGridEntry = CollisionSystemGridEntry; 

})(jiglib);


(function(jiglib) {

	var CollDetectBoxPlane = jiglib.CollDetectBoxPlane;
	var CollDetectBoxMesh = jiglib.CollDetectBoxMesh;
	var CollDetectBoxBox = jiglib.CollDetectBoxBox;
	var CollDetectSphereTerrain = jiglib.CollDetectSphereTerrain;
	var CollDetectSphereBox = jiglib.CollDetectSphereBox;
	var CollDetectCapsuleTerrain = jiglib.CollDetectCapsuleTerrain;
	var CollDetectSphereCapsule = jiglib.CollDetectSphereCapsule;
	var CollisionSystemBrute = jiglib.CollisionSystemBrute;
	var CollDetectCapsuleBox = jiglib.CollDetectCapsuleBox;
	var CollDetectSphereMesh = jiglib.CollDetectSphereMesh;
	var CollDetectBoxTerrain = jiglib.CollDetectBoxTerrain;
	var CollisionSystemGrid = jiglib.CollisionSystemGrid;
	var CollDetectCapsuleCapsule = jiglib.CollDetectCapsuleCapsule;
	var CollPointInfo = jiglib.CollPointInfo;
	var CollisionInfo = jiglib.CollisionInfo;
	var CollisionSystemAbstract = jiglib.CollisionSystemAbstract;
	var CollDetectCapsulePlane = jiglib.CollDetectCapsulePlane;
	var CollDetectInfo = jiglib.CollDetectInfo;
	var CollDetectSphereSphere = jiglib.CollDetectSphereSphere;
	var CollisionSystemGridEntry = jiglib.CollisionSystemGridEntry;
	var CollDetectFunctor = jiglib.CollDetectFunctor;
	var Vector3D = jiglib.Vector3D;
	var JConfig = jiglib.JConfig;
	var JIndexedTriangle = jiglib.JIndexedTriangle;
	var JOctree = jiglib.JOctree;
	var JCapsule = jiglib.JCapsule;
	var JBox = jiglib.JBox;
	var JRay = jiglib.JRay;
	var JAABox = jiglib.JAABox;
	var JTerrain = jiglib.JTerrain;
	var JPlane = jiglib.JPlane;
	var JTriangleMesh = jiglib.JTriangleMesh;
	var JTriangle = jiglib.JTriangle;
	var JSphere = jiglib.JSphere;
	var JSegment = jiglib.JSegment;
	var JMatrix3D = jiglib.JMatrix3D;
	var Matrix3D = jiglib.Matrix3D;
	var JMath3D = jiglib.JMath3D;
	var JNumber3D = jiglib.JNumber3D;
	var MaterialProperties = jiglib.MaterialProperties;
	var RigidBody = jiglib.RigidBody;

	var CollDetectSpherePlane = function()
	{

		this.name = "SpherePlane";
		this.type0 = "SPHERE";
		this.type1 = "PLANE";
		
	}

	jiglib.extend(CollDetectSpherePlane, CollDetectFunctor);

	CollDetectSpherePlane.prototype.collDetect = function(info, collArr)
	{

		var tempBody;
		if (info.body0.get_type() == "PLANE")
		{
			tempBody = info.body0;
			info.body0 = info.body1;
			info.body1 = tempBody;
		}

		var sphere = info.body0;
		var plane = info.body1;

		var oldDist, newDist, depth;
		oldDist = plane.pointPlaneDistance(sphere.get_oldState().position);
		newDist = plane.pointPlaneDistance(sphere.get_currentState().position);

		if (Math.min(newDist, oldDist) > sphere.get_boundingSphere() + JConfig.collToll)
		{
			info.body0.removeCollideBodies(info.body1);
			info.body1.removeCollideBodies(info.body0);
			return;
		}

		var collPts = [];
		var cpInfo;
		depth = sphere.get_radius() - oldDist;

		var worldPos = sphere.get_oldState().position.subtract(JNumber3D.getScaleVector(plane.get_normal(), sphere.get_radius()));
		cpInfo = new CollPointInfo();
		cpInfo.r0 = worldPos.subtract(sphere.get_oldState().position);
		cpInfo.r1 = worldPos.subtract(plane.get_oldState().position);
		cpInfo.initialPenetration = depth;
		collPts[0]=cpInfo;
		
		var collInfo = new CollisionInfo();
		collInfo.objInfo = info;
		collInfo.dirToBody = plane.get_normal().clone();
		collInfo.pointInfo = collPts;
		
		var mat = new MaterialProperties();
		mat.restitution = 0.5*(sphere.get_material().restitution + plane.get_material().restitution);
		mat.friction = 0.5*(sphere.get_material().friction + plane.get_material().friction);
		collInfo.mat = mat;
		collArr.push(collInfo);
		info.body0.collisions.push(collInfo);
		info.body1.collisions.push(collInfo);
		info.body0.addCollideBody(info.body1);
		info.body1.addCollideBody(info.body0);
		
	}



	jiglib.CollDetectSpherePlane = CollDetectSpherePlane; 

})(jiglib);


(function(jiglib) {

	var JMath3D = jiglib.JMath3D;
	var JNumber3D = jiglib.JNumber3D;
	var Matrix3D = jiglib.Matrix3D;
	var Vector3D = jiglib.Vector3D;

	var JMatrix3D = function()
	{
	}


	JMatrix3D.getTranslationMatrix = function(x, y, z)
	{

			var matrix3D = new Matrix3D();
			matrix3D.appendTranslation(x, y, z);
			return matrix3D;
		
	}

	JMatrix3D.getScaleMatrix = function(x, y, z)
	{

			var matrix3D = new Matrix3D();
			matrix3D.prependScale(x, y, z);
			return matrix3D;
		
	}

	JMatrix3D.getRotationMatrix = function(x, y, z, degree, pivotPoint)
	{

			var matrix3D = new Matrix3D();
			matrix3D.appendRotation(degree, new Vector3D(x,y,z),pivotPoint);
			return matrix3D;
		
	}

	JMatrix3D.getInverseMatrix = function(m)
	{

			var matrix3D = m.clone();
			matrix3D.invert();
			return matrix3D;
		
	}

	JMatrix3D.getTransposeMatrix = function(m)
	{

			var matrix3D = m.clone();
			matrix3D.transpose();
			return matrix3D;
		
	}

	JMatrix3D.getAppendMatrix3D = function(a, b)
	{

			var matrix3D = a.clone();
			matrix3D.append(b);
			return matrix3D;
		
	}

	JMatrix3D.getPrependMatrix = function(a, b)
	{

			var matrix3D = a.clone();
			matrix3D.prepend(b);
			return matrix3D;
		
	}

	JMatrix3D.getSubMatrix = function(a, b)
	{

			var ar = a.get_rawData();
			var br = b.get_rawData();
			return new Matrix3D([[
				ar[0] - br[0],
				ar[1] - br[1],
				ar[2] - br[2],
				ar[3] - br[3],
				ar[4] - br[4],
				ar[5] - br[5],
				ar[6] - br[6],
				ar[7] - br[7],
				ar[8] - br[8],
				ar[9] - br[9],
				ar[10] - br[10],
				ar[11] - br[11],
				ar[12] - br[12],
				ar[13] - br[13],
				ar[14] - br[14],
				ar[15] - br[15]
			]]);
		
	}

	JMatrix3D.getRotationMatrixAxis = function(degree, rotateAxis)
	{

    		var matrix3D = new Matrix3D();
    		matrix3D.appendRotation(degree, rotateAxis?rotateAxis:Vector3D.X_AXIS);
    		return matrix3D;
		
	}

	JMatrix3D.getCols = function(matrix3D)
	{

			var rawData =  matrix3D.get_rawData();
			var cols = [];
			
			cols[0] = new Vector3D(rawData[0], rawData[4], rawData[8]);
			cols[1] = new Vector3D(rawData[1], rawData[5], rawData[9]);
			cols[2] = new Vector3D(rawData[2], rawData[6], rawData[10]);
			
			return cols;
		
	}

	JMatrix3D.multiplyVector = function(matrix3D, v)
	{

			v = matrix3D.transformVector(v);
			
			/*
			var vx = v.x;
			var vy = v.y;
			var vz = v.z;

			if (vx == 0 && vy == 0 && vz == 0) { return; }
			
			var _rawData =  matrix3D.get_rawData();
			
			v.x = vx * _rawData[0] + vy * _rawData[4] + vz * _rawData[8]  + _rawData[12];
			v.y = vx * _rawData[1] + vy * _rawData[5] + vz * _rawData[9]  + _rawData[13];
			v.z = vx * _rawData[2] + vy * _rawData[6] + vz * _rawData[10] + _rawData[14];
			*/
		
	}


	jiglib.JMatrix3D = JMatrix3D; 

})(jiglib);


(function(jiglib) {

	var JMatrix3D = jiglib.JMatrix3D;
	var Matrix3D = jiglib.Matrix3D;
	var JMath3D = jiglib.JMath3D;
	var JNumber3D = jiglib.JNumber3D;

	var Vector3D = function()
	{
	}

	Vector3D.prototype.get_length = function()
	{
 
	}

	Vector3D.prototype.get_lengthSquared = function()
	{
 
	}

	Vector3D.prototype.add = function()
	{
 
	}

	Vector3D.prototype.clone = function()
	{
 
	}

	Vector3D.prototype.crossProduct = function()
	{
 
	}

	Vector3D.prototype.subtract = function()
	{
 
	}



	jiglib.Vector3D = Vector3D; 

})(jiglib);


(function(jiglib) {

	var JMatrix3D = jiglib.JMatrix3D;
	var Vector3D = jiglib.Vector3D;
	var JMath3D = jiglib.JMath3D;
	var JNumber3D = jiglib.JNumber3D;

	var Matrix3D = function()
	{
	}

	Matrix3D.prototype.get_determinant = function()
	{
 
	}

	Matrix3D.prototype.get_position = function()
	{
 
	}

	Matrix3D.prototype.get_rawData = function()
	{
 
	}

	Matrix3D.prototype.set_rawData = function(rawData)
	{
 
	}

	Matrix3D.prototype.clone = function()
	{
 
	}

	Matrix3D.prototype.deltaTransformVector = function()
	{
 
	}

	Matrix3D.prototype.interpolate = function()
	{
 
	}

	Matrix3D.prototype.transformVector = function()
	{
 
	}



	jiglib.Matrix3D = Matrix3D; 

})(jiglib);


(function(jiglib) {

	var JMatrix3D = jiglib.JMatrix3D;
	var JNumber3D = jiglib.JNumber3D;
	var Matrix3D = jiglib.Matrix3D;
	var Vector3D = jiglib.Vector3D;

	var JMath3D = function()
	{
	}

	JMath3D.NUM_TINY =  0.000001; // Number
	JMath3D.NUM_HUGE =  1000000; // Number

	JMath3D.fromNormalAndPoint = function(normal, point)
	{

        	var v = new Vector3D(normal.x, normal.y, normal.z);
        	v.w = -(v.x*point.x + v.y*point.y + v.z*point.z);
        	
        	return v;
        
	}

	JMath3D.getIntersectionLine = function(v, v0, v1)
	{

			var d0 = v.x * v0.x + v.y * v0.y + v.z * v0.z - v.w;
			var d1 = v.x * v1.x + v.y * v1.y + v.z * v1.z - v.w;
			var m = d1 / (d1 - d0);
			return new Vector3D(
				v1.x + (v0.x - v1.x) * m,
				v1.y + (v0.y - v1.y) * m,
				v1.z + (v0.z - v1.z) * m);
		
	}

	JMath3D.wrap = function(val, min, max)
	{

			var delta = max - min;
			if (val > delta)
			{
				val = val / delta;
				val = val - Number(Math.floor(val));
				val = val * delta;
			}
			return val;
		
	}

	JMath3D.getLimiteNumber = function(num, min, max)
	{

			var n = num;
			if (n < min)
			{
				n = min;
			}
			else if (n > max)
			{
				n = max;
			}
			return n;
		
	}


	jiglib.JMath3D = JMath3D; 

})(jiglib);


(function(jiglib) {

	var JMatrix3D = jiglib.JMatrix3D;
	var Matrix3D = jiglib.Matrix3D;
	var JMath3D = jiglib.JMath3D;
	var Vector3D = jiglib.Vector3D;

	var JNumber3D = function()
	{
	}


	JNumber3D.toArray = function(v)
	{

			var arr=[];
			arr[0]=v.x;
			arr[1]=v.y;
			arr[2]=v.z;
			return arr;
		
	}

	JNumber3D.copyFromArray = function(v, arr)
	{

			if (arr.length >= 3)
			{
				v.x = arr[0];
				v.y = arr[1];
				v.z = arr[2];
			}
		
	}

	JNumber3D.getScaleVector = function(v, s)
	{

			return new Vector3D(v.x*s,v.y*s,v.z*s,v.w);
		
	}

	JNumber3D.getDivideVector = function(v, w)
	{

			if (w != 0)
			{
				return new Vector3D(v.x / w, v.y / w, v.z / w);
			}
			else
			{
				return new Vector3D(0, 0, 0);
			}
		
	}

	JNumber3D.getNormal = function(v0, v1, v2)
	{

			var E = v1.clone();
			var F = v2.clone();
			var N = E.subtract(v0).crossProduct(F.subtract(v1));
			N.normalize();

			return N;
		
	}


	jiglib.JNumber3D = JNumber3D; 

})(jiglib);


(function(jiglib) {

	var JOctree = jiglib.JOctree;
	var JCapsule = jiglib.JCapsule;
	var JBox = jiglib.JBox;
	var JRay = jiglib.JRay;
	var JAABox = jiglib.JAABox;
	var JTerrain = jiglib.JTerrain;
	var JPlane = jiglib.JPlane;
	var JTriangleMesh = jiglib.JTriangleMesh;
	var JTriangle = jiglib.JTriangle;
	var JSphere = jiglib.JSphere;
	var JSegment = jiglib.JSegment;
	var Vector3D = jiglib.Vector3D;
	var PlaneData = jiglib.PlaneData;
	var JNumber3D = jiglib.JNumber3D;

	var JIndexedTriangle = function()
	{
		this.counter = null; // int
		this._vertexIndices = null; // uint
		this._plane = null; // PlaneData
		this._boundingBox = null; // JAABox

		this.counter = 0;
		this._vertexIndices = [];
		this._vertexIndices[0] = -1;
		this._vertexIndices[1] = -1;
		this._vertexIndices[2] = -1;
		this._plane = new PlaneData();
		this._boundingBox = new JAABox();
		
	}

	JIndexedTriangle.prototype.setVertexIndices = function(i0, i1, i2, vertexArray)
	{

		this._vertexIndices[0] = i0;
		this._vertexIndices[1] = i1;
		this._vertexIndices[2] = i2;
		
		this._plane.setWithPoint(vertexArray[i0], vertexArray[i1], vertexArray[i2]);
		
		this._boundingBox.clear();
		this._boundingBox.addPoint(vertexArray[i0]);
		this._boundingBox.addPoint(vertexArray[i1]);
		this._boundingBox.addPoint(vertexArray[i2]);
		
	}

	JIndexedTriangle.prototype.updateVertexIndices = function(vertexArray)
	{

		var i0, i1, i2;
		i0=this._vertexIndices[0];
		i1=this._vertexIndices[1];
		i2=this._vertexIndices[2];
		
		this._plane.setWithPoint(vertexArray[i0], vertexArray[i1], vertexArray[i2]);
		
		this._boundingBox.clear();
		this._boundingBox.addPoint(vertexArray[i0]);
		this._boundingBox.addPoint(vertexArray[i1]);
		this._boundingBox.addPoint(vertexArray[i2]);
		
	}

	JIndexedTriangle.prototype.get_vertexIndices = function()
	{

		return this._vertexIndices;
		
	}

	JIndexedTriangle.prototype.getVertexIndex = function(iCorner)
	{

		return this._vertexIndices[iCorner];
		
	}

	JIndexedTriangle.prototype.get_plane = function()
	{

		return this._plane;
		
	}

	JIndexedTriangle.prototype.get_boundingBox = function()
	{

		return this._boundingBox;
		
	}



	jiglib.JIndexedTriangle = JIndexedTriangle; 

})(jiglib);


(function(jiglib) {

	var JIndexedTriangle = jiglib.JIndexedTriangle;
	var JCapsule = jiglib.JCapsule;
	var JBox = jiglib.JBox;
	var JRay = jiglib.JRay;
	var JAABox = jiglib.JAABox;
	var JTerrain = jiglib.JTerrain;
	var JPlane = jiglib.JPlane;
	var JTriangleMesh = jiglib.JTriangleMesh;
	var JTriangle = jiglib.JTriangle;
	var JSphere = jiglib.JSphere;
	var JSegment = jiglib.JSegment;
	var Vector3D = jiglib.Vector3D;
	var EdgeData = jiglib.EdgeData;
	var OctreeCell = jiglib.OctreeCell;
	var TriangleVertexIndices = jiglib.TriangleVertexIndices;
	var JNumber3D = jiglib.JNumber3D;
	var JMath3D = jiglib.JMath3D;

	var JOctree = function()
	{
		this._cells = null; // OctreeCell
		this._vertices = null; // Vector3D
		this._triangles = null; // JIndexedTriangle
		this._boundingBox = null; // JAABox
		this._cellsToTest = null; // int
		this._testCounter = null; // int

		this._testCounter = 0;
		this._cells = [];
		this._vertices = [];
		this._triangles = [];
		this._cellsToTest = [];
		this._boundingBox = new JAABox();
		
	}

	JOctree.prototype.get_trianglesData = function()
	{

		return this._triangles;
		
	}

	JOctree.prototype.getTriangle = function(iTriangle)
	{

		return this._triangles[iTriangle];
		
	}

	JOctree.prototype.get_verticesData = function()
	{

		return this._vertices;
		
	}

	JOctree.prototype.getVertex = function(iVertex)
	{

		return this._vertices[iVertex];
		
	}

	JOctree.prototype.boundingBox = function()
	{

		return this._boundingBox;
		
	}

	JOctree.prototype.clear = function()
	{

		this._cells.length=0;
		this._vertices.length=0;
		this._triangles.length=0;
		
	}

	JOctree.prototype.addTriangles = function(vertices, numVertices, triangleVertexIndices, numTriangles)
	{

		this.clear();
		
		this._vertices = vertices.concat();
		
		var NLen, tiny=JMath3D.NUM_TINY;
		var i0, i1, i2;
		var dr1, dr2, N;
		var indexedTriangle;
		for (var triangleVertexIndices_i = 0, triangleVertexIndices_l = triangleVertexIndices.length, tri; (triangleVertexIndices_i < triangleVertexIndices_l) && (tri = triangleVertexIndices[triangleVertexIndices_i]); triangleVertexIndices_i++) {
			i0 = tri.i0;
			i1 = tri.i1;
			i2 = tri.i2;
			
			dr1 = vertices[i1].subtract(vertices[i0]);
			dr2 = vertices[i2].subtract(vertices[i0]);
			N = dr1.crossProduct(dr2);
			NLen = N.get_length();
			
			if (NLen > tiny)
			{
				indexedTriangle = new JIndexedTriangle();
				indexedTriangle.setVertexIndices(i0, i1, i2, this._vertices);
				this._triangles.push(indexedTriangle);
			}
		}
		
	}

	JOctree.prototype.buildOctree = function(maxTrianglesPerCell, minCellSize)
	{

		this._boundingBox.clear();
		
		for (var _vertices_i = 0, _vertices_l = this._vertices.length, vt; (_vertices_i < _vertices_l) && (vt = this._vertices[_vertices_i]); _vertices_i++) {
			this._boundingBox.addPoint(vt);
		}
		
		this._cells.length=0;
		this._cells.push(new OctreeCell(this._boundingBox));
		
		var numTriangles = this._triangles.length;
		for (var i = 0; i < numTriangles; i++ ) {
			this._cells[0].triangleIndices[i] = i;
		}
		
		var cellsToProcess = [];
		cellsToProcess.push(0);
		
		var iTri;
		var cellIndex;
		var childCell;
		while (cellsToProcess.length != 0) {
			cellIndex = cellsToProcess.pop();
			
			if (this._cells[cellIndex].triangleIndices.length <= maxTrianglesPerCell || this._cells[cellIndex].AABox.getRadiusAboutCentre() < minCellSize) {
				continue;
			}
			for (i = 0; i < OctreeCell.NUM_CHILDREN; i++ ) {
				this._cells[cellIndex].childCellIndices[i] = this._cells.length;
				cellsToProcess.push(this._cells.length);
				this._cells.push(new OctreeCell(this.createAABox(this._cells[cellIndex].AABox, i)));
				
				childCell = this._cells[this._cells.length - 1];
				numTriangles = this._cells[cellIndex].triangleIndices.length;
				for (var j=0; j < numTriangles; j++ ) {
				iTri = this._cells[cellIndex].triangleIndices[j];
				if (this.doesTriangleIntersectCell(this._triangles[iTri], childCell))
				{
					childCell.triangleIndices.push(iTri);
				}
				}
			}
			this._cells[cellIndex].triangleIndices.length=0;
		}
		
	}

	JOctree.prototype.updateTriangles = function(vertices)
	{

		this._vertices = vertices.concat();
		
		for (var _triangles_i = 0, _triangles_l = this._triangles.length, triangle; (_triangles_i < _triangles_l) && (triangle = this._triangles[_triangles_i]); _triangles_i++){
			triangle.updateVertexIndices(this._vertices);
		}
		
	}

	JOctree.prototype.getTrianglesIntersectingtAABox = function(triangles, aabb)
	{

		if (this._cells.length == 0) return 0;
		
		this._cellsToTest.length=0;
		this._cellsToTest.push(0);
		
		this.incrementTestCounter();
		
		var cellIndex, nTris, cell, triangle;
		
		while (this._cellsToTest.length != 0) {
			cellIndex = this._cellsToTest.pop();
			
			cell = this._cells[cellIndex];
			
			if (!aabb.overlapTest(cell.AABox)) {
				continue;
			}
			
			if (cell.isLeaf()) {
				nTris = cell.triangleIndices.length;
				for (var i = 0 ; i < nTris ; i++) {
				triangle = this.getTriangle(cell.triangleIndices[i]);
				if (triangle.counter != this._testCounter) {
					triangle.counter = this._testCounter;
					if (aabb.overlapTest(triangle.get_boundingBox())) {
						triangles.push(cell.triangleIndices[i]);
					}
				}
				}
			}else {
				for (i = 0 ; i < OctreeCell.NUM_CHILDREN ; i++) {
				this._cellsToTest.push(cell.childCellIndices[i]);
				}
			}
		}
		return triangles.length;
		
	}

	JOctree.prototype.dumpStats = function()
	{

		var maxTris = 0, numTris, cellIndex, cell;
		
		var cellsToProcess = [];
		cellsToProcess.push(0);
		
		while (cellsToProcess.length != 0) {
			cellIndex = cellsToProcess.pop();
			
			cell = cell[cellIndex];
			if (cell.isLeaf()) {
				
				numTris = cell.triangleIndices.length;
				if (numTris > maxTris) {
				maxTris = numTris;
				}
			}else {
				for (var i = 0 ; i < OctreeCell.NUM_CHILDREN ; i++) {
				if ((cell.childCellIndices[i] >= 0) && (cell.childCellIndices[i] < this._cells.length)) {
					cellsToProcess.push(cell.childCellIndices[i]);
				}
				}
			}
		}
		
	}

	JOctree.prototype.createAABox = function(aabb, _id)
	{

		var dims = JNumber3D.getScaleVector(aabb.maxPos.subtract(aabb.minPos), 0.5);
		var offset;
		switch(_id) {
			case 0:
				offset = new Vector3D(1, 1, 1);
				break;
			case 1:
				offset = new Vector3D(1, 1, 0);
				break;
			case 2:
				offset = new Vector3D(1, 0, 1);
				break;
			case 3:
				offset = new Vector3D(1, 0, 0);
				break;
			case 4:
				offset = new Vector3D(0, 1, 1);
				break;
			case 5:
				offset = new Vector3D(0, 1, 0);
				break;
			case 6:
				offset = new Vector3D(0, 0, 1);
				break;
			case 7:
				offset = new Vector3D(0, 0, 0);
				break;
			default:
				offset = new Vector3D(0, 0, 0);
				break;
		}
		
		var result = new JAABox();
		result.minPos = aabb.minPos.add(new Vector3D(offset.x * dims.x, offset.y * dims.y, offset.z * dims.z));
		result.maxPos = result.minPos.add(dims);
		
		dims.scaleBy(0.00001);
		result.minPos = result.minPos.subtract(dims);
		result.maxPos = result.maxPos.add(dims);
		
		return result;
		
	}

	JOctree.prototype.doesTriangleIntersectCell = function(triangle, cell)
	{

		if (!triangle.get_boundingBox().overlapTest(cell.AABox)) {
			return false;
		}
		if (cell.AABox.isPointInside(this.getVertex(triangle.getVertexIndex(0))) ||
		    cell.AABox.isPointInside(this.getVertex(triangle.getVertexIndex(1))) ||
		    cell.AABox.isPointInside(this.getVertex(triangle.getVertexIndex(2)))) {
				return true;
			}
			
		var tri = new JTriangle(this.getVertex(triangle.getVertexIndex(0)), this.getVertex(triangle.getVertexIndex(1)), this.getVertex(triangle.getVertexIndex(2)));
		var edge;
		var seg;
		var edges = cell.get_egdes();
		var pts = cell.get_points();
		for (var i = 0; i < 12; i++ ) {
			edge = edges[i];
			seg = new JSegment(pts[edge.ind0], pts[edge.ind1].subtract(pts[edge.ind0]));
			if (tri.segmentTriangleIntersection(null, seg)) {
				return true;
			}
		}
		
		var pt0;
		var pt1;
		for (i = 0; i < 3; i++ ) {
			pt0 = tri.getVertex(i);
			pt1 = tri.getVertex((i + 1) % 3);
			if (cell.AABox.segmentAABoxOverlap(new JSegment(pt0, pt1.subtract(pt0)))) {
				return true;
			}
		}
		return false;
		
	}

	JOctree.prototype.incrementTestCounter = function()
	{

		++this._testCounter;
		if (this._testCounter == 0) {
			var numTriangles = this._triangles.length;
			for (var i = 0; i < numTriangles; i++) {
				this._triangles[i].counter = 0;
			}
			this._testCounter = 1;
		}
		
	}



	jiglib.JOctree = JOctree; 

})(jiglib);


(function(jiglib) {

	var JIndexedTriangle = jiglib.JIndexedTriangle;
	var JOctree = jiglib.JOctree;
	var JCapsule = jiglib.JCapsule;
	var JBox = jiglib.JBox;
	var JAABox = jiglib.JAABox;
	var JTerrain = jiglib.JTerrain;
	var JPlane = jiglib.JPlane;
	var JTriangleMesh = jiglib.JTriangleMesh;
	var JTriangle = jiglib.JTriangle;
	var JSphere = jiglib.JSphere;
	var JSegment = jiglib.JSegment;
	var Vector3D = jiglib.Vector3D;
	var JMatrix3D = jiglib.JMatrix3D;
	var Matrix3D = jiglib.Matrix3D;
	var JMath3D = jiglib.JMath3D;
	var JNumber3D = jiglib.JNumber3D;

	var JRay = function(_origin, _dir)
	{
		this.origin = null; // Vector3D
		this.dir = null; // Vector3D

		this.origin = _origin;
		this.dir = _dir;
		
	}

	JRay.prototype.getOrigin = function(t)
	{

		return this.origin.add(JNumber3D.getScaleVector(this.dir, t));
		
	}



	jiglib.JRay = JRay; 

})(jiglib);


(function(jiglib) {

	var JIndexedTriangle = jiglib.JIndexedTriangle;
	var JOctree = jiglib.JOctree;
	var JCapsule = jiglib.JCapsule;
	var JBox = jiglib.JBox;
	var JRay = jiglib.JRay;
	var JTerrain = jiglib.JTerrain;
	var JPlane = jiglib.JPlane;
	var JTriangleMesh = jiglib.JTriangleMesh;
	var JTriangle = jiglib.JTriangle;
	var JSphere = jiglib.JSphere;
	var JSegment = jiglib.JSegment;
	var Vector3D = jiglib.Vector3D;
	var EdgeData = jiglib.EdgeData;
	var JNumber3D = jiglib.JNumber3D;
	var JMath3D = jiglib.JMath3D;

	var JAABox = function()
	{
		this.minPos = null; // Vector3D
		this.maxPos = null; // Vector3D

		this.clear();
		
	}

	JAABox.prototype.get_sideLengths = function()
	{

		var pos = this.maxPos.clone();
		pos = pos.subtract( this.minPos ); 
		return pos;
		
	}

	JAABox.prototype.get_centrePos = function()
	{

		var pos = this.minPos.clone();
		return JNumber3D.getScaleVector(pos.add(this.maxPos), 0.5);
		
	}

	JAABox.prototype.getAllPoints = function()
	{

		var center, halfSide;
		var points;
		center = this.get_centrePos();
		halfSide = JNumber3D.getScaleVector(this.get_sideLengths(), 0.5);
		points = [];
		points[0] = center.add(new Vector3D(halfSide.x, -halfSide.y, halfSide.z));
		points[1] = center.add(new Vector3D(halfSide.x, halfSide.y, halfSide.z));
		points[2] = center.add(new Vector3D(-halfSide.x, -halfSide.y, halfSide.z));
		points[3] = center.add(new Vector3D(-halfSide.x, halfSide.y, halfSide.z));
		points[4] = center.add(new Vector3D(-halfSide.x, -halfSide.y, -halfSide.z));
		points[5] = center.add(new Vector3D(-halfSide.x, halfSide.y, -halfSide.z));
		points[6] = center.add(new Vector3D(halfSide.x, -halfSide.y, -halfSide.z));
		points[7] = center.add(new Vector3D(halfSide.x, halfSide.y, -halfSide.z));
		
		return points;
		
	}

	JAABox.prototype.get_edges = function()
	{

		return [
		new EdgeData( 0, 1 ), new EdgeData( 0, 2 ), new EdgeData( 0, 6 ),
		new EdgeData( 2, 3 ), new EdgeData( 2, 4 ), new EdgeData( 6, 7 ),
		new EdgeData( 6, 4 ), new EdgeData( 1, 3 ), new EdgeData( 1, 7 ),
		new EdgeData( 3, 5 ), new EdgeData( 7, 5 ), new EdgeData( 4, 5 )];
		
	}

	JAABox.prototype.getRadiusAboutCentre = function()
	{

		return 0.5 * (this.maxPos.subtract(this.minPos).get_length());
		
	}

	JAABox.prototype.move = function(delta)
	{

		this.minPos.add(delta);
		this.maxPos.add(delta);
		
	}

	JAABox.prototype.clear = function()
	{

		var huge=JMath3D.NUM_HUGE;
		this.minPos = new Vector3D(huge, huge, huge);
		this.maxPos = new Vector3D( -huge, -huge, -huge);
		
	}

	JAABox.prototype.clone = function()
	{

		var aabb = new JAABox();
		aabb.minPos = this.minPos.clone();
		aabb.maxPos = this.maxPos.clone();
		return aabb;
		
	}

	JAABox.prototype.addPoint = function(pos)
	{

		var tiny=JMath3D.NUM_TINY;
		if (pos.x < this.minPos.x) this.minPos.x = pos.x - tiny;
		if (pos.x > this.maxPos.x) this.maxPos.x = pos.x + tiny;
		if (pos.y < this.minPos.y) this.minPos.y = pos.y - tiny;
		if (pos.y > this.maxPos.y) this.maxPos.y = pos.y + tiny;
		if (pos.z < this.minPos.z) this.minPos.z = pos.z - tiny;
		if (pos.z > this.maxPos.z) this.maxPos.z = pos.z + tiny;
		
	}

	JAABox.prototype.addBox = function(box)
	{

		var pts = box.getCornerPoints(box.get_currentState());
		this.addPoint(pts[0]);
		this.addPoint(pts[1]);
		this.addPoint(pts[2]);
		this.addPoint(pts[3]);
		this.addPoint(pts[4]);
		this.addPoint(pts[5]);
		this.addPoint(pts[6]);
		this.addPoint(pts[7]);
		
	}

	JAABox.prototype.addSphere = function(sphere)
	{

		//if (sphere.get_currentState().position.x - sphere.get_radius() < _minPos.x) {
			this.minPos.x = (sphere.get_currentState().position.x - sphere.get_radius()) - 1;
		//}
		//if (sphere.get_currentState().position.x + sphere.get_radius() > _maxPos.x) {
			this.maxPos.x = (sphere.get_currentState().position.x + sphere.get_radius()) + 1;
		//}
		
		//if (sphere.get_currentState().position.y - sphere.get_radius() < _minPos.y) {
			this.minPos.y = (sphere.get_currentState().position.y - sphere.get_radius()) - 1;
		//}
		//if (sphere.get_currentState().position.y + sphere.get_radius() > _maxPos.y) {
			this.maxPos.y = (sphere.get_currentState().position.y + sphere.get_radius()) + 1;
		//}
		
		//if (sphere.get_currentState().position.z - sphere.get_radius() < _minPos.z) {
			this.minPos.z = (sphere.get_currentState().position.z - sphere.get_radius()) - 1;
		//}
		//if (sphere.get_currentState().position.z + sphere.get_radius() > _maxPos.z) {
			this.maxPos.z = (sphere.get_currentState().position.z + sphere.get_radius()) + 1;
		//}
		//trace("jaabox - add sphere:", _minPos.x,_minPos.y,_minPos.z,_maxPos.x,_maxPos.y,_maxPos.z);
		
		// todo: remove this code
			/*
		if (_minPos.x > _maxPos.x) {
			trace("minpos x ouch");
		}
		if (_minPos.y > _maxPos.y) {
			trace("minpos y ouch");
		}
		if (_minPos.z > _maxPos.z) {
			trace("minpos z ouch");
		}
		*/

		
	}

	JAABox.prototype.addCapsule = function(capsule)
	{

		var pos = capsule.getBottomPos(capsule.get_currentState());
		if (pos.x - capsule.get_radius() < this.minPos.x) {
			this.minPos.x = (pos.x - capsule.get_radius()) - 1;
		}
		if (pos.x + capsule.get_radius() > this.maxPos.x) {
			this.maxPos.x = (pos.x + capsule.get_radius()) + 1;
		}
		
		if (pos.y - capsule.get_radius() < this.minPos.y) {
			this.minPos.y = (pos.y - capsule.get_radius()) - 1;
		}
		if (pos.y + capsule.get_radius() > this.maxPos.y) {
			this.maxPos.y = (pos.y + capsule.get_radius()) + 1;
		}
		
		if (pos.z - capsule.get_radius() < this.minPos.z) {
			this.minPos.z = (pos.z - capsule.get_radius()) - 1;
		}
		if (pos.z + capsule.get_radius() > this.maxPos.z) {
			this.maxPos.z = (pos.z + capsule.get_radius()) + 1;
		}
		
		pos = capsule.getEndPos(capsule.get_currentState());
		if (pos.x - capsule.get_radius() < this.minPos.x) {
			this.minPos.x = (pos.x - capsule.get_radius()) - 1;
		}
		if (pos.x + capsule.get_radius() > this.maxPos.x) {
			this.maxPos.x = (pos.x + capsule.get_radius()) + 1;
		}
		
		if (pos.y - capsule.get_radius() < this.minPos.y) {
			this.minPos.y = (pos.y - capsule.get_radius()) - 1;
		}
		if (pos.y + capsule.get_radius() > this.maxPos.y) {
			this.maxPos.y = (pos.y + capsule.get_radius()) + 1;
		}
		
		if (pos.z - capsule.get_radius() < this.minPos.z) {
			this.minPos.z = (pos.z - capsule.get_radius()) - 1;
		}
		if (pos.z + capsule.get_radius() > this.maxPos.z) {
			this.maxPos.z = (pos.z + capsule.get_radius()) + 1;
		}
		
	}

	JAABox.prototype.addSegment = function(seg)
	{

		this.addPoint(seg.origin);
		this.addPoint(seg.getEnd());
		
	}

	JAABox.prototype.overlapTest = function(box)
	{

		return (
			(this.minPos.z >= box.maxPos.z) ||
			(this.maxPos.z <= box.minPos.z) ||
			(this.minPos.y >= box.maxPos.y) ||
			(this.maxPos.y <= box.minPos.y) ||
			(this.minPos.x >= box.maxPos.x) ||
			(this.maxPos.x <= box.minPos.x) ) ? false : true;
		
	}

	JAABox.prototype.isPointInside = function(pos)
	{

		return ((pos.x >= this.minPos.x) && 
			    (pos.x <= this.maxPos.x) && 
			    (pos.y >= this.minPos.y) && 
			    (pos.y <= this.maxPos.y) && 
			    (pos.z >= this.minPos.z) && 
			    (pos.z <= this.maxPos.z));
		
	}

	JAABox.prototype.segmentAABoxOverlap = function(seg)
	{

		var jDir, kDir, i, iFace;
		var frac, dist0, dist1, tiny=JMath3D.NUM_TINY;
		
		var pt, minPosArr, maxPosArr, p0, p1, faceOffsets;
		minPosArr = JNumber3D.toArray(this.minPos);
		maxPosArr = JNumber3D.toArray(this.maxPos);
		p0 = JNumber3D.toArray(seg.origin);
		p1 = JNumber3D.toArray(seg.getEnd());
		for (i = 0; i < 3; i++ ) {
			jDir = (i + 1) % 3;
			kDir = (i + 2) % 3;
			faceOffsets = [[minPosArr[i], maxPosArr[i]]];
			
			for (iFace = 0 ; iFace < 2 ; iFace++) {
				dist0 = p0[i] - faceOffsets[iFace];
			    dist1 = p1[i] - faceOffsets[iFace];
			    frac = -1;
				if (dist0 * dist1 < -tiny)
				frac = -dist0 / (dist1 - dist0);
			    else if (Math.abs(dist0) < tiny)
				frac = 0;
			    else if (Math.abs(dist1) < tiny)
				frac = 1;
				
				if (frac >= 0) {
				pt = JNumber3D.toArray(seg.getPoint(frac));
				if((pt[jDir] > minPosArr[jDir] - tiny) && 
				(pt[jDir] < maxPosArr[jDir] + tiny) && 
				(pt[kDir] > minPosArr[kDir] - tiny) && 
				(pt[kDir] < maxPosArr[kDir] + tiny)) {
					return true;
				}
				}
			}
		}
		return false;
		
	}



	jiglib.JAABox = JAABox; 

})(jiglib);


(function(jiglib) {

	var JIndexedTriangle = jiglib.JIndexedTriangle;
	var JOctree = jiglib.JOctree;
	var JCapsule = jiglib.JCapsule;
	var JBox = jiglib.JBox;
	var JRay = jiglib.JRay;
	var JAABox = jiglib.JAABox;
	var JTerrain = jiglib.JTerrain;
	var JPlane = jiglib.JPlane;
	var JTriangleMesh = jiglib.JTriangleMesh;
	var JSphere = jiglib.JSphere;
	var JSegment = jiglib.JSegment;
	var Vector3D = jiglib.Vector3D;
	var CollOutData = jiglib.CollOutData;
	var PlaneData = jiglib.PlaneData;
	var SpanData = jiglib.SpanData;
	var JNumber3D = jiglib.JNumber3D;
	var JMath3D = jiglib.JMath3D;

	var JTriangle = function(pt0, pt1, pt2)
	{
		this.origin = null; // Vector3D
		this.edge0 = null; // Vector3D
		this.edge1 = null; // Vector3D

		this.origin = pt0.clone();
		this.edge0 = pt1.subtract(pt0);
		this.edge1 = pt2.subtract(pt0);
		
	}

	JTriangle.prototype.get_edge2 = function()
	{

		return this.edge1.subtract(this.edge0);
		
	}

	JTriangle.prototype.get_normal = function()
	{

		var N = this.edge0.crossProduct(this.edge1);
		N.normalize();
		
		return N;
		
	}

	JTriangle.prototype.get_plane = function()
	{

		var pl = new PlaneData();
		pl.setWithNormal(this.origin, this.get_normal());
		
		return pl;
		
	}

	JTriangle.prototype.getPoint = function(t0, t1)
	{

		var d0, d1;
		d0 = this.edge0.clone();
		d1 = this.edge1.clone();
		
		d0.scaleBy(t0);
		d1.scaleBy(t1);
		
		return this.origin.add(d0).add(d1);
		
	}

	JTriangle.prototype.getCentre = function()
	{

		var result = this.edge0.add(this.edge1);
		result.scaleBy(0.333333);
		
		return this.origin.add(result);
		
	}

	JTriangle.prototype.getVertex = function(_id)
	{

		switch(_id) {
			case 1: 
				return this.origin.add(this.edge0);
			case 2:
				return this.origin.add(this.edge1);
			default:
				return this.origin;
		}
		
	}

	JTriangle.prototype.getSpan = function(axis)
	{

		var d0, d1, d2;
		d0 = this.getVertex(0).dotProduct(axis);
		d1 = this.getVertex(1).dotProduct(axis);
		d2 = this.getVertex(2).dotProduct(axis);
		
		var result = new SpanData();
		result.min = Math.min(d0, d1, d2);
		result.max = Math.max(d0, d1, d2);
		
		return result;
		
	}

	JTriangle.prototype.segmentTriangleIntersection = function(out, seg)
	{

		
		var u, v, t, a, f;
		var p, s, q;
		
		p = seg.delta.crossProduct(this.edge1);
		a = this.edge0.dotProduct(p);
		
		if (a > -JMath3D.NUM_TINY && a < JMath3D.NUM_TINY) {
			return false;
		}
		f = 1 / a;
		s = seg.origin.subtract(this.origin);
		u = f * s.dotProduct(p);
		
		if (u < 0 || u > 1) return false;
		
		q = s.crossProduct(this.edge0);
		v = f * seg.delta.dotProduct(q);
		if (v < 0 || (u + v) > 1) return false;
		
		t = f * this.edge1.dotProduct(q);
		if (t < 0 || t > 1) return false;
		
		if (out) out.frac = t;
		return true;
		
	}

	JTriangle.prototype.pointTriangleDistanceSq = function(out, point)
	{

		
		var fA00, fA01, fA11, fB0, fB1, fC, fDet, fS, fT, fSqrDist;
		
		var kDiff = this.origin.subtract(point);
		    fA00 = this.edge0.get_lengthSquared();
		    fA01 = this.edge0.dotProduct(this.edge1);
		    fA11 = this.edge1.get_lengthSquared();
		    fB0 = kDiff.dotProduct(this.edge0);
		    fB1 = kDiff.dotProduct(this.edge1);
		    fC = kDiff.get_lengthSquared();
		    fDet = Math.abs(fA00 * fA11 - fA01 * fA01);
		    fS = fA01 * fB1 - fA11 * fB0;
		    fT = fA01 * fB0 - fA00 * fB1;
		
		  if ( fS + fT <= fDet )
		  {
		if ( fS < 0 )
		{
		  if ( fT < 0 )  // region 4
		  {
			if ( fB0 < 0 )
			{
			  fT = 0;
			  if ( -fB0 >= fA00 )
			  {
				fS = 1;
				fSqrDist = fA00+2*fB0+fC;
			  }
			  else
			  {
				fS = -fB0/fA00;
				fSqrDist = fB0*fS+fC;
			  }
			}
			else
			{
			  fS = 0;
			  if ( fB1 >= 0 )
			  {
				fT = 0;
				fSqrDist = fC;
			  }
			  else if ( -fB1 >= fA11 )
			  {
				fT = 1;
				fSqrDist = fA11+2*fB1+fC;
			  }
			  else
			  {
				fT = -fB1/fA11;
				fSqrDist = fB1*fT+fC;
			  }
			}
		  }
		  else  // region 3
		  {
			fS = 0;
			if ( fB1 >= 0 )
			{
			  fT = 0;
			  fSqrDist = fC;
			}
			else if ( -fB1 >= fA11 )
			{
			  fT = 1;
			  fSqrDist = fA11+2*fB1+fC;
			}
			else
			{
			  fT = -fB1/fA11;
			  fSqrDist = fB1*fT+fC;
			}
		  }
		}
		else if ( fT < 0 )  // region 5
		{
		  fT = 0;
		  if ( fB0 >= 0 )
		  {
			fS = 0;
			fSqrDist = fC;
		  }
		  else if ( -fB0 >= fA00 )
		  {
			fS = 1;
			fSqrDist = fA00+2*fB0+fC;
		  }
		  else
		  {
			fS = -fB0/fA00;
			fSqrDist = fB0*fS+fC;
		  }
		}
		else  // region 0
		{
		  // minimum at interior point
		  var fInvDet = 1/fDet;
		  fS *= fInvDet;
		  fT *= fInvDet;
		  fSqrDist = fS * (fA00 * fS + fA01 * fT + 2 * fB0) +fT * (fA01 * fS + fA11 * fT + 2 * fB1) + fC;
		}
		  }
		  else
		  {
		var fTmp0, fTmp1, fNumer, fDenom;

		if ( fS < 0 )  // region 2
		{
		  fTmp0 = fA01 + fB0;
		  fTmp1 = fA11 + fB1;
		  if ( fTmp1 > fTmp0 )
		  {
			fNumer = fTmp1 - fTmp0;
			fDenom = fA00-2*fA01+fA11;
			if ( fNumer >= fDenom )
			{
			  fS = 1;
			  fT = 0;
			  fSqrDist = fA00+2*fB0+fC;
			}
			else
			{
			  fS = fNumer/fDenom;
			  fT = 1 - fS;
			  fSqrDist = fS * (fA00 * fS + fA01 * fT + 2 * fB0) +fT * (fA01 * fS + fA11 * fT + 2 * fB1) + fC;
			}
		  }
		  else
		  {
			fS = 0;
			if ( fTmp1 <= 0 )
			{
			  fT = 1;
			  fSqrDist = fA11+2*fB1+fC;
			}
			else if ( fB1 >= 0 )
			{
			  fT = 0;
			  fSqrDist = fC;
			}
			else
			{
			  fT = -fB1/fA11;
			  fSqrDist = fB1*fT+fC;
			}
		  }
		}
		else if ( fT < 0 )  // region 6
		{
		  fTmp0 = fA01 + fB1;
		  fTmp1 = fA00 + fB0;
		  if ( fTmp1 > fTmp0 )
		  {
			fNumer = fTmp1 - fTmp0;
			fDenom = fA00-2*fA01+fA11;
			if ( fNumer >= fDenom )
			{
			  fT = 1;
			  fS = 0;
			  fSqrDist = fA11+2*fB1+fC;
			}
			else
			{
			  fT = fNumer/fDenom;
			  fS = 1 - fT;
			  fSqrDist = fS * (fA00 * fS + fA01 * fT + 2 * fB0) +fT * (fA01 * fS + fA11 * fT + 2 * fB1) + fC;
			}
		  }
		  else
		  {
			fT = 0;
			if ( fTmp1 <= 0 )
			{
			  fS = 1;
			  fSqrDist = fA00+2*fB0+fC;
			}
			else if ( fB0 >= 0 )
			{
			  fS = 0;
			  fSqrDist = fC;
			}
			else
			{
			  fS = -fB0/fA00;
			  fSqrDist = fB0*fS+fC;
			}
		  }
		}
		else  // region 1
		{
		  fNumer = fA11 + fB1 - fA01 - fB0;
		  if ( fNumer <= 0 )
		  {
			fS = 0;
			fT = 1;
			fSqrDist = fA11+2*fB1+fC;
		  }
		  else
		  {
			fDenom = fA00-2*fA01+fA11;
			if ( fNumer >= fDenom )
			{
			  fS = 1;
			  fT = 0;
			  fSqrDist = fA00 + 2 * fB0 + fC;
			}
			else
			{
			  fS = fNumer/fDenom;
			  fT = 1 - fS;
			  fSqrDist = fS * (fA00 * fS + fA01 * fT + 2 * fB0) +fT * (fA01 * fS + fA11 * fT + 2 * fB1) + fC;
			}
		  }
		}
		  }
		  out[0] = fS;
		  out[1] = fT;
		 
		  return Math.abs(fSqrDist);
		
	}



	jiglib.JTriangle = JTriangle; 

})(jiglib);


(function(jiglib) {

	var JIndexedTriangle = jiglib.JIndexedTriangle;
	var JOctree = jiglib.JOctree;
	var JCapsule = jiglib.JCapsule;
	var JBox = jiglib.JBox;
	var JRay = jiglib.JRay;
	var JAABox = jiglib.JAABox;
	var JTerrain = jiglib.JTerrain;
	var JPlane = jiglib.JPlane;
	var JTriangleMesh = jiglib.JTriangleMesh;
	var JTriangle = jiglib.JTriangle;
	var JSphere = jiglib.JSphere;
	var Vector3D = jiglib.Vector3D;
	var CollOutData = jiglib.CollOutData;
	var JNumber3D = jiglib.JNumber3D;
	var JMath3D = jiglib.JMath3D;
	var PhysicsState = jiglib.PhysicsState;

	var JSegment = function(_origin, _delta)
	{
		this.origin = null; // Vector3D
		this.delta = null; // Vector3D

		this.origin = _origin;
		this.delta = _delta;
		
	}

	JSegment.prototype.getPoint = function(t)
	{

		return this.origin.add(JNumber3D.getScaleVector(this.delta, t));
		
	}

	JSegment.prototype.getEnd = function()
	{

		return this.origin.add(this.delta);
		
	}

	JSegment.prototype.clone = function()
	{

		return new JSegment(this.origin, this.delta);
		
	}

	JSegment.prototype.segmentSegmentDistanceSq = function(out, seg)
	{

		var fA00, fA01, fA11, fB0, fC, fDet, fB1, fS, fT, fSqrDist, fTmp, fInvDet;
		
		var kDiff = this.origin.subtract(seg.origin);
		fA00 = this.delta.get_lengthSquared();
		fA01 = -this.delta.dotProduct(seg.delta);
		fA11 = seg.delta.get_lengthSquared();
		fB0 = kDiff.dotProduct(this.delta);
		fC = kDiff.get_lengthSquared();
		fDet = Math.abs(fA00 * fA11 - fA01 * fA01);

		if (fDet >= JMath3D.NUM_TINY)
		{
			fB1 = -kDiff.dotProduct(seg.delta);
			fS = fA01 * fB1 - fA11 * fB0;
			fT = fA01 * fB0 - fA00 * fB1;

			if (fS >= 0)
			{
				if (fS <= fDet)
				{
				if (fT >= 0)
				{
					if (fT <= fDet)
					{
						fInvDet = 1 / fDet;
						fS *= fInvDet;
						fT *= fInvDet;
						fSqrDist = fS * (fA00 * fS + fA01 * fT + 2 * fB0) + fT * (fA01 * fS + fA11 * fT + 2 * fB1) + fC;
					}
					else
					{
						fT = 1;
						fTmp = fA01 + fB0;
						if (fTmp >= 0)
						{
						fS = 0;
						fSqrDist = fA11 + 2 * fB1 + fC;
						}
						else if (-fTmp >= fA00)
						{
						fS = 1;
						fSqrDist = fA00 + fA11 + fC + 2 * (fB1 + fTmp);
						}
						else
						{
						fS = -fTmp / fA00;
						fSqrDist = fTmp * fS + fA11 + 2 * fB1 + fC;
						}
					}
				}
				else
				{
					fT = 0;
					if (fB0 >= 0)
					{
						fS = 0;
						fSqrDist = fC;
					}
					else if (-fB0 >= fA00)
					{
						fS = 1;
						fSqrDist = fA00 + 2 * fB0 + fC;
					}
					else
					{
						fS = -fB0 / fA00;
						fSqrDist = fB0 * fS + fC;
					}
				}
				}
				else
				{
				if (fT >= 0)
				{
					if (fT <= fDet)
					{
						fS = 1;
						fTmp = fA01 + fB1;
						if (fTmp >= 0)
						{
						fT = 0;
						fSqrDist = fA00 + 2 * fB0 + fC;
						}
						else if (-fTmp >= fA11)
						{
						fT = 1;
						fSqrDist = fA00 + fA11 + fC + 2 * (fB0 + fTmp);
						}
						else
						{
						fT = -fTmp / fA11;
						fSqrDist = fTmp * fT + fA00 + 2 * fB0 + fC;
						}
					}
					else
					{
						fTmp = fA01 + fB0;
						if (-fTmp <= fA00)
						{
						fT = 1;
						if (fTmp >= 0)
						{
							fS = 0;
							fSqrDist = fA11 + 2 * fB1 + fC;
						}
						else
						{
							fS = -fTmp / fA00;
							fSqrDist = fTmp * fS + fA11 + 2 * fB1 + fC;
						}
						}
						else
						{
						fS = 1;
						fTmp = fA01 + fB1;
						if (fTmp >= 0)
						{
							fT = 0;
							fSqrDist = fA00 + 2 * fB0 + fC;
						}
						else if (-fTmp >= fA11)
						{
							fT = 1;
							fSqrDist = fA00 + fA11 + fC + 2 * (fB0 + fTmp);
						}
						else
						{
							fT = -fTmp / fA11;
							fSqrDist = fTmp * fT + fA00 + 2 * fB0 + fC;
						}
						}
					}
				}
				else
				{
					if (-fB0 < fA00)
					{
						fT = 0;
						if (fB0 >= 0)
						{
						fS = 0;
						fSqrDist = fC;
						}
						else
						{
						fS = -fB0 / fA00;
						fSqrDist = fB0 * fS + fC;
						}
					}
					else
					{
						fS = 1;
						fTmp = fA01 + fB1;
						if (fTmp >= 0)
						{
						fT = 0;
						fSqrDist = fA00 + 2 * fB0 + fC;
						}
						else if (-fTmp >= fA11)
						{
						fT = 1;
						fSqrDist = fA00 + fA11 + fC + 2 * (fB0 + fTmp);
						}
						else
						{
						fT = -fTmp / fA11;
						fSqrDist = fTmp * fT + fA00 + 2 * fB0 + fC;
						}
					}
				}
				}
			}
			else
			{
				if (fT >= 0)
				{
				if (fT <= fDet)
				{
					fS = 0;
					if (fB1 >= 0)
					{
						fT = 0;
						fSqrDist = fC;
					}
					else if (-fB1 >= fA11)
					{
						fT = 1;
						fSqrDist = fA11 + 2 * fB1 + fC;
					}
					else
					{
						fT = -fB1 / fA11;
						fSqrDist = fB1 * fT + fC;
					}
				}
				else
				{
					fTmp = fA01 + fB0;
					if (fTmp < 0)
					{
						fT = 1;
						if (-fTmp >= fA00)
						{
						fS = 1;
						fSqrDist = fA00 + fA11 + fC + 2 * (fB1 + fTmp);
						}
						else
						{
						fS = -fTmp / fA00;
						fSqrDist = fTmp * fS + fA11 + 2 * fB1 + fC;
						}
					}
					else
					{
						fS = 0;
						if (fB1 >= 0)
						{
						fT = 0;
						fSqrDist = fC;
						}
						else if (-fB1 >= fA11)
						{
						fT = 1;
						fSqrDist = fA11 + 2 * fB1 + fC;
						}
						else
						{
						fT = -fB1 / fA11;
						fSqrDist = fB1 * fT + fC;
						}
					}
				}
				}
				else
				{
				if (fB0 < 0)
				{
					fT = 0;
					if (-fB0 >= fA00)
					{
						fS = 1;
						fSqrDist = fA00 + 2 * fB0 + fC;
					}
					else
					{
						fS = -fB0 / fA00;
						fSqrDist = fB0 * fS + fC;
					}
				}
				else
				{
					fS = 0;
					if (fB1 >= 0)
					{
						fT = 0;
						fSqrDist = fC;
					}
					else if (-fB1 >= fA11)
					{
						fT = 1;
						fSqrDist = fA11 + 2 * fB1 + fC;
					}
					else
					{
						fT = -fB1 / fA11;
						fSqrDist = fB1 * fT + fC;
					}
				}
				}
			}
		}
		else
		{
			if (fA01 > 0)
			{
				if (fB0 >= 0)
				{
				fS = 0;
				fT = 0;
				fSqrDist = fC;
				}
				else if (-fB0 <= fA00)
				{
				fS = -fB0 / fA00;
				fT = 0;
				fSqrDist = fB0 * fS + fC;
				}
				else
				{
				fB1 = -kDiff.dotProduct(seg.delta);
				fS = 1;
				fTmp = fA00 + fB0;
				if (-fTmp >= fA01)
				{
					fT = 1;
					fSqrDist = fA00 + fA11 + fC + 2 * (fA01 + fB0 + fB1);
				}
				else
				{
					fT = -fTmp / fA01;
					fSqrDist = fA00 + 2 * fB0 + fC + fT * (fA11 * fT + 2 * (fA01 + fB1));
				}
				}
			}
			else
			{
				if (-fB0 >= fA00)
				{
				fS = 1;
				fT = 0;
				fSqrDist = fA00 + 2 * fB0 + fC;
				}
				else if (fB0 <= 0)
				{
				fS = -fB0 / fA00;
				fT = 0;
				fSqrDist = fB0 * fS + fC;
				}
				else
				{
				fB1 = -kDiff.dotProduct(seg.delta);
				fS = 0;
				if (fB0 >= -fA01)
				{
					fT = 1;
					fSqrDist = fA11 + 2 * fB1 + fC;
				}
				else
				{
					fT = -fB0 / fA01;
					fSqrDist = fC + fT * (2 * fB1 + fA11 * fT);
				}
				}
			}
		}

		out[0] = fS;
		out[1] = fT;
		return Math.abs(fSqrDist);
		
	}

	JSegment.prototype.pointSegmentDistanceSq = function(out, pt)
	{

		var kDiff = pt.subtract(this.origin);
		var fT = kDiff.dotProduct(this.delta);

		if (fT <= 0)
		{
			fT = 0;
		}
		else
		{
			var fSqrLen = this.delta.get_lengthSquared();
			if (fT >= fSqrLen)
			{
				fT = 1;
				kDiff = kDiff.subtract(this.delta);
			}
			else
			{
				fT /= fSqrLen;
				kDiff = kDiff.subtract(JNumber3D.getScaleVector(this.delta, fT));
			}
		}

		out[0] = fT;
		return kDiff.get_lengthSquared();
		
	}

	JSegment.prototype.segmentBoxDistanceSq = function(out, rkBox, boxState)
	{

		out[3] = 0;
		out[0] = 0;
		out[1] = 0;
		out[2] = 0;

		var obj = [];
		var kRay = new JRay(this.origin, this.delta);
		var fSqrDistance = this.sqrDistanceLine(obj, kRay, rkBox, boxState);
		if (obj[3] >= 0)
		{
			if (obj[3] <= 1)
			{
				out[3] = obj[3];
				out[0] = obj[0];
				out[1] = obj[1];
				out[2] = obj[2];
				return Math.max(fSqrDistance, 0);
			}
			else
			{
				fSqrDistance = this.sqrDistancePoint(out, this.origin.add(this.delta), rkBox, boxState);
				out[3] = 1;
				return Math.max(fSqrDistance, 0);
			}
		}
		else
		{
			fSqrDistance = this.sqrDistancePoint(out, this.origin, rkBox, boxState);
			out[3] = 0;
			return Math.max(fSqrDistance, 0);
		}
		
	}

	JSegment.prototype.sqrDistanceLine = function(out, rkLine, rkBox, boxState)
	{

		var kDiff, kPnt, kDir;
		var orientationCols = boxState.getOrientationCols();
		out[3] = 0;
		out[0] = 0;
		out[1] = 0;
		out[2] = 0;

		kDiff = rkLine.origin.subtract(boxState.position);
		kPnt = new Vector3D(kDiff.dotProduct(orientationCols[0]),
			kDiff.dotProduct(orientationCols[1]),
			kDiff.dotProduct(orientationCols[2]));

		kDir = new Vector3D(rkLine.dir.dotProduct(orientationCols[0]),
			rkLine.dir.dotProduct(orientationCols[1]),
			rkLine.dir.dotProduct(orientationCols[2]));
		
		var kPntArr = JNumber3D.toArray(kPnt);
		var kDirArr = JNumber3D.toArray(kDir);

		var bReflect = [];
		for (var i = 0; i < 3; i++)
		{
			if (kDirArr[i] < 0)
			{
				kPntArr[i] = -kPntArr[i];
				kDirArr[i] = -kDirArr[i];
				bReflect[i] = true;
			}
			else
			{
				bReflect[i] = false;
			}
		}

		JNumber3D.copyFromArray(kPnt, kPntArr);
		JNumber3D.copyFromArray(kDir, kDirArr);

		var obj = new SegmentInfo(kPnt.clone(), 0, 0);

		if (kDir.x > 0)
		{
			if (kDir.y > 0)
			{
				if (kDir.z > 0)
				{
				this.caseNoZeros(obj, kDir, rkBox);
				out[3] = obj.pfLParam;
				}
				else
				{
				this.case0(obj, 0, 1, 2, kDir, rkBox);
				out[3] = obj.pfLParam;
				}
			}
			else
			{
				if (kDir.z > 0)
				{
				this.case0(obj, 0, 2, 1, kDir, rkBox);
				out[3] = obj.pfLParam;
				}
				else
				{
				this.case00(obj, 0, 1, 2, kDir, rkBox);
				out[3] = obj.pfLParam;
				}
			}
		}
		else
		{
			if (kDir.y > 0)
			{
				if (kDir.z > 0)
				{
				this.case0(obj, 1, 2, 0, kDir, rkBox);
				out[3] = obj.pfLParam;
				}
				else
				{
				this.case00(obj, 1, 0, 2, kDir, rkBox);
				out[3] = obj.pfLParam;
				}
			}
			else
			{
				if (kDir.z > 0)
				{
				this.case00(obj, 2, 0, 1, kDir, rkBox);
				out[3] = obj.pfLParam;
				}
				else
				{
				this.case000(obj, rkBox);
				out[3] = 0;
				}
			}
		}

		kPntArr = JNumber3D.toArray(obj.rkPnt);
		for (i = 0; i < 3; i++)
		{
			if (bReflect[i])
				kPntArr[i] = -kPntArr[i];
		}
		JNumber3D.copyFromArray(obj.rkPnt, kPntArr);

		out[0] = obj.rkPnt.x;
		out[1] = obj.rkPnt.y;
		out[2] = obj.rkPnt.z;

		return Math.max(obj.rfSqrDistance, 0);
		
	}

	JSegment.prototype.sqrDistancePoint = function(out, rkPoint, rkBox, boxState)
	{

		var kDiff, kClosest, boxHalfSide;
		var fSqrDistance=0, fDelta;
		
		var orientationVector = boxState.getOrientationCols();
		kDiff = rkPoint.subtract(boxState.position);
		kClosest = new Vector3D(kDiff.dotProduct(orientationVector[0]),
			kDiff.dotProduct(orientationVector[1]),
			kDiff.dotProduct(orientationVector[2]));

		boxHalfSide = rkBox.getHalfSideLengths();

		if (kClosest.x < -boxHalfSide.x)
		{
			fDelta = kClosest.x + boxHalfSide.x;
			fSqrDistance += (fDelta * fDelta);
			kClosest.x = -boxHalfSide.x;
		}
		else if (kClosest.x > boxHalfSide.x)
		{
			fDelta = kClosest.x - boxHalfSide.x;
			fSqrDistance += (fDelta * fDelta);
			kClosest.x = boxHalfSide.x;
		}

		if (kClosest.y < -boxHalfSide.y)
		{
			fDelta = kClosest.y + boxHalfSide.y;
			fSqrDistance += (fDelta * fDelta);
			kClosest.y = -boxHalfSide.y;
		}
		else if (kClosest.y > boxHalfSide.y)
		{
			fDelta = kClosest.y - boxHalfSide.y;
			fSqrDistance += (fDelta * fDelta);
			kClosest.y = boxHalfSide.y;
		}

		if (kClosest.z < -boxHalfSide.z)
		{
			fDelta = kClosest.z + boxHalfSide.z;
			fSqrDistance += (fDelta * fDelta);
			kClosest.z = -boxHalfSide.z;
		}
		else if (kClosest.z > boxHalfSide.z)
		{
			fDelta = kClosest.z - boxHalfSide.z;
			fSqrDistance += (fDelta * fDelta);
			kClosest.z = boxHalfSide.z;
		}

		out[0] = kClosest.x;
		out[1] = kClosest.y;
		out[2] = kClosest.z;

		return Math.max(fSqrDistance, 0);
		
	}

	JSegment.prototype.face = function(out, i0, i1, i2, rkDir, rkBox, rkPmE)
	{

		
		var fLSqr, fInv, fTmp, fParam, fT, fDelta;

		var kPpE = new Vector3D();
		var boxHalfSide = rkBox.getHalfSideLengths();
		
		var boxHalfArr, rkPntArr, rkDirArr, kPpEArr, rkPmEArr;
		boxHalfArr = JNumber3D.toArray(boxHalfSide);
		rkPntArr = JNumber3D.toArray(out.rkPnt);
		rkDirArr = JNumber3D.toArray(rkDir);
		kPpEArr = JNumber3D.toArray(kPpE);
		rkPmEArr = JNumber3D.toArray(rkPmE);

		kPpEArr[i1] = rkPntArr[i1] + boxHalfArr[i1];
		kPpEArr[i2] = rkPntArr[i2] + boxHalfArr[i2];
		JNumber3D.copyFromArray(rkPmE, kPpEArr);

		if (rkDirArr[i0] * kPpEArr[i1] >= rkDirArr[i1] * rkPmEArr[i0])
		{
			if (rkDirArr[i0] * kPpEArr[i2] >= rkDirArr[i2] * rkPmEArr[i0])
			{
				rkPntArr[i0] = boxHalfArr[i0];
				fInv = 1 / rkDirArr[i0];
				rkPntArr[i1] -= (rkDirArr[i1] * rkPmEArr[i0] * fInv);
				rkPntArr[i2] -= (rkDirArr[i2] * rkPmEArr[i0] * fInv);
				out.pfLParam = -rkPmEArr[i0] * fInv;
				JNumber3D.copyFromArray(out.rkPnt, rkPntArr);
			}
			else
			{
				fLSqr = rkDirArr[i0] * rkDirArr[i0] + rkDirArr[i2] * rkDirArr[i2];
				fTmp = fLSqr * kPpEArr[i1] - rkDirArr[i1] * (rkDirArr[i0] * rkPmEArr[i0] + rkDirArr[i2] * kPpEArr[i2]);
				if (fTmp <= 2 * fLSqr * boxHalfArr[i1])
				{
				fT = fTmp / fLSqr;
				fLSqr += (rkDirArr[i1] * rkDirArr[i1]);
				fTmp = kPpEArr[i1] - fT;
				fDelta = rkDirArr[i0] * rkPmEArr[i0] + rkDirArr[i1] * fTmp + rkDirArr[i2] * kPpEArr[i2];
				fParam = -fDelta / fLSqr;
				out.rfSqrDistance += (rkPmEArr[i0] * rkPmEArr[i0] + fTmp * fTmp + kPpEArr[i2] * kPpEArr[i2] + fDelta * fParam);

				out.pfLParam = fParam;
				rkPntArr[i0] = boxHalfArr[i0];
				rkPntArr[i1] = fT - boxHalfArr[i1];
				rkPntArr[i2] = -boxHalfArr[i2];
				JNumber3D.copyFromArray(out.rkPnt, rkPntArr);
				}
				else
				{
				fLSqr += (rkDirArr[i1] * rkDirArr[i1]);
				fDelta = rkDirArr[i0] * rkPmEArr[i0] + rkDirArr[i1] * rkPmEArr[i1] + rkDirArr[i2] * kPpEArr[i2];
				fParam = -fDelta / fLSqr;
				out.rfSqrDistance += (rkPmEArr[i0] * rkPmEArr[i0] + rkPmEArr[i1] * rkPmEArr[i1] + kPpEArr[i2] * kPpEArr[i2] + fDelta * fParam);

				out.pfLParam = fParam;
				rkPntArr[i0] = boxHalfArr[i0];
				rkPntArr[i1] = boxHalfArr[i1];
				rkPntArr[i2] = -boxHalfArr[i2];
				JNumber3D.copyFromArray(out.rkPnt, rkPntArr);
				}
			}
		}
		else
		{
			if (rkDirArr[i0] * kPpEArr[i2] >= rkDirArr[i2] * rkPmEArr[i0])
			{
				fLSqr = rkDirArr[i0] * rkDirArr[i0] + rkDirArr[i1] * rkDirArr[i1];
				fTmp = fLSqr * kPpEArr[i2] - rkDirArr[i2] * (rkDirArr[i0] * rkPmEArr[i0] + rkDirArr[i1] * kPpEArr[i1]);
				if (fTmp <= 2 * fLSqr * boxHalfArr[i2])
				{
				fT = fTmp / fLSqr;
				fLSqr += (rkDirArr[i2] * rkDirArr[i2]);
				fTmp = kPpEArr[i2] - fT;
				fDelta = rkDirArr[i0] * rkPmEArr[i0] + rkDirArr[i1] * kPpEArr[i1] + rkDirArr[i2] * fTmp;
				fParam = -fDelta / fLSqr;
				out.rfSqrDistance += (rkPmEArr[i0] * rkPmEArr[i0] + kPpEArr[i1] * kPpEArr[i1] + fTmp * fTmp + fDelta * fParam);

				out.pfLParam = fParam;
				rkPntArr[i0] = boxHalfArr[i0];
				rkPntArr[i1] = -boxHalfArr[i1];
				rkPntArr[i2] = fT - boxHalfArr[i2];
				JNumber3D.copyFromArray(out.rkPnt, rkPntArr);
				}
				else
				{
				fLSqr += (rkDirArr[i2] * rkDirArr[i2]);
				fDelta = rkDirArr[i0] * rkPmEArr[i0] + rkDirArr[i1] * kPpEArr[i1] + rkDirArr[i2] * rkPmEArr[i2];
				fParam = -fDelta / fLSqr;
				out.rfSqrDistance += (rkPmEArr[i0] * rkPmEArr[i0] + kPpEArr[i1] * kPpEArr[i1] + rkPmEArr[i2] * rkPmEArr[i2] + fDelta * fParam);

				out.pfLParam = fParam;
				rkPntArr[i0] = boxHalfArr[i0];
				rkPntArr[i1] = -boxHalfArr[i1];
				rkPntArr[i2] = boxHalfArr[i2];
				JNumber3D.copyFromArray(out.rkPnt, rkPntArr);
				}
			}
			else
			{
				fLSqr = rkDirArr[i0] * rkDirArr[i0] + rkDirArr[i2] * rkDirArr[i2];
				fTmp = fLSqr * kPpEArr[i1] - rkDirArr[i1] * (rkDirArr[i0] * rkPmEArr[i0] + rkDirArr[i2] * kPpEArr[i2]);
				if (fTmp >= 0)
				{
				if (fTmp <= 2 * fLSqr * boxHalfArr[i1])
				{
					fT = fTmp / fLSqr;
					fLSqr += (rkDirArr[i1] * rkDirArr[i1]);
					fTmp = kPpEArr[i1] - fT;
					fDelta = rkDirArr[i0] * rkPmEArr[i0] + rkDirArr[i1] * fTmp + rkDirArr[i2] * kPpEArr[i2];
					fParam = -fDelta / fLSqr;
					out.rfSqrDistance += (rkPmEArr[i0] * rkPmEArr[i0] + fTmp * fTmp + kPpEArr[i2] * kPpEArr[i2] + fDelta * fParam);

					out.pfLParam = fParam;
					rkPntArr[i0] = boxHalfArr[i0];
					rkPntArr[i1] = fT - boxHalfArr[i1];
					rkPntArr[i2] = -boxHalfArr[i2];
					JNumber3D.copyFromArray(out.rkPnt, rkPntArr);
				}
				else
				{
					fLSqr += (rkDirArr[i1] * rkDirArr[i1]);
					fDelta = rkDirArr[i0] * rkPmEArr[i0] + rkDirArr[i1] * rkPmEArr[i1] + rkDirArr[i2] * kPpEArr[i2];
					fParam = -fDelta / fLSqr;
					out.rfSqrDistance += (rkPmEArr[i0] * rkPmEArr[i0] + rkPmEArr[i1] * rkPmEArr[i1] + kPpEArr[i2] * kPpEArr[i2] + fDelta * fParam);

					out.pfLParam = fParam;
					rkPntArr[i0] = boxHalfArr[i0];
					rkPntArr[i1] = boxHalfArr[i1];
					rkPntArr[i2] = -boxHalfArr[i2];
					JNumber3D.copyFromArray(out.rkPnt, rkPntArr);
				}
				return;
				}

				fLSqr = rkDirArr[i0] * rkDirArr[i0] + rkDirArr[i1] * rkDirArr[i1];
				fTmp = fLSqr * kPpEArr[i2] - rkDirArr[i2] * (rkDirArr[i0] * rkPmEArr[i0] + rkDirArr[i1] * kPpEArr[i1]);
				if (fTmp >= 0)
				{
				if (fTmp <= 2 * fLSqr * boxHalfArr[i2])
				{
					fT = fTmp / fLSqr;
					fLSqr += (rkDirArr[i2] * rkDirArr[i2]);
					fTmp = kPpEArr[i2] - fT;
					fDelta = rkDirArr[i0] * rkPmEArr[i0] + rkDirArr[i1] * kPpEArr[i1] + rkDirArr[i2] * fTmp;
					fParam = -fDelta / fLSqr;
					out.rfSqrDistance += (rkPmEArr[i0] * rkPmEArr[i0] + kPpEArr[i1] * kPpEArr[i1] + fTmp * fTmp + fDelta * fParam);

					out.pfLParam = fParam;
					rkPntArr[i0] = boxHalfArr[i0];
					rkPntArr[i1] = -boxHalfArr[i1];
					rkPntArr[i2] = fT - boxHalfArr[i2];
					JNumber3D.copyFromArray(out.rkPnt, rkPntArr);
				}
				else
				{
					fLSqr += (rkDirArr[i2] * rkDirArr[i2]);
					fDelta = rkDirArr[i0] * rkPmEArr[i0] + rkDirArr[i1] * kPpEArr[i1] + rkDirArr[i2] * rkPmEArr[i2];
					fParam = -fDelta / fLSqr;
					out.rfSqrDistance += (rkPmEArr[i0] * rkPmEArr[i0] + kPpEArr[i1] * kPpEArr[i1] + rkPmEArr[i2] * rkPmEArr[i2] + fDelta * fParam);

					out.pfLParam = fParam;
					rkPntArr[i0] = boxHalfArr[i0];
					rkPntArr[i1] = -boxHalfArr[i1];
					rkPntArr[i2] = boxHalfArr[i2];
					JNumber3D.copyFromArray(out.rkPnt, rkPntArr);
				}
				return;
				}

				fLSqr += (rkDirArr[i2] * rkDirArr[i2]);
				fDelta = rkDirArr[i0] * rkPmEArr[i0] + rkDirArr[i1] * kPpEArr[i1] + rkDirArr[i2] * kPpEArr[i2];
				fParam = -fDelta / fLSqr;
				out.rfSqrDistance += (rkPmEArr[i0] * rkPmEArr[i0] + kPpEArr[i1] * kPpEArr[i1] + kPpEArr[i2] * kPpEArr[i2] + fDelta * fParam);

				out.pfLParam = fParam;
				rkPntArr[i0] = boxHalfArr[i0];
				rkPntArr[i1] = -boxHalfArr[i1];
				rkPntArr[i2] = -boxHalfArr[i2];
				JNumber3D.copyFromArray(out.rkPnt, rkPntArr);
			}
		}
		
	}

	JSegment.prototype.caseNoZeros = function(out, rkDir, rkBox)
	{

		var boxHalfSide = rkBox.getHalfSideLengths();
		var kPmE = new Vector3D(out.rkPnt.x - boxHalfSide.x, out.rkPnt.y - boxHalfSide.y, out.rkPnt.z - boxHalfSide.z);

		var fProdDxPy = rkDir.x * kPmE.y, fProdDyPx = rkDir.y * kPmE.x, fProdDzPx, fProdDxPz, fProdDzPy, fProdDyPz;

		if (fProdDyPx >= fProdDxPy)
		{
			fProdDzPx = rkDir.z * kPmE.x;
			fProdDxPz = rkDir.x * kPmE.z;
			if (fProdDzPx >= fProdDxPz)
			{
				this.face(out, 0, 1, 2, rkDir, rkBox, kPmE);
			}
			else
			{
				this.face(out, 2, 0, 1, rkDir, rkBox, kPmE);
			}
		}
		else
		{
			fProdDzPy = rkDir.z * kPmE.y;
			fProdDyPz = rkDir.y * kPmE.z;
			if (fProdDzPy >= fProdDyPz)
			{
				this.face(out, 1, 2, 0, rkDir, rkBox, kPmE);
			}
			else
			{
				this.face(out, 2, 0, 1, rkDir, rkBox, kPmE);
			}
		}
		
	}

	JSegment.prototype.case0 = function(out, i0, i1, i2, rkDir, rkBox)
	{

		var boxHalfSide = rkBox.getHalfSideLengths();
		var boxHalfArr, rkPntArr, rkDirArr;
		boxHalfArr = JNumber3D.toArray(boxHalfSide);
		rkPntArr = JNumber3D.toArray(out.rkPnt);
		rkDirArr = JNumber3D.toArray(rkDir);
		
		var fPmE0 = rkPntArr[i0] - boxHalfArr[i0], fPmE1 = rkPntArr[i1] - boxHalfArr[i1], fProd0 = rkDirArr[i1] * fPmE0, fProd1 = rkDirArr[i0] * fPmE1, fDelta, fInvLSqr, fInv, fPpE1, fPpE0;

		if (fProd0 >= fProd1)
		{
			rkPntArr[i0] = boxHalfArr[i0];

			fPpE1 = rkPntArr[i1] + boxHalfArr[i1];
			fDelta = fProd0 - rkDirArr[i0] * fPpE1;
			if (fDelta >= 0)
			{
				fInvLSqr = 1 / (rkDirArr[i0] * rkDirArr[i0] + rkDirArr[i1] * rkDirArr[i1]);
				out.rfSqrDistance += (fDelta * fDelta * fInvLSqr);

				rkPntArr[i1] = -boxHalfArr[i1];
				out.pfLParam = -(rkDirArr[i0] * fPmE0 + rkDirArr[i1] * fPpE1) * fInvLSqr;
			}
			else
			{
				fInv = 1 / rkDirArr[i0];
				rkPntArr[i1] -= (fProd0 * fInv);
				out.pfLParam = -fPmE0 * fInv;
			}
			JNumber3D.copyFromArray(out.rkPnt, rkPntArr);
		}
		else
		{
			rkPntArr[i1] = boxHalfArr[i1];

			fPpE0 = rkPntArr[i0] + boxHalfArr[i0];
			fDelta = fProd1 - rkDirArr[i1] * fPpE0;
			if (fDelta >= 0)
			{
				fInvLSqr = 1 / (rkDirArr[i0] * rkDirArr[i0] + rkDirArr[i1] * rkDirArr[i1]);
				out.rfSqrDistance += (fDelta * fDelta * fInvLSqr);

				rkPntArr[i0] = -boxHalfArr[i0];
				out.pfLParam = -(rkDirArr[i0] * fPpE0 + rkDirArr[i1] * fPmE1) * fInvLSqr;
			}
			else
			{
				fInv = 1 / rkDirArr[i1];
				rkPntArr[i0] -= (fProd1 * fInv);
				out.pfLParam = -fPmE1 * fInv;
			}
			JNumber3D.copyFromArray(out.rkPnt, rkPntArr);
		}

		if (rkPntArr[i2] < -boxHalfArr[i2])
		{
			fDelta = rkPntArr[i2] + boxHalfArr[i2];
			out.rfSqrDistance += (fDelta * fDelta);
			rkPntArr[i2] = -boxHalfArr[i2];
		}
		else if (rkPntArr[i2] > boxHalfArr[i2])
		{
			fDelta = rkPntArr[i2] - boxHalfArr[i2];
			out.rfSqrDistance += (fDelta * fDelta);
			rkPntArr[i2] = boxHalfArr[i2];
		}
		JNumber3D.copyFromArray(out.rkPnt, rkPntArr);
		
	}

	JSegment.prototype.case00 = function(out, i0, i1, i2, rkDir, rkBox)
	{

		var fDelta = 0;
		var boxHalfSide = rkBox.getHalfSideLengths();
		
		var boxHalfArr, rkPntArr, rkDirArr;
		boxHalfArr = JNumber3D.toArray(boxHalfSide);
		rkPntArr = JNumber3D.toArray(out.rkPnt);
		rkDirArr = JNumber3D.toArray(rkDir);
		out.pfLParam = (boxHalfArr[i0] - rkPntArr[i0]) / rkDirArr[i0];

		rkPntArr[i0] = boxHalfArr[i0];

		if (rkPntArr[i1] < -boxHalfArr[i1])
		{
			fDelta = rkPntArr[i1] + boxHalfArr[i1];
			out.rfSqrDistance += (fDelta * fDelta);
			rkPntArr[i1] = -boxHalfArr[i1];
		}
		else if (rkPntArr[i1] > boxHalfArr[i1])
		{
			fDelta = rkPntArr[i1] - boxHalfArr[i1];
			out.rfSqrDistance += (fDelta * fDelta);
			rkPntArr[i1] = boxHalfArr[i1];
		}

		if (rkPntArr[i2] < -boxHalfArr[i2])
		{
			fDelta = rkPntArr[i2] + boxHalfArr[i2];
			out.rfSqrDistance += (fDelta * fDelta);
			rkPntArr[i2] = -boxHalfArr[i2];
		}
		else if (rkPntArr[i2] > boxHalfArr[i2])
		{
			fDelta = rkPntArr[i2] - boxHalfArr[i2];
			out.rfSqrDistance += (fDelta * fDelta);
			rkPntArr[i2] = boxHalfArr[i2];
		}

		JNumber3D.copyFromArray(out.rkPnt, rkPntArr);
		
	}

	JSegment.prototype.case000 = function(out, rkBox)
	{

		var fDelta = 0;
		var boxHalfSide = rkBox.getHalfSideLengths();

		if (out.rkPnt.x < -boxHalfSide.x)
		{
			fDelta = out.rkPnt.x + boxHalfSide.x;
			out.rfSqrDistance += (fDelta * fDelta);
			out.rkPnt.x = -boxHalfSide.x;
		}
		else if (out.rkPnt.x > boxHalfSide.x)
		{
			fDelta = out.rkPnt.x - boxHalfSide.x;
			out.rfSqrDistance += (fDelta * fDelta);
			out.rkPnt.x = boxHalfSide.x;
		}

		if (out.rkPnt.y < -boxHalfSide.y)
		{
			fDelta = out.rkPnt.y + boxHalfSide.y;
			out.rfSqrDistance += (fDelta * fDelta);
			out.rkPnt.y = -boxHalfSide.y;
		}
		else if (out.rkPnt.y > boxHalfSide.y)
		{
			fDelta = out.rkPnt.y - boxHalfSide.y;
			out.rfSqrDistance += (fDelta * fDelta);
			out.rkPnt.y = boxHalfSide.y;
		}

		if (out.rkPnt.z < -boxHalfSide.z)
		{
			fDelta = out.rkPnt.z + boxHalfSide.z;
			out.rfSqrDistance += (fDelta * fDelta);
			out.rkPnt.z = -boxHalfSide.z;
		}
		else if (out.rkPnt.z > boxHalfSide.z)
		{
			fDelta = out.rkPnt.z - boxHalfSide.z;
			out.rfSqrDistance += (fDelta * fDelta);
			out.rkPnt.z = boxHalfSide.z;
		}
		
	}



	jiglib.JSegment = JSegment; 

})(jiglib);


(function(jiglib) {


	var JConfig = function()
	{
	}

	JConfig.solverType =  "ACCUMULATED"; // String
	JConfig.rotationType =  "DEGREES"; // String
	JConfig.doShockStep =  false; // Boolean
	JConfig.allowedPenetration =  0.01; // Number
	JConfig.collToll =  0.05; // Number
	JConfig.velThreshold =  0.5; // Number
	JConfig.angVelThreshold =  0.5; // Number
	JConfig.posThreshold =  0.2; // Number
	JConfig.orientThreshold =  0.2; // Number
	JConfig.deactivationTime =  0.5; // Number
	JConfig.numPenetrationRelaxationTimesteps =  10; // Number
	JConfig.numCollisionIterations =  1; // Number
	JConfig.numContactIterations =  2; // Number
	JConfig.numConstraintIterations =  2; // Number


	jiglib.JConfig = JConfig; 

})(jiglib);


(function(jiglib) {

	var PhysicsSystem = jiglib.PhysicsSystem;
	var RigidBody = jiglib.RigidBody;

	var AbstractPhysics = function(speed)
	{
		this.initTime = null; // int
		this.stepTime = null; // int
		this.speed = null; // Number
		this.deltaTime =  0; // Number
		this.physicsSystem = null; // PhysicsSystem
		this._frameTime =  0; // uint

		this.speed = speed;
		this.initTime = getTimer();
		this.physicsSystem = PhysicsSystem.getInstance();
		this.physicsSystem.setCollisionSystem(false); // bruteforce
		//this.physicsSystem.setCollisionSystem(true,20,20,20,200,200,200); // grid
		
	}

	AbstractPhysics.prototype.addBody = function(body)
	{

		this.physicsSystem.addBody(body);
		
	}

	AbstractPhysics.prototype.removeBody = function(body)
	{

		this.physicsSystem.removeBody(body);
		
	}

	AbstractPhysics.prototype.get_engine = function()
	{

		return this.physicsSystem ;
		
	}

	AbstractPhysics.prototype.get_frameTime = function()
	{

		return this._frameTime;
		
	}

	AbstractPhysics.prototype.afterPause = function()
	{

		this.initTime = getTimer();
		
	}

	AbstractPhysics.prototype.step = function(dt)
	{

			this.stepTime = getTimer();
			this.deltaTime = ((this.stepTime - this.initTime) / 1000) * this.speed;
			this.initTime = this.stepTime;
			this.physicsSystem.integrate((dt>0)?dt:this.deltaTime);
			this._frameTime = getTimer()-this.initTime;
		
	}



	jiglib.AbstractPhysics = AbstractPhysics; 

})(jiglib);


(function(jiglib) {

	var AbstractPhysics = jiglib.AbstractPhysics;
	var Matrix3D = jiglib.Matrix3D;
	var Vector3D = jiglib.Vector3D;
	var JBox = jiglib.JBox;
	var JPlane = jiglib.JPlane;
	var JSphere = jiglib.JSphere;
	var JTerrain = jiglib.JTerrain;
	var JTriangleMesh = jiglib.JTriangleMesh;
	var RigidBody = jiglib.RigidBody;

	var Away3D4Physics = function(view, speed)
	{
		this.view = null; // View3D

		this.view = view;
		jiglib.AbstractPhysics.apply(this, [ speed ]);
		
	}

	jiglib.extend(Away3D4Physics, AbstractPhysics);

	Away3D4Physics.prototype.getMesh = function(body)
	{

		if(body.get_skin()!=null){
			return Away3D4Mesh(body.get_skin()).mesh;
		}else {
			return null;
		}
		
	}

	Away3D4Physics.prototype.createGround = function(material, width, height, segmentsW, segmentsH, yUp, level)
	{
		if (width == null) width = 500;
		if (height == null) height = 500;
		if (segmentsW == null) segmentsW = 1;
		if (segmentsH == null) segmentsH = 1;
		if (yUp == null) yUp = true;

		var groundGeometry = new PlaneGeometry(width,height,segmentsW,segmentsH,yUp);
		var groundMesh = new Mesh();
		groundMesh.geometry = groundGeometry;
		groundMesh.material = material;

		this.view.scene.addChild(groundMesh);
		
		var jGround = new JPlane(new Away3D4Mesh(groundMesh),new Vector3D(0, 1, 0));
		jGround.set_y(level);
		jGround.set_movable(false);
		this.addBody(jGround);
		return jGround;
		
	}

	Away3D4Physics.prototype.createCube = function(material, width, height, depth, segmentsW, segmentsH, segmentsD, tile6)
	{
		if (width == null) width = 500;
		if (height == null) height = 500;
		if (depth == null) depth = 500;
		if (segmentsW == null) segmentsW = 1;
		if (segmentsH == null) segmentsH = 1;
		if (segmentsD == null) segmentsD = 1;
		if (tile6 == null) tile6 = true;

		var cubeGeometry = new CubeGeometry(width,height,depth,segmentsW,segmentsH,segmentsD,tile6);
		var cubeMesh = new Mesh();
		cubeMesh.geometry = cubeGeometry;
		cubeMesh.material = material;
		this.view.scene.addChild(cubeMesh);
		
		var jBox = new JBox(new Away3D4Mesh(cubeMesh), width, depth, height);
		this.addBody(jBox);
		return jBox;
		
	}

	Away3D4Physics.prototype.createSphere = function(material, radius, segmentsW, segmentsH, yUp)
	{
		if (radius == null) radius = 50;
		if (segmentsW == null) segmentsW = 16;
		if (segmentsH == null) segmentsH = 12;
		if (yUp == null) yUp = true;

		var sphereGeometry = new SphereGeometry(radius,segmentsW,segmentsH,yUp);
		var sphereMesh = new Mesh();
		sphereMesh.geometry = sphereGeometry;
		sphereMesh.material = material;
		this.view.scene.addChild(sphereMesh);

		var jsphere = new JSphere(new Away3D4Mesh(sphereMesh), radius);
		this.addBody(jsphere);
		return jsphere;
		
	}

	Away3D4Physics.prototype.createTerrain = function(material, heightMap, width, height, depth, segmentsW, segmentsH, maxElevation, minElevation, smoothMap)
	{
		if (width == null) width = 1000;
		if (height == null) height = 100;
		if (depth == null) depth = 1000;
		if (segmentsW == null) segmentsW = 30;
		if (segmentsH == null) segmentsH = 30;
		if (maxElevation == null) maxElevation = 255;
		if (smoothMap == null) smoothMap = false;

		var terrainMap = new Away3D4Terrain(material, heightMap, width, height, depth, segmentsW, segmentsH, maxElevation, minElevation, smoothMap);
		this.view.scene.addChild(terrainMap);
		
		var terrain = new JTerrain(terrainMap);
		this.addBody(terrain);
		
		return terrain;
		
	}

	Away3D4Physics.prototype.createMesh = function(skin, initPosition, initOrientation, maxTrianglesPerCell, minCellSize)
	{
		if (maxTrianglesPerCell == null) maxTrianglesPerCell = 10;
		if (minCellSize == null) minCellSize = 10;

		var mesh=new JTriangleMesh(new Away3D4Mesh(skin),initPosition,initOrientation,maxTrianglesPerCell,minCellSize);
		this.addBody(mesh);
		
		return mesh;
		
	}



	jiglib.Away3D4Physics = Away3D4Physics; 

})(jiglib);


(function(jiglib) {

	var ContactData = jiglib.ContactData;
	var PlaneData = jiglib.PlaneData;
	var EdgeData = jiglib.EdgeData;
	var TerrainData = jiglib.TerrainData;
	var OctreeCell = jiglib.OctreeCell;
	var CollOutBodyData = jiglib.CollOutBodyData;
	var TriangleVertexIndices = jiglib.TriangleVertexIndices;
	var SpanData = jiglib.SpanData;
	var Vector3D = jiglib.Vector3D;
	var RigidBody = jiglib.RigidBody;

	var CollOutData = function(frac, position, normal)
	{
		this.frac = null; // Number
		this.position = null; // Vector3D
		this.normal = null; // Vector3D

		this.frac = isNaN(frac) ? 0 : frac;
		this.position = position ? position : new Vector3D;
		this.normal = normal ? normal : new Vector3D;
		
	}



	jiglib.CollOutData = CollOutData; 

})(jiglib);


(function(jiglib) {

	var CollOutData = jiglib.CollOutData;
	var PlaneData = jiglib.PlaneData;
	var EdgeData = jiglib.EdgeData;
	var TerrainData = jiglib.TerrainData;
	var OctreeCell = jiglib.OctreeCell;
	var CollOutBodyData = jiglib.CollOutBodyData;
	var TriangleVertexIndices = jiglib.TriangleVertexIndices;
	var SpanData = jiglib.SpanData;
	var BodyPair = jiglib.BodyPair;
	var CachedImpulse = jiglib.CachedImpulse;

	var ContactData = function()
	{
		this.pair = null; // BodyPair
		this.impulse = null; // CachedImpulse
	}



	jiglib.ContactData = ContactData; 

})(jiglib);


(function(jiglib) {

	var CollOutData = jiglib.CollOutData;
	var ContactData = jiglib.ContactData;
	var EdgeData = jiglib.EdgeData;
	var TerrainData = jiglib.TerrainData;
	var OctreeCell = jiglib.OctreeCell;
	var CollOutBodyData = jiglib.CollOutBodyData;
	var TriangleVertexIndices = jiglib.TriangleVertexIndices;
	var SpanData = jiglib.SpanData;
	var Vector3D = jiglib.Vector3D;
	var JNumber3D = jiglib.JNumber3D;
	var JMath3D = jiglib.JMath3D;

	var PlaneData = function()
	{
		this._position = null; // Vector3D
		this._normal = null; // Vector3D
		this._distance = null; // Number

		this._position = new Vector3D();
		this._normal = new Vector3D(0, 1, 0);
		this._distance = 0;
		
	}

	PlaneData.prototype.get_position = function()
	{

		return this._position;
		
	}

	PlaneData.prototype.get_normal = function()
	{

		return this._normal;
		
	}

	PlaneData.prototype.get_distance = function()
	{

		return this._distance;
		
	}

	PlaneData.prototype.pointPlaneDistance = function(pt)
	{

		return this._normal.dotProduct(pt) - this._distance;
		
	}

	PlaneData.prototype.setWithNormal = function(pos, nor)
	{

		this._position = pos.clone();
		this._normal = nor.clone();
		this._distance = pos.dotProduct(nor);
		
	}

	PlaneData.prototype.setWithPoint = function(pos0, pos1, pos2)
	{

		this._position = pos0.clone();
		
		var dr1 = pos1.subtract(pos0);
		var dr2 = pos2.subtract(pos0);
		this._normal = dr1.crossProduct(dr2);
		
		var nLen = this._normal.get_length();
		if (nLen < JMath3D.NUM_TINY) {
			this._normal = new Vector3D(0, 1, 0);
			this._distance = 0;
		}else {
			this._normal.scaleBy(1 / nLen);
			this._distance = pos0.dotProduct(this._normal);
		}
		
	}



	jiglib.PlaneData = PlaneData; 

})(jiglib);


(function(jiglib) {

	var CollOutData = jiglib.CollOutData;
	var ContactData = jiglib.ContactData;
	var PlaneData = jiglib.PlaneData;
	var TerrainData = jiglib.TerrainData;
	var OctreeCell = jiglib.OctreeCell;
	var CollOutBodyData = jiglib.CollOutBodyData;
	var TriangleVertexIndices = jiglib.TriangleVertexIndices;
	var SpanData = jiglib.SpanData;

	var EdgeData = function(ind0, ind1)
	{
		this.ind0 = null; // int
		this.ind1 = null; // int

		this.ind0 = ind0;
		this.ind1 = ind1;
		
	}



	jiglib.EdgeData = EdgeData; 

})(jiglib);


(function(jiglib) {

	var CollOutData = jiglib.CollOutData;
	var ContactData = jiglib.ContactData;
	var PlaneData = jiglib.PlaneData;
	var EdgeData = jiglib.EdgeData;
	var OctreeCell = jiglib.OctreeCell;
	var CollOutBodyData = jiglib.CollOutBodyData;
	var TriangleVertexIndices = jiglib.TriangleVertexIndices;
	var SpanData = jiglib.SpanData;
	var Vector3D = jiglib.Vector3D;

	var TerrainData = function(height, normal)
	{
		this.height = null; // Number
		this.normal = null; // Vector3D

		this.height = isNaN(height) ? 0 : height;
		this.normal = normal ? normal : new Vector3D();
		
	}



	jiglib.TerrainData = TerrainData; 

})(jiglib);


(function(jiglib) {

	var CollOutData = jiglib.CollOutData;
	var ContactData = jiglib.ContactData;
	var PlaneData = jiglib.PlaneData;
	var EdgeData = jiglib.EdgeData;
	var TerrainData = jiglib.TerrainData;
	var CollOutBodyData = jiglib.CollOutBodyData;
	var TriangleVertexIndices = jiglib.TriangleVertexIndices;
	var SpanData = jiglib.SpanData;
	var Vector3D = jiglib.Vector3D;
	var JAABox = jiglib.JAABox;
	var JNumber3D = jiglib.JNumber3D;

	var OctreeCell = function(aabox)
	{
		this.childCellIndices = null; // int
		this.triangleIndices = null; // int
		this.AABox = null; // JAABox
		this._points = null; // Vector3D
		this._egdes = null; // EdgeData

		this.childCellIndices = [];
		this.triangleIndices = [];
		
		this.clear();
		
		if(aabox){
			this.AABox = aabox.clone();
		}else {
			this.AABox = new JAABox();
		}
		this._points = this.AABox.getAllPoints();
		this._egdes = this.AABox.get_edges();
		
	}

	OctreeCell.prototype.isLeaf = function()
	{

		return this.childCellIndices[0] == -1;
		
	}

	OctreeCell.prototype.clear = function()
	{

		for (var i = 0; i < OctreeCell.NUM_CHILDREN; i++ ) {
			this.childCellIndices[i] = -1;
		}
		this.triangleIndices.splice(0, this.triangleIndices.length);
		
	}

	OctreeCell.prototype.get_points = function()
	{

		return this._points;
		
	}

	OctreeCell.prototype.get_egdes = function()
	{

		return this._egdes;
		
	}

	OctreeCell.NUM_CHILDREN =  8; // uint


	jiglib.OctreeCell = OctreeCell; 

})(jiglib);


(function(jiglib) {

	var ContactData = jiglib.ContactData;
	var PlaneData = jiglib.PlaneData;
	var EdgeData = jiglib.EdgeData;
	var TerrainData = jiglib.TerrainData;
	var OctreeCell = jiglib.OctreeCell;
	var TriangleVertexIndices = jiglib.TriangleVertexIndices;
	var SpanData = jiglib.SpanData;
	var CollOutData = jiglib.CollOutData;
	var Vector3D = jiglib.Vector3D;
	var RigidBody = jiglib.RigidBody;

	var CollOutBodyData = function(frac, position, normal, rigidBody)
	{
		this.rigidBody = null; // RigidBody

		jiglib.CollOutData.apply(this, [ frac, position, normal ]);
		this.rigidBody = rigidBody;
		
	}

	jiglib.extend(CollOutBodyData, CollOutData);



	jiglib.CollOutBodyData = CollOutBodyData; 

})(jiglib);


(function(jiglib) {

	var CollOutData = jiglib.CollOutData;
	var ContactData = jiglib.ContactData;
	var PlaneData = jiglib.PlaneData;
	var EdgeData = jiglib.EdgeData;
	var TerrainData = jiglib.TerrainData;
	var OctreeCell = jiglib.OctreeCell;
	var CollOutBodyData = jiglib.CollOutBodyData;
	var SpanData = jiglib.SpanData;

	var TriangleVertexIndices = function(_i0, _i1, _i2)
	{
		this.i0 = null; // uint
		this.i1 = null; // uint
		this.i2 = null; // uint

		this.i0 = _i0;
		this.i1 = _i1;
		this.i2 = _i2;
		
	}



	jiglib.TriangleVertexIndices = TriangleVertexIndices; 

})(jiglib);


(function(jiglib) {

	var CollOutData = jiglib.CollOutData;
	var ContactData = jiglib.ContactData;
	var PlaneData = jiglib.PlaneData;
	var EdgeData = jiglib.EdgeData;
	var TerrainData = jiglib.TerrainData;
	var OctreeCell = jiglib.OctreeCell;
	var CollOutBodyData = jiglib.CollOutBodyData;
	var TriangleVertexIndices = jiglib.TriangleVertexIndices;

	var SpanData = function()
	{
		this.min = null; // Number
		this.max = null; // Number
		this.flag = null; // Boolean
		this.depth = null; // Number
	}



	jiglib.SpanData = SpanData; 

})(jiglib);


(function(jiglib) {

	var RigidBody = jiglib.RigidBody;

	var JCollisionEvent = function(type)
	{
		this.body = null; // RigidBody

		
		
	}

	JCollisionEvent.COLLISION_START =  "collisionStart"; // String
	JCollisionEvent.COLLISION_END =  "collisionEnd"; // String


	jiglib.JCollisionEvent = JCollisionEvent; 

})(jiglib);


(function(jiglib) {

	var JConstraintWorldPoint = jiglib.JConstraintWorldPoint;
	var JConstraintMaxDistance = jiglib.JConstraintMaxDistance;
	var JConstraintPoint = jiglib.JConstraintPoint;

	var JConstraint = function()
	{
		this.satisfied = null; // Boolean
		this._constraintEnabled = null; // Boolean

		
	}

	JConstraint.prototype.preApply = function(dt)
	{

		this.satisfied = false;
		
	}

	JConstraint.prototype.apply = function(dt)
	{

		return false;
		
	}

	JConstraint.prototype.enableConstraint = function()
	{

		
	}

	JConstraint.prototype.disableConstraint = function()
	{

		
	}

	JConstraint.prototype.get_constraintEnabled = function()
	{

		return this._constraintEnabled;
		
	}



	jiglib.JConstraint = JConstraint; 

})(jiglib);


(function(jiglib) {

	var JConstraintWorldPoint = jiglib.JConstraintWorldPoint;
	var JConstraintPoint = jiglib.JConstraintPoint;
	var JConstraint = jiglib.JConstraint;
	var Vector3D = jiglib.Vector3D;
	var JMatrix3D = jiglib.JMatrix3D;
	var Matrix3D = jiglib.Matrix3D;
	var JMath3D = jiglib.JMath3D;
	var JNumber3D = jiglib.JNumber3D;
	var RigidBody = jiglib.RigidBody;
	var PhysicsSystem = jiglib.PhysicsSystem;

	var JConstraintMaxDistance = function(body0, body0Pos, body1, body1Pos, maxDistance)
	{
		this._maxVelMag =  20; // Number
		this._minVelForProcessing =  0.01; // Number
		this._body0 = null; // RigidBody
		this._body1 = null; // RigidBody
		this._body0Pos = null; // Vector3D
		this._body1Pos = null; // Vector3D
		this._maxDistance = null; // Number
		this.r0 = null; // Vector3D
		this.r1 = null; // Vector3D
		this._worldPos = null; // Vector3D
		this._currentRelPos0 = null; // Vector3D

		jiglib.JConstraint.apply(this, [  ]);
		this._body0 = body0;
		this._body0Pos = body0Pos;
		this._body1 = body1;
		this._body1Pos = body1Pos;
		this._maxDistance = maxDistance;
		
		this._constraintEnabled = false;
		this.enableConstraint();
		
	}

	jiglib.extend(JConstraintMaxDistance, JConstraint);

	JConstraintMaxDistance.prototype.enableConstraint = function()
	{

		if (this._constraintEnabled)
		{
			return;
		}
		this._constraintEnabled = true;
		this._body0.addConstraint(this);
		this._body1.addConstraint(this);
		PhysicsSystem.getInstance().addConstraint(this);
		
	}

	JConstraintMaxDistance.prototype.disableConstraint = function()
	{

		if (!this._constraintEnabled)
		{
			return;
		}
		this._constraintEnabled = false;
		this._body0.removeConstraint(this);
		this._body1.removeConstraint(this);
		PhysicsSystem.getInstance().removeConstraint(this);
		
	}

	JConstraintMaxDistance.prototype.preApply = function(dt)
	{

		this.satisfied = false;

		this.r0 = this._body0.get_currentState().orientation.transformVector(this._body0Pos);
		this.r1 = this._body1.get_currentState().orientation.transformVector(this._body1Pos);

		var worldPos0, worldPos1;
		worldPos0 = this._body0.get_currentState().position.add(this.r0);
		worldPos1 = this._body1.get_currentState().position.add(this.r1);
		this._worldPos = JNumber3D.getScaleVector(worldPos0.add(worldPos1), 0.5);

		this._currentRelPos0 = worldPos0.subtract(worldPos1);
		
	}

	JConstraintMaxDistance.prototype.apply = function(dt)
	{

		this.satisfied = true;

		if (!this._body0.isActive && !this._body1.isActive)
		{
			return false;
		}

		var clampedRelPos0Mag, normalVel, denominator, tiny=JMath3D.NUM_TINY;
		var currentVel0, currentVel1, predRelPos0, clampedRelPos0, desiredRelVel0, Vr, N, tempVec1, tempVec2, normalImpulse;
		
		currentVel0 = this._body0.getVelocity(this.r0);
		currentVel1 = this._body1.getVelocity(this.r1);

		predRelPos0 = this._currentRelPos0.add(JNumber3D.getScaleVector(currentVel0.subtract(currentVel1), dt));
		clampedRelPos0 = predRelPos0.clone();
		clampedRelPos0Mag = clampedRelPos0.get_length();
		if (clampedRelPos0Mag <= tiny)
		{
			return false;
		}
		if (clampedRelPos0Mag > this._maxDistance)
		{
			clampedRelPos0 = JNumber3D.getScaleVector(clampedRelPos0, this._maxDistance / clampedRelPos0Mag);
		}

		desiredRelVel0 = JNumber3D.getDivideVector(clampedRelPos0.subtract(this._currentRelPos0), dt);
		Vr = currentVel0.subtract(currentVel1).subtract(desiredRelVel0);

		normalVel = Vr.get_length();
		if (normalVel > this._maxVelMag)
		{
			Vr = JNumber3D.getScaleVector(Vr, this._maxVelMag / normalVel);
			normalVel = this._maxVelMag;
		}
		else if (normalVel < this._minVelForProcessing)
		{
			return false;
		}

		N = JNumber3D.getDivideVector(Vr, normalVel);
		tempVec1 = this.r0.crossProduct(N);
		tempVec1 = this._body0.get_worldInvInertia().transformVector(tempVec1);
		tempVec2 = this.r1.crossProduct(N);
		tempVec2 = this._body1.get_worldInvInertia().transformVector(tempVec2);
		denominator = this._body0.get_invMass() + this._body1.get_invMass() + N.dotProduct(tempVec1.crossProduct(this.r0)) + N.dotProduct(tempVec2.crossProduct(this.r1));
		if (denominator < tiny)
		{
			return false;
		}

		normalImpulse = JNumber3D.getScaleVector(N, -normalVel / denominator);
		this._body0.applyWorldImpulse(normalImpulse, this._worldPos, false);
		this._body1.applyWorldImpulse(JNumber3D.getScaleVector(normalImpulse, -1), this._worldPos, false);

		this._body0.setConstraintsAndCollisionsUnsatisfied();
		this._body1.setConstraintsAndCollisionsUnsatisfied();
		this.satisfied = true;
		return true;
		
	}



	jiglib.JConstraintMaxDistance = JConstraintMaxDistance; 

})(jiglib);


(function(jiglib) {

	var JConstraintMaxDistance = jiglib.JConstraintMaxDistance;
	var JConstraintPoint = jiglib.JConstraintPoint;
	var JConstraint = jiglib.JConstraint;
	var Vector3D = jiglib.Vector3D;
	var JMatrix3D = jiglib.JMatrix3D;
	var Matrix3D = jiglib.Matrix3D;
	var JMath3D = jiglib.JMath3D;
	var JNumber3D = jiglib.JNumber3D;
	var RigidBody = jiglib.RigidBody;
	var PhysicsSystem = jiglib.PhysicsSystem;
	var CollisionInfo = jiglib.CollisionInfo;

	var JConstraintWorldPoint = function(body, pointOnBody, worldPosition)
	{
		this.minVelForProcessing =  0.001; // Number
		this.allowedDeviation =  0.01; // Number
		this.timescale =  4; // Number
		this._body = null; // RigidBody
		this._pointOnBody = null; // Vector3D
		this._worldPosition = null; // Vector3D

		jiglib.JConstraint.apply(this, [  ]);
		this._body = body;
		this._pointOnBody = pointOnBody;
		this._worldPosition = worldPosition;
		
		this._constraintEnabled = false;
		this.enableConstraint();
		
	}

	jiglib.extend(JConstraintWorldPoint, JConstraint);

	JConstraintWorldPoint.prototype.set_worldPosition = function(pos)
	{

		this._worldPosition = pos;
		
	}

	JConstraintWorldPoint.prototype.get_worldPosition = function()
	{

		return this._worldPosition;
		
	}

	JConstraintWorldPoint.prototype.enableConstraint = function()
	{

		if (this._constraintEnabled)
		{
			return;
		}
		this._constraintEnabled = true;
		this._body.addConstraint(this);
		PhysicsSystem.getInstance().addConstraint(this);
		
	}

	JConstraintWorldPoint.prototype.disableConstraint = function()
	{

		if (!this._constraintEnabled)
		{
			return;
		}
		this._constraintEnabled = false;
		this._body.removeConstraint(this);
		PhysicsSystem.getInstance().removeConstraint(this);
		
	}

	JConstraintWorldPoint.prototype.apply = function(dt)
	{

		this.satisfied = true;

		var deviationDistance, normalVel, denominator, normalImpulse, dot;
		var worldPos, R, currentVel, desiredVel, deviationDir, deviation, N, tempV;
		
		worldPos = this._body.get_currentState().orientation.transformVector(this._pointOnBody);
		worldPos = worldPos.add( this._body.get_currentState().position);
		R = worldPos.subtract(this._body.get_currentState().position);
		currentVel = this._body.get_currentState().linVelocity.add(this._body.get_currentState().rotVelocity.crossProduct(R));
		
		deviation = worldPos.subtract(this._worldPosition);
		deviationDistance = deviation.get_length();
		if (deviationDistance > this.allowedDeviation) {
			deviationDir = JNumber3D.getDivideVector(deviation, deviationDistance);
			desiredVel = JNumber3D.getScaleVector(deviationDir, (this.allowedDeviation - deviationDistance) / (this.timescale * dt));
		} else {
			desiredVel = new Vector3D();
		}
		
		N = currentVel.subtract(desiredVel);
		normalVel = N.get_length();
		if (normalVel < this.minVelForProcessing) {
			return false;
		}
		N = JNumber3D.getDivideVector(N, normalVel);
		
		tempV = R.crossProduct(N);
		tempV = this._body.get_worldInvInertia().transformVector(tempV);
		denominator = this._body.get_invMass() + N.dotProduct(tempV.crossProduct(R));
		 
		if (denominator < JMath3D.NUM_TINY) {
			return false;
		}
		 
		normalImpulse = -normalVel / denominator;
		
		this._body.applyWorldImpulse(JNumber3D.getScaleVector(N, normalImpulse), worldPos, false);
		
		this._body.setConstraintsAndCollisionsUnsatisfied();
		this.satisfied = true;
		
		return true;
		
	}



	jiglib.JConstraintWorldPoint = JConstraintWorldPoint; 

})(jiglib);


(function(jiglib) {

	var JConstraintWorldPoint = jiglib.JConstraintWorldPoint;
	var JConstraintMaxDistance = jiglib.JConstraintMaxDistance;
	var JConstraint = jiglib.JConstraint;
	var Vector3D = jiglib.Vector3D;
	var JMatrix3D = jiglib.JMatrix3D;
	var Matrix3D = jiglib.Matrix3D;
	var JMath3D = jiglib.JMath3D;
	var JNumber3D = jiglib.JNumber3D;
	var RigidBody = jiglib.RigidBody;
	var PhysicsSystem = jiglib.PhysicsSystem;

	var JConstraintPoint = function(body0, body0Pos, body1, body1Pos, allowedDistance, timescale)
	{
		this._maxVelMag =  20; // Number
		this._minVelForProcessing =  0.01; // Number
		this._body0 = null; // RigidBody
		this._body1 = null; // RigidBody
		this._body0Pos = null; // Vector3D
		this._body1Pos = null; // Vector3D
		this._timescale = null; // Number
		this._allowedDistance = null; // Number
		this.r0 = null; // Vector3D
		this.r1 = null; // Vector3D
		this._worldPos = null; // Vector3D
		this._vrExtra = null; // Vector3D

		jiglib.JConstraint.apply(this, [  ]);
		this._body0 = body0;
		this._body0Pos = body0Pos;
		this._body1 = body1;
		this._body1Pos = body1Pos;
		this._allowedDistance = allowedDistance;
		this._timescale = timescale;
		if (this._timescale < JMath3D.NUM_TINY)
		{
			this._timescale = JMath3D.NUM_TINY;
		}
		
		this._constraintEnabled = false;
		this.enableConstraint();
		
	}

	jiglib.extend(JConstraintPoint, JConstraint);

	JConstraintPoint.prototype.enableConstraint = function()
	{

		if (this._constraintEnabled)
		{
			return;
		}
		this._constraintEnabled = true;
		this._body0.addConstraint(this);
		this._body1.addConstraint(this);
		PhysicsSystem.getInstance().addConstraint(this);
		
	}

	JConstraintPoint.prototype.disableConstraint = function()
	{

		if (!this._constraintEnabled)
		{
			return;
		}
		this._constraintEnabled = false;
		this._body0.removeConstraint(this);
		this._body1.removeConstraint(this);
		PhysicsSystem.getInstance().removeConstraint(this);
		
	}

	JConstraintPoint.prototype.preApply = function(dt)
	{

		this.satisfied = false;

		this.r0 = this._body0.get_currentState().orientation.transformVector(this._body0Pos);
		this.r1 = this._body1.get_currentState().orientation.transformVector(this._body1Pos);

		var worldPos0, worldPos1, deviation, deviationAmount;
		worldPos0 = this._body0.get_currentState().position.add(this.r0);
		worldPos1 = this._body1.get_currentState().position.add(this.r1);
		this._worldPos = JNumber3D.getScaleVector(worldPos0.add(worldPos1), 0.5);

		deviation = worldPos0.subtract(worldPos1);
		deviationAmount = deviation.get_length();
		if (deviationAmount > this._allowedDistance)
		{
			this._vrExtra = JNumber3D.getScaleVector(deviation, (deviationAmount - this._allowedDistance) / (deviationAmount * Math.max(this._timescale, dt)));
		}
		else
		{
			this._vrExtra = new Vector3D();
		}
		
	}

	JConstraintPoint.prototype.apply = function(dt)
	{

		this.satisfied = true;

		if (!this._body0.isActive && !this._body1.isActive)
		{
			return false;
		}

		var normalVel, denominator;
		var currentVel0, currentVel1, Vr, N, tempVec1, tempVec2, normalImpulse;
		currentVel0 = this._body0.getVelocity(this.r0);
		currentVel1 = this._body1.getVelocity(this.r1);
		Vr = this._vrExtra.add(currentVel0.subtract(currentVel1));

		normalVel = Vr.get_length();
		if (normalVel < this._minVelForProcessing)
		{
			return false;
		}

		if (normalVel > this._maxVelMag)
		{
			Vr = JNumber3D.getScaleVector(Vr, this._maxVelMag / normalVel);
			normalVel = this._maxVelMag;
		}

		N = JNumber3D.getDivideVector(Vr, normalVel);
		tempVec1 = this.r0.crossProduct(N);
		tempVec1 = this._body0.get_worldInvInertia().transformVector(tempVec1);
		tempVec2 = this.r1.crossProduct(N);
		tempVec2 = this._body1.get_worldInvInertia().transformVector(tempVec2);
		denominator = this._body0.get_invMass() + this._body1.get_invMass() + N.dotProduct(tempVec1.crossProduct(this.r0)) + N.dotProduct(tempVec2.crossProduct(this.r1));
		if (denominator < JMath3D.NUM_TINY)
		{
			return false;
		}

		normalImpulse = JNumber3D.getScaleVector(N, -normalVel / denominator);
		this._body0.applyWorldImpulse(normalImpulse, this._worldPos, false);
		this._body1.applyWorldImpulse(JNumber3D.getScaleVector(normalImpulse, -1), this._worldPos, false);

		this._body0.setConstraintsAndCollisionsUnsatisfied();
		this._body1.setConstraintsAndCollisionsUnsatisfied();
		this.satisfied = true;
		return true;
		
	}



	jiglib.JConstraintPoint = JConstraintPoint; 

})(jiglib);


(function(jiglib) {

	var PhysicsController = jiglib.PhysicsController;
	var CachedImpulse = jiglib.CachedImpulse;
	var PhysicsState = jiglib.PhysicsState;
	var RigidBody = jiglib.RigidBody;
	var HingeJoint = jiglib.HingeJoint;
	var BodyPair = jiglib.BodyPair;
	var PhysicsSystem = jiglib.PhysicsSystem;

	var MaterialProperties = function(_restitution, _friction)
	{
		this.restitution = null; // Number
		this.friction = null; // Number

		this.restitution = _restitution;
		this.friction = _friction;
		
	}



	jiglib.MaterialProperties = MaterialProperties; 

})(jiglib);


(function(jiglib) {

	var MaterialProperties = jiglib.MaterialProperties;
	var CachedImpulse = jiglib.CachedImpulse;
	var PhysicsState = jiglib.PhysicsState;
	var RigidBody = jiglib.RigidBody;
	var HingeJoint = jiglib.HingeJoint;
	var BodyPair = jiglib.BodyPair;
	var PhysicsSystem = jiglib.PhysicsSystem;

	var PhysicsController = function()
	{
		this._controllerEnabled = null; // Boolean

		this._controllerEnabled = false;
		
	}

	PhysicsController.prototype.updateController = function(dt)
	{

		
	}

	PhysicsController.prototype.enableController = function()
	{

		
	}

	PhysicsController.prototype.disableController = function()
	{

		
	}

	PhysicsController.prototype.get_controllerEnabled = function()
	{

		return this._controllerEnabled;
		
	}



	jiglib.PhysicsController = PhysicsController; 

})(jiglib);


(function(jiglib) {

	var MaterialProperties = jiglib.MaterialProperties;
	var PhysicsController = jiglib.PhysicsController;
	var PhysicsState = jiglib.PhysicsState;
	var RigidBody = jiglib.RigidBody;
	var HingeJoint = jiglib.HingeJoint;
	var BodyPair = jiglib.BodyPair;
	var PhysicsSystem = jiglib.PhysicsSystem;
	var Vector3D = jiglib.Vector3D;

	var CachedImpulse = function(_normalImpulse, _normalImpulseAux, _frictionImpulse)
	{
		this.normalImpulse = null; // Number
		this.normalImpulseAux = null; // Number
		this.frictionImpulse = null; // Vector3D

		this.normalImpulse = _normalImpulse;
		this.normalImpulseAux = _normalImpulseAux;
		this.frictionImpulse = _frictionImpulse;
		
	}



	jiglib.CachedImpulse = CachedImpulse; 

})(jiglib);


(function(jiglib) {

	var MaterialProperties = jiglib.MaterialProperties;
	var PhysicsController = jiglib.PhysicsController;
	var CachedImpulse = jiglib.CachedImpulse;
	var RigidBody = jiglib.RigidBody;
	var HingeJoint = jiglib.HingeJoint;
	var BodyPair = jiglib.BodyPair;
	var PhysicsSystem = jiglib.PhysicsSystem;
	var Matrix3D = jiglib.Matrix3D;
	var Vector3D = jiglib.Vector3D;
	var JMatrix3D = jiglib.JMatrix3D;
	var JMath3D = jiglib.JMath3D;
	var JNumber3D = jiglib.JNumber3D;

	var PhysicsState = function()
	{
		this.position =  new Vector3D(); // Vector3D
		this.orientation =  new Matrix3D(); // Matrix3D
		this.linVelocity =  new Vector3D(); // Vector3D
		this.rotVelocity =  new Vector3D(); // Vector3D
		this.orientationCols =  []; // Vector3D

		//this.orientationCols[0] = new Vector3D();
		//this.orientationCols[1] = new Vector3D();
		//this.orientationCols[2] = new Vector3D();
		
	}

	PhysicsState.prototype.getOrientationCols = function()
	{

		return JMatrix3D.getCols(this.orientation);
		
	}



	jiglib.PhysicsState = PhysicsState; 

})(jiglib);


(function(jiglib) {

	var MaterialProperties = jiglib.MaterialProperties;
	var PhysicsController = jiglib.PhysicsController;
	var CachedImpulse = jiglib.CachedImpulse;
	var PhysicsState = jiglib.PhysicsState;
	var HingeJoint = jiglib.HingeJoint;
	var BodyPair = jiglib.BodyPair;
	var PhysicsSystem = jiglib.PhysicsSystem;
	var Matrix3D = jiglib.Matrix3D;
	var Vector3D = jiglib.Vector3D;
	var JConfig = jiglib.JConfig;
	var CollisionInfo = jiglib.CollisionInfo;
	var CollisionSystemAbstract = jiglib.CollisionSystemAbstract;
	var CollisionSystemGridEntry = jiglib.CollisionSystemGridEntry;
	var CollOutData = jiglib.CollOutData;
	var JAABox = jiglib.JAABox;
	var JSegment = jiglib.JSegment;
	var JMatrix3D = jiglib.JMatrix3D;
	var JMath3D = jiglib.JMath3D;
	var JNumber3D = jiglib.JNumber3D;
	var JCollisionEvent = jiglib.JCollisionEvent;
	var JConstraint = jiglib.JConstraint;

	var RigidBody = function(skin)
	{
		this._id = null; // int
		this._skin = null; // ISkin3D
		this._type = null; // String
		this._boundingSphere = null; // Number
		this._boundingBox = null; // JAABox
		this._currState = null; // PhysicsState
		this._oldState = null; // PhysicsState
		this._storeState = null; // PhysicsState
		this._invOrientation = null; // Matrix3D
		this._currLinVelocityAux = null; // Vector3D
		this._currRotVelocityAux = null; // Vector3D
		this._mass = null; // Number
		this._invMass = null; // Number
		this._bodyInertia = null; // Matrix3D
		this._bodyInvInertia = null; // Matrix3D
		this._worldInertia = null; // Matrix3D
		this._worldInvInertia = null; // Matrix3D
		this._force = null; // Vector3D
		this._torque = null; // Vector3D
		this._linVelDamping = null; // Vector3D
		this._rotVelDamping = null; // Vector3D
		this._maxLinVelocities = null; // Vector3D
		this._maxRotVelocities = null; // Vector3D
		this._movable = null; // Boolean
		this._origMovable = null; // Boolean
		this._inactiveTime = null; // Number
		this._bodiesToBeActivatedOnMovement = null; // RigidBody
		this._storedPositionForActivation = null; // Vector3D
		this._lastPositionForDeactivation = null; // Vector3D
		this._lastOrientationForDeactivation = null; // Matrix3D
		this._material = null; // MaterialProperties
		this._rotationX =  0; // Number
		this._rotationY =  0; // Number
		this._rotationZ =  0; // Number
		this._useDegrees = null; // Boolean
		this._nonCollidables = null; // RigidBody
		this._collideBodies = null; // RigidBody
		this._constraints = null; // JConstraint
		this._gravity = null; // Vector3D
		this._gravityAxis = null; // int
		this._gravityForce = null; // Vector3D
		this.collisions = null; // CollisionInfo
		this.externalData = null; // CollisionSystemGridEntry
		this.collisionSystem = null; // CollisionSystemAbstract
		this.isActive = null; // Boolean

		this._useDegrees = (JConfig.rotationType == "DEGREES") ? true : false;

		this._id = RigidBody.idCounter++;

		this._skin = skin;
		this._material = new MaterialProperties();

		this._bodyInertia = new Matrix3D();
		this._bodyInvInertia = JMatrix3D.getInverseMatrix(this._bodyInertia);

		this._currState = new PhysicsState();
		this._oldState = new PhysicsState();
		this._storeState = new PhysicsState();
		this._currLinVelocityAux = new Vector3D();
		this._currRotVelocityAux = new Vector3D();

		this._force = new Vector3D();
		this._torque = new Vector3D();

		this._invOrientation = JMatrix3D.getInverseMatrix(this._currState.orientation);
		this._linVelDamping = new Vector3D(0.999, 0.999, 0.999);
		this._rotVelDamping = new Vector3D(0.999, 0.999, 0.999);
		this._maxLinVelocities = new Vector3D(JMath3D.NUM_HUGE,JMath3D.NUM_HUGE,JMath3D.NUM_HUGE);
		this._maxRotVelocities = new Vector3D(JMath3D.NUM_HUGE,JMath3D.NUM_HUGE,JMath3D.NUM_HUGE);

		this._inactiveTime = 0;
		this.isActive = true;
		this._movable = true;
		this._origMovable = true;

		this.collisions = [];
		this._constraints = [];
		this._nonCollidables = [];
		this._collideBodies = [];

		this._storedPositionForActivation = new Vector3D();
		this._bodiesToBeActivatedOnMovement = [];
		this._lastPositionForDeactivation = this._currState.position.clone();
		this._lastOrientationForDeactivation = this._currState.orientation.clone();

		this._type = "Object3D";
		this._boundingSphere = JMath3D.NUM_HUGE;
		this._boundingBox = new JAABox();
		
		this.externalData=null;
		
	}

	RigidBody.prototype.radiansToDegrees = function(rad)
	{

		return rad * 180 / Math.PI;
		
	}

	RigidBody.prototype.degreesToRadians = function(deg)
	{

		return deg * Math.PI / 180;
		
	}

	RigidBody.prototype.updateRotationValues = function()
	{

		var rotationVector = this._currState.orientation.decompose()[1];
		this._rotationX = RigidBody.formatRotation(this.radiansToDegrees(rotationVector.x));
		this._rotationY = RigidBody.formatRotation(this.radiansToDegrees(rotationVector.y));
		this._rotationZ = RigidBody.formatRotation(this.radiansToDegrees(rotationVector.z));
		
	}

	RigidBody.prototype.get_rotationX = function()
	{

		return this._rotationX; //(this._useDegrees) ? this.radiansToDegrees(this._rotationX) : this._rotationX;
		
	}

	RigidBody.prototype.get_rotationY = function()
	{

		return this._rotationY; //(this._useDegrees) ? this.radiansToDegrees(this._rotationY) : this._rotationY;
		
	}

	RigidBody.prototype.get_rotationZ = function()
	{

		return this._rotationZ; //(this._useDegrees) ? this.radiansToDegrees(this._rotationZ) : this._rotationZ;
		
	}

	RigidBody.prototype.set_rotationX = function(px)
	{

		//var rad = (this._useDegrees) ? this.degreesToRadians(px) : px;
		this._rotationX = px;
		this.setOrientation(this.createRotationMatrix());
		
	}

	RigidBody.prototype.set_rotationY = function(py)
	{

		//var rad = (this._useDegrees) ? this.degreesToRadians(py) : py;
		this._rotationY = py;
		this.setOrientation(this.createRotationMatrix());
		
	}

	RigidBody.prototype.set_rotationZ = function(pz)
	{

		//var rad = (this._useDegrees) ? this.degreesToRadians(pz) : pz;
		this._rotationZ = pz;
		this.setOrientation(this.createRotationMatrix());
		
	}

	RigidBody.prototype.pitch = function(rot)
	{

		this.setOrientation(JMatrix3D.getAppendMatrix3D(this.get_currentState().orientation, JMatrix3D.getRotationMatrixAxis(rot, Vector3D.X_AXIS)));
		
	}

	RigidBody.prototype.yaw = function(rot)
	{

		this.setOrientation(JMatrix3D.getAppendMatrix3D(this.get_currentState().orientation, JMatrix3D.getRotationMatrixAxis(rot, Vector3D.Y_AXIS)));
		
	}

	RigidBody.prototype.roll = function(rot)
	{

		this.setOrientation(JMatrix3D.getAppendMatrix3D(this.get_currentState().orientation, JMatrix3D.getRotationMatrixAxis(rot, Vector3D.Z_AXIS)));
		
	}

	RigidBody.prototype.createRotationMatrix = function()
	{

		var matrix3D = new Matrix3D();
		matrix3D.appendRotation(this._rotationX, Vector3D.X_AXIS);
		matrix3D.appendRotation(this._rotationY, Vector3D.Y_AXIS);
		matrix3D.appendRotation(this._rotationZ, Vector3D.Z_AXIS);
		return matrix3D;
		
	}

	RigidBody.prototype.setOrientation = function(orient)
	{

		this._currState.orientation = orient.clone();
		this.updateInertia();
		this.updateState();
		
	}

	RigidBody.prototype.get_x = function()
	{

		return this._currState.position.x;
		
	}

	RigidBody.prototype.get_y = function()
	{

		return this._currState.position.y;
		
	}

	RigidBody.prototype.get_z = function()
	{

		return this._currState.position.z;
		
	}

	RigidBody.prototype.set_x = function(px)
	{

		this._currState.position.x = px;
		this.updateState();
		
	}

	RigidBody.prototype.set_y = function(py)
	{

		this._currState.position.y = py;
		this.updateState();
		
	}

	RigidBody.prototype.set_z = function(pz)
	{

		this._currState.position.z = pz;
		this.updateState();
		
	}

	RigidBody.prototype.moveTo = function(pos)
	{

		this._currState.position = pos.clone();
		this.updateState();
		
	}

	RigidBody.prototype.updateState = function()
	{

		this._currState.linVelocity.setTo(0,0,0);
		this._currState.rotVelocity.setTo(0,0,0);
		this.copyCurrentStateToOld();
		this.updateBoundingBox(); // todo: is making invalid boundingboxes, shouldn't this only be update when it's scaled?
		this.updateObject3D();
		
		if (this.collisionSystem) {
			this.collisionSystem.collisionSkinMoved(this);
		}
		
	}

	RigidBody.prototype.setLineVelocity = function(vel)
	{

		this._currState.linVelocity = vel.clone();
		
	}

	RigidBody.prototype.setAngleVelocity = function(angVel)
	{

		this._currState.rotVelocity = angVel.clone();
		
	}

	RigidBody.prototype.setLineVelocityAux = function(vel)
	{

		this._currLinVelocityAux = vel.clone();
		
	}

	RigidBody.prototype.setAngleVelocityAux = function(angVel)
	{

		this._currRotVelocityAux = angVel.clone();
		
	}

	RigidBody.prototype.updateGravity = function(gravity, gravityAxis)
	{

		this._gravity = gravity;
		this._gravityAxis = gravityAxis;
		
		this._gravityForce = JNumber3D.getScaleVector(this._gravity, this._mass);
		
	}

	RigidBody.prototype.addWorldTorque = function(t, active)
	{
		if (active == null) active = true;

		if (!this._movable)
		{
			return;
		}
		this._torque = this._torque.add(t);
		
		if (active) this.setActive();
		
	}

	RigidBody.prototype.addBodyTorque = function(t, active)
	{
		if (active == null) active = true;

		if (!this._movable)
			return;

		this.addWorldTorque(this._currState.orientation.transformVector(t), active);
		
	}

	RigidBody.prototype.addWorldForce = function(f, p, active)
	{
		if (active == null) active = true;

		if (!this._movable)
			return;

		this._force = this._force.add(f);
		this.addWorldTorque(p.subtract(this._currState.position).crossProduct(f));
		
		if (active) this.setActive();
		
	}

	RigidBody.prototype.addBodyForce = function(f, p, active)
	{
		if (active == null) active = true;

		if (!this._movable)
			return;

		f = this._currState.orientation.transformVector(f);
		p = this._currState.orientation.transformVector(p);
		this.addWorldForce(f, this._currState.position.add(p), active);
		
	}

	RigidBody.prototype.clearForces = function()
	{

		this._force.setTo(0,0,0);
		this._torque.setTo(0,0,0);
		
	}

	RigidBody.prototype.applyWorldImpulse = function(impulse, pos, active)
	{
		if (active == null) active = true;

		if (!this._movable)
		{
			return;
		}
		this._currState.linVelocity = this._currState.linVelocity.add(JNumber3D.getScaleVector(impulse, this._invMass));

		var rotImpulse = pos.subtract(this._currState.position).crossProduct(impulse);
		rotImpulse = this._worldInvInertia.transformVector(rotImpulse);
		this._currState.rotVelocity = this._currState.rotVelocity.add(rotImpulse);

		if (active) this.setActive();
		
	}

	RigidBody.prototype.applyWorldImpulseAux = function(impulse, pos, active)
	{
		if (active == null) active = true;

		if (!this._movable)
		{
			return;
		}
		this._currLinVelocityAux = this._currLinVelocityAux.add(JNumber3D.getScaleVector(impulse, this._invMass));

		var rotImpulse = pos.subtract(this._currState.position).crossProduct(impulse);
		rotImpulse = this._worldInvInertia.transformVector(rotImpulse);
		this._currRotVelocityAux = this._currRotVelocityAux.add(rotImpulse);

		if (active) this.setActive();
		
	}

	RigidBody.prototype.applyBodyWorldImpulse = function(impulse, delta, active)
	{
		if (active == null) active = true;

		if (!this._movable)
		{
			return;
		}
		this._currState.linVelocity = this._currState.linVelocity.add(JNumber3D.getScaleVector(impulse, this._invMass));

		var rotImpulse = delta.crossProduct(impulse);
		rotImpulse = this._worldInvInertia.transformVector(rotImpulse);
		this._currState.rotVelocity = this._currState.rotVelocity.add(rotImpulse);

		if (active) this.setActive();
		
	}

	RigidBody.prototype.applyBodyWorldImpulseAux = function(impulse, delta, active)
	{
		if (active == null) active = true;

		if (!this._movable)
		{
			return;
		}
		this._currLinVelocityAux = this._currLinVelocityAux.add(JNumber3D.getScaleVector(impulse, this._invMass));

		var rotImpulse = delta.crossProduct(impulse);
		rotImpulse = this._worldInvInertia.transformVector(rotImpulse);
		this._currRotVelocityAux = this._currRotVelocityAux.add(rotImpulse);

		if (active) this.setActive();
		
	}

	RigidBody.prototype.updateVelocity = function(dt)
	{

		if (!this._movable || !this.isActive)
			return;

		this._currState.linVelocity = this._currState.linVelocity.add(JNumber3D.getScaleVector(this._force, this._invMass * dt));

		var rac = JNumber3D.getScaleVector(this._torque, dt);
		rac = this._worldInvInertia.transformVector(rac);
		this._currState.rotVelocity = this._currState.rotVelocity.add(rac);
		
	}

	RigidBody.prototype.updatePosition = function(dt)
	{

		   if (!this._movable || !this.isActive)
		   {
		   return;
		   }

		   var angMomBefore = this._currState.rotVelocity.clone();
		   angMomBefore = this._worldInertia.transformVector(angMomBefore);

		   this._currState.position = this._currState.position.add(JNumber3D.getScaleVector(this._currState.linVelocity, dt));

		   var dir = this._currState.rotVelocity.clone();
		   var ang = dir.get_length();
		   if (ang > 0)
		   {
		   dir.normalize();
		   ang *= dt;
		   var rot = JMatrix3D.rotationMatrix(dir.x, dir.y, dir.z, ang);
		   this._currState.orientation = JMatrix3D.getMatrix3D(JMatrix3D.multiply(rot, JMatrix3D.getJMatrix3D(this._currState.orientation)));
		   this.updateInertia();
		   }

		   angMomBefore = this._worldInvInertia.transformVector(angMomBefore);
		   this._currState.rotVelocity = angMomBefore.clone();
		   
	}

	RigidBody.prototype.updatePositionWithAux = function(dt)
	{

		if (!this._movable || !this.isActive)
		{
			this._currLinVelocityAux.setTo(0,0,0);
			this._currRotVelocityAux.setTo(0,0,0);
			return;
		}

		var ga = this._gravityAxis;
		if (ga != -1)
		{
			var arr = JNumber3D.toArray(this._currLinVelocityAux);
			arr[(ga + 1) % 3] *= 0.1;
			arr[(ga + 2) % 3] *= 0.1;
			JNumber3D.copyFromArray(this._currLinVelocityAux, arr);
		}

		this._currState.position = this._currState.position.add(JNumber3D.getScaleVector(this._currState.linVelocity.add(this._currLinVelocityAux), dt));

		var dir = this._currState.rotVelocity.add(this._currRotVelocityAux);
		var ang = dir.get_length() * 180 / Math.PI;
		if (ang > 0)
		{
			dir.normalize();
			ang *= dt;


			var rot = JMatrix3D.getRotationMatrix(dir.x, dir.y, dir.z, ang);
			this._currState.orientation = JMatrix3D.getAppendMatrix3D(this._currState.orientation, rot);

			this.updateInertia();
		}

		this._currLinVelocityAux.setTo(0,0,0);
		this._currRotVelocityAux.setTo(0,0,0);

		
	}

	RigidBody.prototype.tryToFreeze = function(dt)
	{

		if (!this._movable || !this.isActive)
		{
			return;
		}
		
		if (this._currState.position.subtract(this._lastPositionForDeactivation).get_length() > JConfig.posThreshold)
		{
			this._lastPositionForDeactivation = this._currState.position.clone();
			this._inactiveTime = 0;
			return;
		}

		var ot = JConfig.orientThreshold;
		var deltaMat = JMatrix3D.getSubMatrix(this._currState.orientation, this._lastOrientationForDeactivation);

		var cols = JMatrix3D.getCols(deltaMat);

		if (cols[0].get_length() > ot || cols[1].get_length() > ot || cols[2].get_length() > ot)
		{
			this._lastOrientationForDeactivation = this._currState.orientation.clone();
			this._inactiveTime = 0;
			return;
		}

		if (this.getShouldBeActive())
		{
			return;
		}

		this._inactiveTime += dt;
		if (this._inactiveTime > JConfig.deactivationTime)
		{
			this._lastPositionForDeactivation = this._currState.position.clone();
			this._lastOrientationForDeactivation = this._currState.orientation.clone();
			this.setInactive();
		}
		
	}

	RigidBody.prototype.postPhysics = function(dt)
	{

		if (!this._movable || !this.isActive)
		{
			return;
		}
		
		this.limitVel();
		this.limitAngVel();
		
		this.updatePositionWithAux(dt);
		this.updateBoundingBox(); // todo: is making invalid boundingboxes, shouldn't this only be update when it's scaled?
		this.updateObject3D();
		
		if (this.collisionSystem) {
			this.collisionSystem.collisionSkinMoved(this);
		}
		
		this.clearForces();
		
		//add gravity
		this._force = this._force.add(this._gravityForce);
		
	}

	RigidBody.prototype.set_mass = function(m)
	{

		this._mass = m;
		this._invMass = 1 / m;
		this.setInertia(this.getInertiaProperties(m));
		
		// this.get_mass() is dirty have to recalculate gravity this.get_force()
		var physicsSystem = PhysicsSystem.getInstance();
		this.updateGravity(physicsSystem.get_gravity(), physicsSystem.get_gravityAxis());
		
	}

	RigidBody.prototype.setInertia = function(matrix3D)
	{

		this._bodyInertia = matrix3D.clone();
		this._bodyInvInertia = JMatrix3D.getInverseMatrix(this._bodyInertia.clone());

		this.updateInertia();
		
	}

	RigidBody.prototype.updateInertia = function()
	{

		this._invOrientation = JMatrix3D.getTransposeMatrix(this._currState.orientation);

		this._worldInertia = JMatrix3D.getAppendMatrix3D(this._invOrientation, JMatrix3D.getAppendMatrix3D(this._currState.orientation, this._bodyInertia));

		this._worldInvInertia = JMatrix3D.getAppendMatrix3D(this._invOrientation, JMatrix3D.getAppendMatrix3D(this._currState.orientation, this._bodyInvInertia));
		
	}

	RigidBody.prototype.get_movable = function()
	{

		return this._movable;
		
	}

	RigidBody.prototype.set_movable = function(mov)
	{

		if (this._type == "PLANE" || this._type == "TERRAIN" || this._type == "TRIANGLEMESH")
			return;
		
		this._movable = mov;
		this.isActive = mov;
		this._origMovable = mov;
		
	}

	RigidBody.prototype.internalSetImmovable = function()
	{

		this._origMovable = this._movable;
		this._movable = false;
		
	}

	RigidBody.prototype.internalRestoreImmovable = function()
	{

		this._movable = this._origMovable;
		
	}

	RigidBody.prototype.setActive = function()
	{

		if (this._movable)
		{
			if (this.isActive) return;
			this._inactiveTime = 0;
			this.isActive = true;
		}
		
	}

	RigidBody.prototype.setInactive = function()
	{

		if (this._movable) {
			this._inactiveTime = JConfig.deactivationTime;
			this.isActive = false;
		}
		
	}

	RigidBody.prototype.getVelocity = function(relPos)
	{

		return this._currState.linVelocity.add(this._currState.rotVelocity.crossProduct(relPos));
		
	}

	RigidBody.prototype.getVelocityAux = function(relPos)
	{

		return this._currLinVelocityAux.add(this._currRotVelocityAux.crossProduct(relPos));
		
	}

	RigidBody.prototype.getShouldBeActive = function()
	{

		return ((this._currState.linVelocity.get_length() > JConfig.velThreshold) || (this._currState.rotVelocity.get_length() > JConfig.angVelThreshold));
		
	}

	RigidBody.prototype.getShouldBeActiveAux = function()
	{

		return ((this._currLinVelocityAux.get_length() > JConfig.velThreshold) || (this._currRotVelocityAux.get_length() > JConfig.angVelThreshold));
		
	}

	RigidBody.prototype.dampForDeactivation = function()
	{

		this._currState.linVelocity.x *= this._linVelDamping.x;
		this._currState.linVelocity.y *= this._linVelDamping.y;
		this._currState.linVelocity.z *= this._linVelDamping.z;
		this._currState.rotVelocity.x *= this._rotVelDamping.x;
		this._currState.rotVelocity.y *= this._rotVelDamping.y;
		this._currState.rotVelocity.z *= this._rotVelDamping.z;

		this._currLinVelocityAux.x *= this._linVelDamping.x;
		this._currLinVelocityAux.y *= this._linVelDamping.y;
		this._currLinVelocityAux.z *= this._linVelDamping.z;
		this._currRotVelocityAux.x *= this._rotVelDamping.x;
		this._currRotVelocityAux.y *= this._rotVelDamping.y;
		this._currRotVelocityAux.z *= this._rotVelDamping.z;

		var r = 0.5;
		var frac = this._inactiveTime / JConfig.deactivationTime;
		if (frac < r)
			return;

		var scale = 1 - ((frac - r) / (1 - r));
		if (scale < 0)
		{
			scale = 0;
		}
		else if (scale > 1)
		{
			scale = 1;
		}

		this._currState.linVelocity.scaleBy(scale);
		this._currState.rotVelocity.scaleBy(scale);
		
	}

	RigidBody.prototype.doMovementActivations = function(physicsSystem)
	{

		if (this._bodiesToBeActivatedOnMovement.length == 0 || this._currState.position.subtract(this._storedPositionForActivation).get_length() < JConfig.posThreshold)
			return;

		for (var _bodiesToBeActivatedOnMovement_i = 0, _bodiesToBeActivatedOnMovement_l = this._bodiesToBeActivatedOnMovement.length, body; (_bodiesToBeActivatedOnMovement_i < _bodiesToBeActivatedOnMovement_l) && (body = this._bodiesToBeActivatedOnMovement[_bodiesToBeActivatedOnMovement_i]); _bodiesToBeActivatedOnMovement_i++)
		{
			physicsSystem.activateObject(body);
		}

		this._bodiesToBeActivatedOnMovement.length=0;
		
	}

	RigidBody.prototype.addMovementActivation = function(pos, otherBody)
	{

		if (this._bodiesToBeActivatedOnMovement.indexOf(otherBody) > -1)
			return;

		if (this._bodiesToBeActivatedOnMovement.length == 0)
			this._storedPositionForActivation = pos;

		this._bodiesToBeActivatedOnMovement.push(otherBody);
		
	}

	RigidBody.prototype.setConstraintsAndCollisionsUnsatisfied = function()
	{

		for (var _constraints_i = 0, _constraints_l = this._constraints.length, _constraint; (_constraints_i < _constraints_l) && (_constraint = this._constraints[_constraints_i]); _constraints_i++)
			_constraint.satisfied = false;

		for (var collisions_i = 0, collisions_l = this.collisions.length, _collision; (collisions_i < collisions_l) && (_collision = this.collisions[collisions_i]); collisions_i++)
			_collision.satisfied = false;
		
	}

	RigidBody.prototype.segmentIntersect = function(out, seg, state)
	{

		return false;
		
	}

	RigidBody.prototype.getInertiaProperties = function(m)
	{

		return new Matrix3D();
		
	}

	RigidBody.prototype.updateBoundingBox = function()
	{

		
	}

	RigidBody.prototype.hitTestObject3D = function(obj3D)
	{

		var num1, num2;
		num1 = this._currState.position.subtract(obj3D.get_currentState().position).get_length();
		num2 = this._boundingSphere + obj3D.get_boundingSphere();

		if (num1 <= num2)
			return true;

		return false;
		
	}

	RigidBody.prototype.disableCollisions = function(body)
	{

		if (this._nonCollidables.indexOf(body) < 0)
			this._nonCollidables.push(body);
		
	}

	RigidBody.prototype.enableCollisions = function(body)
	{

		if (this._nonCollidables.indexOf(body) >= 0)
			this._nonCollidables.splice(this._nonCollidables.indexOf(body), 1);
		
	}

	RigidBody.prototype.addCollideBody = function(body)
	{

		if (this._collideBodies.indexOf(body) < 0) {
			
			this._collideBodies.push(body);
			
			
			
			
		}
		
	}

	RigidBody.prototype.removeCollideBodies = function(body)
	{

		var i = this._collideBodies.indexOf(body);
		if (i >= 0) {
			
			this._collideBodies.splice(i, 1);
			
			
			
			
		}
		
	}

	RigidBody.prototype.addConstraint = function(constraint)
	{

		if (this._constraints.indexOf(constraint) < 0)
		{
			this._constraints.push(constraint);
		}
		
	}

	RigidBody.prototype.removeConstraint = function(constraint)
	{

		if (this._constraints.indexOf(constraint) >= 0)
		{
			this._constraints.splice(this._constraints.indexOf(constraint), 1);
		}
		
	}

	RigidBody.prototype.copyCurrentStateToOld = function()
	{

		this._oldState.position = this._currState.position.clone();
		this._oldState.orientation = this._currState.orientation.clone();
		this._oldState.linVelocity = this._currState.linVelocity.clone();
		this._oldState.rotVelocity = this._currState.rotVelocity.clone();
		
	}

	RigidBody.prototype.storeState = function()
	{

		this._storeState.position = this._currState.position.clone();
		this._storeState.orientation = this._currState.orientation.clone();
		this._storeState.linVelocity = this._currState.linVelocity.clone();
		this._storeState.rotVelocity = this._currState.rotVelocity.clone();
		
	}

	RigidBody.prototype.restoreState = function()
	{

		this._currState.position = this._storeState.position.clone();
		this._currState.orientation = this._storeState.orientation.clone();
		this._currState.linVelocity = this._storeState.linVelocity.clone();
		this._currState.rotVelocity = this._storeState.rotVelocity.clone();
		
		this.updateInertia();
		
	}

	RigidBody.prototype.get_currentState = function()
	{

		return this._currState;
		
	}

	RigidBody.prototype.get_oldState = function()
	{

		return this._oldState;
		
	}

	RigidBody.prototype.get_id = function()
	{

		return this._id;
		
	}

	RigidBody.prototype.get_type = function()
	{

		return this._type;
		
	}

	RigidBody.prototype.get_skin = function()
	{

		return this._skin;
		
	}

	RigidBody.prototype.get_boundingSphere = function()
	{

		return this._boundingSphere;
		
	}

	RigidBody.prototype.get_boundingBox = function()
	{

		return this._boundingBox;
		
	}

	RigidBody.prototype.get_force = function()
	{

		return this._force;
		
	}

	RigidBody.prototype.get_mass = function()
	{

		return this._mass;
		
	}

	RigidBody.prototype.get_invMass = function()
	{

		return this._invMass;
		
	}

	RigidBody.prototype.get_worldInertia = function()
	{

		return this._worldInertia;
		
	}

	RigidBody.prototype.get_worldInvInertia = function()
	{

		return this._worldInvInertia;
		
	}

	RigidBody.prototype.get_nonCollidables = function()
	{

		return this._nonCollidables;
		
	}

	RigidBody.prototype.get_constraints = function()
	{

		return this._constraints;
		
	}

	RigidBody.prototype.set_linVelocityDamping = function(vel)
	{

		this._linVelDamping.x = JMath3D.getLimiteNumber(vel.x, 0, 1);
		this._linVelDamping.y = JMath3D.getLimiteNumber(vel.y, 0, 1);
		this._linVelDamping.z = JMath3D.getLimiteNumber(vel.z, 0, 1);
		
	}

	RigidBody.prototype.get_linVelocityDamping = function()
	{

		return this._linVelDamping;
		
	}

	RigidBody.prototype.set_rotVelocityDamping = function(vel)
	{

		this._rotVelDamping.x = JMath3D.getLimiteNumber(vel.x, 0, 1);
		this._rotVelDamping.y = JMath3D.getLimiteNumber(vel.y, 0, 1);
		this._rotVelDamping.z = JMath3D.getLimiteNumber(vel.z, 0, 1);
		
	}

	RigidBody.prototype.get_rotVelocityDamping = function()
	{

		return this._rotVelDamping;
		
	}

	RigidBody.prototype.set_maxLinVelocities = function(vel)
	{

		this._maxLinVelocities = new Vector3D(Math.abs(vel.x),Math.abs(vel.y),Math.abs(vel.z));
		
	}

	RigidBody.prototype.get_maxLinVelocities = function()
	{

		return this._maxLinVelocities;
		
	}

	RigidBody.prototype.set_maxRotVelocities = function(vel)
	{

		this._maxRotVelocities = new Vector3D(Math.abs(vel.x),Math.abs(vel.y),Math.abs(vel.z));
		
	}

	RigidBody.prototype.get_maxRotVelocities = function()
	{

		return this._maxRotVelocities;
		
	}

	RigidBody.prototype.limitVel = function()
	{

		this._currState.linVelocity.x = JMath3D.getLimiteNumber(this._currState.linVelocity.x, -this._maxLinVelocities.x, this._maxLinVelocities.x);
		this._currState.linVelocity.y = JMath3D.getLimiteNumber(this._currState.linVelocity.y, -this._maxLinVelocities.y, this._maxLinVelocities.y);
		this._currState.linVelocity.z = JMath3D.getLimiteNumber(this._currState.linVelocity.z, -this._maxLinVelocities.z, this._maxLinVelocities.z);
		
	}

	RigidBody.prototype.limitAngVel = function()
	{

		var fx = Math.abs(this._currState.rotVelocity.x) / this._maxRotVelocities.x;
		var fy = Math.abs(this._currState.rotVelocity.y) / this._maxRotVelocities.y;
		var fz = Math.abs(this._currState.rotVelocity.z) / this._maxRotVelocities.z;
		var f = Math.max(fx, fy, fz);

		if (f > 1)
			this._currState.rotVelocity = JNumber3D.getDivideVector(this._currState.rotVelocity, f);
		
	}

	RigidBody.prototype.getTransform = function()
	{

		return this._skin ? this._skin.transform : null;
		
	}

	RigidBody.prototype.updateObject3D = function()
	{

		if (this._skin)
		{
			this._skin.transform = JMatrix3D.getAppendMatrix3D(this._currState.orientation, JMatrix3D.getTranslationMatrix(this._currState.position.x, this._currState.position.y, this._currState.position.z));
		}
		
	}

	RigidBody.prototype.get_material = function()
	{

		return this._material;
		
	}

	RigidBody.prototype.get_restitution = function()
	{

		return this._material.restitution;
		
	}

	RigidBody.prototype.set_restitution = function(restitution)
	{

		this._material.restitution = JMath3D.getLimiteNumber(restitution, 0, 1);
		
	}

	RigidBody.prototype.get_friction = function()
	{

		return this._material.friction;
		
	}

	RigidBody.prototype.set_friction = function(friction)
	{

		this._material.friction = JMath3D.getLimiteNumber(friction, 0, 1);
		
	}

	RigidBody.idCounter =  0; // int

	RigidBody.formatRotation = function(angle)
	{

			if (angle >= -180 && angle <= 180)
				return angle;
			
			var angle2 = angle % 360;
			if (angle2 < -180)
				return angle2 + 360;
			
			if (angle2 > 180)
				return angle2 - 360;
			
			return angle2;
		
	}


	jiglib.RigidBody = RigidBody; 

})(jiglib);


(function(jiglib) {

	var JIndexedTriangle = jiglib.JIndexedTriangle;
	var JOctree = jiglib.JOctree;
	var JCapsule = jiglib.JCapsule;
	var JBox = jiglib.JBox;
	var JRay = jiglib.JRay;
	var JAABox = jiglib.JAABox;
	var JTerrain = jiglib.JTerrain;
	var JPlane = jiglib.JPlane;
	var JTriangleMesh = jiglib.JTriangleMesh;
	var JTriangle = jiglib.JTriangle;
	var JSegment = jiglib.JSegment;
	var RigidBody = jiglib.RigidBody;
	var Matrix3D = jiglib.Matrix3D;
	var Vector3D = jiglib.Vector3D;
	var CollOutData = jiglib.CollOutData;
	var JMatrix3D = jiglib.JMatrix3D;
	var JMath3D = jiglib.JMath3D;
	var JNumber3D = jiglib.JNumber3D;
	var PhysicsState = jiglib.PhysicsState;

	var JSphere = function(skin, r)
	{
		this.name = null; // String
		this._radius = null; // Number


		jiglib.RigidBody.apply(this, [ skin ]);
		this._type = "SPHERE";
		this._radius = r;
		this._boundingSphere = this._radius;
		this.set_mass(1);
		this.updateBoundingBox();
		
	}

	jiglib.extend(JSphere, RigidBody);

	JSphere.prototype.set_radius = function(r)
	{

		this._radius = r;
		this._boundingSphere = this._radius;
		this.setInertia(this.getInertiaProperties(this.get_mass()));
		this.setActive();
		this.updateBoundingBox();
		
	}

	JSphere.prototype.get_radius = function()
	{

		return this._radius;
		
	}

	JSphere.prototype.segmentIntersect = function(out, seg, state)
	{

		out.frac = 0;
		out.position = new Vector3D();
		out.normal = new Vector3D();

		var frac = 0, radiusSq, rSq, sDotr, sSq, sigma, sigmaSqrt, lambda1, lambda2;
		var r, s;
		r = seg.delta;
		s = seg.origin.subtract(state.position);

		radiusSq = this._radius * this._radius;
		rSq = r.get_lengthSquared();
		if (rSq < radiusSq)
		{
			out.frac = 0;
			out.position = seg.origin.clone();
			out.normal = out.position.subtract(state.position);
			out.normal.normalize();
			return true;
		}

		sDotr = s.dotProduct(r);
		sSq = s.get_lengthSquared();
		sigma = sDotr * sDotr - rSq * (sSq - radiusSq);
		if (sigma < 0)
		{
			return false;
		}
		sigmaSqrt = Math.sqrt(sigma);
		lambda1 = (-sDotr - sigmaSqrt) / rSq;
		lambda2 = (-sDotr + sigmaSqrt) / rSq;
		if (lambda1 > 1 || lambda2 < 0)
		{
			return false;
		}
		frac = Math.max(lambda1, 0);
		out.frac = frac;
		out.position = seg.getPoint(frac);
		out.normal = out.position.subtract(state.position);
		out.normal.normalize();
		return true;
		
	}

	JSphere.prototype.getInertiaProperties = function(m)
	{

		var Ixx = 0.4 * m * this._radius * this._radius;
		return JMatrix3D.getScaleMatrix(Ixx, Ixx, Ixx);
		
	}

	JSphere.prototype.updateBoundingBox = function()
	{

		this._boundingBox.clear();
		this._boundingBox.addSphere(this); // todo: only when needed like changing the scale?
		
	}



	jiglib.JSphere = JSphere; 

})(jiglib);


(function(jiglib) {

	var JIndexedTriangle = jiglib.JIndexedTriangle;
	var JOctree = jiglib.JOctree;
	var JCapsule = jiglib.JCapsule;
	var JBox = jiglib.JBox;
	var JRay = jiglib.JRay;
	var JAABox = jiglib.JAABox;
	var JTerrain = jiglib.JTerrain;
	var JPlane = jiglib.JPlane;
	var JTriangle = jiglib.JTriangle;
	var JSphere = jiglib.JSphere;
	var JSegment = jiglib.JSegment;
	var RigidBody = jiglib.RigidBody;
	var Matrix3D = jiglib.Matrix3D;
	var Vector3D = jiglib.Vector3D;
	var CollOutData = jiglib.CollOutData;
	var TriangleVertexIndices = jiglib.TriangleVertexIndices;
	var JMatrix3D = jiglib.JMatrix3D;
	var JMath3D = jiglib.JMath3D;
	var JNumber3D = jiglib.JNumber3D;
	var PhysicsState = jiglib.PhysicsState;

	var JTriangleMesh = function(skin, initPosition, initOrientation, maxTrianglesPerCell, minCellSize)
	{
		this._octree = null; // JOctree
		this._maxTrianglesPerCell = null; // int
		this._minCellSize = null; // Number
		this._skinVertices = null; // Vector3D

		jiglib.RigidBody.apply(this, [ skin ]);
		
		this.get_currentState().position=initPosition.clone();
		this.get_currentState().orientation=initOrientation.clone();
		this._maxTrianglesPerCell = maxTrianglesPerCell;
		this._minCellSize = minCellSize;
		
		this.set_movable(false);
		
		if(skin){
			this._skinVertices=skin.vertices;
			this.createMesh(this._skinVertices,skin.indices);
			
			this._boundingBox=this._octree.boundingBox().clone();
			skin.transform = JMatrix3D.getAppendMatrix3D(this.get_currentState().orientation, JMatrix3D.getTranslationMatrix(this.get_currentState().position.x, this.get_currentState().position.y, this.get_currentState().position.z));
		}
		
		this._type = "TRIANGLEMESH";
		
	}

	jiglib.extend(JTriangleMesh, RigidBody);

	JTriangleMesh.prototype.createMesh = function(vertices, triangleVertexIndices)
	{

		
		var len=vertices.length;
		var vts=[];
		
		var transform = JMatrix3D.getTranslationMatrix(this.get_currentState().position.x, this.get_currentState().position.y, this.get_currentState().position.z);
		transform = JMatrix3D.getAppendMatrix3D(this.get_currentState().orientation, transform);
		
		var i = 0;
		for (var vertices_i = 0, vertices_l = vertices.length, _point; (vertices_i < vertices_l) && (_point = vertices[vertices_i]); vertices_i++){
			vts[i++] = transform.transformVector(_point);
		}
		
		this._octree = new JOctree();
		
		this._octree.addTriangles(vts, vts.length, triangleVertexIndices, triangleVertexIndices.length);
		this._octree.buildOctree(this._maxTrianglesPerCell, this._minCellSize);
		
		
	}

	JTriangleMesh.prototype.get_octree = function()
	{

		return this._octree;
		
	}

	JTriangleMesh.prototype.segmentIntersect = function(out, seg, state)
	{

		var segBox = new JAABox();
		segBox.addSegment(seg);
		
		var potentialTriangles = [];
		var numTriangles = this._octree.getTrianglesIntersectingtAABox(potentialTriangles, segBox);
		
		var bestFrac = JMath3D.NUM_HUGE;
		var tri;
		var meshTriangle;
		for (var iTriangle = 0 ; iTriangle < numTriangles ; iTriangle++) {
			meshTriangle = this._octree.getTriangle(potentialTriangles[iTriangle]);
			
			tri = new JTriangle(this._octree.getVertex(meshTriangle.getVertexIndex(0)), this._octree.getVertex(meshTriangle.getVertexIndex(1)), this._octree.getVertex(meshTriangle.getVertexIndex(2)));
			
			if (tri.segmentTriangleIntersection(out, seg)) {
				if (out.frac < bestFrac) {
				bestFrac = out.frac;
				out.position = seg.getPoint(bestFrac);
				out.normal = meshTriangle.get_plane().get_normal();
				}
			}
		}
		out.frac = bestFrac;
		if (bestFrac < JMath3D.NUM_HUGE) {
			return true;
		}else {
			return false;
		}
		
	}

	JTriangleMesh.prototype.updateState = function()
	{

		jiglib.RigidBody.prototype.updateState.apply(this, [  ]);
		
		var len=this._skinVertices.length;
		var vts=[];
		
		var transform = JMatrix3D.getTranslationMatrix(this.get_currentState().position.x, this.get_currentState().position.y, this.get_currentState().position.z);
		transform = JMatrix3D.getAppendMatrix3D(this.get_currentState().orientation, transform);
		
		var i = 0;
		for (var _skinVertices_i = 0, _skinVertices_l = this._skinVertices.length, _point; (_skinVertices_i < _skinVertices_l) && (_point = this._skinVertices[_skinVertices_i]); _skinVertices_i++){
			vts[i++] = transform.transformVector(_point);
		}
		
		this._octree.updateTriangles(vts);
		this._octree.buildOctree(this._maxTrianglesPerCell, this._minCellSize);
		
		this._boundingBox=this._octree.boundingBox().clone();
		
	}

	JTriangleMesh.prototype.getInertiaProperties = function(m)
	{

		return new Matrix3D();
		
	}

	JTriangleMesh.prototype.updateBoundingBox = function()
	{

		
	}



	jiglib.JTriangleMesh = JTriangleMesh; 

})(jiglib);


(function(jiglib) {

	var JIndexedTriangle = jiglib.JIndexedTriangle;
	var JOctree = jiglib.JOctree;
	var JCapsule = jiglib.JCapsule;
	var JBox = jiglib.JBox;
	var JRay = jiglib.JRay;
	var JAABox = jiglib.JAABox;
	var JTerrain = jiglib.JTerrain;
	var JTriangleMesh = jiglib.JTriangleMesh;
	var JTriangle = jiglib.JTriangle;
	var JSphere = jiglib.JSphere;
	var JSegment = jiglib.JSegment;
	var RigidBody = jiglib.RigidBody;
	var Vector3D = jiglib.Vector3D;
	var CollOutData = jiglib.CollOutData;
	var JMatrix3D = jiglib.JMatrix3D;
	var Matrix3D = jiglib.Matrix3D;
	var JMath3D = jiglib.JMath3D;
	var JNumber3D = jiglib.JNumber3D;
	var PhysicsState = jiglib.PhysicsState;

	var JPlane = function(skin, initNormal)
	{
		this._initNormal = null; // Vector3D
		this._normal = null; // Vector3D
		this._distance = null; // Number

		jiglib.RigidBody.apply(this, [ skin ]);

		this._initNormal = initNormal ? initNormal.clone() : new Vector3D(0, 0, -1);
		this._normal = this._initNormal.clone();

		this._distance = 0;
		this.set_movable(false);
		
		var huge=JMath3D.NUM_HUGE;
		this._boundingBox.minPos = new Vector3D(-huge, -huge, -huge);
		this._boundingBox.maxPos = new Vector3D(huge, huge, huge);

		this._type = "PLANE";
		
	}

	jiglib.extend(JPlane, RigidBody);

	JPlane.prototype.get_normal = function()
	{

		return this._normal;
		
	}

	JPlane.prototype.get_distance = function()
	{

		return this._distance;
		
	}

	JPlane.prototype.pointPlaneDistance = function(pt)
	{

		return this._normal.dotProduct(pt) - this._distance;
		
	}

	JPlane.prototype.segmentIntersect = function(out, seg, state)
	{

		out.frac = 0;
		out.position = new Vector3D();
		out.normal = new Vector3D();

		var frac = 0, t, denom;

		denom = this._normal.dotProduct(seg.delta);
		if (Math.abs(denom) > JMath3D.NUM_TINY)
		{
			t = -1 * (this._normal.dotProduct(seg.origin) - this._distance) / denom;

			if (t < 0 || t > 1)
			{
				return false;
			}
			else
			{
				frac = t;
				out.frac = frac;
				out.position = seg.getPoint(frac);
				out.normal = this._normal.clone();
				out.normal.normalize();
				return true;
			}
		}
		else
		{
			return false;
		}
		
	}

	JPlane.prototype.updateState = function()
	{

		jiglib.RigidBody.prototype.updateState.apply(this, [  ]);

		this._normal = this._currState.orientation.transformVector(this._initNormal);
		this._distance = this._currState.position.dotProduct(this._normal);
		
	}



	jiglib.JPlane = JPlane; 

})(jiglib);


(function(jiglib) {

	var JIndexedTriangle = jiglib.JIndexedTriangle;
	var JOctree = jiglib.JOctree;
	var JCapsule = jiglib.JCapsule;
	var JBox = jiglib.JBox;
	var JRay = jiglib.JRay;
	var JAABox = jiglib.JAABox;
	var JPlane = jiglib.JPlane;
	var JTriangleMesh = jiglib.JTriangleMesh;
	var JTriangle = jiglib.JTriangle;
	var JSphere = jiglib.JSphere;
	var JSegment = jiglib.JSegment;
	var RigidBody = jiglib.RigidBody;
	var Vector3D = jiglib.Vector3D;
	var CollOutData = jiglib.CollOutData;
	var PlaneData = jiglib.PlaneData;
	var TerrainData = jiglib.TerrainData;
	var JNumber3D = jiglib.JNumber3D;
	var JMath3D = jiglib.JMath3D;
	var PhysicsState = jiglib.PhysicsState;

	var JTerrain = function(tr, yUp)
	{
		this._terrain = null; // ITerrain
		this._yUp = null; // Boolean

		jiglib.RigidBody.apply(this, [ null ]);

		// yUp for lite
		this._yUp = yUp;

		this._terrain = tr;
		this.set_movable(false);
		
		this._boundingBox.minPos=new Vector3D(tr.minW,-tr.maxHeight,tr.minH);
		this._boundingBox.maxPos=new Vector3D(tr.maxW,tr.maxHeight,tr.maxH);
		this._boundingSphere=this._boundingBox.getRadiusAboutCentre();
		
		this._type = "TERRAIN";
		
	}

	jiglib.extend(JTerrain, RigidBody);

	JTerrain.prototype.get_terrainMesh = function()
	{

		return this._terrain;
		
	}

	JTerrain.prototype.getHeightByIndex = function(i, j)
	{

		i = this.limiteInt(i, 0, this._terrain.sw);
		j = this.limiteInt(j, 0, this._terrain.sh);

		return this._terrain.heights[i][j];
		
	}

	JTerrain.prototype.getNormalByIndex = function(i, j)
	{

		   var i0 = i - 1;
		   var i1 = i + 1;
		   var j0 = j - 1;
		   var j1 = j + 1;
		   i0 = this.limiteInt(i0, 0, this._terrain.sw);
		   i1 = this.limiteInt(i1, 0, this._terrain.sw);
		   j0 = this.limiteInt(j0, 0, this._terrain.sh);
		   j1 = this.limiteInt(j1, 0, this._terrain.sh);

		   var dx = (i1 - i0) * this._terrain.dw;
		   var dy = (j1 - j0) * this._terrain.dh;
		   if (i0 == i1) dx = 1;
		   if (j0 == j1) dy = 1;
		   if (i0 == i1 && j0 == j1) return Vector3D.Y_AXIS;

		   var hFwd = this._terrain.heights[i1][j];
		   var hBack = this._terrain.heights[i0][j];
		   var hLeft = this._terrain.heights[i][j1];
		   var hRight = this._terrain.heights[i][j0];

		   var normal = new Vector3D(dx, hFwd - hBack, 0);
		   normal = new Vector3D(0, hLeft - hRight, dy).crossProduct(normal);
		   normal.normalize();
		   return normal;
		   
	}

	JTerrain.prototype.getSurfacePosByIndex = function(i, j)
	{

		   return new Vector3D(this._terrain.minW + i * this._terrain.dw, this.getHeightByIndex(i, j), this._terrain.minH + j * this._terrain.dh);
		   
	}

	JTerrain.prototype.getHeightAndNormalByPoint = function(point)
	{

		var i0, j0, i1, j1;
		var w, h, iFrac, jFrac, h00, h01, h10, h11;
		
		w = this.limiteInt(point.x, this._terrain.minW, this._terrain.maxW);
		h = this.limiteInt(point.z, this._terrain.minH, this._terrain.maxH);

		i0 = (w - this._terrain.minW) / this._terrain.dw;
		j0 = (h - this._terrain.minH) / this._terrain.dh;
		i0 = this.limiteInt(i0, 0, this._terrain.sw);
		j0 = this.limiteInt(j0, 0, this._terrain.sh);

		i1 = i0 + 1;
		j1 = j0 + 1;
		i1 = this.limiteInt(i1, 0, this._terrain.sw);
		j1 = this.limiteInt(j1, 0, this._terrain.sh);

		iFrac = 1 - (w - (i0 * this._terrain.dw + this._terrain.minW)) / this._terrain.dw;
		jFrac = (h - (j0 * this._terrain.dh + this._terrain.minH)) / this._terrain.dh;
		iFrac = JMath3D.getLimiteNumber(iFrac, 0, 1);
		jFrac = JMath3D.getLimiteNumber(jFrac, 0, 1);

		// yUp for lite
		h00 = this._yUp ? this._terrain.heights[i0][j0] : -this._terrain.heights[i0][j0];
		h01 = this._yUp ? this._terrain.heights[i0][j1] : -this._terrain.heights[i0][j1];
		h10 = this._yUp ? this._terrain.heights[i1][j0] : -this._terrain.heights[i1][j0];
		h11 = this._yUp ? this._terrain.heights[i1][j1] : -this._terrain.heights[i1][j1];

		var obj = new TerrainData();
		var plane;
		if (iFrac < jFrac || i0 == i1 || j0 == j1)
		{
			obj.normal = new Vector3D(0, h11 - h10, this._terrain.dh).crossProduct(new Vector3D(this._terrain.dw, h11 - h01, 0));
			// yUp for lite
			if (!this._yUp)
				obj.normal.negate();
			obj.normal.normalize();

			plane = new PlaneData();
			plane.setWithNormal(new Vector3D((i1 * this._terrain.dw + this._terrain.minW), h11, (j1 * this._terrain.dh + this._terrain.minH)), obj.normal);
			obj.height = plane.pointPlaneDistance(point);
		}
		else
		{
			obj.normal = new Vector3D(0, h01 - h00, this._terrain.dh).crossProduct(new Vector3D(this._terrain.dw, h10 - h00, 0));
			// yUp for lite
			if (!this._yUp)
				obj.normal.negate();
			obj.normal.normalize();

			plane = new PlaneData();
			plane.setWithNormal(new Vector3D((i0 * this._terrain.dw + this._terrain.minW), h00, (j0 * this._terrain.dh + this._terrain.minH)), obj.normal);
			obj.height = plane.pointPlaneDistance(point);
		}

		return obj;
		
	}

	JTerrain.prototype.getHeightByPoint = function(point)
	{

		return this.getHeightAndNormalByPoint(point).height;
		
	}

	JTerrain.prototype.getNormalByPoint = function(point)
	{

		return this.getHeightAndNormalByPoint(point).normal;
		
	}

	JTerrain.prototype.getSurfacePosByPoint = function(point)
	{

		return new Vector3D(point.x, this.getHeightAndNormalByPoint(point).height, point.z);
		
	}

	JTerrain.prototype.segmentIntersect = function(out, seg, state)
	{

		out.frac = 0;
		out.position = new Vector3D();
		out.normal = new Vector3D();

		var segY, depthEnd, weightStart, weightEnd, tiny=JMath3D.NUM_TINY;
		// yUp for lite
		segY = this._yUp ? seg.delta.y : -seg.delta.y;

		if (segY > tiny)
			return false;

		var obj1 = this.getHeightAndNormalByPoint(seg.origin);
		if (obj1.height < 0)
			return false;

		var obj2 = this.getHeightAndNormalByPoint(seg.getEnd());
		if (obj2.height > 0)
			return false;

		depthEnd = -obj2.height;
		weightStart = 1 / (tiny + obj1.height);
		weightEnd = 1 / (tiny + obj2.height);

		obj1.normal.scaleBy(weightStart);
		obj2.normal.scaleBy(weightEnd);
		out.normal = obj1.normal.add(obj2.normal);
		out.normal.scaleBy(1 / (weightStart + weightEnd));

		out.frac = obj1.height / (obj1.height + depthEnd + tiny);
		out.position = seg.getPoint(out.frac);

		return true;
		
	}

	JTerrain.prototype.limiteInt = function(num, min, max)
	{

		var n = num;
		if (n < min)
			n = min;
		else if (n > max)
			n = max;

		return n;
		
	}

	JTerrain.prototype.updateState = function()
	{

		
	}



	jiglib.JTerrain = JTerrain; 

})(jiglib);


(function(jiglib) {

	var JIndexedTriangle = jiglib.JIndexedTriangle;
	var JOctree = jiglib.JOctree;
	var JCapsule = jiglib.JCapsule;
	var JRay = jiglib.JRay;
	var JAABox = jiglib.JAABox;
	var JTerrain = jiglib.JTerrain;
	var JPlane = jiglib.JPlane;
	var JTriangleMesh = jiglib.JTriangleMesh;
	var JTriangle = jiglib.JTriangle;
	var JSphere = jiglib.JSphere;
	var JSegment = jiglib.JSegment;
	var RigidBody = jiglib.RigidBody;
	var Matrix3D = jiglib.Matrix3D;
	var Vector3D = jiglib.Vector3D;
	var CollOutData = jiglib.CollOutData;
	var EdgeData = jiglib.EdgeData;
	var SpanData = jiglib.SpanData;
	var JMatrix3D = jiglib.JMatrix3D;
	var JMath3D = jiglib.JMath3D;
	var JNumber3D = jiglib.JNumber3D;
	var PhysicsState = jiglib.PhysicsState;

	var JBox = function(skin, width, depth, height)
	{
		this._sideLengths = null; // Vector3D
		this._points = null; // Vector3D
		this._edges =  [
			new EdgeData( 0, 1 ), new EdgeData( 0, 2 ), new EdgeData( 0, 6 ),
			new EdgeData( 2, 3 ), new EdgeData( 2, 4 ), new EdgeData( 6, 7 ),
			new EdgeData( 6, 4 ), new EdgeData( 1, 3 ), new EdgeData( 1, 7 ),
			new EdgeData( 3, 5 ), new EdgeData( 7, 5 ), new EdgeData( 4, 5 )]; // EdgeData
		this._face =  [
			[[6, 7, 1, 0]], [[5, 4, 2, 3]],
			[[3, 1, 7, 5]], [[4, 6, 0, 2]],
			[[1, 3, 2, 0]], [[7, 6, 4, 5]]]; // Vector.<Vector.<Number>>

		jiglib.RigidBody.apply(this, [ skin ]);
		this._type = "BOX";

		this._sideLengths = new Vector3D(width, height, depth);
		this._boundingSphere = 0.5 * this._sideLengths.get_length();
		this.initPoint();
		this.set_mass(1);
		this.updateBoundingBox();
		
	}

	jiglib.extend(JBox, RigidBody);

	JBox.prototype.initPoint = function()
	{

		var halfSide = this.getHalfSideLengths();
		this._points = [];
		this._points[0] = new Vector3D(halfSide.x, -halfSide.y, halfSide.z);
		this._points[1] = new Vector3D(halfSide.x, halfSide.y, halfSide.z);
		this._points[2] = new Vector3D(-halfSide.x, -halfSide.y, halfSide.z);
		this._points[3] = new Vector3D(-halfSide.x, halfSide.y, halfSide.z);
		this._points[4] = new Vector3D(-halfSide.x, -halfSide.y, -halfSide.z);
		this._points[5] = new Vector3D(-halfSide.x, halfSide.y, -halfSide.z);
		this._points[6] = new Vector3D(halfSide.x, -halfSide.y, -halfSide.z);
		this._points[7] = new Vector3D(halfSide.x, halfSide.y, -halfSide.z);
		
	}

	JBox.prototype.set_sideLengths = function(size)
	{

		this._sideLengths = size.clone();
		this._boundingSphere = 0.5 * this._sideLengths.get_length();
		this.initPoint();
		this.setInertia(this.getInertiaProperties(this.get_mass()));
		this.setActive();
		this.updateBoundingBox();
		
	}

	JBox.prototype.get_sideLengths = function()
	{

		return this._sideLengths;
		
	}

	JBox.prototype.get_edges = function()
	{

		return this._edges;
		
	}

	JBox.prototype.getVolume = function()
	{

		return (this._sideLengths.x * this._sideLengths.y * this._sideLengths.z);
		
	}

	JBox.prototype.getSurfaceArea = function()
	{

		return 2 * (this._sideLengths.x * this._sideLengths.y + this._sideLengths.x * this._sideLengths.z + this._sideLengths.y * this._sideLengths.z);
		
	}

	JBox.prototype.getHalfSideLengths = function()
	{

		return JNumber3D.getScaleVector(this._sideLengths, 0.5);
		
	}

	JBox.prototype.getSpan = function(axis)
	{

		var s, u, d, r, p;
		var cols = this.get_currentState().getOrientationCols();
		var obj = new SpanData();
		s = Math.abs(axis.dotProduct(cols[0])) * (0.5 * this._sideLengths.x);
		u = Math.abs(axis.dotProduct(cols[1])) * (0.5 * this._sideLengths.y);
		d = Math.abs(axis.dotProduct(cols[2])) * (0.5 * this._sideLengths.z);
		r = s + u + d;
		p = this.get_currentState().position.dotProduct(axis);
		obj.min = p - r;
		obj.max = p + r;

		return obj;
		
	}

	JBox.prototype.getCornerPoints = function(state)
	{

		var _points_length = this._points.length;
		var arr = [];

		var transform = JMatrix3D.getTranslationMatrix(state.position.x, state.position.y, state.position.z);
		transform = JMatrix3D.getAppendMatrix3D(state.orientation, transform);
		
		var i=0;
		for (var _points_i = 0, _points_l = this._points.length, _point; (_points_i < _points_l) && (_point = this._points[_points_i]); _points_i++){
			arr[i++] = transform.transformVector(_point);
		}
		
		return arr;
		
	}

	JBox.prototype.getCornerPointsInBoxSpace = function(thisState, boxState)
	{

		
		var max, orient, transform;
		
		max = JMatrix3D.getTransposeMatrix(boxState.orientation);
		var pos = thisState.position.subtract(boxState.position);
		pos = max.transformVector(pos);
		
		orient = JMatrix3D.getAppendMatrix3D(thisState.orientation, max);
		
		var arr = [];
		
		transform = JMatrix3D.getTranslationMatrix(pos.x, pos.y, pos.z);
		transform = JMatrix3D.getAppendMatrix3D(orient, transform);
		
		var i = 0;
		for (var _points_i = 0, _points_l = this._points.length, _point; (_points_i < _points_l) && (_point = this._points[_points_i]); _points_i++)
			arr[i++] = transform.transformVector(_point);
		
		return arr;
		
	}

	JBox.prototype.getSqDistanceToPoint = function(state, closestBoxPoint, point)
	{

		var _closestBoxPoint, halfSideLengths;
		var delta=0, sqDistance=0;
		
		_closestBoxPoint = point.subtract(state.position);
		_closestBoxPoint = JMatrix3D.getTransposeMatrix(state.orientation).transformVector(_closestBoxPoint);

		halfSideLengths = this.getHalfSideLengths();

		if (_closestBoxPoint.x < -halfSideLengths.x)
		{
			delta = _closestBoxPoint.x + halfSideLengths.x;
			sqDistance += (delta * delta);
			_closestBoxPoint.x = -halfSideLengths.x;
		}
		else if (_closestBoxPoint.x > halfSideLengths.x)
		{
			delta = _closestBoxPoint.x - halfSideLengths.x;
			sqDistance += (delta * delta);
			_closestBoxPoint.x = halfSideLengths.x;
		}

		if (_closestBoxPoint.y < -halfSideLengths.y)
		{
			delta = _closestBoxPoint.y + halfSideLengths.y;
			sqDistance += (delta * delta);
			_closestBoxPoint.y = -halfSideLengths.y;
		}
		else if (_closestBoxPoint.y > halfSideLengths.y)
		{
			delta = _closestBoxPoint.y - halfSideLengths.y;
			sqDistance += (delta * delta);
			_closestBoxPoint.y = halfSideLengths.y;
		}

		if (_closestBoxPoint.z < -halfSideLengths.z)
		{
			delta = _closestBoxPoint.z + halfSideLengths.z;
			sqDistance += (delta * delta);
			_closestBoxPoint.z = -halfSideLengths.z;
		}
		else if (_closestBoxPoint.z > halfSideLengths.z)
		{
			delta = (_closestBoxPoint.z - halfSideLengths.z);
			sqDistance += (delta * delta);
			_closestBoxPoint.z = halfSideLengths.z;
		}
		_closestBoxPoint = state.orientation.transformVector(_closestBoxPoint);
		closestBoxPoint[0] = state.position.add(_closestBoxPoint);
		return sqDistance;
		
	}

	JBox.prototype.getDistanceToPoint = function(state, closestBoxPoint, point)
	{

		return Math.sqrt(this.getSqDistanceToPoint(state, closestBoxPoint, point));
		
	}

	JBox.prototype.pointIntersect = function(pos)
	{

		var p, h, dirVec;
		
		p = pos.subtract(this.get_currentState().position);
		h = JNumber3D.getScaleVector(this._sideLengths, 0.5);
		
		var cols = this.get_currentState().getOrientationCols();
		for (var dir; dir < 3; dir++)
		{
			dirVec = cols[dir].clone();
			dirVec.normalize();
			if (Math.abs(dirVec.dotProduct(p)) > JNumber3D.toArray(h)[dir] + JMath3D.NUM_TINY)
			{
				return false;
			}
		}
		return true;
		
	}

	JBox.prototype.segmentIntersect = function(out, seg, state)
	{

		out.frac = 0;
		out.position = new Vector3D();
		out.normal = new Vector3D();
		
		var tiny=JMath3D.NUM_TINY, huge=JMath3D.NUM_HUGE, frac, min, max, dirMin=0, dirMax=0, dir=0, e, f, t, t1, t2, directionVectorNumber;
		var p, h;

		frac = huge;
		min = -huge;
		max = huge;
		p = state.position.subtract(seg.origin);
		h = JNumber3D.getScaleVector(this._sideLengths, 0.5);
		
		var orientationCol = state.getOrientationCols();
		var directionVectorArray = JNumber3D.toArray(h);
		for (dir = 0; dir < 3; dir++)
		{
			directionVectorNumber = directionVectorArray[dir];
			e = orientationCol[dir].dotProduct(p);
			f = orientationCol[dir].dotProduct(seg.delta);
			if (Math.abs(f) > tiny)
			{
				t1 = (e + directionVectorNumber) / f;
				t2 = (e - directionVectorNumber) / f;
				if (t1 > t2)
				{
				t = t1;
				t1 = t2;
				t2 = t;
				}
				if (t1 > min)
				{
				min = t1;
				dirMin = dir;
				}
				if (t2 < max)
				{
				max = t2;
				dirMax = dir;
				}
				if (min > max)
				return false;
				if (max < 0)
				return false;
			}
			else if (-e - directionVectorNumber > 0 || -e + directionVectorNumber < 0)
			{
				return false;
			}
		}

		if (min > 0)
		{
			dir = dirMin;
			frac = min;
		}
		else
		{
			dir = dirMax;
			frac = max;
		}
		if (frac < 0)
			frac = 0;
		/*if (frac > 1)
			frac = 1;*/
		if (frac > 1 - tiny)
		{
			return false;
		}
		out.frac = frac;
		out.position = seg.getPoint(frac);
		if (orientationCol[dir].dotProduct(seg.delta) < 0)
		{
			out.normal = JNumber3D.getScaleVector(orientationCol[dir], -1);
		}
		else
		{
			out.normal = orientationCol[dir];
		}
		return true;
		
	}

	JBox.prototype.getInertiaProperties = function(m)
	{

		return JMatrix3D.getScaleMatrix(
		(m / 12) * (this._sideLengths.y * this._sideLengths.y + this._sideLengths.z * this._sideLengths.z),
		(m / 12) * (this._sideLengths.x * this._sideLengths.x + this._sideLengths.z * this._sideLengths.z),
		(m / 12) * (this._sideLengths.x * this._sideLengths.x + this._sideLengths.y * this._sideLengths.y))
		
	}

	JBox.prototype.updateBoundingBox = function()
	{

		this._boundingBox.clear();
		this._boundingBox.addBox(this);
		
	}



	jiglib.JBox = JBox; 

})(jiglib);


(function(jiglib) {

	var JCar = jiglib.JCar;
	var JWheel = jiglib.JWheel;
	var JBox = jiglib.JBox;

	var JChassis = function(car, skin, width, depth, height)
	{
		this._car = null; // JCar

		jiglib.JBox.apply(this, [ skin, width, depth, height ]);

		this._car = car;
		
	}

	jiglib.extend(JChassis, JBox);

	JChassis.prototype.get_car = function()
	{

		return this._car;
		
	}

	JChassis.prototype.postPhysics = function(dt)
	{

		jiglib.JBox.prototype.postPhysics.apply(this, [ dt ]);
		this._car.addExternalForces(dt);
		this._car.postPhysics(dt);
		
	}



	jiglib.JChassis = JChassis; 

})(jiglib);


(function(jiglib) {

	var JIndexedTriangle = jiglib.JIndexedTriangle;
	var JOctree = jiglib.JOctree;
	var JBox = jiglib.JBox;
	var JRay = jiglib.JRay;
	var JAABox = jiglib.JAABox;
	var JTerrain = jiglib.JTerrain;
	var JPlane = jiglib.JPlane;
	var JTriangleMesh = jiglib.JTriangleMesh;
	var JTriangle = jiglib.JTriangle;
	var JSphere = jiglib.JSphere;
	var JSegment = jiglib.JSegment;
	var RigidBody = jiglib.RigidBody;
	var Matrix3D = jiglib.Matrix3D;
	var Vector3D = jiglib.Vector3D;
	var CollOutData = jiglib.CollOutData;
	var JMatrix3D = jiglib.JMatrix3D;
	var JMath3D = jiglib.JMath3D;
	var JNumber3D = jiglib.JNumber3D;
	var PhysicsState = jiglib.PhysicsState;

	var JCapsule = function(skin, r, l)
	{
		this._length = null; // Number
		this._radius = null; // Number

		jiglib.RigidBody.apply(this, [ skin ]);
		this._type = "CAPSULE";
		this._radius = r;
		this._length = l;
		this._boundingSphere = this.getBoundingSphere(r, l);
		this.set_mass(1);
		this.updateBoundingBox();
		
	}

	jiglib.extend(JCapsule, RigidBody);

	JCapsule.prototype.set_radius = function(r)
	{

		this._radius = r;
		this._boundingSphere = this.getBoundingSphere(this._radius, this._length);
		this.setInertia(this.getInertiaProperties(this.get_mass()));
		this.updateBoundingBox();
		this.setActive();
		
	}

	JCapsule.prototype.get_radius = function()
	{

		return this._radius;
		
	}

	JCapsule.prototype.set_length = function(l)
	{

		this._length = l;
		this._boundingSphere = this.getBoundingSphere(this._radius, this._length);
		this.setInertia(this.getInertiaProperties(this.get_mass()));
		this.updateBoundingBox();
		this.setActive();
		
	}

	JCapsule.prototype.get_length = function()
	{

		return this._length;
		
	}

	JCapsule.prototype.getBottomPos = function(state)
	{

		return state.position.add(JNumber3D.getScaleVector(state.getOrientationCols()[1], -this._length / 2));
		
	}

	JCapsule.prototype.getEndPos = function(state)
	{

		return state.position.add(JNumber3D.getScaleVector(state.getOrientationCols()[1], this._length / 2));
		
	}

	JCapsule.prototype.segmentIntersect = function(out, seg, state)
	{

	}

	JCapsule.prototype.getInertiaProperties = function(m)
	{

		var cylinderMass, Ixx, Iyy, Izz, endMass;
		cylinderMass = m * Math.PI * this._radius * this._radius * this._length / this.getVolume();
		Ixx = 0.25 * cylinderMass * this._radius * this._radius + (1 / 12) * cylinderMass * this._length * this._length;
		Iyy = 0.5 * cylinderMass * this._radius * this._radius;
		Izz = Ixx;
		 
		endMass = m - cylinderMass;
		Ixx += (0.4 * endMass * this._radius * this._radius + endMass * Math.pow(0.5 * this._length, 2));
		Iyy += (0.2 * endMass * this._radius * this._radius);
		Izz += (0.4 * endMass * this._radius * this._radius + endMass * Math.pow(0.5 * this._length, 2));
		
		return JMatrix3D.getScaleMatrix(Ixx, Iyy, Izz);
		
	}

	JCapsule.prototype.updateBoundingBox = function()
	{

		this._boundingBox.clear();
		this._boundingBox.addCapsule(this);
		
	}

	JCapsule.prototype.getBoundingSphere = function(r, l)
	{

		return Math.sqrt(Math.pow(l / 2, 2) + r * r) + r;
		
	}

	JCapsule.prototype.getVolume = function()
	{

		return (4 / 3) * Math.PI * this._radius * this._radius * this._radius + this._length * Math.PI * this._radius * this._radius;
		
	}



	jiglib.JCapsule = JCapsule; 

})(jiglib);


(function(jiglib) {

	var MaterialProperties = jiglib.MaterialProperties;
	var CachedImpulse = jiglib.CachedImpulse;
	var PhysicsState = jiglib.PhysicsState;
	var RigidBody = jiglib.RigidBody;
	var BodyPair = jiglib.BodyPair;
	var PhysicsSystem = jiglib.PhysicsSystem;
	var PhysicsController = jiglib.PhysicsController;
	var Vector3D = jiglib.Vector3D;
	var JMatrix3D = jiglib.JMatrix3D;
	var Matrix3D = jiglib.Matrix3D;
	var JMath3D = jiglib.JMath3D;
	var JNumber3D = jiglib.JNumber3D;
	var JConstraintWorldPoint = jiglib.JConstraintWorldPoint;
	var JConstraintMaxDistance = jiglib.JConstraintMaxDistance;
	var JConstraint = jiglib.JConstraint;
	var JConstraintPoint = jiglib.JConstraintPoint;

	var HingeJoint = function(body0, body1, hingeAxis, hingePosRel0, hingeHalfWidth, hingeFwdAngle, hingeBckAngle, sidewaysSlack, damping)
	{
		this.MAX_HINGE_ANGLE_LIMIT =  150; // Number
		this._hingeAxis = null; // Vector3D
		this._hingePosRel0 = null; // Vector3D
		this._body0 = null; // RigidBody
		this._body1 = null; // RigidBody
		this._usingLimit = null; // Boolean
		this._broken = null; // Boolean
		this._damping = null; // Number
		this._extraTorque = null; // Number
		this.sidePointConstraints = null; // JConstraintMaxDistance
		this.midPointConstraint = null; // JConstraintPoint
		this.maxDistanceConstraint = null; // JConstraintMaxDistance

		this._body0 = body0;
		this._body1 = body1;
		this._hingeAxis = hingeAxis.clone();
		this._hingePosRel0 = hingePosRel0.clone();
		this._usingLimit = false;
		this._controllerEnabled = false;
		this._broken = false;
		this._damping = damping;
		this._extraTorque = 0;

		this._hingeAxis.normalize();
		var _hingePosRel1 = this._body0.get_currentState().position.add(this._hingePosRel0.subtract(this._body1.get_currentState().position));

		var relPos0a = this._hingePosRel0.add(JNumber3D.getScaleVector(this._hingeAxis, hingeHalfWidth));
		var relPos0b = this._hingePosRel0.subtract(JNumber3D.getScaleVector(this._hingeAxis, hingeHalfWidth));

		var relPos1a = _hingePosRel1.add(JNumber3D.getScaleVector(this._hingeAxis, hingeHalfWidth));
		var relPos1b = _hingePosRel1.subtract(JNumber3D.getScaleVector(this._hingeAxis, hingeHalfWidth));

		var timescale = 1 / 20;
		var allowedDistanceMid = 0.005;
		var allowedDistanceSide = sidewaysSlack * hingeHalfWidth;

		this.sidePointConstraints = [];
		this.sidePointConstraints[0] = new JConstraintMaxDistance(this._body0, relPos0a, this._body1, relPos1a, allowedDistanceSide);
		this.sidePointConstraints[1] = new JConstraintMaxDistance(this._body0, relPos0b, this._body1, relPos1b, allowedDistanceSide);

		this.midPointConstraint = new JConstraintPoint(this._body0, this._hingePosRel0, this._body1, _hingePosRel1, allowedDistanceMid, timescale);

		if (hingeFwdAngle <= this.MAX_HINGE_ANGLE_LIMIT)
		{
			var perpDir = Vector3D.Y_AXIS;
			if (perpDir.dotProduct(this._hingeAxis) > 0.1)
			{
				perpDir.x = 1;
				perpDir.y = 0;
				perpDir.z = 0;
			}
			var sideAxis = this._hingeAxis.crossProduct(perpDir);
			perpDir = sideAxis.crossProduct(this._hingeAxis);
			perpDir.normalize();

			var len = 10 * hingeHalfWidth;
			var hingeRelAnchorPos0 = JNumber3D.getScaleVector(perpDir, len);
			var angleToMiddle = 0.5 * (hingeFwdAngle - hingeBckAngle);
			var hingeRelAnchorPos1 = JMatrix3D.getRotationMatrix(this._hingeAxis.x, this._hingeAxis.y, this._hingeAxis.z, -angleToMiddle).transformVector(hingeRelAnchorPos0);

			var hingeHalfAngle = 0.5 * (hingeFwdAngle + hingeBckAngle);
			var allowedDistance = len * 2 * Math.sin(0.5 * hingeHalfAngle * Math.PI / 180);

			var hingePos = this._body1.get_currentState().position.add(this._hingePosRel0);
			var relPos0c = hingePos.add(hingeRelAnchorPos0.subtract(this._body0.get_currentState().position));
			var relPos1c = hingePos.add(hingeRelAnchorPos1.subtract(this._body1.get_currentState().position));

			this.maxDistanceConstraint = new JConstraintMaxDistance(this._body0, relPos0c, this._body1, relPos1c, allowedDistance);
			this._usingLimit = true;
		}
		if (this._damping <= 0)
		{
			this._damping = -1;
		}
		else
		{
			this._damping = JMath3D.getLimiteNumber(this._damping, 0, 1);
		}

		this.enableController();
		
	}

	jiglib.extend(HingeJoint, PhysicsController);

	HingeJoint.prototype.enableController = function()
	{

		if (this._controllerEnabled)
		{
			return;
		}
		this.midPointConstraint.enableConstraint();
		this.sidePointConstraints[0].enableConstraint();
		this.sidePointConstraints[1].enableConstraint();
		if (this._usingLimit && !this._broken)
		{
			this.maxDistanceConstraint.enableConstraint();
		}
		this._controllerEnabled = true;
		PhysicsSystem.getInstance().addController(this);
		
	}

	HingeJoint.prototype.disableController = function()
	{

		if (!this._controllerEnabled)
		{
			return;
		}
		this.midPointConstraint.disableConstraint();
		this.sidePointConstraints[0].disableConstraint();
		this.sidePointConstraints[1].disableConstraint();
		if (this._usingLimit && !this._broken)
		{
			this.maxDistanceConstraint.disableConstraint();
		}
		this._controllerEnabled = false;
		PhysicsSystem.getInstance().removeController(this);
		
	}

	HingeJoint.prototype.breakHinge = function()
	{

		if (this._broken)
		{
			return;
		}
		if (this._usingLimit)
		{
			this.maxDistanceConstraint.disableConstraint();
		}
		this._broken = true;
		
	}

	HingeJoint.prototype.mendHinge = function()
	{

		if (!this._broken)
		{
			return;
		}
		if (this._usingLimit)
		{
			this.maxDistanceConstraint.enableConstraint();
		}
		this._broken = false;
		
	}

	HingeJoint.prototype.setExtraTorque = function(torque)
	{

		this._extraTorque = torque;
		
	}

	HingeJoint.prototype.isBroken = function()
	{

		return this._broken;
		
	}

	HingeJoint.prototype.getHingePosRel0 = function()
	{

		return this._hingePosRel0;
		
	}

	HingeJoint.prototype.updateController = function(dt)
	{

		if (this._damping > 0)
		{
			var hingeAxis, newAngVel1, newAngVel2;
			var angRot1, angRot2, avAngRot, frac, newAngRot1, newAngRot2;
			
			hingeAxis = this._body1.get_currentState().rotVelocity.subtract(this._body0.get_currentState().rotVelocity);
			hingeAxis.normalize();

			angRot1 = this._body0.get_currentState().rotVelocity.dotProduct(hingeAxis);
			angRot2 = this._body1.get_currentState().rotVelocity.dotProduct(hingeAxis);

			avAngRot = 0.5 * (angRot1 + angRot2);
			frac = 1 - this._damping;
			newAngRot1 = avAngRot + (angRot1 - avAngRot) * frac;
			newAngRot2 = avAngRot + (angRot2 - avAngRot) * frac;

			newAngVel1 = this._body0.get_currentState().rotVelocity.add(JNumber3D.getScaleVector(hingeAxis, newAngRot1 - angRot1));
			newAngVel2 = this._body1.get_currentState().rotVelocity.add(JNumber3D.getScaleVector(hingeAxis, newAngRot2 - angRot2));

			this._body0.setAngleVelocity(newAngVel1);
			this._body1.setAngleVelocity(newAngVel2);
		}

		if (this._extraTorque != 0)
		{
			var torque1 = this._body0.get_currentState().orientation.transformVector(this._hingeAxis);
			torque1 = JNumber3D.getScaleVector(torque1, this._extraTorque);

			this._body0.addWorldTorque(torque1);
			this._body1.addWorldTorque(JNumber3D.getScaleVector(torque1, -1));
		}
		
	}



	jiglib.HingeJoint = HingeJoint; 

})(jiglib);


(function(jiglib) {

	var MaterialProperties = jiglib.MaterialProperties;
	var PhysicsController = jiglib.PhysicsController;
	var CachedImpulse = jiglib.CachedImpulse;
	var PhysicsState = jiglib.PhysicsState;
	var RigidBody = jiglib.RigidBody;
	var HingeJoint = jiglib.HingeJoint;
	var PhysicsSystem = jiglib.PhysicsSystem;
	var Vector3D = jiglib.Vector3D;

	var BodyPair = function(_body0, _body1, r0, r1)
	{
		this.body0 = null; // RigidBody
		this.body1 = null; // RigidBody
		this.r = null; // Vector3D


		var id1 = -1;
		if (_body1 != null) id1 = _body1.get_id();
		
		if (_body0.get_id() > id1)
		{
			this.body0 = _body0;
			this.body1 = _body1;
			this.r = r0;
		}
		else
		{
			this.body0 = _body1;
			this.body1 = _body0;
			this.r = r1;
		}
		
	}



	jiglib.BodyPair = BodyPair; 

})(jiglib);


(function(jiglib) {

	var MaterialProperties = jiglib.MaterialProperties;
	var PhysicsController = jiglib.PhysicsController;
	var CachedImpulse = jiglib.CachedImpulse;
	var PhysicsState = jiglib.PhysicsState;
	var RigidBody = jiglib.RigidBody;
	var HingeJoint = jiglib.HingeJoint;
	var BodyPair = jiglib.BodyPair;
	var Vector3D = jiglib.Vector3D;
	var JConfig = jiglib.JConfig;
	var CollDetectInfo = jiglib.CollDetectInfo;
	var CollPointInfo = jiglib.CollPointInfo;
	var CollisionInfo = jiglib.CollisionInfo;
	var CollisionSystemAbstract = jiglib.CollisionSystemAbstract;
	var CollisionSystemBrute = jiglib.CollisionSystemBrute;
	var CollisionSystemGrid = jiglib.CollisionSystemGrid;
	var ContactData = jiglib.ContactData;
	var JNumber3D = jiglib.JNumber3D;
	var JMath3D = jiglib.JMath3D;
	var JConstraint = jiglib.JConstraint;

	var PhysicsSystem = function()
	{
		this._maxVelMag =  0.5; // Number
		this._minVelForProcessing =  0.001; // Number
		this._bodies = null; // RigidBody
		this._activeBodies = null; // RigidBody
		this._collisions = null; // CollisionInfo
		this._constraints = null; // JConstraint
		this._controllers = null; // PhysicsController
		this._gravityAxis = null; // int
		this._gravity = null; // Vector3D
		this._doingIntegration = null; // Boolean
		this.preProcessCollisionFn = null; // Function
		this.preProcessContactFn = null; // Function
		this.processCollisionFn = null; // Function
		this.processContactFn = null; // Function
		this._cachedContacts = null; // ContactData
		this._collisionSystem = null; // CollisionSystemAbstract

		this.setSolverType(JConfig.solverType);
		this._doingIntegration = false;
		this._bodies = [];
		this._collisions = [];
		this._activeBodies = [];
		this._constraints = [];
		this._controllers = [];
		
		this._cachedContacts = [];
		
		this.setGravity(JNumber3D.getScaleVector(Vector3D.Y_AXIS, -10));
		
	}

	PhysicsSystem.prototype.setCollisionSystem = function(collisionSystemGrid, sx, sy, sz, nx, ny, nz, dx, dy, dz)
	{
		if (collisionSystemGrid == null) collisionSystemGrid = false;
		if (nx == null) nx = 20;
		if (ny == null) ny = 20;
		if (nz == null) nz = 20;
		if (dx == null) dx = 200;
		if (dy == null) dy = 200;
		if (dz == null) dz = 200;

		// which collisionsystem to use grid / brute
		if (collisionSystemGrid)
		{
			this._collisionSystem = new CollisionSystemGrid(sx, sy, sz, nx, ny, nz, dx, dy, dz);
		}
		else {
			this._collisionSystem = new CollisionSystemBrute(); // brute by default	
		}
		
	}

	PhysicsSystem.prototype.getCollisionSystem = function()
	{

		return this._collisionSystem;
		
	}

	PhysicsSystem.prototype.setGravity = function(gravity)
	{

		this._gravity = gravity;
		if (this._gravity.x == this._gravity.y && this._gravity.y == this._gravity.z)
			this._gravityAxis = -1;
		
		this._gravityAxis = 0;
		if (Math.abs(this._gravity.y) > Math.abs(this._gravity.z))
			this._gravityAxis = 1;
		
		if (Math.abs(this._gravity.z) > Math.abs(JNumber3D.toArray(this._gravity)[this._gravityAxis]))
			this._gravityAxis = 2;
		
		// do update only when dirty, faster than call every time in step
		for (var _bodies_i = 0, _bodies_l = this._bodies.length, body; (_bodies_i < _bodies_l) && (body = this._bodies[_bodies_i]); _bodies_i++)
			body.updateGravity(this._gravity, this._gravityAxis);
		
	}

	PhysicsSystem.prototype.get_gravity = function()
	{

		return this._gravity;
		
	}

	PhysicsSystem.prototype.get_gravityAxis = function()
	{

		return this._gravityAxis;
		
	}

	PhysicsSystem.prototype.get_bodies = function()
	{

		return this._bodies;
		
	}

	PhysicsSystem.prototype.get_activeBodies = function()
	{

		return this._activeBodies;
		
	}

	PhysicsSystem.prototype.get_constraints = function()
	{

		return this._constraints;
		
	}

	PhysicsSystem.prototype.addBody = function(body)
	{

		if (this._bodies.indexOf(body) < 0)
		{
			this._bodies.push(body);
			this._collisionSystem.addCollisionBody(body);
			
			// update only once, and callback later when dirty
			body.updateGravity(this._gravity, this._gravityAxis);
		}
		
	}

	PhysicsSystem.prototype.removeBody = function(body)
	{

		if (this._bodies.indexOf(body) >= 0)
		{
			this._bodies.splice(this._bodies.indexOf(body), 1);
			this._collisionSystem.removeCollisionBody(body);
		}
		
	}

	PhysicsSystem.prototype.removeAllBodies = function()
	{

		this._bodies.length=0;
		this._collisionSystem.removeAllCollisionBodies();
		
	}

	PhysicsSystem.prototype.addConstraint = function(constraint)
	{

		if (this._constraints.indexOf(constraint) < 0)
			this._constraints.push(constraint);
		
	}

	PhysicsSystem.prototype.removeConstraint = function(constraint)
	{

		if (this._constraints.indexOf(constraint) >= 0)
			this._constraints.splice(this._constraints.indexOf(constraint), 1);
		
	}

	PhysicsSystem.prototype.removeAllConstraints = function()
	{

		for (var _constraints_i = 0, _constraints_l = this._constraints.length, constraint; (_constraints_i < _constraints_l) && (constraint = this._constraints[_constraints_i]); _constraints_i++) {
			constraint.disableConstraint();
		}
		this._constraints.length = 0;
		
	}

	PhysicsSystem.prototype.addController = function(controller)
	{

		if (this._controllers.indexOf(controller) < 0)
			this._controllers.push(controller);
		
	}

	PhysicsSystem.prototype.removeController = function(controller)
	{

		if (this._controllers.indexOf(controller) >= 0)
			this._controllers.splice(this._controllers.indexOf(controller), 1);
		
	}

	PhysicsSystem.prototype.removeAllControllers = function()
	{

		for (var _controllers_i = 0, _controllers_l = this._controllers.length, controller; (_controllers_i < _controllers_l) && (controller = this._controllers[_controllers_i]); _controllers_i++) {
			controller.disableController();
		}
		this._controllers.length=0;
		
	}

	PhysicsSystem.prototype.setSolverType = function(type)
	{

		switch (type)
		{
			case "FAST":
				this.preProcessCollisionFn = this.preProcessCollisionFast;
				this.preProcessContactFn = this.preProcessCollisionFast;
				this.processCollisionFn = this.processCollisionNormal;
				this.processContactFn = this.processCollisionNormal;
				return;
			case "NORMAL":
				this.preProcessCollisionFn = this.preProcessCollisionNormal;
				this.preProcessContactFn = this.preProcessCollisionNormal;
				this.processCollisionFn = this.processCollisionNormal;
				this.processContactFn = this.processCollisionNormal;
				return;
			case "ACCUMULATED":
				this.preProcessCollisionFn = this.preProcessCollisionNormal;
				this.preProcessContactFn = this.preProcessCollisionAccumulated;
				this.processCollisionFn = this.processCollisionNormal;
				this.processContactFn = this.processCollisionAccumulated;
				return;
			default:
				this.preProcessCollisionFn = this.preProcessCollisionNormal;
				this.preProcessContactFn = this.preProcessCollisionNormal;
				this.processCollisionFn = this.processCollisionNormal;
				this.processContactFn = this.processCollisionNormal;
				return;
		}
		
	}

	PhysicsSystem.prototype.moreCollPtPenetration = function(info0, info1)
	{

		if (info0.initialPenetration < info1.initialPenetration)
			return 1;
		else if (info0.initialPenetration > info1.initialPenetration)
			return -1;
		else
			return 0;
		
	}

	PhysicsSystem.prototype.preProcessCollisionFast = function(collision, dt)
	{

		collision.satisfied = false;
		
		var body0, body1;
		
		body0 = collision.objInfo.body0;
		body1 = collision.objInfo.body1;
		
		var N = collision.dirToBody, tempV;
		var timescale = JConfig.numPenetrationRelaxationTimesteps * dt, approachScale = 0, tiny=JMath3D.NUM_TINY, allowedPenetration=JConfig.allowedPenetration;
		var ptInfo;
		var collision_pointInfo = collision.pointInfo;
		
		if (collision_pointInfo.length > 3)
		{
			collision_pointInfo=collision_pointInfo.sort(this.moreCollPtPenetration);
			collision_pointInfo.fixed=false;
			collision_pointInfo.length=3;
			collision_pointInfo.fixed=true;
		}
		
		for (var collision_pointInfo_i = 0, collision_pointInfo_l = collision_pointInfo.length, ptInfo; (collision_pointInfo_i < collision_pointInfo_l) && (ptInfo = collision_pointInfo[collision_pointInfo_i]); collision_pointInfo_i++)
		{
			if (!body0.get_movable())
			{
				ptInfo.denominator = 0;
			}
			else
			{
				tempV = ptInfo.r0.crossProduct(N);
				tempV = body0.get_worldInvInertia().transformVector(tempV);
				ptInfo.denominator = body0.get_invMass() + N.dotProduct(tempV.crossProduct(ptInfo.r0));
			}
			
			if (body1 && body1.get_movable())
			{
				tempV = ptInfo.r1.crossProduct(N);
				tempV = body1.get_worldInvInertia().transformVector(tempV);
				ptInfo.denominator += (body1.get_invMass() + N.dotProduct(tempV.crossProduct(ptInfo.r1)));
			}
			
			if (ptInfo.denominator < tiny)
				ptInfo.denominator = tiny;
			
			if (ptInfo.initialPenetration > allowedPenetration)
			{
				ptInfo.minSeparationVel = (ptInfo.initialPenetration - allowedPenetration) / timescale;
			}
			else
			{
				approachScale = -0.1 * (ptInfo.initialPenetration - allowedPenetration) / allowedPenetration;
				
				if (approachScale < tiny)
				{
				approachScale = tiny;
				}
				else if (approachScale > 1)
				{
				approachScale = 1;
				}
				
				ptInfo.minSeparationVel = approachScale * (ptInfo.initialPenetration - allowedPenetration) / dt;
			}
			
			if (ptInfo.minSeparationVel > this._maxVelMag)
				ptInfo.minSeparationVel = this._maxVelMag;
		}
		
	}

	PhysicsSystem.prototype.preProcessCollisionNormal = function(collision, dt)
	{

		collision.satisfied = false;
		
		var body0, body1;
		
		body0 = collision.objInfo.body0;
		body1 = collision.objInfo.body1;
		
		var N = collision.dirToBody, tempV;
		var timescale = JConfig.numPenetrationRelaxationTimesteps * dt, approachScale = 0, tiny=JMath3D.NUM_TINY, allowedPenetration=JConfig.allowedPenetration;
		var ptInfo;
		var collision_pointInfo = collision.pointInfo;
		
		for (var collision_pointInfo_i = 0, collision_pointInfo_l = collision_pointInfo.length, ptInfo; (collision_pointInfo_i < collision_pointInfo_l) && (ptInfo = collision_pointInfo[collision_pointInfo_i]); collision_pointInfo_i++)
		{
			if (!body0.get_movable())
			{
				ptInfo.denominator = 0;
			}
			else
			{
				tempV = ptInfo.r0.crossProduct(N);
				tempV = body0.get_worldInvInertia().transformVector(tempV);
				ptInfo.denominator = body0.get_invMass() + N.dotProduct(tempV.crossProduct(ptInfo.r0));
			}
			
			if (body1 && body1.get_movable())
			{
				tempV = ptInfo.r1.crossProduct(N);
				tempV = body1.get_worldInvInertia().transformVector(tempV);
				ptInfo.denominator += (body1.get_invMass() + N.dotProduct(tempV.crossProduct(ptInfo.r1)));
			}
			
			if (ptInfo.denominator < tiny)
				ptInfo.denominator = tiny;
			
			if (ptInfo.initialPenetration > allowedPenetration)
			{
				ptInfo.minSeparationVel = (ptInfo.initialPenetration - allowedPenetration) / timescale;
			}
			else
			{
				approachScale = -0.1 * (ptInfo.initialPenetration - allowedPenetration) / allowedPenetration;
				
				if (approachScale < tiny)
				{
				approachScale = tiny;
				}
				else if (approachScale > 1)
				{
				approachScale = 1;
				}
				ptInfo.minSeparationVel = approachScale * (ptInfo.initialPenetration - allowedPenetration) / dt;
			}
			
			if (ptInfo.minSeparationVel > this._maxVelMag)
				ptInfo.minSeparationVel = this._maxVelMag;
		}
		
	}

	PhysicsSystem.prototype.preProcessCollisionAccumulated = function(collision, dt)
	{

		collision.satisfied = false;
		
		var body0, body1;
		
		body0 = collision.objInfo.body0;
		body1 = collision.objInfo.body1;
		
		var N = collision.dirToBody, tempV;
		var timescale = JConfig.numPenetrationRelaxationTimesteps * dt, approachScale = 0, numTiny = JMath3D.NUM_TINY, allowedPenetration = JConfig.allowedPenetration;
		var ptInfo;
		var collision_pointInfo = collision.pointInfo;
		
		for (var collision_pointInfo_i = 0, collision_pointInfo_l = collision_pointInfo.length, ptInfo; (collision_pointInfo_i < collision_pointInfo_l) && (ptInfo = collision_pointInfo[collision_pointInfo_i]); collision_pointInfo_i++)
		{
			if (!body0.get_movable())
			{
				ptInfo.denominator = 0;
			}
			else
			{
				tempV = ptInfo.r0.crossProduct(N);
				tempV = body0.get_worldInvInertia().transformVector(tempV);
				ptInfo.denominator = body0.get_invMass() + N.dotProduct(tempV.crossProduct(ptInfo.r0));
			}
			
			if (body1 && body1.get_movable())
			{
				tempV = ptInfo.r1.crossProduct(N);
				tempV = body1.get_worldInvInertia().transformVector(tempV);
				ptInfo.denominator += (body1.get_invMass() + N.dotProduct(tempV.crossProduct(ptInfo.r1)));
			}
			
			if (ptInfo.denominator < numTiny)
			{
				ptInfo.denominator = numTiny;
			}
			
			if (ptInfo.initialPenetration > allowedPenetration)
			{
				ptInfo.minSeparationVel = (ptInfo.initialPenetration - allowedPenetration) / timescale;
			}
			else
			{
				approachScale = -0.1 * (ptInfo.initialPenetration - allowedPenetration) / allowedPenetration;
				
				if (approachScale < numTiny)
				{
				approachScale = numTiny;
				}
				else if (approachScale > 1)
				{
				approachScale = 1;
				}
				
				ptInfo.minSeparationVel = approachScale * (ptInfo.initialPenetration - allowedPenetration) / Math.max(dt, numTiny);
			}
			
			ptInfo.accumulatedNormalImpulse = 0;
			ptInfo.accumulatedNormalImpulseAux = 0;
			ptInfo.accumulatedFrictionImpulse = new Vector3D();
			
			var bestDistSq = 0.04;
			var bp = new BodyPair(body0, body1, new Vector3D(), new Vector3D());
			
			for (var _cachedContacts_i = 0, _cachedContacts_l = this._cachedContacts.length, cachedContact; (_cachedContacts_i < _cachedContacts_l) && (cachedContact = this._cachedContacts[_cachedContacts_i]); _cachedContacts_i++)
			{
				if (!(bp.body0 == cachedContact.pair.body0 && bp.body1 == cachedContact.pair.body1))
				continue;
				
				var distSq = (cachedContact.pair.body0 == body0) ? cachedContact.pair.r.subtract(ptInfo.r0).get_lengthSquared() : cachedContact.pair.r.subtract(ptInfo.r1).get_lengthSquared();
				
				if (distSq < bestDistSq)
				{
				bestDistSq = distSq;
				ptInfo.accumulatedNormalImpulse = cachedContact.impulse.normalImpulse;
				ptInfo.accumulatedNormalImpulseAux = cachedContact.impulse.normalImpulseAux;
				ptInfo.accumulatedFrictionImpulse = cachedContact.impulse.frictionImpulse;
				
				if (cachedContact.pair.body0 != body0)
					ptInfo.accumulatedFrictionImpulse = JNumber3D.getScaleVector(ptInfo.accumulatedFrictionImpulse, -1);
				}
			}
			
			if (ptInfo.accumulatedNormalImpulse != 0)
			{
				var impulse = JNumber3D.getScaleVector(N, ptInfo.accumulatedNormalImpulse);
				impulse = impulse.add(ptInfo.accumulatedFrictionImpulse);
				body0.applyBodyWorldImpulse(impulse, ptInfo.r0, false);
				if (body1)
				body1.applyBodyWorldImpulse(JNumber3D.getScaleVector(impulse, -1), ptInfo.r1, false);
			}
			
			if (ptInfo.accumulatedNormalImpulseAux != 0)
			{
				impulse = JNumber3D.getScaleVector(N, ptInfo.accumulatedNormalImpulseAux);
				body0.applyBodyWorldImpulseAux(impulse, ptInfo.r0, false);
				if (body1)
				body1.applyBodyWorldImpulseAux(JNumber3D.getScaleVector(impulse, -1), ptInfo.r1, false);
			}
		}
		
	}

	PhysicsSystem.prototype.processCollisionNormal = function(collision, dt)
	{

		collision.satisfied = true;
		
		var body0, body1;
		
		body0 = collision.objInfo.body0;
		body1 = collision.objInfo.body1;
		
		var gotOne=false;
		var deltaVel=0, normalVel=0, finalNormalVel=0, normalImpulse=0, tangent_speed, denominator, impulseToReverse, impulseFromNormalImpulse, frictionImpulse, tiny=JMath3D.NUM_TINY;
		var N = collision.dirToBody, impulse, Vr0, Vr1, tempV, VR, tangent_vel, T;
		var ptInfo;
		
		var collision_pointInfo = collision.pointInfo;
		
		for (var collision_pointInfo_i = 0, collision_pointInfo_l = collision_pointInfo.length, ptInfo; (collision_pointInfo_i < collision_pointInfo_l) && (ptInfo = collision_pointInfo[collision_pointInfo_i]); collision_pointInfo_i++)
		{
			Vr0 = body0.getVelocity(ptInfo.r0);
			if (body1){
				Vr1 = body1.getVelocity(ptInfo.r1);
				normalVel = Vr0.subtract(Vr1).dotProduct(N);
			}else{
				normalVel = Vr0.dotProduct(N);
			} 
			if (normalVel > ptInfo.minSeparationVel)
				continue;
			
			finalNormalVel = -1 * collision.mat.restitution * normalVel;
			
			if (finalNormalVel < this._minVelForProcessing)
				finalNormalVel = ptInfo.minSeparationVel;
			
			deltaVel = finalNormalVel - normalVel;
			
			if (deltaVel <= this._minVelForProcessing)
				continue;
			
			normalImpulse = deltaVel / ptInfo.denominator;
			
			gotOne = true;
			impulse = JNumber3D.getScaleVector(N, normalImpulse);
			
			body0.applyBodyWorldImpulse(impulse, ptInfo.r0, false);
			if(body1)body1.applyBodyWorldImpulse(JNumber3D.getScaleVector(impulse, -1), ptInfo.r1, false);
			
			VR = Vr0.clone();
			if (body1) VR = VR.subtract(Vr1);
			tangent_vel = VR.subtract(JNumber3D.getScaleVector(N, VR.dotProduct(N)));
			tangent_speed = tangent_vel.get_length();
			
			if (tangent_speed > this._minVelForProcessing)
			{
				T = JNumber3D.getDivideVector(tangent_vel, -tangent_speed);
				denominator = 0;
				
				if (body0.get_movable())
				{
				tempV = ptInfo.r0.crossProduct(T);
				tempV = body0.get_worldInvInertia().transformVector(tempV);
				denominator = body0.get_invMass() + T.dotProduct(tempV.crossProduct(ptInfo.r0));
				}
				
				if (body1 && body1.get_movable())
				{
				tempV = ptInfo.r1.crossProduct(T);
				tempV = body1.get_worldInvInertia().transformVector(tempV);
				denominator += (body1.get_invMass() + T.dotProduct(tempV.crossProduct(ptInfo.r1)));
				}
				
				if (denominator > tiny)
				{
				impulseToReverse = tangent_speed / denominator;
				
				impulseFromNormalImpulse = collision.mat.friction * normalImpulse;
				if (impulseToReverse < impulseFromNormalImpulse) {
					frictionImpulse = impulseToReverse;
				}else {
					frictionImpulse = collision.mat.friction * normalImpulse;
				}
				T.scaleBy(frictionImpulse);
				body0.applyBodyWorldImpulse(T, ptInfo.r0, false);
				if(body1)body1.applyBodyWorldImpulse(JNumber3D.getScaleVector(T, -1), ptInfo.r1, false);
				}
			}
		}
		
		if (gotOne)
		{
			body0.setConstraintsAndCollisionsUnsatisfied();
			if(body1)body1.setConstraintsAndCollisionsUnsatisfied();
		}
		
		return gotOne;
		
	}

	PhysicsSystem.prototype.processCollisionAccumulated = function(collision, dt)
	{

		collision.satisfied = true;
		
		var body0, body1;
		body0 = collision.objInfo.body0;
		body1 = collision.objInfo.body1;
		
		var gotOne=false;
		var deltaVel=0, normalVel=0, finalNormalVel=0, normalImpulse=0, tangent_speed, denominator, impulseToReverse, AFIMag, maxAllowedAFIMag, tiny=JMath3D.NUM_TINY;
		var N = collision.dirToBody, impulse, Vr0, Vr1, tempV, VR, tangent_vel, T, frictionImpulseVec, origAccumulatedFrictionImpulse, actualFrictionImpulse;
		var ptInfo;
		
		var collision_pointInfo = collision.pointInfo;
		
		for (var collision_pointInfo_i = 0, collision_pointInfo_l = collision_pointInfo.length, ptInfo; (collision_pointInfo_i < collision_pointInfo_l) && (ptInfo = collision_pointInfo[collision_pointInfo_i]); collision_pointInfo_i++)
		{
			Vr0 = body0.getVelocity(ptInfo.r0);
			if (body1){
				Vr1 = body1.getVelocity(ptInfo.r1);
				normalVel = Vr0.subtract(Vr1).dotProduct(N);
			}else{
				normalVel = Vr0.dotProduct(N);
			}
			deltaVel = -normalVel;
			
			if (ptInfo.minSeparationVel < 0)
				deltaVel += ptInfo.minSeparationVel;
			
			if (Math.abs(deltaVel) > this._minVelForProcessing)
			{
				normalImpulse = deltaVel / ptInfo.denominator;
				var origAccumulatedNormalImpulse = ptInfo.accumulatedNormalImpulse;
				ptInfo.accumulatedNormalImpulse = Math.max(ptInfo.accumulatedNormalImpulse + normalImpulse, 0);
				var actualImpulse = ptInfo.accumulatedNormalImpulse - origAccumulatedNormalImpulse;
				
				impulse = JNumber3D.getScaleVector(N, actualImpulse);
				body0.applyBodyWorldImpulse(impulse, ptInfo.r0, false);
				if(body1)body1.applyBodyWorldImpulse(JNumber3D.getScaleVector(impulse, -1), ptInfo.r1, false);
				
				gotOne = true;
			}
			
			Vr0 = body0.getVelocityAux(ptInfo.r0);
			if (body1){
				Vr1 = body1.getVelocityAux(ptInfo.r1);
				normalVel = Vr0.subtract(Vr1).dotProduct(N);
			}else{
				normalVel = Vr0.dotProduct(N);
			}
			
			deltaVel = -normalVel;
			
			if (ptInfo.minSeparationVel > 0)
				deltaVel += ptInfo.minSeparationVel;
			
			if (Math.abs(deltaVel) > this._minVelForProcessing)
			{
				normalImpulse = deltaVel / ptInfo.denominator;
				origAccumulatedNormalImpulse = ptInfo.accumulatedNormalImpulseAux;
				ptInfo.accumulatedNormalImpulseAux = Math.max(ptInfo.accumulatedNormalImpulseAux + normalImpulse, 0);
				actualImpulse = ptInfo.accumulatedNormalImpulseAux - origAccumulatedNormalImpulse;
				
				impulse = JNumber3D.getScaleVector(N, actualImpulse);
				body0.applyBodyWorldImpulseAux(impulse, ptInfo.r0, false);
				if(body1)body1.applyBodyWorldImpulseAux(JNumber3D.getScaleVector(impulse, -1), ptInfo.r1, false);
				
				gotOne = true;
			}
			
			if (ptInfo.accumulatedNormalImpulse > 0)
			{
				Vr0 = body0.getVelocity(ptInfo.r0);
				VR = Vr0.clone();
				if (body1){
				Vr1 = body1.getVelocity(ptInfo.r1);
				VR = VR.subtract(Vr1);
				} 
				tangent_vel = VR.subtract(JNumber3D.getScaleVector(N, VR.dotProduct(N)));
				tangent_speed = tangent_vel.get_length();
				
				if (tangent_speed > this._minVelForProcessing)
				{
				
				T = JNumber3D.getScaleVector(JNumber3D.getDivideVector(tangent_vel, tangent_speed), -1);
				denominator = 0;
				if (body0.get_movable())
				{
					tempV = ptInfo.r0.crossProduct(T);
					tempV = body0.get_worldInvInertia().transformVector(tempV);
					denominator = body0.get_invMass() + T.dotProduct(tempV.crossProduct(ptInfo.r0));
				}
				
				if (body1 && body1.get_movable())
				{
					tempV = ptInfo.r1.crossProduct(T);
					tempV = body1.get_worldInvInertia().transformVector(tempV);
					denominator += (body1.get_invMass() + T.dotProduct(tempV.crossProduct(ptInfo.r1)));
				}
				
				if (denominator > tiny)
				{
					impulseToReverse = tangent_speed / denominator;
					frictionImpulseVec = JNumber3D.getScaleVector(T, impulseToReverse);
					
					origAccumulatedFrictionImpulse = ptInfo.accumulatedFrictionImpulse.clone();
					ptInfo.accumulatedFrictionImpulse = ptInfo.accumulatedFrictionImpulse.add(frictionImpulseVec);
					
					AFIMag = ptInfo.accumulatedFrictionImpulse.get_length();
					maxAllowedAFIMag = collision.mat.friction * ptInfo.accumulatedNormalImpulse;
					
					if (AFIMag > tiny && AFIMag > maxAllowedAFIMag)
						ptInfo.accumulatedFrictionImpulse = JNumber3D.getScaleVector(ptInfo.accumulatedFrictionImpulse, maxAllowedAFIMag / AFIMag);
					
					actualFrictionImpulse = ptInfo.accumulatedFrictionImpulse.subtract(origAccumulatedFrictionImpulse);
					
					body0.applyBodyWorldImpulse(actualFrictionImpulse, ptInfo.r0, false);
					if(body1)body1.applyBodyWorldImpulse(JNumber3D.getScaleVector(actualFrictionImpulse, -1), ptInfo.r1, false);
				}
				}
			}
		}
		
		if (gotOne)
		{
			body0.setConstraintsAndCollisionsUnsatisfied();
			if(body1)body1.setConstraintsAndCollisionsUnsatisfied();
		}
		
		return gotOne;
		
	}

	PhysicsSystem.prototype.processCollisionForShock = function(collision, dt)
	{

		
		collision.satisfied = true;
		var N = collision.dirToBody;
		
		var timescale = JConfig.numPenetrationRelaxationTimesteps * dt;
		var body0 = collision.objInfo.body0;
		var body1 = collision.objInfo.body1;
		
		if (!body0.get_movable())
			body0 = null;
		if (body1 && !body1.get_movable())
			body1 = null;
		
		if (!body0 && !body1) {
			return false;
		}
		
		var normalVel=0;
		var finalNormalVel;
		var impulse;
		var orig;
		var actualImpulse;
		
		for (var pointInfo_i = 0, pointInfo_l = collision.pointInfo.length, ptInfo; (pointInfo_i < pointInfo_l) && (ptInfo = collision.pointInfo[pointInfo_i]); pointInfo_i++) {
			normalVel=0;
			if (body0) {
				normalVel = body0.getVelocity(ptInfo.r0).dotProduct(N) + body0.getVelocityAux(ptInfo.r0).dotProduct(N);
			}
			if (body1) {
				normalVel -= (body1.getVelocity(ptInfo.r1).dotProduct(N) + body1.getVelocityAux(ptInfo.r1).dotProduct(N));
			}
			
			finalNormalVel = (ptInfo.initialPenetration - JConfig.allowedPenetration) / timescale;
			if (finalNormalVel < 0) {
				continue;
			}
			impulse = (finalNormalVel - normalVel) / ptInfo.denominator;
			orig = ptInfo.accumulatedNormalImpulseAux;
			ptInfo.accumulatedNormalImpulseAux = Math.max(ptInfo.accumulatedNormalImpulseAux + impulse, 0);
			actualImpulse = JNumber3D.getScaleVector(N, ptInfo.accumulatedNormalImpulseAux - orig);
			
			if (body0)body0.applyBodyWorldImpulse(actualImpulse, ptInfo.r0, false);
			if (body1)body1.applyBodyWorldImpulse(JNumber3D.getScaleVector(actualImpulse, -1), ptInfo.r1, false);
		}
		
		if (body0)body0.setConstraintsAndCollisionsUnsatisfied();
		if (body1)body1.setConstraintsAndCollisionsUnsatisfied();
		return true;
		
	}

	PhysicsSystem.prototype.sortPositionX = function(body0, body1)
	{

		if (body0.get_currentState().position.x < body1.get_currentState().position.x)
			return -1;
		else if (body0.get_currentState().position.x > body1.get_currentState().position.x)
			return 1;
		else
			return 0;
		
	}

	PhysicsSystem.prototype.sortPositionY = function(body0, body1)
	{

		if (body0.get_currentState().position.y < body1.get_currentState().position.y)
			return -1;
		else if (body0.get_currentState().position.y > body1.get_currentState().position.y)
			return 1;
		else
			return 0;
		
	}

	PhysicsSystem.prototype.sortPositionZ = function(body0, body1)
	{

		if (body0.get_currentState().position.z < body1.get_currentState().position.z)
			return -1;
		else if (body0.get_currentState().position.z > body1.get_currentState().position.z)
			return 1;
		else
			return 0;
		
	}

	PhysicsSystem.prototype.doShockStep = function(dt)
	{

		if (Math.abs(this._gravity.x) > Math.abs(this._gravity.y) && Math.abs(this._gravity.x) > Math.abs(this._gravity.z))
		{
			this._bodies = this._bodies.sort(this.sortPositionX);
			this._collisionSystem.collBody = this._collisionSystem.collBody.sort(this.sortPositionX);
		}
		else if (Math.abs(this._gravity.y) > Math.abs(this._gravity.z) && Math.abs(this._gravity.y) > Math.abs(this._gravity.x))
		{
			this._bodies = this._bodies.sort(this.sortPositionY);
			this._collisionSystem.collBody = this._collisionSystem.collBody.sort(this.sortPositionY);
		}
		else if (Math.abs(this._gravity.z) > Math.abs(this._gravity.x) && Math.abs(this._gravity.z) > Math.abs(this._gravity.y))
		{
			this._bodies = this._bodies.sort(this.sortPositionZ);
			this._collisionSystem.collBody = this._collisionSystem.collBody.sort(this.sortPositionZ);
		}
		
		var setImmovable, gotOne=true;
		var info;
		var body0, body1;
		 
		for (var _bodies_i = 0, _bodies_l = this._bodies.length, body; (_bodies_i < _bodies_l) && (body = this._bodies[_bodies_i]); _bodies_i++)
		{
			if (body.get_movable())
			{
				if (body.collisions.length == 0 || !body.isActive)
				{
				body.internalSetImmovable();
				}
				else
				{
				setImmovable = false;
				for (var collisions_i = 0, collisions_l = body.collisions.length, info; (collisions_i < collisions_l) && (info = body.collisions[collisions_i]); collisions_i++)
				{
					body0 = info.objInfo.body0;
					body1 = info.objInfo.body1;
					
					if ((body0 == body && (!body1 || !body1.get_movable())) || (body1 == body && (!body0 || !body0.get_movable())))
					{
						this.preProcessCollisionFn(info, dt);
						this.processCollisionForShock(info, dt);
						setImmovable = true;
					}
				}
				
				if (setImmovable)
				{
					body.internalSetImmovable();
				}
				}
			}
		}
		
		for (var _bodies_i = 0, _bodies_l = this._bodies.length, body; (_bodies_i < _bodies_l) && (body = this._bodies[_bodies_i]); _bodies_i++)
		{
			body.internalRestoreImmovable();
		}
		
	}

	PhysicsSystem.prototype.updateContactCache = function()
	{

		this._cachedContacts = [];
		
		var fricImpulse, body0, body1, contact, collInfo_objInfo, collInfo_pointInfo;
		var i = 0, id1;
		for (var _collisions_i = 0, _collisions_l = this._collisions.length, collInfo; (_collisions_i < _collisions_l) && (collInfo = this._collisions[_collisions_i]); _collisions_i++)
		{
			collInfo_objInfo = collInfo.objInfo;
			body0 = collInfo_objInfo.body0;
			body1 = collInfo_objInfo.body1;
			
			collInfo_pointInfo = collInfo.pointInfo;
			this._cachedContacts.fixed = false;
			this._cachedContacts.length += collInfo_pointInfo.length;
			this._cachedContacts.fixed = true;
			
			for (var collInfo_pointInfo_i = 0, collInfo_pointInfo_l = collInfo_pointInfo.length, ptInfo; (collInfo_pointInfo_i < collInfo_pointInfo_l) && (ptInfo = collInfo_pointInfo[collInfo_pointInfo_i]); collInfo_pointInfo_i++)
			{
				id1=-1;
				if (body1) id1=body1.get_id();
				fricImpulse = (body0.get_id() > id1) ? ptInfo.accumulatedFrictionImpulse : JNumber3D.getScaleVector(ptInfo.accumulatedFrictionImpulse, -1);
				
				this._cachedContacts[i++] = contact = new ContactData();
				contact.pair = new BodyPair(body0, body1, ptInfo.r0, ptInfo.r1);
				contact.impulse = new CachedImpulse(ptInfo.accumulatedNormalImpulse, ptInfo.accumulatedNormalImpulseAux, ptInfo.accumulatedFrictionImpulse);
			}
		}
		
	}

	PhysicsSystem.prototype.handleAllConstraints = function(dt, iter, forceInelastic)
	{

		var origNumCollisions = this._collisions.length, iteration = JConfig.numConstraintIterations, step, i, len;
		var collInfo;
		var constraint;
		var flag, gotOne;
		
		if (this._constraints.length > 0)
		{
			for (var _constraints_i = 0, _constraints_l = this._constraints.length, constraint; (_constraints_i < _constraints_l) && (constraint = this._constraints[_constraints_i]); _constraints_i++)
				constraint.preApply(dt);
			
			for (step = 0; step < iteration; step++)
			{
				gotOne = false;
				for (var _constraints_i = 0, _constraints_l = this._constraints.length, constraint; (_constraints_i < _constraints_l) && (constraint = this._constraints[_constraints_i]); _constraints_i++)
				{
				if (!constraint.satisfied)
				{
					flag = constraint.apply(dt);
					gotOne = gotOne || flag;
				}
				}
				if (!gotOne)
				break;
			}
		}
		
		if (forceInelastic)
		{
			for (var _collisions_i = 0, _collisions_l = this._collisions.length, collInfo; (_collisions_i < _collisions_l) && (collInfo = this._collisions[_collisions_i]); _collisions_i++)
			{
				this.preProcessContactFn(collInfo, dt);
				collInfo.mat.restitution = 0;
				collInfo.satisfied = false;
			}
		}
		else
		{
			for (var _collisions_i = 0, _collisions_l = this._collisions.length, collInfo; (_collisions_i < _collisions_l) && (collInfo = this._collisions[_collisions_i]); _collisions_i++)
				this.preProcessCollisionFn(collInfo, dt);
		}
		
		for (step = 0; step < iter; step++)
		{
			gotOne = true;
			
			for (var _collisions_i = 0, _collisions_l = this._collisions.length, collInfo; (_collisions_i < _collisions_l) && (collInfo = this._collisions[_collisions_i]); _collisions_i++)
			{
				if (!collInfo.satisfied)
				{
				if (forceInelastic)
					flag = this.processContactFn(collInfo, dt);
				else
					flag = this.processCollisionFn(collInfo, dt);
				
				gotOne = gotOne || flag;
				}
			}
			
			len = this._collisions.length;
			if (forceInelastic)
			{
				for (i = origNumCollisions; i < len; i++)
				{
				collInfo = this._collisions[i];
				collInfo.mat.restitution = 0;
				collInfo.satisfied = false;
				this.preProcessContactFn(collInfo, dt);
				}
			}
			else
			{
				for (i = origNumCollisions; i < len; i++)
				this.preProcessCollisionFn(this._collisions[i], dt);
			}
			
			origNumCollisions = len;
			
			if (!gotOne)
				break;
		}
		
	}

	PhysicsSystem.prototype.activateObject = function(body)
	{

		if (!body.get_movable() || body.isActive) return;
		
		if (this._activeBodies.indexOf(body) < 0) {
			body.setActive();
			this._activeBodies.fixed = false;
			this._activeBodies.push(body);
			this._activeBodies.fixed = true;
		}
		
	}

	PhysicsSystem.prototype.tryToActivateAllFrozenObjects = function()
	{

		for (var _bodies_i = 0, _bodies_l = this._bodies.length, body; (_bodies_i < _bodies_l) && (body = this._bodies[_bodies_i]); _bodies_i++)
		{
			if (!body.isActive)
			{
				if (body.getShouldBeActive())
				{
				this.activateObject(body);
				}
				else
				{
				body.setLineVelocity(new Vector3D());
				body.setAngleVelocity(new Vector3D());
				}
			}
		}
		
	}

	PhysicsSystem.prototype.tryToFreezeAllObjects = function(dt)
	{

		for (var _activeBodies_i = 0, _activeBodies_l = this._activeBodies.length, activeBody; (_activeBodies_i < _activeBodies_l) && (activeBody = this._activeBodies[_activeBodies_i]); _activeBodies_i++){
			activeBody.dampForDeactivation();
			activeBody.tryToFreeze(dt);
		}
		
	}

	PhysicsSystem.prototype.activateAllFrozenObjectsLeftHanging = function()
	{

		var other_body;
		var body_collisions;
		
		for (var _activeBodies_i = 0, _activeBodies_l = this._activeBodies.length, body; (_activeBodies_i < _activeBodies_l) && (body = this._activeBodies[_activeBodies_i]); _activeBodies_i++)
		{
			body.doMovementActivations(this);
			body_collisions = body.collisions;
			if (body_collisions.length > 0)
			{
				for (var body_collisions_i = 0, body_collisions_l = body_collisions.length, collisionInfo; (body_collisions_i < body_collisions_l) && (collisionInfo = body_collisions[body_collisions_i]); body_collisions_i++)
				{
				other_body = collisionInfo.objInfo.body0;
				if (other_body == body)
					other_body = collisionInfo.objInfo.body1;
				
				if (!other_body.isActive)
					body.addMovementActivation(body.get_currentState().position, other_body);
				}
			}
		}
		
	}

	PhysicsSystem.prototype.updateAllController = function(dt)
	{

		for (var _controllers_i = 0, _controllers_l = this._controllers.length, controller; (_controllers_i < _controllers_l) && (controller = this._controllers[_controllers_i]); _controllers_i++)
		controller.updateController(dt);
		
	}

	PhysicsSystem.prototype.updateAllVelocities = function(dt)
	{

		for (var _activeBodies_i = 0, _activeBodies_l = this._activeBodies.length, activeBody; (_activeBodies_i < _activeBodies_l) && (activeBody = this._activeBodies[_activeBodies_i]); _activeBodies_i++)
			activeBody.updateVelocity(dt);
		
	}

	PhysicsSystem.prototype.notifyAllPostPhysics = function(dt)
	{

		for (var _activeBodies_i = 0, _activeBodies_l = this._activeBodies.length, activeBody; (_activeBodies_i < _activeBodies_l) && (activeBody = this._activeBodies[_activeBodies_i]); _activeBodies_i++)
			activeBody.postPhysics(dt);
		
	}

	PhysicsSystem.prototype.detectAllCollisions = function(dt)
	{

		for (var _bodies_i = 0, _bodies_l = this._bodies.length, body; (_bodies_i < _bodies_l) && (body = this._bodies[_bodies_i]); _bodies_i++) {
			if (body.isActive) {
				body.storeState();
				body.updateVelocity(dt);
				body.updatePositionWithAux(dt);
			}
			body.collisions.length = 0;
		}
		
		this._collisions.length=0;
		this._collisionSystem.detectAllCollisions(this._activeBodies, this._collisions);
		
		for (var _activeBodies_i = 0, _activeBodies_l = this._activeBodies.length, activeBody; (_activeBodies_i < _activeBodies_l) && (activeBody = this._activeBodies[_activeBodies_i]); _activeBodies_i++)
			activeBody.restoreState();
		
	}

	PhysicsSystem.prototype.findAllActiveBodiesAndCopyStates = function()
	{

		this._activeBodies = [];
		var i = 0;
		
		for (var _bodies_i = 0, _bodies_l = this._bodies.length, body; (_bodies_i < _bodies_l) && (body = this._bodies[_bodies_i]); _bodies_i++)
		{
			// findAllActiveBodies
			if (body.isActive)
			{
				this._activeBodies[i++] = body;
				body.copyCurrentStateToOld();
			}
			
		}
		
		// correct length
		this._activeBodies.fixed = false;
		this._activeBodies.length = i;
		
		// fixed is faster
		this._activeBodies.fixed = true;
		
	}

	PhysicsSystem.prototype.integrate = function(dt)
	{

		this._doingIntegration = true;
		
		this.findAllActiveBodiesAndCopyStates();
		this.updateAllController(dt);
		this.detectAllCollisions(dt);
		this.handleAllConstraints(dt, JConfig.numCollisionIterations, false);
		this.updateAllVelocities(dt);
		this.handleAllConstraints(dt, JConfig.numContactIterations, true);
		
		if (JConfig.doShockStep) this.doShockStep(dt);
		
		this.tryToActivateAllFrozenObjects();
		this.tryToFreezeAllObjects(dt);
		this.activateAllFrozenObjectsLeftHanging();
		
		this.notifyAllPostPhysics(dt);
		
		if (JConfig.solverType == "ACCUMULATED")
			this.updateContactCache();
		
		this._doingIntegration = false;
		
	}

	PhysicsSystem._currentPhysicsSystem = null ; // PhysicsSystem

	PhysicsSystem.getInstance = function()
	{

			if (!PhysicsSystem._currentPhysicsSystem)
			{
				trace("version: JigLibFlash fp11 (2011-10-31)");
				PhysicsSystem._currentPhysicsSystem = new PhysicsSystem();
			}
			return PhysicsSystem._currentPhysicsSystem;
		
	}


	jiglib.PhysicsSystem = PhysicsSystem; 

})(jiglib);


(function(jiglib) {

	var PhysicsSystem = jiglib.PhysicsSystem;
	var Away3D4Physics = jiglib.Away3D4Physics;

	var Stats = function(view3d, physics, grid)
	{
		this.WIDTH =  182; // uint
		this.HEIGHT =  126; // uint
		this.textFpsLabel = null; // TextField
		this.textFps = null; // TextField
		this.textMsLabel = null; // TextField
		this.textMs = null; // TextField
		this.textCDT = null; // TextField
		this.textBottomLeft = null; // TextField
		this.textBottomRight = null; // TextField
		this.textBottom = null; // TextField
		this.timer = null; // uint
		this.fps = null; // uint
		this.ms = null; // uint
		this.ms_prev = null; // uint
		this.mem = null; // Number
		this.mem_max = null; // Number
		this.statsSkinBm = null; // Bitmap
		this.physics = null; // Away3D4Physics
		this.view3d = null; // View3D
		this.grid =  false; // Boolean
		this.StatsSkinBitmap = null; // Class

		this.view3d = view3d;
		this.physics = physics;
		this.grid = grid;
		// TODO: is temp var, make auto detection
		this.mem_max = 0;
		this.textFpsLabel = new TextField();
		this.textFps = new TextField();
		this.textMsLabel = new TextField();
		this.textMs = new TextField();
		this.textBottom = new TextField();
		this.textBottomLeft = new TextField();
		this.textBottomRight = new TextField();
		this.addTextField(this.textFpsLabel, 0xFFFFFF, 10, true, "left", 1, 10, 10);
		this.addTextField(this.textFps, 0x1abfff, 10, true, "left", 1, 40, 10);
		this.addTextField(this.textMsLabel, 0xFFFFFF, 10, true, "left", 1, 96, 10);
		this.addTextField(this.textMs, 0xffcc1a, 10, true, "left", 1, 135, 10, 44);
		this.addTextField(this.textBottomLeft, 0x000000, 10, true, "right", 6, 10, 65, 80, 36);
		this.addTextField(this.textBottomRight, 0x000000, 10, true, "left", 6, 88, 65, 80, 36);
		this.addTextField(this.textBottom, 0x000000, 10, true, "center", 0, 10, 101, 160, 14);

		// headers
		this.textFpsLabel.htmlText = "FPS:<br>CDC:<br>TRI.:";
		this.textMsLabel.htmlText = "TOTAL:<br>JIGLIB:<br>3D:";
		this.textBottom.htmlText = "CDT BRUTEFORCE";
		// TODO once we got a grid system

		// skin used
		this.statsSkinBm = new this.StatsSkinBitmap();

		// add listeners
		addEventListener(Event.ADDED_TO_STAGE, this.init, false, 0, true);
		addEventListener(Event.REMOVED_FROM_STAGE, this.destroy, false, 0, true);
		
	}

	Stats.prototype.init = function(e)
	{

		addChild(this.statsSkinBm);
		addChild(this.textFpsLabel);
		addChild(this.textFps);
		addChild(this.textMsLabel);
		addChild(this.textMs);
		addChild(this.textBottomLeft);
		addChild(this.textBottomRight);
		addChild(this.textBottom);

		addEventListener(Event.ENTER_FRAME, this.update);
		
	}

	Stats.prototype.disableSkin = function()
	{

		removeChild(this.statsSkinBm);
		
	}

	Stats.prototype.update = function(e)
	{

		this.timer = getTimer();

		if ( this.timer - 1000 > this.ms_prev ) {
			this.ms_prev = this.timer;
			this.mem = Number((System.totalMemory * 0.000000954).toFixed(2));
			this.mem_max = this.mem_max > this.mem ? this.mem_max : this.mem;

			this.fps = this.fps > stage.frameRate ? stage.frameRate : this.fps;

			this.textFps.htmlText = this.fps + " / " + stage.frameRate + "<br>" + PhysicsSystem.getInstance().getCollisionSystem().numCollisionsChecks + "<br>" + this.view3d.renderedFacesCount;

			// todo temp. till away3d got _deltatime avail.
			var ms3D = (this.timer - this.ms) - this.physics.get_frameTime();

			this.textMs.htmlText = (this.timer - this.ms) + " this.ms<br>" + this.physics.get_frameTime() + " this.ms<br>" + ms3D + " this.ms";
			this.textBottomLeft.htmlText = "MEM " + this.mem + "<br>RIGIDB. " + PhysicsSystem.getInstance().get_bodies().length;
			this.textBottomRight.htmlText = "/ MAX <font color='#cb2929'>" + this.mem_max + "</font><br>/ ACTIVE <font color='#cb2929'>" + PhysicsSystem.getInstance().get_activeBodies().length + "</font>";
			if (this.grid) {
				this.textBottom.htmlText = "CDT GRID";
			} else {
				this.textBottom.htmlText = "CDT BRUTEFORCE";
			}
			this.fps = 0;
		}
		this.fps++;
		this.ms = this.timer;
		
	}

	Stats.prototype.destroy = function(event)
	{

		while (numChildren > 0)
			removeChildAt(0);

		removeEventListener(Event.ENTER_FRAME, this.update);
		
	}

	Stats.prototype.addTextField = function(text, colorText, textSize, bold, alignText, leading, xPos, yPos, widthText, heightText)
	{
		if (widthText == null) widthText = 52;
		if (heightText == null) heightText = 45;

		text.x = xPos;
		text.y = yPos;
		// setup format
		var format = new TextFormat();
		format.font = "_sans";
		format.color = colorText;
		format.bold = bold;
		format.size = textSize;
		format.align = alignText;
		format.leading = leading;
		text.defaultTextFormat = format;
		// setup text
		text.antiAliasType = AntiAliasType.ADVANCED;
		text.multiline = true;
		text.width = widthText;
		text.height = heightText;
		text.selectable = false;
		text.mouseEnabled = false;
		
	}



	jiglib.Stats = Stats; 

})(jiglib);

