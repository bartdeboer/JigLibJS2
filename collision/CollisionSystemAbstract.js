
var JigLib_CollisionSystemAbstract = function()
{
	this.detectionFunctors = null; // Dictionary
	this.collBody = null; // RigidBody
	this._numCollisionsChecks =  0; // uint
	this.startPoint = null; // Vector3D

		this.collBody = [];
		this.detectionFunctors = [];
		this.detectionFunctors["BOX_BOX"] = new JigLib_CollDetectBoxBox();
		this.detectionFunctors["BOX_SPHERE"] = new JigLib_CollDetectSphereBox();
		this.detectionFunctors["BOX_CAPSULE"] = new JigLib_CollDetectCapsuleBox();
		this.detectionFunctors["BOX_PLANE"] = new JigLib_CollDetectBoxPlane();
		this.detectionFunctors["BOX_TERRAIN"] = new JigLib_CollDetectBoxTerrain();
		this.detectionFunctors["BOX_TRIANGLEMESH"] = new JigLib_CollDetectBoxMesh();
		this.detectionFunctors["SPHERE_BOX"] = new JigLib_CollDetectSphereBox();
		this.detectionFunctors["SPHERE_SPHERE"] = new JigLib_CollDetectSphereSphere();
		this.detectionFunctors["SPHERE_CAPSULE"] = new JigLib_CollDetectSphereCapsule();
		this.detectionFunctors["SPHERE_PLANE"] = new JigLib_CollDetectSpherePlane();
		this.detectionFunctors["SPHERE_TERRAIN"] = new JigLib_CollDetectSphereTerrain();
		this.detectionFunctors["SPHERE_TRIANGLEMESH"] = new JigLib_CollDetectSphereMesh();
		this.detectionFunctors["CAPSULE_CAPSULE"] = new JigLib_CollDetectCapsuleCapsule();
		this.detectionFunctors["CAPSULE_BOX"] = new JigLib_CollDetectCapsuleBox();
		this.detectionFunctors["CAPSULE_SPHERE"] = new JigLib_CollDetectSphereCapsule();
		this.detectionFunctors["CAPSULE_PLANE"] = new JigLib_CollDetectCapsulePlane();
		this.detectionFunctors["CAPSULE_TERRAIN"] = new JigLib_CollDetectCapsuleTerrain();
		this.detectionFunctors["PLANE_BOX"] = new JigLib_CollDetectBoxPlane();
		this.detectionFunctors["PLANE_SPHERE"] = new JigLib_CollDetectSpherePlane();
		this.detectionFunctors["PLANE_CAPSULE"] = new JigLib_CollDetectCapsulePlane();
		this.detectionFunctors["TERRAIN_SPHERE"] = new JigLib_CollDetectSphereTerrain();
		this.detectionFunctors["TERRAIN_BOX"] = new JigLib_CollDetectBoxTerrain();
		this.detectionFunctors["TERRAIN_CAPSULE"] = new JigLib_CollDetectCapsuleTerrain();
		this.detectionFunctors["TRIANGLEMESH_SPHERE"] = new JigLib_CollDetectSphereMesh();
		this.detectionFunctors["TRIANGLEMESH_BOX"] = new JigLib_CollDetectBoxMesh();
		
}

JigLib_CollisionSystemAbstract.prototype.addCollisionBody = function(body)
{

		if (this.collBody.indexOf(body) < 0)
			this.collBody.push(body);
		
}

JigLib_CollisionSystemAbstract.prototype.removeCollisionBody = function(body)
{

		if (this.collBody.indexOf(body) >= 0)
			this.collBody.splice(this.collBody.indexOf(body), 1);
		
}

JigLib_CollisionSystemAbstract.prototype.removeAllCollisionBodies = function()
{

		this.collBody.length=0;
		
}

JigLib_CollisionSystemAbstract.prototype.detectCollisions = function(body, collArr)
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
				info = new JigLib_CollDetectInfo();
				info.body0 = body;
				info.body1 = _collBody;
				fu = this.detectionFunctors[info.body0.get_type() + "_" + info.body1.get_type()];
				fu.collDetect(info, collArr);
			}
		}
		
}

JigLib_CollisionSystemAbstract.prototype.detectAllCollisions = function(bodies, collArr)
{

		
}

JigLib_CollisionSystemAbstract.prototype.collisionSkinMoved = function(colBody)
{

		// used for grid
		
}

JigLib_CollisionSystemAbstract.prototype.segmentIntersect = function(out, seg, ownerBody)
{

		out.frac = JigLib_JMath3D.NUM_HUGE;
		out.position = new JigLib_Vector3D();
		out.normal = new JigLib_Vector3D();
		
		var obj = new JigLib_CollOutBodyData();
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

JigLib_CollisionSystemAbstract.prototype.segmentBounding = function(seg, obj)
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

JigLib_CollisionSystemAbstract.prototype.get_numCollisionsChecks = function()
{

		return this._numCollisionsChecks;	
		
}

JigLib_CollisionSystemAbstract.prototype.checkCollidables = function(body0, body1)
{

		if (body0.get_nonCollidables().length == 0 && body1.get_nonCollidables().length == 0)
			return true;
		
		if(body0.get_nonCollidables().indexOf(body1) > -1)
			return false;
		
		if(body1.get_nonCollidables().indexOf(body0) > -1)
			return false;
		
		return true;
		
}



JigLib.CollisionSystemAbstract = JigLib_CollisionSystemAbstract; 
