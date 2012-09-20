
JigLib.CollisionSystemAbstract = function()
{
	this.detectionFunctors = null; // Dictionary
	this.collBody = null; // RigidBody
	this._numCollisionsChecks =  0; // uint
	this.startPoint = null; // Vector3D

		this.collBody = [];
		this.detectionFunctors = [];
		this.detectionFunctors["BOX_BOX"] = new JigLib.CollDetectBoxBox();
		this.detectionFunctors["BOX_SPHERE"] = new JigLib.CollDetectSphereBox();
		this.detectionFunctors["BOX_CAPSULE"] = new JigLib.CollDetectCapsuleBox();
		this.detectionFunctors["BOX_PLANE"] = new JigLib.CollDetectBoxPlane();
		this.detectionFunctors["BOX_TERRAIN"] = new JigLib.CollDetectBoxTerrain();
		this.detectionFunctors["BOX_TRIANGLEMESH"] = new JigLib.CollDetectBoxMesh();
		this.detectionFunctors["SPHERE_BOX"] = new JigLib.CollDetectSphereBox();
		this.detectionFunctors["SPHERE_SPHERE"] = new JigLib.CollDetectSphereSphere();
		this.detectionFunctors["SPHERE_CAPSULE"] = new JigLib.CollDetectSphereCapsule();
		this.detectionFunctors["SPHERE_PLANE"] = new JigLib.CollDetectSpherePlane();
		this.detectionFunctors["SPHERE_TERRAIN"] = new JigLib.CollDetectSphereTerrain();
		this.detectionFunctors["SPHERE_TRIANGLEMESH"] = new JigLib.CollDetectSphereMesh();
		this.detectionFunctors["CAPSULE_CAPSULE"] = new JigLib.CollDetectCapsuleCapsule();
		this.detectionFunctors["CAPSULE_BOX"] = new JigLib.CollDetectCapsuleBox();
		this.detectionFunctors["CAPSULE_SPHERE"] = new JigLib.CollDetectSphereCapsule();
		this.detectionFunctors["CAPSULE_PLANE"] = new JigLib.CollDetectCapsulePlane();
		this.detectionFunctors["CAPSULE_TERRAIN"] = new JigLib.CollDetectCapsuleTerrain();
		this.detectionFunctors["PLANE_BOX"] = new JigLib.CollDetectBoxPlane();
		this.detectionFunctors["PLANE_SPHERE"] = new JigLib.CollDetectSpherePlane();
		this.detectionFunctors["PLANE_CAPSULE"] = new JigLib.CollDetectCapsulePlane();
		this.detectionFunctors["TERRAIN_SPHERE"] = new JigLib.CollDetectSphereTerrain();
		this.detectionFunctors["TERRAIN_BOX"] = new JigLib.CollDetectBoxTerrain();
		this.detectionFunctors["TERRAIN_CAPSULE"] = new JigLib.CollDetectCapsuleTerrain();
		this.detectionFunctors["TRIANGLEMESH_SPHERE"] = new JigLib.CollDetectSphereMesh();
		this.detectionFunctors["TRIANGLEMESH_BOX"] = new JigLib.CollDetectBoxMesh();
		
}

JigLib.CollisionSystemAbstract.prototype.addCollisionBody = function(body)
{

		if (this.collBody.indexOf(body) < 0)
			this.collBody.push(body);
		
}

JigLib.CollisionSystemAbstract.prototype.removeCollisionBody = function(body)
{

		if (this.collBody.indexOf(body) >= 0)
			this.collBody.splice(this.collBody.indexOf(body), 1);
		
}

JigLib.CollisionSystemAbstract.prototype.removeAllCollisionBodies = function()
{

		this.collBody.length=0;
		
}

JigLib.CollisionSystemAbstract.prototype.detectCollisions = function(body, collArr)
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
				info = new JigLib.CollDetectInfo();
				info.body0 = body;
				info.body1 = _collBody;
				fu = this.detectionFunctors[info.body0.get_type() + "_" + info.body1.get_type()];
				fu.collDetect(info, collArr);
			}
		}
		
}

JigLib.CollisionSystemAbstract.prototype.detectAllCollisions = function(bodies, collArr)
{

		
}

JigLib.CollisionSystemAbstract.prototype.collisionSkinMoved = function(colBody)
{

		// used for grid
		
}

JigLib.CollisionSystemAbstract.prototype.segmentIntersect = function(out, seg, ownerBody)
{

		out.frac = JigLib.JMath3D.NUM_HUGE;
		out.position = new JigLib.Vector3D();
		out.normal = new JigLib.Vector3D();
		
		var obj = new JigLib.CollOutBodyData();
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

JigLib.CollisionSystemAbstract.prototype.segmentBounding = function(seg, obj)
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

JigLib.CollisionSystemAbstract.prototype.get_numCollisionsChecks = function()
{

		return this._numCollisionsChecks;	
		
}

JigLib.CollisionSystemAbstract.prototype.checkCollidables = function(body0, body1)
{

		if (body0.get_nonCollidables().length == 0 && body1.get_nonCollidables().length == 0)
			return true;
		
		if(body0.get_nonCollidables().indexOf(body1) > -1)
			return false;
		
		if(body1.get_nonCollidables().indexOf(body0) > -1)
			return false;
		
		return true;
		
}



