
(function(JigLib) {


	var CollisionSystemAbstract = function()
	{
		this.detectionFunctors = null; // Dictionary
		this.collBody = null; // RigidBody
		this._numCollisionsChecks =  0; // uint
		this.startPoint = null; // Vector3D

		this.collBody = [];
		this.detectionFunctors = [];

		
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
				info = new JigLib.CollDetectInfo();
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



	JigLib.CollisionSystemAbstract = CollisionSystemAbstract; 

})(JigLib);

