
JigLib.CollisionSystemBrute = function()
{

		JigLib.CollisionSystemAbstract.apply(this, [  ]);
		
}

JigLib.extend(JigLib.CollisionSystemBrute, JigLib.CollisionSystemAbstract);

JigLib.CollisionSystemBrute.prototype.detectAllCollisions = function(bodies, collArr)
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
				info = new JigLib.CollDetectInfo();
				info.body0 = _body;
				info.body1 = _collBody;
				fu = this.detectionFunctors[info.body0.get_type() + "_" + info.body1.get_type()];
				fu.collDetect(info, collArr);
				this._numCollisionsChecks += 1;
				}
			}
		}
		
}



