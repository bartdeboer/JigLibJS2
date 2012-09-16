
(function(JigLib) {


	var CollDetectSphereBox = function()
	{

		this.name = "SphereBox";
		this.type0 = "SPHERE";
		this.type1 = "BOX";
		
	}

	JigLib.extend(CollDetectSphereBox, JigLib.CollDetectFunctor);

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
		
		var oldBoxPoint = [new JigLib.Vector3D()];
		var newBoxPoint = [new JigLib.Vector3D()];
		
		var oldDist, newDist, oldDepth, newDepth, tiny=JigLib.JMath3D.NUM_TINY;
		oldDist = box.getDistanceToPoint(box.get_oldState(), oldBoxPoint, sphere.get_oldState().position);
		newDist = box.getDistanceToPoint(box.get_currentState(), newBoxPoint, sphere.get_currentState().position);
		
		var _oldBoxPosition = oldBoxPoint[0];
		
		oldDepth = sphere.get_radius() - oldDist;
		newDepth = sphere.get_radius() - newDist;
		if (Math.max(oldDepth, newDepth) > -JigLib.JConfig.collToll)
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
			
			var cpInfo = new JigLib.CollPointInfo();
			cpInfo.r0 = _oldBoxPosition.subtract(sphere.get_oldState().position);
			cpInfo.r1 = _oldBoxPosition.subtract(box.get_oldState().position);
			cpInfo.initialPenetration = oldDepth;
			collPts[0]=cpInfo;
			
			var collInfo = new JigLib.CollisionInfo();
			collInfo.objInfo = info;
			collInfo.dirToBody = dir;
			collInfo.pointInfo = collPts;
			
			var mat = new JigLib.MaterialProperties();
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



	JigLib.CollDetectSphereBox = CollDetectSphereBox; 

})(JigLib);

