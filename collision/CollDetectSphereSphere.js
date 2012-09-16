
(function(JigLib) {


	var CollDetectSphereSphere = function()
	{

		this.name = "SphereSphere";
		this.type0 = "SPHERE";
		this.type1 = "SPHERE";
		
	}

	JigLib.extend(CollDetectSphereSphere, JigLib.CollDetectFunctor);

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

		if (Math.min(oldDistSq, newDistSq) < Math.pow(radSum + JigLib.JConfig.collToll, 2))
		{
			oldDist = Math.sqrt(oldDistSq);
			depth = radSum - oldDist;
			if (oldDist > JigLib.JMath3D.NUM_TINY)
			{
				oldDelta = JigLib.JNumber3D.getDivideVector(oldDelta, oldDist);
			}
			else
			{
				oldDelta = JigLib.JMatrix3D.getRotationMatrix(0, 0, 1, 360 * Math.random()).transformVector(JigLib.Vector3D.Y_AXIS);
			}
			
			var worldPos = sphere1.get_oldState().position.add(JigLib.JNumber3D.getScaleVector(oldDelta, sphere1.get_radius() - 0.5 * depth));

			var collPts = [];
			var cpInfo = new JigLib.CollPointInfo();
			cpInfo.r0 = worldPos.subtract(sphere0.get_oldState().position);
			cpInfo.r1 = worldPos.subtract(sphere1.get_oldState().position);
			cpInfo.initialPenetration = depth;
			collPts[0]=cpInfo;
			
			var collInfo = new JigLib.CollisionInfo();
			collInfo.objInfo = info;
			collInfo.dirToBody = oldDelta;
			collInfo.pointInfo = collPts;
			
			var mat = new JigLib.MaterialProperties();
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



	JigLib.CollDetectSphereSphere = CollDetectSphereSphere; 

})(JigLib);

