
var JigLib_CollDetectSphereCapsule = function()
{

		this.name = "SphereCapsule";
		this.type0 = "SPHERE";
		this.type1 = "CAPSULE";
		
}

JigLib.extend(JigLib_CollDetectSphereCapsule, JigLib_CollDetectFunctor);

JigLib_CollDetectSphereCapsule.prototype.collDetect = function(info, collArr)
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

		var oldSeg = new JigLib_JSegment(capsule.getBottomPos(capsule.get_oldState()), JigLib_JNumber3D.getScaleVector(capsule.get_oldState().getOrientationCols()[1], capsule.get_length()));
		var newSeg = new JigLib_JSegment(capsule.getBottomPos(capsule.get_currentState()), JigLib_JNumber3D.getScaleVector(capsule.get_currentState().getOrientationCols()[1], capsule.get_length()));
		var radSum = sphere.get_radius() + capsule.get_radius();

		var oldObj = [];
		var oldDistSq = oldSeg.pointSegmentDistanceSq(oldObj, sphere.get_oldState().position);
		var newObj = [];
		var newDistSq = newSeg.pointSegmentDistanceSq(newObj, sphere.get_currentState().position);

		if (Math.min(oldDistSq, newDistSq) < Math.pow(radSum + JigLib_JConfig.collToll, 2))
		{
			var segPos = oldSeg.getPoint(oldObj[0]);
			var delta = sphere.get_oldState().position.subtract(segPos);

			var dist = Math.sqrt(oldDistSq);
			var depth = radSum - dist;

			if (dist > JigLib_JMath3D.NUM_TINY)
			{
				delta = JigLib_JNumber3D.getDivideVector(delta, dist);
			}
			else
			{
				delta = JigLib_JMatrix3D.getRotationMatrix(0, 0, 1, 360 * Math.random()).transformVector(JigLib_Vector3D.Y_AXIS);
			}

			var worldPos = segPos.add(JigLib_JNumber3D.getScaleVector(delta, capsule.get_radius() - 0.5 * depth));

			var collPts = [];
			var cpInfo = new JigLib_CollPointInfo();
			cpInfo.r0 = worldPos.subtract(sphere.get_oldState().position);
			cpInfo.r1 = worldPos.subtract(capsule.get_oldState().position);
			cpInfo.initialPenetration = depth;
			collPts[0]=cpInfo;
			
			var collInfo = new JigLib_CollisionInfo();
			collInfo.objInfo = info;
			collInfo.dirToBody = delta;
			collInfo.pointInfo = collPts;
			
			var mat = new JigLib_MaterialProperties();
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



JigLib.CollDetectSphereCapsule = JigLib_CollDetectSphereCapsule; 
