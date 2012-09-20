
JigLib.CollDetectCapsuleTerrain = function()
{

		this.name = "BoxTerrain";
		this.type0 = "CAPSULE";
		this.type1 = "TERRAIN";
		
}

JigLib.extend(JigLib.CollDetectCapsuleTerrain, JigLib.CollDetectFunctor);

JigLib.CollDetectCapsuleTerrain.prototype.collDetect = function(info, collArr)
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
		
		var averageNormal = new JigLib.Vector3D();
		var pos1 = capsule.getBottomPos(capsule.get_oldState());
		var pos2 = capsule.getBottomPos(capsule.get_currentState());
		var obj1 = terrain.getHeightAndNormalByPoint(pos1);
		var obj2 = terrain.getHeightAndNormalByPoint(pos2);
		if (Math.min(obj1.height, obj2.height) < JigLib.JConfig.collToll + capsule.get_radius()) {
			var oldDepth = capsule.get_radius() - obj1.height;
			var worldPos = pos1.subtract(JigLib.JNumber3D.getScaleVector(obj2.normal, capsule.get_radius()));
			cpInfo = new JigLib.CollPointInfo();
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
		if (Math.min(obj1.height, obj2.height) < JigLib.JConfig.collToll + capsule.get_radius()) {
			oldDepth = capsule.get_radius() - obj1.height;
			worldPos = pos1.subtract(JigLib.JNumber3D.getScaleVector(obj2.normal, capsule.get_radius()));
			cpInfo = new JigLib.CollPointInfo();
			cpInfo.r0 = worldPos.subtract(capsule.get_oldState().position);
			cpInfo.r1 = worldPos.subtract(terrain.get_oldState().position);
			cpInfo.initialPenetration = oldDepth;
			collPts.push(cpInfo);
			averageNormal = averageNormal.add(obj2.normal);
		}
		
		if (collPts.length > 0)
		{
			averageNormal.normalize();
			
			var collInfo = new JigLib.CollisionInfo();
			collInfo.objInfo = info;
			collInfo.dirToBody = averageNormal;
			collInfo.pointInfo = collPts;
			
			var mat = new JigLib.MaterialProperties();
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



