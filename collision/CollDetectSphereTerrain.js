
JigLib.CollDetectSphereTerrain = function()
{

		this.name = "SphereTerrain";
		this.type0 = "SPHERE";
		this.type1 = "TERRAIN";
		
}

JigLib.extend(JigLib.CollDetectSphereTerrain, JigLib.CollDetectFunctor);

JigLib.CollDetectSphereTerrain.prototype.collDetect = function(info, collArr)
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
		if (obj.height < JigLib.JConfig.collToll + sphere.get_radius()) {
			var dist = terrain.getHeightByPoint(sphere.get_oldState().position);
			var depth = sphere.get_radius() - dist;
			
			var Pt = sphere.get_oldState().position.subtract(JigLib.JNumber3D.getScaleVector(obj.normal, sphere.get_radius()));
			
			var collPts = [];
			var cpInfo = new JigLib.CollPointInfo();
			cpInfo.r0 = Pt.subtract(sphere.get_oldState().position);
			cpInfo.r1 = Pt.subtract(terrain.get_oldState().position);
			cpInfo.initialPenetration = depth;
			collPts[0]=cpInfo;
			
			var collInfo = new JigLib.CollisionInfo();
			collInfo.objInfo = info;
			collInfo.dirToBody = obj.normal;
			collInfo.pointInfo = collPts;
			
			var mat = new JigLib.MaterialProperties();
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



