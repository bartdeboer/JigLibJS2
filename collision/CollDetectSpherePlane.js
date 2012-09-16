
(function(JigLib) {


	var CollDetectSpherePlane = function()
	{

		this.name = "SpherePlane";
		this.type0 = "SPHERE";
		this.type1 = "PLANE";
		
	}

	JigLib.extend(CollDetectSpherePlane, JigLib.CollDetectFunctor);

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

		if (Math.min(newDist, oldDist) > sphere.get_boundingSphere() + JigLib.JConfig.collToll)
		{
			info.body0.removeCollideBodies(info.body1);
			info.body1.removeCollideBodies(info.body0);
			return;
		}

		var collPts = [];
		var cpInfo;
		depth = sphere.get_radius() - oldDist;

		var worldPos = sphere.get_oldState().position.subtract(JigLib.JNumber3D.getScaleVector(plane.get_normal(), sphere.get_radius()));
		cpInfo = new JigLib.CollPointInfo();
		cpInfo.r0 = worldPos.subtract(sphere.get_oldState().position);
		cpInfo.r1 = worldPos.subtract(plane.get_oldState().position);
		cpInfo.initialPenetration = depth;
		collPts[0]=cpInfo;
		
		var collInfo = new JigLib.CollisionInfo();
		collInfo.objInfo = info;
		collInfo.dirToBody = plane.get_normal().clone();
		collInfo.pointInfo = collPts;
		
		var mat = new JigLib.MaterialProperties();
		mat.restitution = 0.5*(sphere.get_material().restitution + plane.get_material().restitution);
		mat.friction = 0.5*(sphere.get_material().friction + plane.get_material().friction);
		collInfo.mat = mat;
		collArr.push(collInfo);
		info.body0.collisions.push(collInfo);
		info.body1.collisions.push(collInfo);
		info.body0.addCollideBody(info.body1);
		info.body1.addCollideBody(info.body0);
		
	}



	JigLib.CollDetectSpherePlane = CollDetectSpherePlane; 

})(JigLib);

