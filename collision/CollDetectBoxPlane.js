
JigLib.CollDetectBoxPlane = function()
{

		this.name = "BoxPlane";
		this.type0 = "BOX";
		this.type1 = "PLANE";
		
}

JigLib.extend(JigLib.CollDetectBoxPlane, JigLib.CollDetectFunctor);

JigLib.CollDetectBoxPlane.prototype.collDetect = function(info, collArr)
{

		var tempBody;
		if (info.body0.get_type() == "PLANE")
		{
			tempBody = info.body0;
			info.body0 = info.body1;
			info.body1 = tempBody;
		}

		var box = info.body0;
		var plane = info.body1;

		var centreDist = plane.pointPlaneDistance(box.get_currentState().position);
		if (centreDist > box.get_boundingSphere() + JigLib.JConfig.collToll)
			return;

		var newPts = box.getCornerPoints(box.get_currentState());
		var oldPts = box.getCornerPoints(box.get_oldState());
		var collPts = [];
		var cpInfo;
		var newPt, oldPt;
		var newDepth, oldDepth;
		var newPts_length = newPts.length;
		
		for (var i = 0; i < newPts_length; i++)
		{
			newPt = newPts[i];
			oldPt = oldPts[i];
			newDepth = -1 * plane.pointPlaneDistance(newPt);
			oldDepth = -1 * plane.pointPlaneDistance(oldPt);
			
			if (Math.max(newDepth, oldDepth) > -JigLib.JConfig.collToll)
			{
				cpInfo = new JigLib.CollPointInfo();
				cpInfo.r0 = oldPt.subtract(box.get_oldState().position);
				cpInfo.r1 = oldPt.subtract(plane.get_oldState().position);
				cpInfo.initialPenetration = oldDepth;
				collPts.push(cpInfo);
			}
		}
		
		if (collPts.length > 0)
		{
			var collInfo = new JigLib.CollisionInfo();
			collInfo.objInfo = info;
			collInfo.dirToBody = plane.get_normal().clone();
			collInfo.pointInfo = collPts;
			
			var mat = new JigLib.MaterialProperties();
			mat.restitution = 0.5*(box.get_material().restitution + plane.get_material().restitution);
			mat.friction = 0.5*(box.get_material().friction + plane.get_material().friction);
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



