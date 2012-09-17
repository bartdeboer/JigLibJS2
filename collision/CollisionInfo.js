
var JigLib_CollisionInfo = function()
{
	this.mat =  new JigLib_MaterialProperties(); // MaterialProperties
	this.objInfo = null; // CollDetectInfo
	this.dirToBody = null; // Vector3D
	this.pointInfo = null; // CollPointInfo
	this.satisfied = null; // Boolean
}



JigLib.CollisionInfo = JigLib_CollisionInfo; 
