
JigLib.PhysicsState = function()
{
	this.position =  new JigLib.Vector3D(); // Vector3D
	this.orientation =  new JigLib.Matrix3D(); // Matrix3D
	this.linVelocity =  new JigLib.Vector3D(); // Vector3D
	this.rotVelocity =  new JigLib.Vector3D(); // Vector3D
	this.orientationCols =  []; // Vector3D

		//this.orientationCols[0] = new JigLib.Vector3D();
		//this.orientationCols[1] = new JigLib.Vector3D();
		//this.orientationCols[2] = new JigLib.Vector3D();
		
}

JigLib.PhysicsState.prototype.getOrientationCols = function()
{

		return JigLib.JMatrix3D.getCols(this.orientation);
		
}



