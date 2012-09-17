
var JigLib_PhysicsState = function()
{
	this.position =  new JigLib_Vector3D(); // Vector3D
	this.orientation =  new JigLib_Matrix3D(); // Matrix3D
	this.linVelocity =  new JigLib_Vector3D(); // Vector3D
	this.rotVelocity =  new JigLib_Vector3D(); // Vector3D
	this.orientationCols =  []; // Vector3D

		//this.orientationCols[0] = new JigLib_Vector3D();
		//this.orientationCols[1] = new JigLib_Vector3D();
		//this.orientationCols[2] = new JigLib_Vector3D();
		
}

JigLib_PhysicsState.prototype.getOrientationCols = function()
{

		return JigLib_JMatrix3D.getCols(this.orientation);
		
}



JigLib.PhysicsState = JigLib_PhysicsState; 
