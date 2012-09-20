
JigLib.CollPointInfo = function()
{
	this.initialPenetration = null; // Number
	this.r0 = null; // Vector3D
	this.r1 = null; // Vector3D
	this.position = null; // Vector3D
	this.minSeparationVel =  0; // Number
	this.denominator =  0; // Number
	this.accumulatedNormalImpulse =  0; // Number
	this.accumulatedNormalImpulseAux =  0; // Number
	this.accumulatedFrictionImpulse =  new JigLib.Vector3D(); // Vector3D
}



