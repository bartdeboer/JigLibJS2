
var JigLib_CollOutData = function(frac, position, normal)
{
	this.frac = null; // Number
	this.position = null; // Vector3D
	this.normal = null; // Vector3D

		this.frac = isNaN(frac) ? 0 : frac;
		this.position = position ? position : new JigLib_Vector3D;
		this.normal = normal ? normal : new JigLib_Vector3D;
		
}



JigLib.CollOutData = JigLib_CollOutData; 
