
var JigLib_JRay = function(_origin, _dir)
{
	this.origin = null; // Vector3D
	this.dir = null; // Vector3D

		this.origin = _origin;
		this.dir = _dir;
		
}

JigLib_JRay.prototype.getOrigin = function(t)
{

		return this.origin.add(JigLib_JNumber3D.getScaleVector(this.dir, t));
		
}



JigLib.JRay = JigLib_JRay; 
