
JigLib.JRay = function(_origin, _dir)
{
	this.origin = null; // Vector3D
	this.dir = null; // Vector3D

		this.origin = _origin;
		this.dir = _dir;
		
}

JigLib.JRay.prototype.getOrigin = function(t)
{

		return this.origin.add(JigLib.JNumber3D.getScaleVector(this.dir, t));
		
}



