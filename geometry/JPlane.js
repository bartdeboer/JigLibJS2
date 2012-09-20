
JigLib.JPlane = function(skin, initNormal)
{
	this._initNormal = null; // Vector3D
	this._normal = null; // Vector3D
	this._distance = null; // Number

		JigLib.RigidBody.apply(this, [ skin ]);

		this._initNormal = initNormal ? initNormal.clone() : new JigLib.Vector3D(0, 0, -1);
		this._normal = this._initNormal.clone();

		this._distance = 0;
		this.set_movable(false);
		
		var huge=JigLib.JMath3D.NUM_HUGE;
		this._boundingBox.minPos = new JigLib.Vector3D(-huge, -huge, -huge);
		this._boundingBox.maxPos = new JigLib.Vector3D(huge, huge, huge);

		this._type = "PLANE";
		
}

JigLib.extend(JigLib.JPlane, JigLib.RigidBody);

JigLib.JPlane.prototype.get_normal = function()
{

		return this._normal;
		
}

JigLib.JPlane.prototype.get_distance = function()
{

		return this._distance;
		
}

JigLib.JPlane.prototype.pointPlaneDistance = function(pt)
{

		return this._normal.dotProduct(pt) - this._distance;
		
}

JigLib.JPlane.prototype.segmentIntersect = function(out, seg, state)
{

		out.frac = 0;
		out.position = new JigLib.Vector3D();
		out.normal = new JigLib.Vector3D();

		var frac = 0, t, denom;

		denom = this._normal.dotProduct(seg.delta);
		if (Math.abs(denom) > JigLib.JMath3D.NUM_TINY)
		{
			t = -1 * (this._normal.dotProduct(seg.origin) - this._distance) / denom;

			if (t < 0 || t > 1)
			{
				return false;
			}
			else
			{
				frac = t;
				out.frac = frac;
				out.position = seg.getPoint(frac);
				out.normal = this._normal.clone();
				out.normal.normalize();
				return true;
			}
		}
		else
		{
			return false;
		}
		
}

JigLib.JPlane.prototype.updateState = function()
{

		JigLib.RigidBody.prototype.updateState.apply(this, [  ]);

		this._normal = this._currState.orientation.transformVector(this._initNormal);
		this._distance = this._currState.position.dotProduct(this._normal);
		
}



