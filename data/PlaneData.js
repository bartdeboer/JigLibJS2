
(function(JigLib) {


	var PlaneData = function()
	{
		this._position = null; // Vector3D
		this._normal = null; // Vector3D
		this._distance = null; // Number

		this._position = new JigLib.Vector3D();
		this._normal = new JigLib.Vector3D(0, 1, 0);
		this._distance = 0;
		
	}

	PlaneData.prototype.get_position = function()
	{

		return this._position;
		
	}

	PlaneData.prototype.get_normal = function()
	{

		return this._normal;
		
	}

	PlaneData.prototype.get_distance = function()
	{

		return this._distance;
		
	}

	PlaneData.prototype.pointPlaneDistance = function(pt)
	{

		return this._normal.dotProduct(pt) - this._distance;
		
	}

	PlaneData.prototype.setWithNormal = function(pos, nor)
	{

		this._position = pos.clone();
		this._normal = nor.clone();
		this._distance = pos.dotProduct(nor);
		
	}

	PlaneData.prototype.setWithPoint = function(pos0, pos1, pos2)
	{

		this._position = pos0.clone();
		
		var dr1 = pos1.subtract(pos0);
		var dr2 = pos2.subtract(pos0);
		this._normal = dr1.crossProduct(dr2);
		
		var nLen = this._normal.get_length();
		if (nLen < JigLib.JMath3D.NUM_TINY) {
			this._normal = new JigLib.Vector3D(0, 1, 0);
			this._distance = 0;
		}else {
			this._normal.scaleBy(1 / nLen);
			this._distance = pos0.dotProduct(this._normal);
		}
		
	}



	JigLib.PlaneData = PlaneData; 

})(JigLib);

