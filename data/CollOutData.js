
(function(JigLib) {


	var CollOutData = function(frac, position, normal)
	{
		this.frac = null; // Number
		this.position = null; // Vector3D
		this.normal = null; // Vector3D

		this.frac = isNaN(frac) ? 0 : frac;
		this.position = position ? position : new JigLib.Vector3D;
		this.normal = normal ? normal : new JigLib.Vector3D;
		
	}



	JigLib.CollOutData = CollOutData; 

})(JigLib);

