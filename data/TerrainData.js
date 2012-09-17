
var JigLib_TerrainData = function(height, normal)
{
	this.height = null; // Number
	this.normal = null; // Vector3D

		this.height = isNaN(height) ? 0 : height;
		this.normal = normal ? normal : new JigLib_Vector3D();
		
}



JigLib.TerrainData = JigLib_TerrainData; 
