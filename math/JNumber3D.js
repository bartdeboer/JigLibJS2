
var JigLib_JNumber3D = function()
{
}


JigLib_JNumber3D.toArray = function(v)
{

			var arr=[];
			arr[0]=v.x;
			arr[1]=v.y;
			arr[2]=v.z;
			return arr;
		
}

JigLib_JNumber3D.copyFromArray = function(v, arr)
{

			if (arr.length >= 3)
			{
				v.x = arr[0];
				v.y = arr[1];
				v.z = arr[2];
			}
		
}

JigLib_JNumber3D.getScaleVector = function(v, s)
{

			return new JigLib_Vector3D(v.x*s,v.y*s,v.z*s,v.w);
		
}

JigLib_JNumber3D.getDivideVector = function(v, w)
{

			if (w != 0)
			{
				return new JigLib_Vector3D(v.x / w, v.y / w, v.z / w);
			}
			else
			{
				return new JigLib_Vector3D(0, 0, 0);
			}
		
}

JigLib_JNumber3D.getNormal = function(v0, v1, v2)
{

			var E = v1.clone();
			var F = v2.clone();
			var N = E.subtract(v0).crossProduct(F.subtract(v1));
			N.normalize();

			return N;
		
}


JigLib.JNumber3D = JigLib_JNumber3D; 
