
var JigLib_JMath3D = function()
{
}

JigLib_JMath3D.NUM_TINY =  0.000001; // Number
JigLib_JMath3D.NUM_HUGE =  1000000; // Number

JigLib_JMath3D.fromNormalAndPoint = function(normal, point)
{

        	var v = new JigLib_Vector3D(normal.x, normal.y, normal.z);
        	v.w = -(v.x*point.x + v.y*point.y + v.z*point.z);
        	
        	return v;
        
}

JigLib_JMath3D.getIntersectionLine = function(v, v0, v1)
{

			var d0 = v.x * v0.x + v.y * v0.y + v.z * v0.z - v.w;
			var d1 = v.x * v1.x + v.y * v1.y + v.z * v1.z - v.w;
			var m = d1 / (d1 - d0);
			return new JigLib_Vector3D(
				v1.x + (v0.x - v1.x) * m,
				v1.y + (v0.y - v1.y) * m,
				v1.z + (v0.z - v1.z) * m);
		
}

JigLib_JMath3D.wrap = function(val, min, max)
{

			var delta = max - min;
			if (val > delta)
			{
				val = val / delta;
				val = val - Number(Math.floor(val));
				val = val * delta;
			}
			return val;
		
}

JigLib_JMath3D.getLimiteNumber = function(num, min, max)
{

			var n = num;
			if (n < min)
			{
				n = min;
			}
			else if (n > max)
			{
				n = max;
			}
			return n;
		
}


JigLib.JMath3D = JigLib_JMath3D; 
