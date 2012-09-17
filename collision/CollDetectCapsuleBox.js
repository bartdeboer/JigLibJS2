
var JigLib_CollDetectCapsuleBox = function()
{

		this.name = "CapsuleBox";
		this.type0 = "CAPSULE";
		this.type1 = "BOX";
		
}

JigLib.extend(JigLib_CollDetectCapsuleBox, JigLib_CollDetectFunctor);

JigLib_CollDetectCapsuleBox.prototype.collDetect = function(info, collArr)
{

}



JigLib.CollDetectCapsuleBox = JigLib_CollDetectCapsuleBox; 
