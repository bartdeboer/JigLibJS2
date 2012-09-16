
(function(JigLib) {


	var CollDetectCapsuleBox = function()
	{

		this.name = "CapsuleBox";
		this.type0 = "CAPSULE";
		this.type1 = "BOX";
		
	}

	JigLib.extend(CollDetectCapsuleBox, JigLib.CollDetectFunctor);

	CollDetectCapsuleBox.prototype.collDetect = function(info, collArr)
	{

	}



	JigLib.CollDetectCapsuleBox = CollDetectCapsuleBox; 

})(JigLib);

