
(function(JigLib) {


	var BodyPair = function(_body0, _body1, r0, r1)
	{
		this.body0 = null; // RigidBody
		this.body1 = null; // RigidBody
		this.r = null; // Vector3D


		var id1 = -1;
		if (_body1 != null) id1 = _body1.get_id();
		
		if (_body0.get_id() > id1)
		{
			this.body0 = _body0;
			this.body1 = _body1;
			this.r = r0;
		}
		else
		{
			this.body0 = _body1;
			this.body1 = _body0;
			this.r = r1;
		}
		
	}



	JigLib.BodyPair = BodyPair; 

})(JigLib);

