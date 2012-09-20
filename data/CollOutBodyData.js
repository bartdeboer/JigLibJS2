
JigLib.CollOutBodyData = function(frac, position, normal, rigidBody)
{
	this.rigidBody = null; // RigidBody

		JigLib.CollOutData.apply(this, [ frac, position, normal ]);
		this.rigidBody = rigidBody;
		
}

JigLib.extend(JigLib.CollOutBodyData, JigLib.CollOutData);



