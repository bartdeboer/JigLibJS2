
var JigLib_CollOutBodyData = function(frac, position, normal, rigidBody)
{
	this.rigidBody = null; // RigidBody

		JigLib_CollOutData.apply(this, [ frac, position, normal ]);
		this.rigidBody = rigidBody;
		
}

JigLib.extend(JigLib_CollOutBodyData, JigLib_CollOutData);



JigLib.CollOutBodyData = JigLib_CollOutBodyData; 
