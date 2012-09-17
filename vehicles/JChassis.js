
var JigLib_JChassis = function(car, skin, width, depth, height)
{
	this._car = null; // JCar

		JigLib_JBox.apply(this, [ skin, width, depth, height ]);

		this._car = car;
		
}

JigLib.extend(JigLib_JChassis, JigLib_JBox);

JigLib_JChassis.prototype.get_car = function()
{

		return this._car;
		
}

JigLib_JChassis.prototype.postPhysics = function(dt)
{

		JigLib_JBox.prototype.postPhysics.apply(this, [ dt ]);
		this._car.addExternalForces(dt);
		this._car.postPhysics(dt);
		
}



JigLib.JChassis = JigLib_JChassis; 
