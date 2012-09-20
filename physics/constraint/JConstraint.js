
JigLib.JConstraint = function()
{
	this.satisfied = null; // Boolean
	this._constraintEnabled = null; // Boolean

		
}

JigLib.JConstraint.prototype.preApply = function(dt)
{

		this.satisfied = false;
		
}

JigLib.JConstraint.prototype.apply = function(dt)
{

		return false;
		
}

JigLib.JConstraint.prototype.enableConstraint = function()
{

		
}

JigLib.JConstraint.prototype.disableConstraint = function()
{

		
}

JigLib.JConstraint.prototype.get_constraintEnabled = function()
{

		return this._constraintEnabled;
		
}



