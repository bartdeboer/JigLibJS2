
var JigLib_JConstraint = function()
{
	this.satisfied = null; // Boolean
	this._constraintEnabled = null; // Boolean

		
}

JigLib_JConstraint.prototype.preApply = function(dt)
{

		this.satisfied = false;
		
}

JigLib_JConstraint.prototype.apply = function(dt)
{

		return false;
		
}

JigLib_JConstraint.prototype.enableConstraint = function()
{

		
}

JigLib_JConstraint.prototype.disableConstraint = function()
{

		
}

JigLib_JConstraint.prototype.get_constraintEnabled = function()
{

		return this._constraintEnabled;
		
}



JigLib.JConstraint = JigLib_JConstraint; 
