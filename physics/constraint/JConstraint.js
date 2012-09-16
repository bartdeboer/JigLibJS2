
(function(JigLib) {


	var JConstraint = function()
	{
		this.satisfied = null; // Boolean
		this._constraintEnabled = null; // Boolean

		
	}

	JConstraint.prototype.preApply = function(dt)
	{

		this.satisfied = false;
		
	}

	JConstraint.prototype.apply = function(dt)
	{

		return false;
		
	}

	JConstraint.prototype.enableConstraint = function()
	{

		
	}

	JConstraint.prototype.disableConstraint = function()
	{

		
	}

	JConstraint.prototype.get_constraintEnabled = function()
	{

		return this._constraintEnabled;
		
	}



	JigLib.JConstraint = JConstraint; 

})(JigLib);

