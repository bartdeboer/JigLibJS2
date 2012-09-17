
var JigLib_JConstraintWorldPoint = function(body, pointOnBody, worldPosition)
{
	this.minVelForProcessing =  0.001; // Number
	this.allowedDeviation =  0.01; // Number
	this.timescale =  4; // Number
	this._body = null; // RigidBody
	this._pointOnBody = null; // Vector3D
	this._worldPosition = null; // Vector3D

		JigLib_JConstraint.apply(this, [  ]);
		this._body = body;
		this._pointOnBody = pointOnBody;
		this._worldPosition = worldPosition;
		
		this._constraintEnabled = false;
		this.enableConstraint();
		
}

JigLib.extend(JigLib_JConstraintWorldPoint, JigLib_JConstraint);

JigLib_JConstraintWorldPoint.prototype.set_worldPosition = function(pos)
{

		this._worldPosition = pos;
		
}

JigLib_JConstraintWorldPoint.prototype.get_worldPosition = function()
{

		return this._worldPosition;
		
}

JigLib_JConstraintWorldPoint.prototype.enableConstraint = function()
{

		if (this._constraintEnabled)
		{
			return;
		}
		this._constraintEnabled = true;
		this._body.addConstraint(this);
		JigLib_PhysicsSystem.getInstance().addConstraint(this);
		
}

JigLib_JConstraintWorldPoint.prototype.disableConstraint = function()
{

		if (!this._constraintEnabled)
		{
			return;
		}
		this._constraintEnabled = false;
		this._body.removeConstraint(this);
		JigLib_PhysicsSystem.getInstance().removeConstraint(this);
		
}

JigLib_JConstraintWorldPoint.prototype.apply = function(dt)
{

		this.satisfied = true;

		var deviationDistance, normalVel, denominator, normalImpulse, dot;
		var worldPos, R, currentVel, desiredVel, deviationDir, deviation, N, tempV;
		
		worldPos = this._body.get_currentState().orientation.transformVector(this._pointOnBody);
		worldPos = worldPos.add( this._body.get_currentState().position);
		R = worldPos.subtract(this._body.get_currentState().position);
		currentVel = this._body.get_currentState().linVelocity.add(this._body.get_currentState().rotVelocity.crossProduct(R));
		
		deviation = worldPos.subtract(this._worldPosition);
		deviationDistance = deviation.get_length();
		if (deviationDistance > this.allowedDeviation) {
			deviationDir = JigLib_JNumber3D.getDivideVector(deviation, deviationDistance);
			desiredVel = JigLib_JNumber3D.getScaleVector(deviationDir, (this.allowedDeviation - deviationDistance) / (this.timescale * dt));
		} else {
			desiredVel = new JigLib_Vector3D();
		}
		
		N = currentVel.subtract(desiredVel);
		normalVel = N.get_length();
		if (normalVel < this.minVelForProcessing) {
			return false;
		}
		N = JigLib_JNumber3D.getDivideVector(N, normalVel);
		
		tempV = R.crossProduct(N);
		tempV = this._body.get_worldInvInertia().transformVector(tempV);
		denominator = this._body.get_invMass() + N.dotProduct(tempV.crossProduct(R));
		 
		if (denominator < JigLib_JMath3D.NUM_TINY) {
			return false;
		}
		 
		normalImpulse = -normalVel / denominator;
		
		this._body.applyWorldImpulse(JigLib_JNumber3D.getScaleVector(N, normalImpulse), worldPos, false);
		
		this._body.setConstraintsAndCollisionsUnsatisfied();
		this.satisfied = true;
		
		return true;
		
}



JigLib.JConstraintWorldPoint = JigLib_JConstraintWorldPoint; 
