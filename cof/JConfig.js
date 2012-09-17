
var JigLib_JConfig = function()
{
}

JigLib_JConfig.solverType =  "ACCUMULATED"; // String
JigLib_JConfig.rotationType =  "DEGREES"; // String
JigLib_JConfig.doShockStep =  false; // Boolean
JigLib_JConfig.allowedPenetration =  0.01; // Number
JigLib_JConfig.collToll =  0.05; // Number
JigLib_JConfig.velThreshold =  0.5; // Number
JigLib_JConfig.angVelThreshold =  0.5; // Number
JigLib_JConfig.posThreshold =  0.2; // Number
JigLib_JConfig.orientThreshold =  0.2; // Number
JigLib_JConfig.deactivationTime =  0.5; // Number
JigLib_JConfig.numPenetrationRelaxationTimesteps =  10; // Number
JigLib_JConfig.numCollisionIterations =  1; // Number
JigLib_JConfig.numContactIterations =  2; // Number
JigLib_JConfig.numConstraintIterations =  2; // Number


JigLib.JConfig = JigLib_JConfig; 
