
JigLib.CollisionSystemGrid = function(sx, sy, sz, nx, ny, nz, dx, dy, dz)
{
	this.gridEntries = null; // CollisionSystemGridEntry
	this.overflowEntries = null; // CollisionSystemGridEntry
	this.nx = null; // int
	this.ny = null; // int
	this.nz = null; // int
	this.dx = null; // Number
	this.dy = null; // Number
	this.dz = null; // Number
	this.sizeX = null; // Number
	this.sizeY = null; // Number
	this.sizeZ = null; // Number
	this.minDelta = null; // Number

		JigLib.CollisionSystemAbstract.apply(this, [  ]);
		
		this.nx = nx; this.ny = ny; this.nz = nz;
		this.dx = dx; this.dy = dy; this.dz = dz;
		this.sizeX = nx * dx;
		this.sizeY = ny * dy;
		this.sizeZ = nz * dz;
		this.minDelta = Math.min(dx, dy, dz);
		
		this.startPoint = new JigLib.Vector3D(sx, sy, sz);
		
		this.gridEntries = [];
		
		var len=this.gridEntries.length;
		for (var j = 0; j < len; ++j)
		{
			var gridEntry = new JigLib.CollisionSystemGridEntry(null);
			gridEntry.gridIndex = j;
			this.gridEntries[j]=gridEntry;
		}
		
		this.overflowEntries = new JigLib.CollisionSystemGridEntry(null);
		this.overflowEntries.gridIndex = -1;
		
}

JigLib.extend(JigLib.CollisionSystemGrid, JigLib.CollisionSystemAbstract);

JigLib.CollisionSystemGrid.prototype.calcIndex = function(i, j, k)
{

		var _i = i % this.nx;
		var _j = j % this.ny;
		var _k = k % this.nz;
		
		return (_i + this.nx * _j + (this.nx + this.ny) * _k);
		
}

JigLib.CollisionSystemGrid.prototype.calcGridForSkin3 = function(colBody)
{

		var i;var j;var k;
		var sides = colBody.get_boundingBox().get_sideLengths();
		
		if ((sides.x > this.dx) || (sides.y > this.dy) || (sides.z > this.dz))
		{
			i = j = k = -1;
			return new JigLib.Vector3D(i,j,k);
		}
		
		var min = colBody.get_boundingBox().minPos.clone();
		min.x = JigLib.JMath3D.getLimiteNumber(min.x, this.startPoint.x, this.startPoint.x + this.sizeX);
		min.y = JigLib.JMath3D.getLimiteNumber(min.y, this.startPoint.y, this.startPoint.y + this.sizeY);
		min.z = JigLib.JMath3D.getLimiteNumber(min.z, this.startPoint.z, this.startPoint.z + this.sizeZ);
		
		i =  ((min.x - this.startPoint.x) / this.dx) % this.nx;
		j =  ((min.y - this.startPoint.y) / this.dy) % this.ny;
		k =  ((min.z - this.startPoint.z) / this.dz) % this.nz;
		
		return new JigLib.Vector3D(i,j,k);
		
}

JigLib.CollisionSystemGrid.prototype.calcGridForSkin6 = function(colBody)
{

		var tempStoreObject = new Object;
		var i;var j;var k;
		var fi;var fj;var fk;
		
		var sides = colBody.get_boundingBox().get_sideLengths();
		
		if ((sides.x > this.dx) || (sides.y > this.dy) || (sides.z > this.dz))
		{
			//trace("this.calcGridForSkin6 -- Rigidbody to big for gridsystem - putting it into overflow list (lengths,type,id):", sides.x,sides.y,sides.z,colBody.get_type(),colBody.get_id(),colBody.get_boundingBox().minPos,colBody.get_boundingBox().maxPos);
			i = j = k = -1;
			fi = fj = fk = 0.0;
			tempStoreObject.i = i; tempStoreObject.j = j; tempStoreObject.k = k; tempStoreObject.fi = fi; tempStoreObject.fj = fj; tempStoreObject.fk = fk;
			return tempStoreObject;
		}
		
		var min = colBody.get_boundingBox().minPos.clone();

		min.x = JigLib.JMath3D.getLimiteNumber(min.x, this.startPoint.x, this.startPoint.x + this.sizeX);
		min.y = JigLib.JMath3D.getLimiteNumber(min.y, this.startPoint.y, this.startPoint.y + this.sizeY);
		min.z = JigLib.JMath3D.getLimiteNumber(min.z, this.startPoint.z, this.startPoint.z + this.sizeZ);
		
		fi = (min.x - this.startPoint.x) / this.dx;
		fj = (min.y - this.startPoint.y) / this.dy;
		fk = (min.z - this.startPoint.z) / this.dz;
		
		i = fi;
		j = fj;
		k = fk;
		
		if (i < 0) { i = 0; fi = 0.0; }
		else if (i >= this.nx) { i = 0; fi = 0.0; }
		else fi -= Number(i);
		
		if (j < 0) { j = 0; fj = 0.0; }
		else if (j >= this.ny) { j = 0; fj = 0.0; }
		else fj -= Number(j);
		
		if (k < 0) { k = 0; fk = 0.0; }
		else if (k >= this.nz) { k = 0; fk = 0.0; }
		else fk -= Number(k);
		
		tempStoreObject.i = i; tempStoreObject.j = j; tempStoreObject.k = k; tempStoreObject.fi = fi; tempStoreObject.fj = fj; tempStoreObject.fk = fk;
		//trace(i,j,k,fi,fj,fk);
		//trace(colBody.get_x(),colBody.get_y(),colBody.get_z());
		return tempStoreObject;
		
}

JigLib.CollisionSystemGrid.prototype.calcGridIndexForBody = function(colBody)
{

		var tempStoreVector = this.calcGridForSkin3(colBody);
		
		if (tempStoreVector.x == -1) return -1;
		return this.calcIndex(tempStoreVector.x, tempStoreVector.y, tempStoreVector.z);
		
}

JigLib.CollisionSystemGrid.prototype.addCollisionBody = function(body)
{

		if (this.collBody.indexOf(body) < 0)
			this.collBody.push(body);
		
		body.collisionSystem = this;

		// also do the grid stuff - for now put it on the overflow list
		var entry = new JigLib.CollisionSystemGridEntry(body);
		body.externalData = entry;
		
		// add entry to the start of the list
		JigLib.CollisionSystemGridEntry.insertGridEntryAfter(entry, this.overflowEntries);
		this.collisionSkinMoved(body);
		
}

JigLib.CollisionSystemGrid.prototype.removeCollisionBody = function(body)
{

		if (body.externalData != null)
		{
			body.externalData.collisionBody = null;
			JigLib.CollisionSystemGridEntry.removeGridEntry(body.externalData);
			body.externalData = null;
		}

		if (this.collBody.indexOf(body) >= 0)
			this.collBody.splice(this.collBody.indexOf(body), 1);
		
}

JigLib.CollisionSystemGrid.prototype.removeAllCollisionBodies = function()
{

		for (var collBody_i = 0, collBody_l = this.collBody.length, body; (collBody_i < collBody_l) && (body = this.collBody[collBody_i]); collBody_i++){
			if (body.externalData != null)
			{
				body.externalData.collisionBody = null;
				JigLib.CollisionSystemGridEntry.removeGridEntry(body.externalData);
			}
		}
		this.collBody.length=0;
		
}

JigLib.CollisionSystemGrid.prototype.collisionSkinMoved = function(colBody)
{

		var entry = colBody.externalData;
		if (entry == null)
		{
			//trace("Warning rigidbody has grid entry null!");
			return;
		}
		
		var gridIndex = this.calcGridIndexForBody(colBody);
				
		// see if it's moved grid
		if (gridIndex == entry.gridIndex)
			return;

		//trace(gridIndex);
		var start;
		//if (gridIndex >= 0**)
		if (this.gridEntries.length-1 > gridIndex && gridIndex >=0) // check if it's outside the gridspace, if so add to overflow
			start = this.gridEntries[gridIndex];
		else
			start = this.overflowEntries;
		
		JigLib.CollisionSystemGridEntry.removeGridEntry(entry);
		JigLib.CollisionSystemGridEntry.insertGridEntryAfter(entry, start);
		
}

JigLib.CollisionSystemGrid.prototype.getListsToCheck = function(colBody)
{

		var entries = []; 
		
		var entry = colBody.externalData;
		if (entry == null)
		{
			//trace("Warning skin has grid entry null!");
			return null;
		}
		
		// todo - work back from the mGridIndex rather than calculating it again...
		var i, j, k;
		var fi, fj, fk;
		var tempStoreObject = this.calcGridForSkin6(colBody);
		i = tempStoreObject.i; j = tempStoreObject.j; k = tempStoreObject.k; fi = tempStoreObject.fi; fj = tempStoreObject.fj; fk = tempStoreObject.fk;
		
		if (i == -1)
		{
			//trace("ADD ALL!");
			entries=this.gridEntries.concat();
			entries.push(this.overflowEntries);
			return entries;
		}
		
		// always add the overflow
		entries.push(this.overflowEntries);
		
		var delta = colBody.get_boundingBox().get_sideLengths(); // skin.WorldBoundingBox.Max - skin.WorldBoundingBox.Min;
		var maxI = 1, maxJ = 1, maxK = 1;
		if (fi + (delta.x / this.dx) < 1)
			maxI = 0;
		if (fj + (delta.y / this.dy) < 1)
			maxJ = 0;
		if (fk + (delta.z / this.dz) < 1)
			maxK = 0;
		
		// now add the contents of all grid boxes - their contents may extend beyond the bounds
		for (var di = -1; di <= maxI; ++di)
		{
			for (var dj = -1; dj <= maxJ; ++dj)
			{
				for (var dk = -1; dk <= maxK; ++dk)
				{
				var thisIndex = this.calcIndex(i + di, j + dj, k + dk); // + ((this.nx*this.ny*this.nz)*0.5);
				//trace("ge", this.gridEntries.length);
				if (this.gridEntries.length-1 > thisIndex && thisIndex >=0) {
					var start = this.gridEntries[thisIndex];
				 
					//trace(thisIndex,this.gridEntries.length);
					if (start != null && start.next != null)
					{
						entries.push(start);
					}
				}
				}
			}
		}
		return entries;
		
}

JigLib.CollisionSystemGrid.prototype.detectAllCollisions = function(bodies, collArr)
{

		var info;
		var fu;
		var bodyID;
		var bodyType;
		this._numCollisionsChecks = 0;
		
		for (var bodies_i = 0, bodies_l = bodies.length, body; (bodies_i < bodies_l) && (body = bodies[bodies_i]); bodies_i++)
		{
			if (!body.isActive)
				continue;

			bodyID = body.get_id();
			bodyType = body.get_type();
			
			var lists=this.getListsToCheck(body);
			
			for (var lists_i = 0, lists_l = lists.length, entry; (lists_i < lists_l) && (entry = lists[lists_i]); lists_i++)
			{
				
				for (entry = entry.next; entry != null; entry = entry.next)
				{
				if (body == entry.collisionBody)
					continue;
				
				if (entry.collisionBody.isActive && bodyID > entry.collisionBody.get_id())
					continue;
				
				if (this.checkCollidables(body, entry.collisionBody) && this.detectionFunctors[bodyType + "_" + entry.collisionBody.get_type()] != undefined)
				{
					info = new JigLib.CollDetectInfo();
					info.body0 = body;
					info.body1 = entry.collisionBody;
					fu = this.detectionFunctors[info.body0.get_type() + "_" + info.body1.get_type()];
					fu.collDetect(info, collArr);
					this._numCollisionsChecks += 1;
				} //check collidables
 				}// loop over entries
			} // loop over lists
		} // loop over bodies
		
}



