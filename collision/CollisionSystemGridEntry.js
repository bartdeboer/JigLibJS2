
JigLib.CollisionSystemGridEntry = function(collisionBody)
{
	this.collisionBody = null; // RigidBody
	this.previous = null; // CollisionSystemGridEntry
	this.next = null; // CollisionSystemGridEntry
	this.gridIndex = null; // int

		this.collisionBody = collisionBody;
		this.previous = this.next = null;
		
}


JigLib.CollisionSystemGridEntry.removeGridEntry = function(entry)
{

			// link the JigLib.CollisionSystemGridEntry.previous to the JigLib.CollisionSystemGridEntry.next (may be 0)
			entry.previous.next = entry.next;
			// link the JigLib.CollisionSystemGridEntry.next (if it exists) to the JigLib.CollisionSystemGridEntry.previous.
			if (entry.next != null)
				entry.next.previous = entry.previous;
			// tidy up this entry
			entry.previous = entry.next = null;
			entry.gridIndex = -2;
		
}

JigLib.CollisionSystemGridEntry.insertGridEntryAfter = function(entry, prev)
{

			var next = prev.next;
			prev.next = entry;
			entry.previous = prev;
			entry.next = next;
			if (next != null)
				next.previous = entry;
			entry.gridIndex = prev.gridIndex;
		
}


