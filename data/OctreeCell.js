
(function(JigLib) {


	var OctreeCell = function(aabox)
	{
		this.childCellIndices = null; // int
		this.triangleIndices = null; // int
		this.AABox = null; // JAABox
		this._points = null; // Vector3D
		this._egdes = null; // EdgeData

		this.childCellIndices = [];
		this.triangleIndices = [];
		
		this.clear();
		
		if(aabox){
			this.AABox = aabox.clone();
		}else {
			this.AABox = new JigLib.JAABox();
		}
		this._points = this.AABox.getAllPoints();
		this._egdes = this.AABox.get_edges();
		
	}

	OctreeCell.prototype.isLeaf = function()
	{

		return this.childCellIndices[0] == -1;
		
	}

	OctreeCell.prototype.clear = function()
	{

		for (var i = 0; i < JigLib.OctreeCell.NUM_CHILDREN; i++ ) {
			this.childCellIndices[i] = -1;
		}
		this.triangleIndices.splice(0, this.triangleIndices.length);
		
	}

	OctreeCell.prototype.get_points = function()
	{

		return this._points;
		
	}

	OctreeCell.prototype.get_egdes = function()
	{

		return this._egdes;
		
	}

	OctreeCell.NUM_CHILDREN =  8; // uint


	JigLib.OctreeCell = OctreeCell; 

})(JigLib);

