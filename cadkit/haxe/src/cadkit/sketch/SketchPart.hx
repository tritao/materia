package cadkit.sketch;

/**
	Constraints that share variables, directly or through each other, and the
	variables they reach, both ascending. `position`/`first` describe the
	envelope of its normal matrix under a reverse Cuthill-McKee ordering (local
	index → row, and each row's first structurally nonzero column); they are
	filled on first use, since a part reused from a previous solve never needs
	them. `envelope` is the solve's storage for that matrix.
*/
typedef SketchPart = {
	var id:Int;
	var constraints:Array<Int>;
	var variables:Array<Int>;
	var position:Array<Int>;
	var first:Array<Int>;
	var ordered:Bool;
	var envelope:Array<Array<Float>>;
}
