package materia.sheet;

/** Coordinates are measured from the lower-left corner of the full sheet, in plan units. */
typedef SheetPartPlacement = {
	var id:String;
	var requirementId:String;
	var x:Float;
	var y:Float;
	var width:Float;
	var height:Float;
	/** Clockwise quarter-turns: 0 or 90 degrees. */
	var rotation:Int;
	/** Leaf region produced by the ordered cut sequence. */
	var regionId:String;
}
