package materia.sheet;

/** A full straight cut through one current rectangular region. */
typedef SheetCut = {
	var id:String;
	var regionId:String;
	/** `vertical` splits left/right; `horizontal` splits lower/upper. */
	var axis:String;
	/** Distance from the region's minimum X or Y edge to the near side of the kerf. */
	var position:Float;
	var firstRegionId:String;
	var secondRegionId:String;
}
