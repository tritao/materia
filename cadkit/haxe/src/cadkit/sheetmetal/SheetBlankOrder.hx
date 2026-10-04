package cadkit.sheetmetal;

/** Caller-owned part/order identities and tolerances; geometry never invents production identity. */
typedef SheetBlankOrder = {
	final id:String;
	final revision:Int;
	final partId:String;
	final partRevision:Int;
	final quantity:Int;
	final thicknessTolerance:SheetTolerance;
	final widthTolerance:SheetTolerance;
	final heightTolerance:SheetTolerance;
}
