package materia.sheet;

/** One physical sheet or reusable remnant. Its measured dimensions use `lengthUnit`. */
typedef StockPiece = {
	var id:String;
	var stockSpecId:String;
	var materialId:String;
	var lengthUnit:String;
	var width:Float;
	var height:Float;
	var thickness:Float;
	var state:String;
	@:optional var batch:String;
	@:optional var location:String;
	@:optional var parentPieceId:String;
	@:optional var sourceExecutionId:String;
	@:optional var allocationPlanId:String;
	/** Captured physical identity and dimensions, checked again at execution time. */
	@:optional var allocationFingerprint:String;
}
