package materia.sheet;

/** Reusable purchased-sheet description. Dimensions use `lengthUnit`. */
typedef SheetStockSpec = {
	var id:String;
	var revision:Int;
	var materialId:String;
	var lengthUnit:String;
	var width:Float;
	var height:Float;
	var thickness:Float;
	@:optional var supplier:String;
	@:optional var supplierCode:String;
}
