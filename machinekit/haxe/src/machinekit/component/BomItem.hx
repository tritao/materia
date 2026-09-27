package machinekit.component;

typedef BomItem = {
	var partNumber:String;
	var description:String;
	var quantity:Int;
	var material:Null<String>;
	var ?typeId:String;
	var ?valuesKey:String;
}
