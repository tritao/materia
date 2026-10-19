package machinekit.component;

@:wire typedef EnclosurePanelBom = {
	@:id(1) var construction:machinekit.enclosure.LaminatedPanel.PanelBomProperties;
	@:id(2) var widthMm:Float;
	@:id(3) var heightMm:Float;
	@:id(4) var areaM2:Float;
	@:id(5) var massKg:Float;
}

typedef BomItem = {
	var partNumber:String;
	var description:String;
	var quantity:Int;
	var material:Null<String>;
	var ?familyId:String;
	var ?manufacturingVariant:String;
	var ?manufacturingProcess:String;
	var ?typeId:String;
	var ?valuesKey:String;
	var ?enclosurePanel:EnclosurePanelBom;
}
