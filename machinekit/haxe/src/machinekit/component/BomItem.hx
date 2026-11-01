package machinekit.component;

@:wire typedef BomProperties = {
  @:id(1) var construction:machinekit.enclosure.LaminatedPanel.PanelBomProperties;
  @:id(2) var widthMm:Float;
  @:id(3) var heightMm:Float;
  @:id(4) var areaM2:Float;
  @:id(5) var massKg:Float;
}

/** Quantity per item, for stock ordered by length, area, mass or volume. */
@:wire typedef BomMeasure = {
  @:id(1) var value:Float;
  @:id(2) var unit:String;
}

@:wire typedef BomCut = {
  @:id(1) var length:Float;
  @:id(2) var count:Int;
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
  var ?fingerprint:String;
  var ?procurement:Procurement;
  var ?maturity:String;
  var ?requirements:Array<machinekit.component.Procurement.ProcurementRequirement>;
  var ?properties:BomProperties;
  var ?measure:BomMeasure;
  var ?cuts:Array<BomCut>;
}
