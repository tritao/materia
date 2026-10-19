package machinekit.enclosure;

import materia.project.MaterialLibrary;
import machinekit.transmission.ValueBasis;

@:wire typedef PanelLayer = {
  @:id(1) var materialId:String;
  @:id(2) var thicknessMm:Float;
}

@:wire typedef PanelProperty = {
  @:id(1) var value:Float;
  @:id(2) var basis:ValueBasis;
  @:id(3) var source:String;
}

/** Unknown qualification stays null; nominal material names never establish a rating. */
@:wire typedef PanelQualifications = {
  @:id(1) var coolantRecord:Null<String>;
  @:id(2) var heatLimitC:Null<PanelProperty>;
  @:id(3) var fireClass:Null<String>;
  @:id(4) var fireRecord:Null<String>;
  @:id(5) var impactRecord:Null<String>;
}

@:wire typedef PanelBomProperties = {
  @:id(1) var constructionId:String;
  @:id(2) var layers:Array<PanelLayer>;
  @:id(3) var thicknessMm:Float;
  @:id(4) var thicknessBasis:ValueBasis;
  @:id(5) var arealMassKgM2:Float;
  @:id(6) var massBasis:ValueBasis;
  @:id(7) var fabrication:String;
  @:id(8) var jointMethod:String;
  @:id(9) var coating:String;
  @:id(10) var qualifications:PanelQualifications;
  @:id(11) var stiffnessPa:Null<PanelProperty>;
  @:id(12) var costPerM2:Null<PanelProperty>;
}

enum PanelCoating {
  None;
  Epoxy;
  Polyurethane;
  PolyureaWithPolyurethane;
  CeramicFilledEpoxy;
  PowderCoat;
}

enum PanelFabrication {
  RouterCut;
  VGrooveFold;
  PressBrakeFold;
  HotGasWeld;
  WetLayup;
  BoughtFinished;
}

/** A layer stack with planning mass, explicit provenance and no inferred protective properties. */
class LaminatedPanel {
  public final id:String;
  public final layers:Array<PanelLayer>;
  public final fabrication:PanelFabrication;
  public final jointMethod:String;
  public final coating:PanelCoating;
  public final qualifications:PanelQualifications;
  public final stiffnessPa:Null<PanelProperty>;
  public final costPerM2:Null<PanelProperty>;

  public function new(id:String, layers:Array<PanelLayer>, fabrication:PanelFabrication,
      jointMethod:String, coating:PanelCoating = None, ?qualifications:PanelQualifications,
      ?stiffnessPa:PanelProperty, ?costPerM2:PanelProperty) {
    this.id = id;
    this.layers = [for (layer in layers) {materialId:layer.materialId, thicknessMm:layer.thicknessMm}];
    this.fabrication = fabrication;
    this.jointMethod = jointMethod;
    this.coating = coating;
    this.qualifications = qualifications == null ? {
      coolantRecord:null, heatLimitC:null, fireClass:null, fireRecord:null, impactRecord:null
    } : qualifications;
    this.stiffnessPa = stiffnessPa;
    this.costPerM2 = costPerM2;
    validate();
  }

  function fail(message:String):Void throw 'Panel "$id": $message';
  static function blank(value:String):Bool return value == null || StringTools.trim(value).length == 0;

  function property(value:Null<PanelProperty>, name:String, positive:Bool):Void {
    if (value == null) return;
    if (!Math.isFinite(value.value) || (positive && value.value <= 0) || blank(value.source))
      fail('invalid $name or missing source');
  }

  public function validate():Void {
    if (blank(id) || blank(jointMethod) || layers.length == 0) fail("missing identity, joints or layers");
    for (layer in layers) {
      MaterialLibrary.require(layer.materialId);
      if (!Math.isFinite(layer.thicknessMm) || layer.thicknessMm <= 0) fail("invalid layer thickness");
      if (layer.materialId == "fr-acm" && coating == PowderCoat) fail("powder coat is not permitted on ACM");
    }
    property(stiffnessPa, "stiffness", true);
    property(costPerM2, "cost", true);
    property(qualifications.heatLimitC, "heat limit", false);
    for (record in [qualifications.coolantRecord, qualifications.fireRecord, qualifications.impactRecord])
      if (record != null && blank(record)) fail("empty qualification record");
    if ((qualifications.fireClass == null) != (qualifications.fireRecord == null))
      fail("fire class requires a source record and vice versa");
    if (qualifications.fireClass != null && blank(qualifications.fireClass)) fail("empty fire class");
  }

  public function thicknessMm():Float {
    var result = 0.0;
    for (layer in layers) result += layer.thicknessMm;
    return result;
  }

  public function arealMassKgM2():Float {
    validate();
    var result = 0.0;
    for (layer in layers) result += MaterialLibrary.require(layer.materialId).physical.density * layer.thicknessMm / 1000;
    return result;
  }

  /** Detailed fabrication/BOM payload; no qualification is promoted from a material name. */
  public function bomProperties():PanelBomProperties {
    validate();
    return {constructionId:id, layers:[for (layer in layers) {materialId:layer.materialId, thicknessMm:layer.thicknessMm}],
      thicknessMm:thicknessMm(), thicknessBasis:ValueBasis.Assumed, arealMassKgM2:arealMassKgM2(),
      massBasis:ValueBasis.Assumed, fabrication:Std.string(fabrication), jointMethod:jointMethod,
      coating:Std.string(coating), qualifications:qualifications, stiffnessPa:stiffnessPa, costPerM2:costPerM2};
  }
}
