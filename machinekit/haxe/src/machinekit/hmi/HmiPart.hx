package machinekit.hmi;

import cadkit.modeling.Footprint;
import cadkit.modeling.FootprintGeometry;
import machinekit.component.AssumedCatalogComponent;
import machinekit.component.ComponentParameter;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.ComponentValues;
import machinekit.component.MachineComponent;
import machinekit.transmission.ValueBasis;

/**
 * A bought operator control for a panel. Unlike an assumed catalogue part its numbers may be a maker's drawing or a
 * measurement; the recipe's `basis` input says which, and travels in its designation and BOM line. Parts
 * are in their own frame: origin at the centre of the panel's front face, +z toward the operator.
 */
class HmiPart extends MachineComponent implements PanelPart {
  /** How trustworthy the part's numbers are. */
  public final basis:ValueBasis;
  final stored:ComponentValues;

  function new(id:String, label:String, material:String, parameters:Array<ComponentParameter>, ?inputs:ComponentValues) {
    var v = ComponentRecipeSupport.resolve(parameters, inputs);
    if (v.get("material") == null) v.setToken("material", material);
    var tag = v.token("basis");
    super(id.toUpperCase() + "-" + tag.toUpperCase() + "-" + haxe.crypto.Sha256.encode(key(v)).substr(0, 12), label, v.token("material"), false, v);
    stored = v.copy();
    basis = switch tag {
      case "catalog": Catalog;
      case "stated": Stated;
      case "measured": Measured;
      case _: Assumed;
    };
  }

  /** The provenance input every part declares. */
  public static function basisParameter(value:String = "assumed"):ComponentParameter
    return ComponentRecipeSupport.choice("basis", ["catalog", "stated", "assumed", "measured"], value);

  static function key(v:ComponentValues):String {
    var names = v.names();
    names.sort(Reflect.compare);
    return haxe.Json.stringify([for (name in names) {name: name, value: Std.string(v.get(name))}]);
  }

  public function n(name:String):Float return parameterNumber(name);
  public function flag(name:String):Bool return stored.boolean(name);
  /** An integer input (a count). */
  public function count(name:String):Int return stored.integer(name);
  public function positive(names:Array<String>):Void {
    for (name in names) if (!Math.isFinite(n(name)) || n(name) <= 0) throw designation + ": " + name + " must be finite and positive";
  }
  public function require(ok:Bool, message:String):Void {
    if (!ok) throw designation + ": " + message;
  }
  override public function hasGeometry():Bool return true;

  public function dataSource():String return switch basis {
    case Catalog: "vendor-drawing";
    case Stated: "listing";
    case Measured: "measured";
    case _: "assumed";
  };

  // The layout interface; each part states its own.
  public function panelCutout():Footprint throw "not defined";
  public function panelKeepOut():Footprint throw "not defined";
  public function panelHoles():Array<{x:Float, y:Float, diameter:Float}> return [];
  public function aboveFace():Float throw "not defined";
  public function behindFace():Float throw "not defined";
  public function rearBody():Footprint return panelCutout();
  public function massGrams():Float throw "not defined";
  public function maxPanelThickness():Float return 1e9;
}
