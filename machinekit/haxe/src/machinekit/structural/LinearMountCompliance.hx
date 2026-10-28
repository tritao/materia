package machinekit.structural;

import machinekit.component.ComponentFacet;
import machinekit.component.MachineComponent;
import materia.assembly.AssemblyRecord.AssemblyFrame;

typedef MountComplianceTerm = {
  var name:String;
  /** Actual assembly members carrying this term; relative to the receiver's parent. */
  var members:Array<String>;
  /** Local translational flexibility, metres/newton. Terms act in series. */
  var xMPerN:Float;
  var yMPerN:Float;
  var zMPerN:Float;
  var reference:String;
  var assumed:Bool;
}

typedef MountComplianceRecord = {
  var schema:Int;
  var host:String;
  var element:String;
  var frame:AssemblyFrame;
  var terms:Array<MountComplianceTerm>;
}

/** Explicit small-displacement translational mount model. It neither adds mass
 * nor asserts that contacts, preload, creep or rotational restraint are qualified.
 * No omitted direction or unknown input is silently treated as rigid.
 */
class LinearMountCompliance implements ComponentFacet {
  final data:MountComplianceRecord;
  public function new(record:MountComplianceRecord) {
    data = copy(record);
    validate();
  }
  public static function decode(text:String):Null<LinearMountCompliance> {
    var value:Dynamic = haxe.Json.parse(text);
    return value == null ? null : new LinearMountCompliance(copy(value));
  }
  static function numeric(value:Dynamic):Bool return Std.isOfType(value, Float) || Std.isOfType(value, Int);
  static function copy(value:Dynamic):MountComplianceRecord {
    if (value == null || value.frame == null || value.terms == null) throw "Incomplete mount compliance record";
    var frame = value.frame, terms:Array<MountComplianceTerm> = [];
    var source:Array<Dynamic> = value.terms;
    for (term in source) {
      if (term == null || term.members == null || !Std.isOfType(term.assumed,
        Bool)) throw "Incomplete mount compliance term";
      for (name in ["xMPerN",
        "yMPerN", "zMPerN"]) if (!numeric(Reflect.field(term, name))) throw "Missing mount flexibility: " + name;
      var ids:Array<String> = [];
      var raw:Array<Dynamic> = term.members;
      for (id in raw) {
        if (!Std.isOfType(id, String)) throw "Invalid mount member identity";
        ids.push(id);
      }
      terms.push({
        name: term.name,
        members: ids,
        xMPerN: term.xMPerN,
        yMPerN: term.yMPerN,
        zMPerN: term.zMPerN,
        reference: term.reference,
        assumed: term.assumed
      });
    }
    for (name in ["x", "y", "z", "qx",
      "qy", "qz", "qw"]) if (!numeric(Reflect.field(frame, name))) throw "Missing mount frame coordinate: " + name;
    return {
      schema: value.schema,
      host: value.host,
      element: value.element,
      frame: {x: frame.x, y: frame.y, z: frame.z, qx: frame.qx, qy: frame.qy, qz: frame.qz, qw: frame.qw},
      terms: terms
    };
  }
  public function snapshot():MountComplianceRecord return copy(data);
  static function member(id:String):Void {
    if (id == null || StringTools.trim(id) == "" || id.indexOf("/") >= 0
      || id == "." || id == "..") throw "Mount compliance needs a sibling member identity: " + id;
  }
  function validate():Void {
    if (data.schema != 1) throw "Unsupported mount compliance schema";
    member(data.host);
    member(data.element);
    if (data.host == data.element) throw "Mount host and mounted element must differ";
    machinekit.design.DesignGeometry.validateFrame(data.frame);
    if (data.terms == null || data.terms.length == 0) throw "Mount compliance needs explicit load-path terms";
    var seen:Map<String, Bool> = [];
    for (term in data.terms) {
      if (term.name == null || StringTools.trim(term.name) == ""
        || seen.exists(term.name)) throw "Invalid or duplicate mount compliance term";
      seen.set(term.name, true);
      if (term.reference == null || StringTools.trim(term.reference) == "" || term.members == null
        || term.members.length == 0) throw "Mount compliance needs provenance and actual carrying members";
      var ids:Map<String, Bool> = [];
      for (id in term.members) {
        member(id);
        if (ids.exists(id)) throw "Duplicate compliance member";
        ids.set(id, true);
      }
      for (value in [term.xMPerN, term.yMPerN, term.zMPerN]) if (!Math.isFinite(value)
        || value <= 0) throw "Mount flexibility must be positive and finite in every direction";
    }
  }
  public function check(component:MachineComponent):Void validate();
  public function describe():String return "Linear mount translational series compliance (SI; rotation unqualified)";
  public static function of(component:MachineComponent):Null<LinearMountCompliance> {
    var found:Null<LinearMountCompliance> = null;
    for (facet in component.facets()) if (Std.isOfType(facet, LinearMountCompliance)) {
      if (found != null) throw "Duplicate mount compliance facet";
      found = cast facet;
    }
    return found;
  }
  public function stiffnessNPerM(): {x: Float, y: Float, z: Float} {
    var x = 0.0, y = 0.0, z = 0.0;
    for (term in data.terms) {
      x += term.xMPerN;
      y += term.yMPerN;
      z += term.zMPerN;
    }
    return {x: 1 / x, y: 1 / y, z: 1 / z};
  }
  /** Axial compression/tension of n equal parallel members. Units mm, Pa. */
  public static function axialMPerN(lengthMm:Float, areaMm2:Float, youngsPa:Float, count:Int = 1):Float {
    positive([lengthMm, areaMm2, youngsPa]);
    if (count < 1) throw "Invalid parallel member count";
    return lengthMm * 1e-3 / (youngsPa * areaMm2 * 1e-6 * count);
  }
  /** Centre point load on a simply supported rectangular bar (not a cantilever). */
  public static function barMPerN(spanMm:Float, widthMm:Float, depthMm:Float, youngsPa:Float):Float {
    positive([spanMm, widthMm, depthMm, youngsPa]);
    var span = spanMm * 1e-3, inertia = widthMm * Math.pow(depthMm, 3) * 1e-12 / 12;
    return Math.pow(span, 3) / (48 * youngsPa * inertia);
  }
  static function positive(values:Array<Float>):Void {
    for (v in values) if (!Math.isFinite(v) || v <= 0) throw "Invalid mount mechanics input";
  }
}
