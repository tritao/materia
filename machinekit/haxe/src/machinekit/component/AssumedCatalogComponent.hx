package machinekit.component;

import machinekit.transmission.ValueBasis;

/** Unverified catalogue-class layout part. Ratings and envelopes are assumptions,
 * never vendor data. Subclasses supply a saved recipe and meaningful interfaces. */
class AssumedCatalogComponent extends MachineComponent {
  final assumedInputs:ComponentValues;
  public final ratingsBasis:ValueBasis = Assumed;
  function new(id:String,
    label:String, material:String, parameters:Array<ComponentParameter>, ?inputs:ComponentValues) {
    var v = ComponentRecipeSupport.resolve(parameters, inputs);
    if (v.get("material") == null) v.setToken("material", material);
    super(
      id.toUpperCase() + "-ASSUMED-" + haxe.crypto.Sha256.encode(key(v)).substr(0, 12),
      "Assumed " + label,
      v.token("material"),
      false,
      v
    );
    assumedInputs = v.copy();
  }
  static function key(v:ComponentValues):String {
    var names = v.names();
    names.sort(Reflect.compare);
    return haxe.Json.stringify([for (name in names) {
      name:name, value:Std.string(v.get(name))
    }
    ]);
  }
  public function flag(name:String):Bool return assumedInputs.boolean(name);
  public function io():MachineIo return {inputs:[], outputs:[], analogOutputs:[]};
  public function n(name:String):Float return parameterNumber(name);
  public function positive(names:Array<String>):Void {
    for (name in names) if (!Math.isFinite(n(name))
      || n(name) <= 0) throw designation + ": " + name + " must be finite and positive";
  }
  public function require(ok:Bool, message:String):Void {
    if (!ok) throw designation + ": " + message;
  }
  override public function hasGeometry():Bool return true;
}
