import cadkit.parametric.Document;
import cadkit.parametric.DocumentCodec;
import cadkit.parametric.ParameterRole;
import cadkit.parametric.recording.DocumentBuilder;

/** FEATURE_PARTS_PLAN FP1c: named parameters say who sets them (editable, derived, fixed), from what, and keep it saved. */
class ParameterRoleSmoke {
  static function check(value:Bool, message:String):Void {
    if (!value) throw "ParameterRoleSmoke: " + message;
  }

  static function throws(action:() -> Void, expected:String, what:String):Void {
    var message:Null<String> = null;
    try action() catch (error:Dynamic) message = Std.string(error);
    check(message != null, what + " did not throw");
    check(message.indexOf(expected) >= 0, what + ' threw "$message", expected "$expected"');
  }

  static function roles(document:Document):String
    return [for (name in ["plate.length", "plate.halfWidth", "plate.thickness", "plate.area"]) {
      var p = document.parameter(name);
      '$name=${p.role}/${p.basis}/${p.note}';
    }].join("; ");

  public static function run():Void {
    var document = DocumentBuilder.build(function(builder) {
      var length = builder.length("plate.length", 326);
      var halfWidth = builder.length("plate.halfWidth", 255).derivedBy("rail centre + block bolt spacing / 2 + edge, up to 5 mm");
      var thickness = builder.length("plate.thickness", 20).fixedBy("ground flat stock 20 mm");
      builder.area("plate.area", 0);
      builder.output(builder.box(length, halfWidth, thickness));
    });
    // An expression parameter is derived by construction.
    document.installExpression("plate.area", "plate.length * plate.halfWidth * 2");
    document.recompute();

    var p = document.parameter("plate.length");
    check(p.role == ParameterRole.Editable && p.basis == "" && p.note == "", "editable by default");
    var half = document.parameter("plate.halfWidth");
    check(half.role == ParameterRole.Derived && half.basis == "Derived", "derivedBy sets role and basis");
    check(half.note.indexOf("block bolt spacing") >= 0, "the rule is the note");
    var thick = document.parameter("plate.thickness");
    check(thick.role == ParameterRole.Fixed && thick.basis == "Catalog" && thick.note == "ground flat stock 20 mm", "fixedBy");
    var area = document.parameter("plate.area");
    check(area.role == ParameterRole.Derived, "an expression parameter is derived");
    throws(() -> area.editable(), "is derived, not editable", "making an expression editable");
    throws(() -> p.describe("guessed"), "unknown parameter role: guessed", "an unknown role");
    p.editable("Assumed", "operator chooses the stroke");

    var before = roles(document);
    var text = DocumentCodec.encode(document);
    check(text.indexOf('"role":"derived"') >= 0 && text.indexOf('"role":"fixed"') >= 0, "roles saved");
    check(text.indexOf('"role":"editable"') < 0, "the default role is not written");
    document.close();

    var restored = DocumentCodec.decode(text);
    check(roles(restored) == before, 'roles restored: ${roles(restored)} vs $before');
    check(DocumentCodec.encode(restored) == text, "save after restore is identical");
    restored.close();
    Sys.println("ParameterRoleSmoke: editable, derived and fixed parameters keep their rule and source");
  }

  public static function main():Void run();
}
