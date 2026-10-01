package humankit.job;

import humankit.HumanBody;

/** Validated target in a canonical job step. */
typedef HumanJobTarget = {
  @:optional var object:String;
  @:optional var point:Array<Float>;
  @:optional var anchor:String;
}

/** Typed canonical step. Action-specific requirements are enforced by parse. */
typedef HumanJobStep = {
  var action:String;
  @:optional var target:HumanJobTarget;
  @:optional var via:Array<Array<Float>>;
  @:optional var object:String;
  /** The surface a picked object rests on, so the worker stands clear of its edge. */
  @:optional var from:String;
  @:optional var onto:String;
  @:optional var offset:Array<Float>;
  @:optional var retreat:String;
  @:optional var hand:String;
  @:optional var seconds:Float;
  @:optional var clip:String;
}

/** Strict, versioned description of an editor-authored human job. */
class HumanJobSpec {
  public final version:Int = 1;
  public final loop:Bool;
  public final steps:Array<HumanJobStep>;

  function new(loop:Bool, steps:Array<HumanJobStep>) {
    this.loop = loop;
    this.steps = steps;
  }

  public static function parse(json:String):HumanJobSpec {
    var raw:Dynamic;
    try raw = haxe.Json.parse(json) catch (error:Dynamic) throw 'Invalid job JSON: $error';
    object(raw, "job");
    fields(raw, ["version", "loop", "steps"], "job");
    var version:Dynamic = Reflect.field(raw, "version");
    if (!Std.isOfType(version, Int)) throw "job.version must be an integer";
    if (version != 1) throw 'Unsupported job version $version';
    var loop:Dynamic = Reflect.field(raw, "loop");
    if (!Std.isOfType(loop, Bool)) throw "job.loop must be a boolean";
    var steps:Dynamic = Reflect.field(raw, "steps");
    if (!Std.isOfType(steps, Array)) throw "job.steps must be an array";
    var held:Map<String, String> = new Map();
    var normalized:Array<HumanJobStep> = [];
    var index = 0;
    for (step in(cast steps:Array<Dynamic>)) {
      var label = 'step $index';
      object(step, label);
      var action = string(step, "action", label);
      var clean:HumanJobStep = {action: action};
      switch action {
        case "walkTo":
          fields(step, ["action", "target", "via"], label);
          var target = Reflect.field(step, "target");
          object(target, '$label.target');
          fields(target, ["object", "point"], '$label.target');
          var hasObject = Reflect.hasField(target, "object");
          var hasPoint = Reflect.hasField(target, "point");
          if (hasObject == hasPoint) throw '$label.target needs exactly one of object or point';
          var walkTarget:HumanJobTarget = hasObject ? {
            object:string(target, "object", '$label.target')
          }
          : {point: point(target, "point", '$label.target')}
          ;
          clean.target = walkTarget;
          if (Reflect.hasField(step, "via")) {
            var via:Dynamic = Reflect.field(step, "via");
            if (!Std.isOfType(via, Array)) throw '$label.via must be an array';
            Reflect.setField(clean, "via", [for (p in(cast via:Array<Dynamic>)) pointValue(p, '$label.via point')]);
          }
        case "pick":
          fields(step, ["action", "object", "hand", "from"], label);
          var id = string(step, "object", label);
          if (Reflect.hasField(step, "from")) Reflect.setField(clean, "from", string(step, "from", label));
          var hand = hand(step, label);
          for (h in hands(hand)) if (held.exists(h)) throw '$label: hand $h already holds an object';
          for (h in hands(hand)) held.set(h, id);
          Reflect.setField(clean, "object", id);
          Reflect.setField(clean, "hand", hand);
        case "place":
          fields(step, ["action", "onto", "offset", "hand", "retreat"], label);
          var onto = string(step, "onto", label);
          var hand = hand(step, label);
          var part:Null<String> = null;
          for (h in hands(hand)) {
            var picked = held.get(h);
            if (picked == null) throw '$label: place without a preceding pick for hand $h';
            if (part != null && part != picked) throw '$label: both hands must hold the same object';
            part = picked;
          }
          for (h in hands(hand)) held.remove(h);
          Reflect.setField(clean, "onto", onto);
          Reflect.setField(clean, "hand", hand);
          if (Reflect.hasField(step, "offset")) Reflect.setField(clean, "offset", point(step, "offset", label));
          if (Reflect.hasField(step, "retreat")) {
            var retreat = string(step, "retreat", label);
            if (retreat != "backward" && retreat != "turn")
              throw '$label.retreat must be "backward" or "turn"';
            Reflect.setField(clean, "retreat", retreat);
          }
        case "press":
          fields(step, ["action", "target", "hand"], label);
          var target = Reflect.field(step, "target");
          object(target, '$label.target');
          fields(target, ["object", "point", "anchor"], '$label.target');
          var hasObject = Reflect.hasField(target, "object");
          var hasPoint = Reflect.hasField(target, "point");
          if (hasObject == hasPoint) throw '$label.target needs exactly one of object or point';
          var anchor:Dynamic = Reflect.field(target, "anchor");
          if (anchor != null && anchor != "top"
            && anchor != "front" && anchor != "center") throw '$label.target.anchor must be top, front, or center';
          if (hasPoint && anchor != null) throw '$label.target.anchor needs an object';
          var resolved:HumanJobTarget = hasObject ? {
            object:string(target, "object", '$label.target')
          }
          : {point: point3(target, "point", '$label.target')};
          if (anchor != null) resolved.anchor = anchor;
          clean.target = resolved;
          Reflect.setField(clean, "hand", hand(step, label));
        case "wait":
          fields(step, ["action", "seconds"], label);
          Reflect.setField(clean, "seconds", nonnegative(step, "seconds", label));
        case "playClip":
          fields(step, ["action", "clip", "seconds"], label);
          Reflect.setField(clean, "clip", string(step, "clip", label));
          Reflect.setField(clean, "seconds", nonnegative(step, "seconds", label));
        default:
          throw '$label: unknown action "$action"';
      }
      normalized.push(clean);
      index++;
    }
    if (loop && normalized.length == 0) throw "A looping job needs at least one step";
    return new HumanJobSpec(loop, normalized);
  }

  public function toJson():String {
    var values:Array<Dynamic> = [];
    for (step in steps) {
      var value:Dynamic = {action:step.action};
      switch step.action {
        case "walkTo", "press":
          var source:Dynamic = Reflect.field(step,"target");
          var object:Null<String> = Reflect.field(source,"object");
          var target:Dynamic = object == null ? {point:Reflect.field(source,"point")} : {object:object};
          var anchor:Null<String> = Reflect.field(source,"anchor");
          if (anchor != null) Reflect.setField(target,"anchor",anchor);
          Reflect.setField(value,"target",target);
          if (step.action == "walkTo") {
            var via:Dynamic = Reflect.field(step,"via");
            if (via != null) Reflect.setField(value,"via",via);
          } else Reflect.setField(value,"hand",Reflect.field(step,"hand"));
        case "pick":
          Reflect.setField(value,"object",Reflect.field(step,"object"));
          Reflect.setField(value,"hand",Reflect.field(step,"hand"));
        case "place":
          Reflect.setField(value,"onto",Reflect.field(step,"onto"));
          Reflect.setField(value,"hand",Reflect.field(step,"hand"));
          var offset:Dynamic = Reflect.field(step,"offset");
          if (offset != null) Reflect.setField(value,"offset",offset);
          var retreat:Dynamic = Reflect.field(step,"retreat");
          if (retreat != null) Reflect.setField(value,"retreat",retreat);
        case "wait": Reflect.setField(value,"seconds",Reflect.field(step,"seconds"));
        case "playClip":
          Reflect.setField(value,"clip",Reflect.field(step,"clip"));
          Reflect.setField(value,"seconds",Reflect.field(step,"seconds"));
      }
      values.push(value);
    }
    return haxe.Json.stringify({version:version,loop:loop,steps:values});
  }

  public static function check(spec:HumanJobSpec, targets:HumanJobTargets, body:HumanBody):Array < String > {
    var warnings:Array<String> = [];
    var heldByHand:Map<String, String> = new Map();
    for (index in 0...spec.steps.length) {
      var step = spec.steps[index];
      var action:String = Reflect.field(step, "action");
      var id:Null<String> = null;
      switch action {
        case "walkTo", "press":
          var target:Dynamic = Reflect.field(step, "target");
          id = Reflect.field(target, "object");
        case "pick":
          id = Reflect.field(step, "object");
        case "place":
          id = Reflect.field(step, "onto");
        case "playClip":
          var clip:String = Reflect.field(step, "clip");
          if (body.character.asset.clipIndex(clip) < 0) warnings.push('step $index: character lacks clip "$clip"');
        default:
      }
      if (id != null) {
        var box = targets.box(id);
        if (box == null) warnings.push('step $index: unknown object "$id"');
        else if (action == "pick" || action == "place" || action == "press") {
          var z = box.center[2] +(action
            == "pick" ? box.halfExtents[2] + 0.01 : action == "place" ? box.halfExtents[2] : 0.0);
          if (action == "place") {
            var hand:String = Reflect.field(step, "hand");
            var heldId = heldByHand.get(hand == "both" ? "left" : hand);
            var held = heldId == null ? null : targets.box(heldId);
            if (held != null) z += held.halfExtents[2];
          }
          var shoulder = body.character.pose.bonePosition(UpperArmR);
          if (shoulder != null) {
            var reach = 0.8 *(body.description.upperArm + body.description.forearm);
            if (z - shoulder[2] >= reach) warnings.push('step $index: target above reach');
            if (shoulder[2] - z >= reach) warnings.push('step $index: target below standing arm reach');
          }
        }
      }
      if (action == "pick") {
        var hand:String = Reflect.field(step, "hand");
        if (hand == "both") { heldByHand.set("left", id); heldByHand.set("right", id); }
        else heldByHand.set(hand, id);
      } else if (action == "place") {
        var hand:String = Reflect.field(step, "hand");
        if (hand == "both") { heldByHand.remove("left"); heldByHand.remove("right"); }
        else heldByHand.remove(hand);
      }
    }
    return warnings;
  }

  static function object(value:Dynamic,
    label:String):Void if (value == null || Std.isOfType(value, Array) || Std.isOfType(
      value,
      String
    ) || Std.isOfType(
      value,
      Float
    ) || Std.isOfType(
      value,
      Bool
    )) throw '$label must be an object';

  static function fields(value:Dynamic,
    allowed:Array<String>, label:String):Void for (field in Reflect.fields(value)) if (allowed.indexOf(field)
    < 0) throw '$label has unknown field "$field"';

  static function string(value:Dynamic, field:String, label:String):String {
    var result:Dynamic = Reflect.field(value, field);
    if (!Std.isOfType(result, String) || result == "") throw '$label.$field needs a nonempty string';
    return result;
  }

  static function number(value:Dynamic, label:String):Float {
    if (!Std.isOfType(value, Int) && !Std.isOfType(value, Float)) throw '$label must be a finite number';
    var result:Float = value;
    if (!Math.isFinite(result)) throw '$label must be a finite number';
    return result;
  }

  static function nonnegative(value:Dynamic, field:String, label:String):Float {
    var result = number(Reflect.field(value, field), '$label.$field');
    if (result < 0.0) throw '$label.$field cannot be negative';
    return result;
  }

  static function point(value:Dynamic, field:String, label:String):Array < Float > return pointValue(
    Reflect.field(
      value,
      field
    ),
    '$label.$field'
  );

  static function point3(value:Dynamic, field:String, label:String):Array<Float> {
    var raw:Dynamic = Reflect.field(value, field);
    if (!Std.isOfType(raw, Array) || (cast raw:Array<Dynamic>).length != 3)
      throw '$label.$field needs [x, y, z]';
    var values:Array<Dynamic> = cast raw;
    return [for (index in 0...3) number(values[index], '$label.$field[$index]')];
  }

  static function pointValue(value:Dynamic, label:String):Array < Float > {
    if (!Std.isOfType(value, Array) ||(cast value:Array<Dynamic>).length != 2) throw '$label needs [x, y]';
    var pair:Array<Dynamic> = cast value;
    return [number(pair[0], '$label[0]'), number(pair[1], '$label[1]')];
  }

  static function hand(value:Dynamic, label:String):String {
    var result:Dynamic = Reflect.field(value, "hand");
    if (result == null && !Reflect.hasField(value, "hand")) return "right";
    if (result != "right" && result != "left" && result != "both") throw '$label.hand must be right, left, or both';
    return result;
  }

  static function hands(hand:String):Array < String > return hand == "both" ?["left", "right"] :[hand];
}
