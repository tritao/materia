package app.editor;

import animkit.AnimationAsset;
import animkit.AnimationInstance;
import app.CadDocumentSession;
import app.EditorScene;
import app.SceneObjectData;
import app.WorkerObjectData;
import humankit.job.HumanJobSpec;
import humankit.HumanBody;
import nativekit.ui.properties.PropertyDescriptor;
import nativekit.ui.properties.PropertyDescriptorOptions;
import nativekit.ui.properties.PropertyOption;
import nativekit.ui.properties.PropertyType;
import nativekit.ui.properties.PropertyValue;

/** Scene object and inspector for an authored human worker. */
@:access(app.EditorScene)
class HumanWorkerKind implements ObjectKindProvider {
  public static inline var KIND = "human-worker";
  public static inline var DEFAULT_ASSET = "animkit/assets/quaternius/worker.glb";
  static final selectedStep:Map<String, Int> = new Map();

  public function new() {}
  public function id():String return KIND;
  public function addSection():String return "People";
  public function addLabel():String return "Worker";
  public function addCommand():String return "scene.create-worker";
  public function isCad():Bool return false;
  public function supportsFaceHover():Bool return false;
  public function supportsEdgeHover():Bool return false;
  public function hasGeneratedGeometry():Bool return false;
  public function supportsSketchEdit():Bool return false;
  public function isPrimitive():Bool return false;

  public static function boundsFor(path:String):Array<Float> {
    var asset = AnimationAsset.load(WorkerAssetPath.resolve(path));
    var instance:Null<AnimationInstance> = null;
    var bounds:Array<Float>;
    try {
      instance = new AnimationInstance(asset);
      bounds = instance.bounds();
    } catch (error:Dynamic) {
      if (instance != null) instance.dispose();
      asset.dispose();
      throw error;
    }
    instance.dispose();
    asset.dispose();
    return bounds;
  }

  public function createDefaultRecord(id:String):SceneObjectData {
    var bounds = boundsFor(DEFAULT_ASSET);
    return {id:id, label:"Worker", type:KIND, x:0.0, y:0.0, z:0.0,
      width:bounds[3]-bounds[0], height:bounds[4]-bounds[1], depth:bounds[5]-bounds[2],
      collisionEnabled:false, dynamicBody:false, mass:1.0,
      red:0.7, green:0.7, blue:0.7, visible:true,
      worker:{asset:DEFAULT_ASSET, job:'{"version":1,"loop":false,"steps":[]}', zones:[]}};
  }

  public function createCadSession(graph:Null<String>, width:Float, height:Float,
      depth:Float):Null<CadDocumentSession> return null;

  public function commands(scene:EditorScene, id:String):Array<String> return [
    "scene.worker-add-step", "scene.worker-remove-step", "scene.worker-step-up", "scene.worker-step-down"
  ];

  static function current(scene:EditorScene, id:String):WorkerObjectData {
    var item = scene.object(id);
    if (item == null || item.kind != KIND || item.worker == null) throw 'Unknown worker "$id"';
    return item.worker;
  }

  static function copy(data:WorkerObjectData, ?job:String, ?asset:String, ?zones:Array<String>):WorkerObjectData
    return {asset:asset == null ? data.asset : asset, job:job == null ? data.job : job,
      zones:zones == null ? data.zones.copy() : zones};

  static function parsed(scene:EditorScene, id:String):HumanJobSpec
    return HumanJobSpec.parse(current(scene, id).job);

  static function editStep(scene:EditorScene, id:String, index:Int, field:String, value:Dynamic):Void {
    var data = current(scene, id);
    var raw:Dynamic = haxe.Json.parse(data.job);
    var steps:Array<Dynamic> = Reflect.field(raw, "steps");
    if (index < 0 || index >= steps.length) throw "Worker step no longer exists";
    if (field == "action") {
      var hand = "right";
      if (value == "place") {
        var held:Array<String> = [];
        for (i in 0...index) {
          var prior = steps[i], action:String = Reflect.field(prior, "action");
          if (action == "pick") held.push(Reflect.field(prior, "hand"));
          if (action == "place") held.remove(Reflect.field(prior, "hand"));
        }
        if (held.length > 0) hand = held[held.length - 1];
      }
      steps[index] = defaultStep(value, hand);
    }
    else {
      if (field == "hand") {
        var action:String = Reflect.field(steps[index], "action");
        if (action == "pick" || action == "place") {
          var mate = -1;
          var oldHand:String = Reflect.field(steps[index], "hand");
          if (action == "pick") {
            for (i in index + 1...steps.length) {
              var other:String = Reflect.field(steps[i], "action");
              if (other == "place" && Reflect.field(steps[i], "hand") == oldHand) { mate = i; break; }
            }
          } else {
            var i = index - 1;
            while (i >= 0) {
              var other:String = Reflect.field(steps[i], "action");
              if (other == "pick" && Reflect.field(steps[i], "hand") == oldHand) { mate = i; break; }
              i--;
            }
          }
          if (mate >= 0) Reflect.setField(steps[mate], "hand", value);
        }
      }
      Reflect.setField(steps[index], field, value);
    }
    var next = HumanJobSpec.parse(haxe.Json.stringify(raw));
    scene.setWorkerData(id, copy(data, next.toJson()));
  }

  static function editTarget(scene:EditorScene, id:String, index:Int, field:String, value:Dynamic):Void {
    var data = current(scene, id);
    var raw:Dynamic = haxe.Json.parse(data.job);
    var steps:Array<Dynamic> = Reflect.field(raw,"steps");
    var target:Dynamic = Reflect.field(steps[index],"target");
    Reflect.setField(target,field,value);
    scene.setWorkerData(id, copy(data, HumanJobSpec.parse(haxe.Json.stringify(raw)).toJson()));
  }

  static function objectOptions(scene:EditorScene, category:String, current:String):PropertyDescriptorOptions {
    var options = settings(category);
    options.options = [for (record in scene.records()) new PropertyOption(record.id,record.label)];
    var seen = false;
    for (option in options.options) if (option.key == current) seen = true;
    if (!seen) options.options.push(new PropertyOption(current,current + " (missing)"));
    return options;
  }

  static function handProperty(result:Array<PropertyDescriptor>, scene:EditorScene, id:String,
      prefix:String, index:Int, category:String):Void {
    var options = settings(category);
    options.options = [new PropertyOption("right","Right"),new PropertyOption("left","Left"),
      new PropertyOption("both","Both")];
    result.push(new PropertyDescriptor(prefix+'worker-step-$index-hand', "Hand", PropertyType.Enum,
      function(_) return PropertyValue.Enum(Reflect.field(parsed(scene,id).steps[index],"hand")),
      function(_, value) switch value {
        case PropertyValue.Enum(next): editStep(scene,id,index,"hand",next);
        default: throw "Hand requires a choice";
      }, options));
  }

  static function pointProperty(result:Array<PropertyDescriptor>, scene:EditorScene, id:String,
      prefix:String, index:Int, category:String, field:String, axis:Int):Void {
    var options = settings(category); options.step=0.1; options.unit="m";
    result.push(new PropertyDescriptor(prefix+'worker-step-$index-$field-$axis',
      axis == 0 ? "Point X" : axis == 1 ? "Point Y" : "Point Z",
      PropertyType.Float, function(_) {
        var step = parsed(scene,id).steps[index];
        var point:Array<Float> = field == "target" ? Reflect.field(Reflect.field(step,"target"),"point")
          : Reflect.field(step,field);
        if (point == null) point = field == "target" &&
          Reflect.field(step,"action") == "press" ? [0.0,0.0,1.0] : [0.0,0.0];
        return PropertyValue.Float(point[axis]);
      }, function(_, value) {
        var number:Float = switch value {
          case PropertyValue.Float(next): next;
          case PropertyValue.Int(next): next;
          default: throw "Point coordinate requires a number";
        };
        var step = parsed(scene,id).steps[index];
        var point:Array<Float> = field == "target" ? Reflect.field(Reflect.field(step,"target"),"point")
          : Reflect.field(step,field);
        if (point == null) point = field == "target" &&
          Reflect.field(step,"action") == "press" ? [0.0,0.0,1.0] : [0.0,0.0];
        var changed = point.copy(); changed[axis] = number;
        if (field == "target") editTarget(scene,id,index,"point",changed);
        else editStep(scene,id,index,field,changed);
      }, options));
  }

  static function defaultStep(action:String, hand:String = "right"):Dynamic return switch action {
    case "walkTo": {action:"walkTo", target:{point:[0.0, 0.0]}};
    case "pick": {action:"pick", object:"part", hand:"right"};
    case "place": {action:"place", onto:"table", hand:hand};
    case "press": {action:"press", target:{object:"button", anchor:"top"}, hand:"right"};
    case "wait": {action:"wait", seconds:1.0};
    case "playClip": {action:"playClip", clip:"Wave", seconds:1.0};
    default: throw 'Unknown worker action "$action"';
  };

  public static function selectStep(id:String, index:Int):Void selectedStep.set(id, index);

  public static function command(scene:EditorScene, command:String):Bool {
    var id = scene.selectedId;
    var data:WorkerObjectData;
    try data = current(scene, id) catch (_:Dynamic) return false;
    var raw:Dynamic;
    try raw = haxe.Json.parse(data.job) catch (_:Dynamic) return false;
    var steps:Array<Dynamic> = Reflect.field(raw, "steps");
    if (steps == null) return false;
    var stored = selectedStep.get(id);
    var index:Int = stored == null ? steps.length - 1 : stored;
    switch command {
      case "scene.worker-add-step":
        steps.push(defaultStep("wait"));
        selectedStep.set(id, steps.length - 1);
      case "scene.worker-remove-step":
        if (index < 0 || index >= steps.length) return false;
        steps.splice(index, 1);
        selectedStep.set(id, Std.int(Math.min(index, steps.length - 1)));
      case "scene.worker-step-up":
        if (index <= 0 || index >= steps.length) return false;
        var step = steps[index]; steps[index] = steps[index - 1]; steps[index - 1] = step;
        selectedStep.set(id, index - 1);
      case "scene.worker-step-down":
        if (index < 0 || index >= steps.length - 1) return false;
        var step = steps[index]; steps[index] = steps[index + 1]; steps[index + 1] = step;
        selectedStep.set(id, index + 1);
      default: return false;
    }
    scene.setWorkerData(id, copy(data, HumanJobSpec.parse(haxe.Json.stringify(raw)).toJson()));
    return true;
  }

  static function settings(category:String):PropertyDescriptorOptions {
    var result = new PropertyDescriptorOptions();
    result.category = category;
    result.recordHistory = false;
    return result;
  }

  static function readOnly(key:String, label:String, text:Void->String):PropertyDescriptor {
    var options = settings("Job"); options.readOnly = true;
    return new PropertyDescriptor(key, label, PropertyType.Text,
      function(_) return PropertyValue.Text(text()), function(_, _) {}, options);
  }

  public function properties(scene:EditorScene, id:String, prefix:String):Array<PropertyDescriptor> {
    var result:Array<PropertyDescriptor> = [];
    var yawOptions = settings("Transform"); yawOptions.unit = "°"; yawOptions.step = 15.0;
    result.push(new PropertyDescriptor(prefix+"worker-yaw", "Yaw", PropertyType.Float,
      function(_) {
        var item = scene.object(id);
        if (item == null) throw "Worker no longer exists";
        var q = item.rotation;
        if (q == null) return PropertyValue.Float(0.0);
        return PropertyValue.Float(Math.atan2(2 * (q[3] * q[2] + q[0] * q[1]),
          1 - 2 * (q[1] * q[1] + q[2] * q[2])) * 180.0 / Math.PI);
      }, function(_, value) {
        var degrees:Float = switch value {
          case PropertyValue.Float(next): next;
          case PropertyValue.Int(next): next;
          default: throw "Yaw requires a number";
        };
        scene.setWorkerYaw(id, degrees * Math.PI / 180.0);
      }, yawOptions));
    var asset = settings("Worker");
    result.push(new PropertyDescriptor(prefix+"worker-asset", "Character asset", PropertyType.Text,
      function(_) return PropertyValue.Text(current(scene,id).asset),
      function(_, value) switch value {
        case PropertyValue.Text(path): scene.setWorkerData(id, copy(current(scene,id), null, path));
        default: throw "Asset path requires text";
      }, asset));
    for (record in scene.records()) if (record.id != id) {
      var zoneId = record.id;
      result.push(new PropertyDescriptor(prefix+"worker-zone-"+zoneId, "Zone: "+record.label,
        PropertyType.Bool, function(_) return PropertyValue.Bool(current(scene,id).zones.indexOf(zoneId) >= 0),
        function(_, value) switch value {
          case PropertyValue.Bool(enabled):
            var zones = current(scene,id).zones.copy();
            if (enabled && zones.indexOf(zoneId) < 0) zones.push(zoneId);
            if (!enabled) zones.remove(zoneId);
            scene.setWorkerData(id, copy(current(scene,id), null, null, zones));
          default: throw "Zone selection requires a boolean";
        }, settings("Safety")));
    }
    for (zoneId in current(scene,id).zones) if (scene.object(zoneId) == null) {
      var missingId = zoneId;
      result.push(new PropertyDescriptor(prefix+"worker-missing-zone-"+missingId,
        "Missing zone: "+missingId, PropertyType.Bool,
        function(_) return PropertyValue.Bool(true), function(_, value) switch value {
          case PropertyValue.Bool(false):
            var zones = current(scene,id).zones.copy();
            zones.remove(missingId);
            scene.setWorkerData(id, copy(current(scene,id), null, null, zones));
          case PropertyValue.Bool(true):
          default: throw "Zone selection requires a boolean";
        }, settings("Safety")));
    }
    var error:Null<String> = null;
    var spec:Null<HumanJobSpec> = null;
    try spec = parsed(scene, id) catch (failure:Dynamic) error = Std.string(failure);
    for (zoneId in current(scene,id).zones) if (scene.object(zoneId) == null)
      error = (error == null ? "" : error + "; ") + 'Missing zone "$zoneId"';
    if (spec != null && error == null) {
      var visual = scene.workerVisuals.get(id);
      if (visual != null) {
        var warnings = HumanJobSpec.check(spec, new WorkerSceneTargets(scene.records(), scene),
          new HumanBody(visual.character));
        if (warnings.length > 0) error = warnings.join("; ");
      }
    }
    if (error != null) result.push(readOnly(prefix+"worker-error", "Job error", function() return error));
    if (spec == null) {
      result.push(new PropertyDescriptor(prefix+"worker-json", "Job JSON", PropertyType.Text,
        function(_) return PropertyValue.Text(current(scene,id).job),
        function(_, value) switch value {
          case PropertyValue.Text(text): scene.setWorkerData(id, copy(current(scene,id), text));
          default: throw "Job JSON requires text";
        }, settings("Job")));
      return workerProperties(scene, id, prefix, result);
    }
    var loop = settings("Job");
    result.push(new PropertyDescriptor(prefix+"worker-loop", "Loop", PropertyType.Bool,
      function(_) return PropertyValue.Bool(parsed(scene,id).loop),
      function(_, value) switch value {
        case PropertyValue.Bool(next):
          var raw:Dynamic = haxe.Json.parse(current(scene,id).job);
          Reflect.setField(raw,"loop",next);
          scene.setWorkerData(id, copy(current(scene,id), HumanJobSpec.parse(haxe.Json.stringify(raw)).toJson()));
        default: throw "Loop requires a boolean";
      }, loop));
    // Choosing among steps needs at least two; a slider over one value has no range.
    if (spec.steps.length > 1) {
      var stepSelect = settings("Job"); stepSelect.minimum = 0; stepSelect.maximum = spec.steps.length-1;
      result.push(new PropertyDescriptor(prefix+"worker-step-index", "Selected step", PropertyType.Int,
        function(_) { var stored = selectedStep.get(id); return PropertyValue.Int(stored == null ? 0 : stored); },
        function(_, value) switch value {
          case PropertyValue.Int(index): selectStep(id,index);
          default: throw "Step index requires a number";
        }, stepSelect));
    }
    for (index in 0...spec.steps.length) appendStepProperties(result, scene, id, prefix, index, spec.steps[index]);
    return workerProperties(scene, id, prefix, result);
  }

  static function workerProperties(scene:EditorScene, id:String, prefix:String,
      fields:Array<PropertyDescriptor>):Array<PropertyDescriptor> {
    var all = ScenePropertyProvider.common(scene, id, prefix, fields);
    return [for (field in all) if (field.id == prefix + "position-x" || field.id == prefix + "position-y" ||
      field.id == prefix + "visible" || field.id == prefix + "name" || fields.indexOf(field) >= 0) field];
  }

  static function appendStepProperties(result:Array<PropertyDescriptor>, scene:EditorScene,
      id:String, prefix:String, index:Int, step:Dynamic):Void {
    var category = 'Step ${index + 1}';
    var action:String = Reflect.field(step,"action");
    var actionOptions = settings(category);
    actionOptions.options = [for (name in ["walkTo","pick","place","press","wait","playClip"])
      new PropertyOption(name,name)];
    result.push(new PropertyDescriptor(prefix+'worker-step-$index-action', "Action", PropertyType.Enum,
      function(_) return PropertyValue.Enum(Reflect.field(parsed(scene,id).steps[index],"action")),
      function(_, value) switch value {
        case PropertyValue.Enum(next): editStep(scene,id,index,"action",next);
        default: throw "Action requires a choice";
      }, actionOptions));
    if (action == "walkTo" || action == "press") {
      var target:Dynamic = Reflect.field(step,"target");
      var objectId:Null<String> = Reflect.field(target,"object");
      var mode = settings(category);
      mode.options = [new PropertyOption("object","Object"),new PropertyOption("point","Point")];
      result.push(new PropertyDescriptor(prefix+'worker-step-$index-target-mode',"Target type",PropertyType.Enum,
        function(_) return PropertyValue.Enum(Reflect.hasField(Reflect.field(parsed(scene,id).steps[index],"target"),"object")
          ? "object" : "point"),
        function(_, value) switch value {
          case PropertyValue.Enum("object"):
            var records = scene.records();
            editStep(scene,id,index,"target",{object:records.length == 0 ? "object" : records[0].id});
          case PropertyValue.Enum("point"): editStep(scene,id,index,"target",
            {point:action == "press" ? [0.0,0.0,1.0] : [0.0,0.0]});
          default: throw "Target type requires object or point";
        }, mode));
      if (objectId != null) {
        result.push(new PropertyDescriptor(prefix+'worker-step-$index-target-object',"Target object",PropertyType.Enum,
          function(_) return PropertyValue.Enum(Reflect.field(Reflect.field(parsed(scene,id).steps[index],"target"),"object")),
          function(_, value) switch value {
            case PropertyValue.Enum(next): editTarget(scene,id,index,"object",next);
            default: throw "Target object requires a choice";
          }, objectOptions(scene,category,objectId)));
        if (action == "press") {
          var anchor = settings(category);
          anchor.options = [new PropertyOption("top","Top"),new PropertyOption("front","Front"),
            new PropertyOption("center","Center")];
          result.push(new PropertyDescriptor(prefix+'worker-step-$index-anchor',"Anchor",PropertyType.Enum,
            function(_) {
              var value:Null<String> = Reflect.field(Reflect.field(parsed(scene,id).steps[index],"target"),"anchor");
              return PropertyValue.Enum(value == null ? "center" : value);
            }, function(_, value) switch value {
              case PropertyValue.Enum(next): editTarget(scene,id,index,"anchor",next);
              default: throw "Anchor requires a choice";
            }, anchor));
        }
      } else for (axis in 0...(action == "press" ? 3 : 2))
        pointProperty(result,scene,id,prefix,index,category,"target",axis);
      if (action == "walkTo") {
        result.push(new PropertyDescriptor(prefix+'worker-step-$index-via',"Via points JSON",PropertyType.Text,
          function(_) {
            var via:Dynamic = Reflect.field(parsed(scene,id).steps[index],"via");
            return PropertyValue.Text(via == null ? "[]" : haxe.Json.stringify(via));
          }, function(_, value) switch value {
            case PropertyValue.Text(text): editStep(scene,id,index,"via",haxe.Json.parse(text));
            default: throw "Via points require JSON";
          }, settings(category)));
      } else handProperty(result,scene,id,prefix,index,category);
    }
    if (action == "pick" || action == "place") {
      var field = action == "pick" ? "object" : "onto";
      var chosen:String = Reflect.field(step,field);
      result.push(new PropertyDescriptor(prefix+'worker-step-$index-$field',
        action == "pick" ? "Pick object" : "Place onto",PropertyType.Enum,
        function(_) return PropertyValue.Enum(Reflect.field(parsed(scene,id).steps[index],field)),
        function(_, value) switch value {
          case PropertyValue.Enum(next): editStep(scene,id,index,field,next);
          default: throw "Object requires a choice";
        }, objectOptions(scene,category,chosen)));
      handProperty(result,scene,id,prefix,index,category);
      if (action == "place") for (axis in 0...2) pointProperty(result,scene,id,prefix,index,category,"offset",axis);
    }
    if (action == "wait" || action == "playClip") {
      var duration = settings(category); duration.minimum=0; duration.step=0.1;
      result.push(new PropertyDescriptor(prefix+'worker-step-$index-seconds', "Seconds", PropertyType.Float,
        function(_) return PropertyValue.Float(Reflect.field(parsed(scene,id).steps[index],"seconds")),
        function(_, value) switch value {
          case PropertyValue.Float(next): editStep(scene,id,index,"seconds",next);
          case PropertyValue.Int(next): editStep(scene,id,index,"seconds",next);
          default: throw "Seconds require a number";
        }, duration));
    }
    if (action == "playClip") result.push(new PropertyDescriptor(prefix+'worker-step-$index-clip',"Clip",PropertyType.Text,
      function(_) return PropertyValue.Text(Reflect.field(parsed(scene,id).steps[index],"clip")),
      function(_, value) switch value {
        case PropertyValue.Text(next): editStep(scene,id,index,"clip",next);
        default: throw "Clip requires text";
      }, settings(category)));
  }
}
