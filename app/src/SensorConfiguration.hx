package app;

import nativekit.ui.core.CommandContext;
import nativekit.ui.core.EditorDocument;
import nativekit.ui.core.EditOperation;
import nativekit.ui.core.PropertyDescriptor;
import nativekit.ui.core.PropertyDescriptorOptions;
import nativekit.ui.core.PropertyOption;
import nativekit.ui.core.PropertyType;
import nativekit.ui.core.PropertyValue;
import robotkit.model.Frame;
import robotkit.model.Link;
import robotkit.model.RobotModel;
import robotkit.model.Sensor;
import robotkit.runtime.RobotRuntimeCompiler;
import robotkit.runtime.RobotRuntime;
import robotkit.runtime.RobotSnapshot;
import robotkit.runtime.Simulation;
import haxe.Int64;

/** Editable RobotKit sensor model used by Materia's sensor panel. */
class SensorConfiguration {
  public final model:RobotModel;
  public final document:EditorDocument;
  public var selectedIndex(default,null):Int = 0;
  public var robotId(default, null):String = "materia/robot";
  public var appliedRevision(default, null):Int = 0;
  public var applyError(default, null):Null<String> = null;
  var nextId:Int = 1;
  var simulation:Null<Simulation> = null;
  var runtime:Null<RobotRuntime> = null;
  var running:Bool = false;

  public function new(?data:Dynamic) {
    document = new EditorDocument("sensors");
    model=new RobotModel("Materia robot");
    var base=model.addLink(new Link("Base","base"));
    model.addFrame(new Frame("Base sensor mount",base,"base/sensors"));
    if (data == null) {
      addDirect("lidar");
    } else {
      load(data);
    }
    document.markSaved();
  }

  public function selected():Null<Sensor>
    return selectedIndex < 0 || selectedIndex >= model.sensors.length ? null : model.sensors[selectedIndex];
  public function select(index:Int):Bool {
    if(index<0||index>=model.sensors.length||index==selectedIndex)return false;
    selectedIndex=index;return true;
  }
  public function add(kind:String):Sensor {
    var sensor = createSensor(kind);
    var previous = selectedIndex;
    document.apply(new EditOperation("Add " + kind + " sensor", function() {
      if (model.sensors.indexOf(sensor) < 0) model.addSensor(sensor);
      selectedIndex = model.sensors.indexOf(sensor);
    }, function() {
      model.sensors.remove(sensor);
      selectedIndex = previous;
    }));
    return sensor;
  }
  public function removeSelected():Bool {
    if(model.sensors.length==0)return false;
    var index = selectedIndex;
    var sensor = model.sensors[index];
    document.apply(new EditOperation("Remove sensor", function() {
      model.sensors.remove(sensor);
      selectedIndex=model.sensors.length==0?-1:Std.int(Math.min(index,model.sensors.length-1));
    }, function() {
      model.sensors.insert(index, sensor);
      selectedIndex = index;
    }));
    return true;
  }

  function createSensor(kind:String):Sensor {
    if(kind!="lidar"&&kind!="imu"&&kind!="joint_encoder")throw "Unsupported sensor kind";
    var sensor=new Sensor(kind+" "+nextId,kind,0.0,"sensor/"+nextId++);
    sensor.frame=model.frames[0];
    return sensor;
  }
  function addDirect(kind:String):Sensor {
    var sensor = createSensor(kind);
    model.addSensor(sensor);
    selectedIndex = model.sensors.length - 1;
    return sensor;
  }

  public function records():Dynamic return {
    robotId: robotId,
    links: [for (link in model.links) {id:link.id, name:link.name}],
    frames: [for (frame in model.frames) {id:frame.id, name:frame.name, linkId:frame.link.id,
      position:frame.position.copy(), rotation:frame.rotation.copy()}],
    sensors: [for (sensor in model.sensors) {id:sensor.id, name:sensor.name, kind:sensor.kind,
      updateRate:sensor.updateRate, frameId:sensor.frame == null ? null : sensor.frame.id,
      rayCount:sensor.rayCount, maxRange:sensor.maxRange, noiseStddev:sensor.noiseStddev,
      noiseSeed:sensor.noiseSeed}]
  };

  function load(data:Dynamic):Void {
    robotId = requiredString(data, "robotId");
    model.links.resize(0); model.frames.resize(0); model.sensors.resize(0);
    var links = new Map<String, Link>();
    for (value in requiredArray(data, "links")) {
      var link = model.addLink(new Link(requiredString(value, "name"), requiredString(value, "id")));
      if (links.exists(link.id)) throw "Duplicate sensor document link ID";
      links.set(link.id, link);
    }
    if (model.links.length == 0) throw "Sensor document requires a robot link";
    var frames = new Map<String, Frame>();
    for (value in requiredArray(data, "frames")) {
      var link = links.get(requiredString(value, "linkId"));
      if (link == null) throw "Sensor frame references an unknown link";
      var frame = new Frame(requiredString(value, "name"), link, requiredString(value, "id"));
      frame.position = vector(value, "position", 3);
      frame.rotation = vector(value, "rotation", 4);
      var norm = 0.0; for (item in frame.rotation) norm += item * item;
      if (Math.abs(norm - 1.0) > 0.000001) throw "Sensor frame rotation must be a unit quaternion";
      model.addFrame(frame); frames.set(frame.id, frame);
    }
    for (value in requiredArray(data, "sensors")) {
      var id = requiredString(value, "id");
      var sensor = new Sensor(requiredString(value, "name"), requiredString(value, "kind"),
        finite(value, "updateRate"), id);
      var frameId:Dynamic = Reflect.field(value, "frameId");
      if (frameId != null) {
        if (!Std.isOfType(frameId, String) || !frames.exists(cast frameId))
          throw "Sensor references an unknown frame";
        sensor.frame = frames.get(cast frameId);
      }
      sensor.rayCount = requiredInteger(value, "rayCount"); sensor.maxRange = finite(value, "maxRange");
      sensor.noiseStddev = finite(value, "noiseStddev"); sensor.noiseSeed = requiredInteger(value, "noiseSeed");
      model.addSensor(sensor);
      var slash = id.lastIndexOf("/");
      var suffix = Std.parseInt(slash < 0 ? id : id.substr(slash + 1));
      if (suffix != null && suffix >= nextId) nextId = suffix + 1;
    }
    selectedIndex = model.sensors.length == 0 ? -1 : 0;
    if (diagnostics().length > 0) throw diagnostics()[0].message;
  }

  static function requiredArray(value:Dynamic, name:String):Array<Dynamic> {
    var field = Reflect.field(value, name); if (!Std.isOfType(field, Array)) throw 'Invalid sensor document field $name'; return cast field;
  }
  static function requiredString(value:Dynamic, name:String):String {
    var field = Reflect.field(value, name); if (!Std.isOfType(field, String) || StringTools.trim(field).length == 0) throw 'Invalid sensor document field $name'; return cast field;
  }
  static function finite(value:Dynamic, name:String):Float {
    var field = Reflect.field(value, name); if (!Std.isOfType(field, Float) && !Std.isOfType(field, Int)) throw 'Invalid sensor document field $name';
    var result:Float = cast field; if (!Math.isFinite(result)) throw 'Non-finite sensor document field $name'; return result;
  }
  static function requiredInteger(value:Dynamic, name:String):Int {
    var field = Reflect.field(value, name); if (!Std.isOfType(field, Int)) throw 'Invalid sensor document field $name'; return cast field;
  }
  static function vector(value:Dynamic, name:String, count:Int):Array<Float> {
    var items = requiredArray(value, name); if (items.length != count) throw 'Invalid sensor document vector $name';
    return [for (index in 0...count) finite({value:items[index]}, "value")];
  }

  public function isRunning():Bool return running;

  /** Atomically replaces the runtime only after validation and native construction succeed. */
  public function apply():Bool {
    var issues = diagnostics();
    if (issues.length > 0) { applyError = issues[0].code + ": " + issues[0].message; return false; }
    var candidate:Null<Simulation> = null;
    try {
      var blueprint = RobotRuntimeCompiler.compile(model, appliedRevision + 1);
      candidate = new Simulation();
      var candidateRuntime = candidate.addRobot(blueprint);
      if (running) candidate.start();
      var previous = simulation;
      simulation = candidate;
      runtime = candidateRuntime;
      appliedRevision++;
      applyError = null;
      if (previous != null) previous.dispose();
      return true;
    } catch (error:Dynamic) {
      applyError = Std.string(error);
      if (candidate != null) candidate.dispose();
      return false;
    }
  }

  public function start():Bool {
    if (simulation == null && !apply()) return false;
    if (!running) simulation.start();
    running = true;
    return true;
  }
  public function stop():Void { if (simulation != null) simulation.stop(); running = false; }
  public function reset():Bool {
    if (simulation == null) return false;
    simulation.reset();
    running = false;
    return true;
  }
  public function step(?timestampNs:Int64):Null<RobotSnapshot> {
    if (simulation == null && !apply()) return null;
    if (running) throw "Stop realtime simulation before deterministic stepping";
    simulation.step(timestampNs == null ? Int64.ofInt(0) : timestampNs);
    return runtime.snapshot();
  }
  public function snapshot():Null<RobotSnapshot> return runtime == null ? null : runtime.snapshot();
  public function dispose():Void { if (simulation != null) simulation.dispose(); simulation = null; runtime = null; }
  public function context():CommandContext {
    var sensor = selected();
    return new CommandContext(document, sensor == null ? [] : [sensor.id],
      "sensor-panel", null, "sensor-editor");
  }
  public function diagnostics():Array<robotkit.runtime.RobotCompileDiagnostic>
    return RobotRuntimeCompiler.validate(model);

  public function properties():Array<PropertyDescriptor> {
    var sensor=selected();if(sensor==null)return [];
    var result:Array<PropertyDescriptor> = [];
    result.push(text(sensor,"name","Name",function()return sensor.name,function(value)sensor.name=value));
    result.push(readonlyText(sensor,"kind","Kind",sensor.kind));
    result.push(number(sensor,"rate","Update rate",function()return sensor.updateRate,
      function(value)sensor.updateRate=value,0.0,10000.0,"Hz",0.1));
    if(sensor.kind=="lidar") {
      result.push(integer(sensor,"rays","Ray count",function()return sensor.rayCount,
        function(value)sensor.rayCount=value,1,64));
      result.push(number(sensor,"range","Maximum range",function()return sensor.maxRange,
        function(value)sensor.maxRange=value,0.000001,1000000.0,"m",0.1));
    }
    result.push(number(sensor,"noise","Noise σ",function()return sensor.noiseStddev,
      function(value)sensor.noiseStddev=value,0.0,1000000.0,null,0.001));
    result.push(integer(sensor,"seed","Noise seed",function()return sensor.noiseSeed,
      function(value)sensor.noiseSeed=value,0,2147483647));
    result.push(choice(sensor, "mount-mode", "Mount ownership", function() return mountMode(sensor),
      function(value) setMountMode(sensor, value), [
        new PropertyOption("shared", "Shared frame"),
        new PropertyOption("independent", "Independent frame")
      ], "Mount"));
    result.push(choice(sensor, "link", "Mounted link", function() return sensorLink(sensor).id,
      function(value) setSensorLink(sensor, value),
      [for (link in model.links) new PropertyOption(link.id, link.name)], "Mount"));
    var frame=sensor.frame;
    if(frame!=null) {
      for(axis in 0...3) result.push(number(sensor,"position-"+axis,"Mount "+["X","Y","Z"][axis],
        function() return frame.position[axis], function(value) { frame.position[axis] = value; },
        -1000000.0,1000000.0,"m",0.01));
      for(axis in 0...4) result.push(number(sensor,"rotation-"+axis,"Rotation "+["X","Y","Z","W"][axis],
        function() return frame.rotation[axis], function(value) { frame.rotation[axis] = value; },
        -1.0,1.0,null,0.01));
    }
    return result;
  }

  function sensorLink(sensor:Sensor):Link return sensor.frame == null ? model.links[0] : sensor.frame.link;
  function mountMode(sensor:Sensor):String {
    var frame = sensor.frame;
    if (frame == null) return "independent";
    var users = 0; for (candidate in model.sensors) if (candidate.frame == frame) users++;
    return users > 1 || frame == model.frames[0] ? "shared" : "independent";
  }
  function setMountMode(sensor:Sensor, mode:String):Void {
    if (mode == "shared") {
      var link = sensorLink(sensor);
      for (frame in model.frames) if (frame.link == link && frame.id.indexOf("shared") >= 0) {
        sensor.frame = frame; return;
      }
      if (model.frames[0].link == link) { sensor.frame = model.frames[0]; return; }
      var shared = new Frame(link.name + " shared sensor mount", link, link.id + "/sensors/shared");
      model.addFrame(shared); sensor.frame = shared;
    } else if (mode == "independent") {
      var source = sensor.frame;
      var frameId = sensor.id + "/mount";
      for (candidate in model.frames) if (candidate.id == frameId) { sensor.frame = candidate; return; }
      var frame = new Frame(sensor.name + " mount", sensorLink(sensor), frameId);
      if (source != null) { frame.position = source.position.copy(); frame.rotation = source.rotation.copy(); }
      model.addFrame(frame); sensor.frame = frame;
    } else throw "Unknown sensor mount ownership";
  }
  function setSensorLink(sensor:Sensor, linkId:String):Void {
    var link:Null<Link> = null; for (candidate in model.links) if (candidate.id == linkId) link = candidate;
    if (link == null) throw "Unknown sensor link";
    var source = sensor.frame;
    var frameId = sensor.id + "/mount/" + link.id;
    for (candidate in model.frames) if (candidate.id == frameId) { sensor.frame = candidate; return; }
    var frame = new Frame(sensor.name + " mount", link, frameId);
    if (source != null) { frame.position = source.position.copy(); frame.rotation = source.rotation.copy(); }
    model.addFrame(frame); sensor.frame = frame;
  }

  static function settings(category:String,min:Null<Float>,max:Null<Float>,unit:Null<String>,step:Float):PropertyDescriptorOptions {
    var value=new PropertyDescriptorOptions();value.category=category;value.minimum=min;value.maximum=max;
    value.unit=unit;value.step=step;return value;
  }
  static function number(sensor:Sensor,id:String,label:String,read:Void->Float,write:Float->Void,
      min:Float,max:Float,unit:Null<String>,step:Float):PropertyDescriptor
    return new PropertyDescriptor(sensor.id+":"+id,label,PropertyType.Float,
      function(_)return PropertyValue.Float(read()),function(_,value)switch value {
        case Float(next):write(next);case Int(next):write(next);default:throw "Numeric sensor value required";
      },settings(id.indexOf("position") >= 0 || id.indexOf("rotation") >= 0 ? "Mount" : "Acquisition",min,max,unit,step));
  static function integer(sensor:Sensor,id:String,label:String,read:Void->Int,write:Int->Void,min:Int,max:Int):PropertyDescriptor
    return new PropertyDescriptor(sensor.id+":"+id,label,PropertyType.Int,
      function(_)return PropertyValue.Int(read()),function(_,value)switch value {case Int(next):write(next);default:throw "Integer sensor value required";},settings("Acquisition",min,max,null,1));
  static function choice(sensor:Sensor,id:String,label:String,read:Void->String,write:String->Void,
      choices:Array<PropertyOption>,category:String):PropertyDescriptor {
    var options=settings(category,null,null,null,1); options.options=choices;
    return new PropertyDescriptor(sensor.id+":"+id,label,PropertyType.Enum,
      function(_)return PropertyValue.Enum(read()),function(_,value)switch value {case Enum(next):write(next);default:throw "Sensor option required";},options);
  }
  static function text(sensor:Sensor,id:String,label:String,read:Void->String,write:String->Void):PropertyDescriptor {
    var options=settings("Identity",null,null,null,1);options.validator=function(_,value)return switch value {
      case Text(next):StringTools.trim(next).length==0?"Name cannot be empty":null;default:"Text required";};
    return new PropertyDescriptor(sensor.id+":"+id,label,PropertyType.Text,function(_)return PropertyValue.Text(read()),
      function(_,value)switch value {case Text(next):write(next);default:throw "Text required";},options);
  }
  static function readonlyText(sensor:Sensor,id:String,label:String,value:String):PropertyDescriptor {
    var options=settings("Identity",null,null,null,1);options.readOnly=true;
    return new PropertyDescriptor(sensor.id+":"+id,label,PropertyType.Text,function(_)return PropertyValue.Text(value),function(_,_){},options);
  }
}
