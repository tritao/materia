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

/** Editable RobotKit sensor model used by Materia's sensor panel. */
class SensorConfiguration {
  public var model(default, null):RobotModel;
  public final document:EditorDocument;
  public var selectedIndex(default,null):Int = 0;
  public var robotId(default, null):String = "materia/robot";
  var nextId:Int = 1;
  var configurationRevision:Int = 0;
  final configurations:Map<String, Dynamic> = new Map();
  final liveModels:Map<String, RobotModel> = new Map();
  final selections:Map<String, Int> = new Map();
  final readOnlyRobots:Map<String, Bool> = new Map();

  public function new(?data:Dynamic) {
    document = new EditorDocument("sensors");
    model=new RobotModel("Materia robot");
    var base=model.addLink(new Link("Base","base"));
    model.addFrame(new Frame("Base sensor mount",base,"base/sensors"));
    if (data == null) {
      addDirect("lidar");
    } else if (Reflect.hasField(data, "robots")) {
      var selected = requiredString(data, "selectedRobotId");
      for (record in requiredArray(data, "robots")) {
        var id = requiredString(record, "robotId");
        if (configurations.exists(id)) throw "Duplicate robot sensor configuration";
        configurations.set(id, record);
      }
      var record = configurations.get(selected);
      if (record == null) throw "Selected robot has no sensor configuration";
      loadRobot(record);
    } else {
      loadRobot(data);
    }
    configurations.set(robotId, singleRecord());
    liveModels.set(robotId, model); selections.set(robotId, selectedIndex);
    document.markSaved();
  }

  public function selected():Null<Sensor>
    return selectedIndex < 0 || selectedIndex >= model.sensors.length ? null : model.sensors[selectedIndex];
  public function select(index:Int):Bool {
    if(index<0||index>=model.sensors.length||index==selectedIndex)return false;
    selectedIndex=index;return true;
  }
  public function selectRobot(id:String):Bool {
    if (id == null || StringTools.trim(id).length == 0 || id == robotId) return false;
    if (readOnlyRobots.exists(id) && !configurations.exists(id) && !liveModels.exists(id)) return false;
    captureCurrent();
    var nextRecord = configurations.get(id);
    if (nextRecord == null) {
      configurationRevision++; document.markExternallyDirty(); createDefault(id); return true;
    }
    activate(id,nextRecord);
    return true;
  }
  public function configuredRobotIds():Array<String> {
    captureCurrent(); var result=[for(id in configurations.keys()) id];
    for(id in liveModels.keys())if(result.indexOf(id)<0)result.push(id);
    result.sort(Reflect.compare); return result;
  }
  public function revision():Int return document.revision + configurationRevision * 1000000;
  public function setReadOnlyRobots(ids:Array<String>):Void {
    readOnlyRobots.clear(); for (id in ids) readOnlyRobots.set(id, true);
  }
  public function removeRobotConfiguration(id:String):Bool {
    captureCurrent();
    if(!configurations.exists(id)||configuredRobotIds().length<=1)return false;
    var record=configurations.get(id),live=liveModels.get(id),selection=selections.get(id);
    var fallback=[for(candidate in configuredRobotIds())if(candidate!=id)candidate][0];
    document.apply(new EditOperation("Remove robot configuration",function(){
      configurations.remove(id);liveModels.remove(id);selections.remove(id);
      if(robotId==id)activate(fallback,configurations.get(fallback));
    },function(){
      configurations.set(id,record);if(live!=null)liveModels.set(id,live);if(selection!=null)selections.set(id,selection);
    }));
    return true;
  }
  public function isEditable():Bool return !readOnlyRobots.exists(robotId);
  public function add(kind:String):Sensor {
    ensureEditable();
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
    ensureEditable();
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

  public function records():Dynamic {
    var values=robotRecords();
    return {selectedRobotId:robotId, robots:values};
  }
  public function robotRecords():Array<Dynamic> {
    captureCurrent();
    for(id in liveModels.keys())configurations.set(id,singleRecordFor(id,liveModels.get(id)));
    return [for(id in configuredRobotIds()) configurations.get(id)];
  }
  public function robotModels():Array<{id:String,model:RobotModel}> {
    captureCurrent(); var selected=robotId; var result:Array<{id:String,model:RobotModel}> = [];
    for(id in configuredRobotIds()) {
      if(!liveModels.exists(id))loadRobot(configurations.get(id));
      var live = liveModels.get(id);
      if (live == null) throw 'Robot "$id" has no editable model';
      result.push({id:id,model:live});
    }
    activate(selected,configurations.get(selected));
    return result;
  }
  function singleRecord():Dynamic return singleRecordFor(robotId,model);
  function singleRecordFor(id:String,value:RobotModel):Dynamic return {
    robotId: id,
    links: [for (link in value.links) {id:link.id, name:link.name}],
    frames: [for (frame in value.frames) {id:frame.id, name:frame.name, linkId:frame.link.id,
      position:frame.position.copy(), rotation:frame.rotation.copy()}],
    sensors: [for (sensor in value.sensors) {id:sensor.id, name:sensor.name, kind:sensor.kind,
      updateRate:sensor.updateRate, frameId:sensor.frame == null ? null : sensor.frame.id,
      rayCount:sensor.rayCount, maxRange:sensor.maxRange, noiseStddev:sensor.noiseStddev,
      noiseSeed:sensor.noiseSeed}]
  };

  function captureCurrent():Void {
    configurations.set(robotId, singleRecord()); liveModels.set(robotId,model);
    selections.set(robotId,selectedIndex);
  }
  function activate(id:String,record:Dynamic):Void {
    var live=liveModels.get(id);
    if(live==null)loadRobot(record); else {robotId=id;model=live;var selected=selections.get(id);selectedIndex=selected==null?0:selected;}
  }
  function createDefault(id:String):Void {
    robotId=id; model=new RobotModel("Materia robot");
    var base=model.addLink(new Link("Base","base"));
    model.addFrame(new Frame("Base sensor mount",base,"base/sensors"));
    nextId=1; addDirect("lidar"); captureCurrent();
  }

  function loadRobot(data:Dynamic):Void {
    robotId = requiredString(data, "robotId");
    model = new RobotModel("Materia robot");
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
    nextId = 1;
    for (sensor in model.sensors) {
      var slash = sensor.id.lastIndexOf("/"); var suffix=Std.parseInt(slash<0?sensor.id:sensor.id.substr(slash+1));
      if (suffix != null && suffix >= nextId) nextId=suffix+1;
    }
    if (diagnostics().length > 0) throw diagnostics()[0].message;
    liveModels.set(robotId,model); selections.set(robotId,selectedIndex);
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
    var result:Array<Float> = [];
    for(item in items) {
      if(item==null||Std.isOfType(item,String)||Std.isOfType(item,Bool))throw 'Invalid sensor document vector $name';
      var number=Std.parseFloat(Std.string(item));
      if(!Math.isFinite(number))throw 'Non-finite sensor document vector $name';
      result.push(number);
    }
    return result;
  }

  public function dispose():Void {}
  public function context():CommandContext {
    var sensor = selected();
    return new CommandContext(document, sensor == null ? [] : [sensor.id],
      "sensor-panel", null, "sensor-editor");
  }
  public function diagnostics():Array<robotkit.runtime.RobotCompileDiagnostic>
    return RobotRuntimeCompiler.validate(model);

  public function properties():Array<PropertyDescriptor> {
    if (!isEditable()) return [];
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
  function ensureEditable():Void if (!isEditable()) throw 'Remote robot "$robotId" is read-only';
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
