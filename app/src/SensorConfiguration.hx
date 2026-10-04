package app;

import nativekit.ui.widgets.text.Text;


import nativekit.ui.core.CommandContext;
import nativekit.ui.editing.EditorDocument;
import nativekit.ui.editing.EditOperation;
import nativekit.ui.properties.PropertyDescriptor;
import nativekit.ui.properties.PropertyDescriptorOptions;
import nativekit.ui.properties.PropertyOption;
import nativekit.ui.properties.PropertyType;
import nativekit.ui.properties.PropertyValue;
import robotkit.model.Frame;
import robotkit.model.CollisionApproximation;
import robotkit.model.Joint;
import robotkit.model.JointLimits;
import robotkit.model.Link;
import robotkit.model.RobotModel;
import robotkit.model.RobotModelCodec;
import robotkit.model.Sensor;
import robotkit.profile.RobotDriveConfiguration;
import robotkit.profile.RobotMobileConfiguration;
import robotkit.profile.RobotForkConfiguration;
import robotkit.runtime.RobotRuntimeCompiler;

/** Editable RobotKit sensor model used by Materia's sensor panel. */
class SensorConfiguration {
  var active:robotkit.model.RobotDefinition;
  public var model(get, never):RobotModel;
  public var profile(get, never):robotkit.profile.RobotProfile;
  function get_model():RobotModel return active.model;
  function get_profile():robotkit.profile.RobotProfile return active.profile;
  public final document:EditorDocument;
  public var selectedIndex(default,null):Int = 0;
  public var robotId(default, null):String = "materia/robot";
  var nextId:Int = 1;
  var empty:Bool = false;
  var configurationRevision:Int = 0;
  var physicsRevision:Int = 1;
  var observedDocumentRevision:Int = -1;
  var observedConfigurationRevision:Int = -1;
  var observedPhysicsState:Null<String> = null;
  final configurations:Map<String, Dynamic> = new Map();
  final liveModels:Map<String, robotkit.model.RobotDefinition> = new Map();
  final selections:Map<String, Int> = new Map();
  final nextIds:Map<String, Int> = new Map();
  final positions:Map<String, Array<Float>> = new Map();
  final rotations:Map<String, Array<Float>> = new Map();
  final readOnlyRobots:Map<String, Bool> = new Map();

  public function new(?data:Dynamic, ?sharedDocument:EditorDocument, empty:Bool = false) {
    document = sharedDocument == null ? new EditorDocument("sensors") : sharedDocument;
    active=new robotkit.model.RobotDefinition(new RobotModel("Materia robot"), new robotkit.profile.RobotProfile());
    var base=model.addLink(new Link("Base","base"));
    model.addFrame(new Frame("Base sensor mount",base,"base/sensors"));
    if (data == null && empty) {
      this.empty = true;
      selectedIndex = -1;
    } else if (data == null) {
      addDirect("lidar");
    } else if (Reflect.hasField(data, "robots")) {
      var loaded = requiredArray(data, "robots");
      if (loaded.length == 0) { this.empty = true; selectedIndex = -1; }
      for (record in loaded) {
        var id = requiredString(record, "robotId");
        if (configurations.exists(id)) throw "Duplicate robot sensor configuration";
        configurations.set(id, record);
      }
      if (!this.empty) {
        var selected = requiredString(data, "selectedRobotId");
        var record = configurations.get(selected);
        if (record == null) throw "Selected robot has no sensor configuration";
        loadRobot(record);
      }
    } else {
      loadRobot(data);
    }
    if (!this.empty) {
      configurations.set(robotId, singleRecord());
      liveModels.set(robotId, active); selections.set(robotId, selectedIndex);
    }
    if (sharedDocument == null) document.markSaved();
  }

  public function selected():Null<Sensor>
    return selectedIndex < 0 || selectedIndex >= model.sensors.length ? null : model.sensors[selectedIndex];
  public function select(index:Int):Bool {
    if(index<0||index>=model.sensors.length||index==selectedIndex)return false;
    selectedIndex=index;return true;
  }
  public function selectRobot(id:String):Bool {
    if (id == null || StringTools.trim(id).length == 0 || (id == robotId && !empty)) return false;
    if (readOnlyRobots.exists(id) && !configurations.exists(id) && !liveModels.exists(id)) return false;
    if (empty) {
      var previousId = robotId;
      var previousDefinition = active, previousSelection = selectedIndex;
      document.apply(new EditOperation("Add robot configuration", function() {
        empty = false; createDefault(id); configurationRevision++;
      }, function() {
        configurations.remove(id); liveModels.remove(id); selections.remove(id);
        nextIds.remove(id); positions.remove(id); rotations.remove(id);
        robotId = previousId; active = previousDefinition; selectedIndex = previousSelection;
        empty = true; configurationRevision++;
      }));
      return true;
    }
    captureCurrent();
    var nextRecord = configurations.get(id);
    if (nextRecord == null) {
      var previousId = robotId;
      document.apply(new EditOperation("Add robot configuration", function() {
        var activeId = robotId;
        createDefault(id);
        if (activeId != previousId && activeId != id)
          activate(activeId, configurations.get(activeId));
        configurationRevision++;
      }, function() {
        var activeId = robotId;
        configurations.remove(id); liveModels.remove(id); selections.remove(id);
        nextIds.remove(id); positions.remove(id); rotations.remove(id);
        if (activeId == id)
          activate(previousId, configurations.get(previousId));
        configurationRevision++;
      }));
      return true;
    }
    activate(id,nextRecord);
    return true;
  }
  public function configuredRobotIds():Array<String> {
    if (empty) return [];
    captureCurrent(); var result=[for(id in configurations.keys()) id];
    for(id in liveModels.keys())if(result.indexOf(id)<0)result.push(id);
    result.sort(Reflect.compare); return result;
  }
  /** Changes only when simulation inputs change, even with a shared editor document. */
  public function revision():Int {
    if (observedDocumentRevision != document.revision ||
        observedConfigurationRevision != configurationRevision) {
      var current = physicsState();
      if (observedPhysicsState != null && current != observedPhysicsState) physicsRevision++;
      observedPhysicsState = current;
      observedDocumentRevision = document.revision;
      observedConfigurationRevision = configurationRevision;
    }
    return physicsRevision;
  }

  function physicsState():String {
    var ids:Array<String> = [for (id in configurations.keys()) id];
    for (id in liveModels.keys()) if (ids.indexOf(id) < 0) ids.push(id);
    ids.sort(Reflect.compare);
    var records:Array<Dynamic> = [];
    for (id in ids) {
      var live = liveModels.get(id);
      var record = live == null ? configurations.get(id) : singleRecordFor(id, live);
      records.push(withoutNames(haxe.Json.parse(haxe.Json.stringify(record))));
    }
    return haxe.Json.stringify({robots:records});
  }

  static function withoutNames(value:Dynamic):Dynamic {
    if (value == null) return null;
    if (Std.isOfType(value, Array)) {
      var values:Array<Dynamic> = cast value;
      return [for (item in values) withoutNames(item)];
    }
    if (Std.isOfType(value, String) || Std.isOfType(value, Int) ||
        Std.isOfType(value, Float) || Std.isOfType(value, Bool)) return value;
    var result:Dynamic = {};
    for (field in Reflect.fields(value)) if (field != "name")
      Reflect.setField(result, field, withoutNames(Reflect.field(value, field)));
    return result;
  }
  public function setReadOnlyRobots(ids:Array<String>):Void {
    readOnlyRobots.clear(); for (id in ids) readOnlyRobots.set(id, true);
  }
  public function removeRobotConfiguration(id:String):Bool {
    captureCurrent();
    if(!configurations.exists(id)||configuredRobotIds().length<=1)return false;
    var record=configurations.get(id),live=liveModels.get(id),selection=selections.get(id),savedNextId=nextIds.get(id);
    var position=robotPosition(id),rotation=robotRotation(id);
    var fallback=[for(candidate in configuredRobotIds())if(candidate!=id)candidate][0];
    document.apply(new EditOperation("Remove robot configuration",function(){
      configurations.remove(id);liveModels.remove(id);selections.remove(id);nextIds.remove(id);positions.remove(id);rotations.remove(id);
      if(robotId==id)activate(fallback,configurations.get(fallback));
    },function(){
      configurations.set(id,record);if(live!=null)liveModels.set(id,live);if(selection!=null)selections.set(id,selection);
      if(savedNextId!=null)nextIds.set(id,savedNextId);
      positions.set(id,position.copy());rotations.set(id,rotation.copy());
    }));
    return true;
  }
  public function isEditable():Bool return !readOnlyRobots.exists(robotId);
  public function add(kind:String):Sensor {
    ensureEditable();
    var owner = model;
    var ownerDefinition = active;
    var ownerId = robotId;
    var sensor = createSensor(kind);
    var previous = selectedIndex;
    document.apply(new EditOperation("Add " + kind + " sensor", function() {
      if (empty) {
        empty = false;
        configurations.set(ownerId, singleRecord()); liveModels.set(ownerId, ownerDefinition);
      }
      if (owner.sensors.indexOf(sensor) < 0) owner.addSensor(sensor);
      if (robotId == ownerId) selectedIndex = owner.sensors.indexOf(sensor);
    }, function() {
      owner.sensors.remove(sensor);
      if (owner.sensors.length == 0 && configurations.get(ownerId) != null &&
          configuredRobotIds().length == 1) {
        configurations.remove(ownerId); liveModels.remove(ownerId); empty = true;
      }
      if (robotId == ownerId) selectedIndex = previous;
    }));
    return sensor;
  }
  public function removeSelected():Bool {
    ensureEditable();
    if(model.sensors.length==0)return false;
    var owner = model;
    var ownerId = robotId;
    var index = selectedIndex;
    var sensor = owner.sensors[index];
    document.apply(new EditOperation("Remove sensor", function() {
      owner.sensors.remove(sensor);
      if (robotId == ownerId)
        selectedIndex=owner.sensors.length==0?-1:Std.int(Math.min(index,owner.sensors.length-1));
    }, function() {
      owner.sensors.insert(index, sensor);
      if (robotId == ownerId) selectedIndex = index;
    }));
    return true;
  }

  function createSensor(kind:String):Sensor {
    if(kind!="lidar"&&kind!="imu"&&kind!="joint_encoder"&&kind!="camera")throw "Unsupported sensor kind";
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
    var result:Dynamic = {selectedRobotId:robotId, robots:values};
    return result;
  }
  public function robotRecords():Array<Dynamic> {
    captureCurrent();
    for(id in liveModels.keys())configurations.set(id,singleRecordFor(id,liveModels.get(id)));
    return [for(id in configuredRobotIds()) configurations.get(id)];
  }
  public function robotModels():Array<{id:String,model:RobotModel,profile:robotkit.profile.RobotProfile,position:Array<Float>,rotation:Array<Float>}> {
    if (empty) return [];
    captureCurrent(); var selected=robotId;
    var result:Array<{id:String,model:RobotModel,profile:robotkit.profile.RobotProfile,position:Array<Float>,rotation:Array<Float>}> = [];
    for(id in configuredRobotIds()) {
      if(!liveModels.exists(id))loadRobot(configurations.get(id));
      var live = liveModels.get(id);
      if (live == null) throw 'Robot "$id" has no editable model';
      result.push({id:id,model:live.model,profile:live.profile,position:robotPosition(id),rotation:robotRotation(id)});
    }
    activate(selected,configurations.get(selected));
    return result;
  }
  function singleRecord():Dynamic return singleRecordFor(robotId, active);
  function singleRecordFor(id:String, value:robotkit.model.RobotDefinition):Dynamic return {
    schemaVersion:1,
    robotId:id,
    model:haxe.Json.parse(RobotModelCodec.encode(value.model).toString()),
    profile:robotkit.profile.RobotProfileCodec.toRecord(value.profile),
    pose:{position:robotPosition(id), rotation:robotRotation(id)}
  };

  function captureCurrent():Void {
    if (empty) return;
    configurations.set(robotId, singleRecord()); liveModels.set(robotId,active);
    selections.set(robotId,selectedIndex);nextIds.set(robotId,nextId);
  }
  function activate(id:String,record:Dynamic):Void {
    var live=liveModels.get(id);
    if(live==null)loadRobot(record); else {
      robotId=id;active=live;var selected=selections.get(id);selectedIndex=selected==null?0:selected;
      var savedNextId=nextIds.get(id);nextId=savedNextId==null?1:savedNextId;
    }
  }
  function createDefault(id:String):Void {
    robotId=id; active=new robotkit.model.RobotDefinition(new RobotModel("Materia robot"), new robotkit.profile.RobotProfile());
    var base=model.addLink(new Link("Base","base"));
    model.addFrame(new Frame("Base sensor mount",base,"base/sensors"));
    nextId=1; addDirect("lidar"); captureCurrent();
  }

  public function robotPosition(id:String):Array<Float> {
    var value=positions.get(id);return value==null?[0.0,0.0,0.0]:value.copy();
  }
  public function robotRotation(id:String):Array<Float> {
    var value=rotations.get(id);return value==null?[0.0,0.0,0.0,1.0]:value.copy();
  }
  public function setRobotPose(id:String,position:Array<Float>,rotation:Array<Float>):Bool {
    ensureEditableRobot(id);
    var nextPosition=checkedVector(position,"robot pose position",3);
    var nextRotation=checkedVector(rotation,"robot pose rotation",4);
    var norm=0.0;for(value in nextRotation)norm+=value*value;
    if(Math.abs(norm-1.0)>0.000001)throw "Robot pose rotation must be a unit quaternion";
    var beforePosition=robotPosition(id),beforeRotation=robotRotation(id);
    if(equalVector(beforePosition,nextPosition)&&equalVector(beforeRotation,nextRotation))return false;
    return document.apply(new EditOperation("Move robot "+id,function(){
      positions.set(id,nextPosition.copy());rotations.set(id,nextRotation.copy());
    },function(){positions.set(id,beforePosition.copy());rotations.set(id,beforeRotation.copy());}));
  }

  function loadRobot(data:Dynamic):Void {
    robotId = requiredString(data, "robotId");
    if (Reflect.field(data, "schemaVersion") != 1)
      throw "Unsupported robot authoring schema version";
    active = new robotkit.model.RobotDefinition(
      RobotModelCodec.decode(haxe.io.Bytes.ofString(haxe.Json.stringify(Reflect.field(data, "model")))),
      robotkit.profile.RobotProfileCodec.fromRecord(Reflect.field(data, "profile")));
    var pose:Dynamic = Reflect.field(data, "pose");
    var loadedPosition = vector(pose, "position", 3);
    var loadedRotation = vector(pose, "rotation", 4);
    var poseNorm = 0.0;
    for (value in loadedRotation) poseNorm += value * value;
    if (Math.abs(poseNorm - 1.0) > 0.000001) throw "Robot pose rotation must be a unit quaternion";
    positions.set(robotId, loadedPosition); rotations.set(robotId, loadedRotation);
    selectedIndex = model.sensors.length == 0 ? -1 : 0;
    nextId = 1;
    for (sensor in model.sensors) {
      var slash = sensor.id.lastIndexOf("/"); var suffix=Std.parseInt(slash<0?sensor.id:sensor.id.substr(slash+1));
      if (suffix != null && suffix >= nextId) nextId=suffix+1;
    }
    if (diagnostics().length > 0) throw diagnostics()[0].message;
    liveModels.set(robotId,active); selections.set(robotId,selectedIndex);nextIds.set(robotId,nextId);
  }

  static function requiredArray(value:Dynamic, name:String):Array<Dynamic> {
    var field = Reflect.field(value, name); if (!Std.isOfType(field, Array)) throw 'Invalid sensor document field $name'; return cast field;
  }
  static function requiredString(value:Dynamic, name:String):String {
    var field = Reflect.field(value, name); if (!Std.isOfType(field, String) || StringTools.trim(field).length == 0) throw 'Invalid sensor document field $name'; return cast field;
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
  static function checkedVector(value:Array<Float>,name:String,count:Int):Array<Float> {
    if(value==null||value.length!=count)throw 'Invalid $name';
    var result:Array<Float> = [];
    for(item in value){if(!Math.isFinite(item))throw 'Invalid $name';result.push(item);}
    return result;
  }
  static function equalVector(left:Array<Float>,right:Array<Float>):Bool {
    for(index in 0...left.length)if(left[index]!=right[index])return false;return true;
  }

  public function dispose():Void {}
  public function context():CommandContext {
    var sensor = selected();
    return new CommandContext(document, sensor == null ? [] : [sensor.id],
      "sensor-panel", null, "sensor-editor");
  }
  public function diagnostics():Array<robotkit.runtime.RobotCompileDiagnostic>
    return RobotRuntimeCompiler.validate(model, profile);

  public function properties():Array<PropertyDescriptor> {
    if (!isEditable()) return [];
    var sensor=selected();if(sensor==null)return [];
    var owner=model;
    var result:Array<PropertyDescriptor> = [];
    result.push(text(sensor,"name","Name",function()return sensor.name,function(value)sensor.name=value));
    result.push(readonlyText(sensor,"kind","Kind",sensor.kind));
    result.push(number(sensor,"rate","Update rate",function()return sensor.updateRate,
      function(value)sensor.updateRate=value,0.0,10000.0,"Hz",0.1));
    if(sensor.kind=="lidar") {
      result.push(integer(sensor,"rays","Ray count",function()return sensor.rayCount,
        function(value)sensor.rayCount=value,1,360));
      result.push(number(sensor,"range","Maximum range",function()return sensor.maxRange,
        function(value)sensor.maxRange=value,0.000001,1000000.0,"m",0.1));
      result.push(number(sensor,"start-angle","Start angle",function()return sensor.startAngleRadians,
        function(value)sensor.startAngleRadians=value,-Math.PI * 2.0,Math.PI * 2.0,"rad",0.01));
      result.push(number(sensor,"field-of-view","Field of view",function()return sensor.fieldOfViewRadians,
        function(value)sensor.fieldOfViewRadians=value,0.000001,Math.PI * 2.0,"rad",0.01));
    }
    result.push(number(sensor,"noise","Noise σ",function()return sensor.noiseStddev,
      function(value)sensor.noiseStddev=value,0.0,1000000.0,null,0.001));
    result.push(integer(sensor,"seed","Noise seed",function()return sensor.noiseSeed,
      function(value)sensor.noiseSeed=value,0,2147483647));
    result.push(choice(sensor, "mount-mode", "Mount ownership", function() return mountMode(owner,sensor),
      function(value) setMountMode(owner,sensor, value), [
        new PropertyOption("shared", "Shared frame"),
        new PropertyOption("independent", "Independent frame")
      ], "Mount"));
    result.push(choice(sensor, "link", "Mounted link", function() return sensorLink(owner,sensor).id,
      function(value) setSensorLink(owner,sensor, value),
      [for (link in owner.links) new PropertyOption(link.id, link.name)], "Mount"));
    var frame=sensor.frame;
    if(frame!=null) {
      for(axis in 0...3) result.push(number(sensor,"position-"+["x","y","z"][axis],"Mount "+["X","Y","Z"][axis],
        function() return frame.position[axis], function(value) { frame.position[axis] = value; },
        -1000000.0,1000000.0,"m",0.01));
      for(axis in 0...4) result.push(number(sensor,"rotation-"+axis,"Rotation "+["X","Y","Z","W"][axis],
        function() return frame.rotation[axis], function(value) { frame.rotation[axis] = value; },
        -1.0,1.0,null,0.01));
    }
    return result;
  }

  function sensorLink(owner:RobotModel,sensor:Sensor):Link return sensor.frame == null ? owner.links[0] : sensor.frame.link;
  function mountMode(owner:RobotModel,sensor:Sensor):String {
    var frame = sensor.frame;
    if (frame == null) return "independent";
    var users = 0; for (candidate in owner.sensors) if (candidate.frame == frame) users++;
    return users > 1 || frame == owner.frames[0] ? "shared" : "independent";
  }
  function setMountMode(owner:RobotModel,sensor:Sensor, mode:String):Void {
    if (mode == "shared") {
      var link = sensorLink(owner,sensor);
      for (frame in owner.frames) if (frame.link == link && frame.id.indexOf("shared") >= 0) {
        sensor.frame = frame; return;
      }
      if (owner.frames[0].link == link) { sensor.frame = owner.frames[0]; return; }
      var shared = new Frame(link.name + " shared sensor mount", link, link.id + "/sensors/shared");
      owner.addFrame(shared); sensor.frame = shared;
    } else if (mode == "independent") {
      var source = sensor.frame;
      var frameId = sensor.id + "/mount";
      for (candidate in owner.frames) if (candidate.id == frameId) { sensor.frame = candidate; return; }
      var frame = new Frame(sensor.name + " mount", sensorLink(owner,sensor), frameId);
      if (source != null) { frame.position = source.position.copy(); frame.rotation = source.rotation.copy(); }
      owner.addFrame(frame); sensor.frame = frame;
    } else throw "Unknown sensor mount ownership";
  }
  function setSensorLink(owner:RobotModel,sensor:Sensor, linkId:String):Void {
    var link:Null<Link> = null; for (candidate in owner.links) if (candidate.id == linkId) link = candidate;
    if (link == null) throw "Unknown sensor link";
    var source = sensor.frame;
    var frameId = sensor.id + "/mount/" + link.id;
    for (candidate in owner.frames) if (candidate.id == frameId) { sensor.frame = candidate; return; }
    var frame = new Frame(sensor.name + " mount", link, frameId);
    if (source != null) { frame.position = source.position.copy(); frame.rotation = source.rotation.copy(); }
    owner.addFrame(frame); sensor.frame = frame;
  }

  static function settings(category:String,min:Null<Float>,max:Null<Float>,unit:Null<String>,step:Float):PropertyDescriptorOptions {
    var value=new PropertyDescriptorOptions();value.category=category;value.minimum=min;value.maximum=max;
    value.unit=unit;value.step=step;return value;
  }
  function ensureEditable():Void if (!isEditable()) throw 'Remote robot "$robotId" is read-only';
  function ensureEditableRobot(id:String):Void {
    if(readOnlyRobots.exists(id))throw 'Remote robot "$id" is read-only';
    if(id!=robotId&&!configurations.exists(id)&&!liveModels.exists(id))throw 'Unknown robot "$id"';
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
