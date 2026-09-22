package app;

import nativekit.ui.core.EditOperation;
import nativekit.ui.core.EditorDocument;
import robotkit.runtime.RobotRuntimeCompiler;
import haxe.Json;

typedef ScriptMaterialization={var scene:EditorScene;var sensors:SensorConfiguration;
  var backend:Int;var timestep:Float;}

/** Owns a script baseline plus stable-ID document overrides. Scripts are unsandboxed compiled Haxe. */
class ScriptOwnership {
  public final document:EditorDocument=new EditorDocument("script-overrides");
  public var reference(default,null):String;
  public var configurationVersion(default,null):Int;
  public var savedVersion(default,null):Int;
  public var overridesEnabled(default,null):Bool;
	public var diagnostics(default,null):Array<String> = [];
  var baseline:ScriptedSetup;
	final overrides:Map<String,Float> = new Map();

  public function new(reference:String,?record:ScriptOwnershipRecord){
    this.reference=reference;overridesEnabled=record!=null&&record.overridesEnabled;
    savedVersion=record==null?0:record.version;
    if(record!=null)for(item in record.overrides)overrides.set(key(item.targetId,item.property),item.value);
    baseline=new ScriptedSetup();var provider=SetupScriptRegistry.provider(reference);
    try provider().evaluate(baseline) catch(error:Dynamic){baseline.dispose();throw error;}
    try validateSetup(baseline,reference) catch(error:Dynamic){baseline.dispose();throw error;}
    configurationVersion=baseline.version;if(record!=null)refreshDiagnostics();document.markSaved();
  }
  public function reload():ScriptMaterialization {
    var candidate=new ScriptedSetup(),previous=baseline;
    var provider=SetupScriptRegistry.provider(reference);
    try provider().evaluate(candidate) catch(error:Dynamic){candidate.dispose();throw error;}
    try validateSetup(candidate,reference) catch(error:Dynamic){candidate.dispose();throw error;}
    baseline=candidate;configurationVersion=candidate.version;refreshDiagnostics();
    try {var result=materialize();previous.dispose();return result;}
    catch(error:Dynamic){baseline=previous;configurationVersion=previous.version;
      candidate.dispose();refreshDiagnostics();throw error;}
  }
  public function materialize():ScriptMaterialization {
    var source:SensorConfiguration=baseline.sensors;
    var record:Dynamic=Json.parse(Json.stringify(source.records()));
    var sensors=new SensorConfiguration(record);
    try {
      applyOverrides(sensors);validateSensors(sensors);
      var objects:Array<SceneObjectData> = Json.parse(Json.stringify(baseline.objects));
      var scene=new EditorScene(objects);
      return {scene:scene,sensors:sensors,backend:baseline.backend,timestep:baseline.timestep};
    } catch(error:Dynamic){sensors.dispose();throw error;}
  }
  public function setOverridesEnabled(value:Bool):Bool {
    if(value==overridesEnabled)return false;var before=overridesEnabled;
    document.apply(new EditOperation(value?"Enable script overrides":"Disable script overrides",
      function()overridesEnabled=value,function()overridesEnabled=before));return true;
  }
  public function setSensorRate(robotId:String,sensorId:String,value:Float):Bool {
    if(!overridesEnabled)throw "Enable overrides before changing script-owned values";
    if(!Math.isFinite(value)||value<0||value>10000)throw "Invalid sensor update rate";
    var target=robotId+"/"+sensorId,k=key(target,"updateRate"),had=overrides.exists(k),before=overrides.get(k);
    if(had&&before==value)return false;
    document.apply(new EditOperation("Override sensor update rate",function()overrides.set(k,value),function(){
      if(had)overrides.set(k,cast before);else overrides.remove(k);
    }));refreshDiagnostics();return true;
  }
  public function revertSensorRate(robotId:String,sensorId:String):Bool {
    var k=key(robotId+"/"+sensorId,"updateRate");if(!overrides.exists(k))return false;var before=overrides.get(k);
    document.apply(new EditOperation("Revert sensor update rate",function()overrides.remove(k),
      function()overrides.set(k,cast before)));refreshDiagnostics();return true;
  }
  public function sensorRateOrigin(robotId:String,sensorId:String):String
    return overridesEnabled&&overrides.exists(key(robotId+"/"+sensorId,"updateRate"))?"override":"script";
  public function backend():Int return baseline.backend;
  public function timestep():Float return baseline.timestep;
  public function markSaved():Void {savedVersion=configurationVersion;document.markSaved();refreshDiagnostics();}
  public function record():ScriptOwnershipRecord {
    var values:Array<ScriptOverrideRecord> = [];
    for(k in overrides.keys()){var split=k.lastIndexOf("|");values.push({targetId:k.substr(0,split),property:k.substr(split+1),value:overrides.get(k)});}
    values.sort(function(a,b)return Reflect.compare(a.targetId+"/"+a.property,b.targetId+"/"+b.property));
    return {reference:reference,version:configurationVersion,overridesEnabled:overridesEnabled,overrides:values};
  }
  function applyOverrides(sensors:SensorConfiguration):Void {
    var found=new Map<String,Bool>();
    for(robot in sensors.robotModels())for(sensor in robot.model.sensors){
      var target=robot.id+"/"+sensor.id,k=key(target,"updateRate");
      if(overrides.exists(k)){if(overridesEnabled)sensor.updateRate=overrides.get(k);found.set(k,true);}
    }
    diagnostics=[];
    if(savedVersion>0&&savedVersion!=configurationVersion)
      diagnostics.push('Script version changed from $savedVersion to $configurationVersion');
    for(k in overrides.keys())if(!found.exists(k))
      diagnostics.push("Stale override target: "+k.substr(0,k.lastIndexOf("|")));
  }
  function refreshDiagnostics():Void {
    var sensors:Null<SensorConfiguration> = null;
    try {var source:SensorConfiguration=baseline.sensors;
      var records:Dynamic=Json.parse(Json.stringify(source.records()));sensors=new SensorConfiguration(records);
      applyOverrides(sensors);validateSensors(sensors);}
    catch(error:Dynamic){if(diagnostics.length==0)diagnostics=[Std.string(error)];}
    if(sensors!=null)sensors.dispose();
  }
  static function validateSensors(sensors:SensorConfiguration):Void
    for(robot in sensors.robotModels()){var issues=RobotRuntimeCompiler.validate(robot.model);if(issues.length>0)throw issues[0].code+": "+issues[0].message;}
  static function validateSetup(value:ScriptedSetup,expectedReference:String):Void {
    if(value.reference!=expectedReference||StringTools.trim(value.reference).length==0||value.version<1||
        value.sensors==null||value.objects==null||
        !Math.isFinite(value.timestep)||value.timestep<=0)throw "Invalid setup script result";
    if(value.backend!=ApplicationSimulation.DETERMINISTIC&&value.backend!=ApplicationSimulation.MUJOCO)
      throw "Unsupported setup physics backend";
    var sensors:SensorConfiguration=value.sensors;validateSensors(sensors);
  }
  static inline function key(target:String,property:String):String return target+"|"+property;
  public function dispose():Void baseline.dispose();
}
