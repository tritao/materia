package app;

import nativekit.ui.core.CommandContext;
import nativekit.ui.core.EditorDocument;
import nativekit.ui.core.PropertyDescriptor;
import nativekit.ui.core.PropertyDescriptorOptions;
import nativekit.ui.core.PropertyType;
import nativekit.ui.core.PropertyValue;
import robotkit.model.Frame;
import robotkit.model.Link;
import robotkit.model.RobotModel;
import robotkit.model.Sensor;
import robotkit.runtime.RobotRuntimeCompiler;

/** Editable RobotKit sensor model used by Materia's sensor panel. */
class SensorConfiguration {
  public final model:RobotModel;
  public final document:EditorDocument;
  public var selectedIndex(default,null):Int = 0;
  var nextId:Int = 1;

  public function new() {
    document = new EditorDocument("sensors");
    model=new RobotModel("Materia robot");
    var base=model.addLink(new Link("Base","base"));
    model.addFrame(new Frame("Base sensor mount",base,"base/sensors"));
    add("lidar");
  }

  public function selected():Null<Sensor>
    return selectedIndex < 0 || selectedIndex >= model.sensors.length ? null : model.sensors[selectedIndex];
  public function select(index:Int):Bool {
    if(index<0||index>=model.sensors.length||index==selectedIndex)return false;
    selectedIndex=index;return true;
  }
  public function add(kind:String):Sensor {
    if(kind!="lidar"&&kind!="imu"&&kind!="joint_encoder")throw "Unsupported sensor kind";
    var sensor=new Sensor(kind+" "+nextId,kind,0.0,"sensor/"+nextId++);
    sensor.frame=model.frames[0];model.addSensor(sensor);selectedIndex=model.sensors.length-1;return sensor;
  }
  public function removeSelected():Bool {
    if(model.sensors.length==0)return false;
    model.sensors.splice(selectedIndex,1);
    selectedIndex=model.sensors.length==0?-1:Std.int(Math.min(selectedIndex,model.sensors.length-1));
    return true;
  }
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
