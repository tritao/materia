import materia.casting.PrecastStrut;

class PrecastStrutTests {
 static var count = 0;
 static function check(value:Bool,message:String):Void {count++;if(!value)throw message;}
 static function rejects(raw:Dynamic,message:String):Void {
  var error = "";
  try PrecastStruts.snapshot(raw) catch(value:Dynamic) error=Std.string(value);
  check(error.length>0 && error.indexOf(message)>=0,"Expected "+message+", got "+error);
 }
 public static function run():Void {
  var lower:PrecastStrut={id:"lower-a",coreFeatureId:"core-a",material:CastingCompound,contact:Lower,
   box:{x:10,y:20,z:0,width:50,depth:50,height:85}};
  var upper:PrecastStrut={id:"upper-a",coreFeatureId:"core-a",material:CastingCompound,contact:Upper,
   box:{x:10,y:20,z:775,width:50,depth:50,height:25}};
  var encoded=PrecastStruts.encode([upper,lower]);
  check(encoded==PrecastStruts.encode([lower,upper]),"Source strut order is canonical");
  var copied=PrecastStruts.decode(encoded);
  check(copied.length==2 && copied[0].id==lower.id && copied[1].contact==Upper,"Strut source round trip");
  lower.box.width=99;
  check(copied[0].box.width==50,"Source strut snapshot owns its dimensions");
  var raw:Array<Dynamic>=PrecastStruts.records(copied);
  raw[0].material="steel";rejects(raw,"Unknown precast strut material");
  raw=PrecastStruts.records(copied);raw[0].contact="side";rejects(raw,"Unknown precast strut contact");
  raw=PrecastStruts.records(copied);raw[0].box.height=0;rejects(raw,"Empty precast strut");
  raw=PrecastStruts.records(copied);raw[0].box.x=Math.NaN;rejects(raw,"Nonfinite casting number");
  raw=PrecastStruts.records(copied);raw[0].id=raw[1].id;rejects(raw,"Duplicate precast strut");
  raw=PrecastStruts.records(copied);raw[0].capacity=1000;rejects(raw,"Unexpected casting record fields");
  Sys.println("Precast source struts: "+count+" assertions passed");
 }
}
