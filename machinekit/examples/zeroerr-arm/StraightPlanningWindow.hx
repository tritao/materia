/** Extra simulation restrictions; physical stops and clearance remain unqualified. */
class StraightPlanningWindow {
  public static function apply(definition:materia.assembly.AssemblyDefinition, layoutPath:String):String {
    var directory = ZeroErrArmPreview.DIRECTORY;
    var path = directory+"/straight-planning-window.json";
    var text = sys.io.File.getContent(path), config:Dynamic = haxe.Json.parse(text);
    if(config.schema!=1 || config.status!="simulation_planning_restriction_only" || config.hardwareQualified!=false ||
       config.requiresPoseAndContinuousCollisionChecks!=true ||
       Std.string(config.layoutSha256)!=haxe.crypto.Sha256.encode(sys.io.File.getContent(layoutPath)) ||
       Std.string(config.reviewConfigSha256)!=haxe.crypto.Sha256.encode(sys.io.File.getContent(directory+"/clearance-review-config.json")))
      throw "Straight planning restriction needs review for the current geometry";
    var ids = Reflect.fields(config.limitsDeg);
    if(ids.length!=2 || ids.indexOf("j3")<0 || ids.indexOf("j4")<0) throw "Expected J3/J4 planning restrictions";
    for(id in ids) {
      var range:Array<Float> = Reflect.field(config.limitsDeg,id);
      if(range.length!=2 || !Math.isFinite(range[0]) || !Math.isFinite(range[1]) || range[0]>=0 || range[1]<=0 ||
         range[0]< -180 || range[1]>180) throw "Invalid simulation planning restriction";
      var joints = [for(j in definition.joints) if(j.id==id && j.type=="revolute") j];
      if(joints.length!=1) throw "Missing restricted joint: "+id;
      joints[0].limits.lower=range[0]*Math.PI/180;
      joints[0].limits.upper=range[1]*Math.PI/180;
    }
    return haxe.crypto.Sha256.encode(text);
  }
}
