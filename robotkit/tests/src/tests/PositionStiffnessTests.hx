package tests;
import materia.assembly.PositionStiffness;

class PositionStiffnessTests {
  static function near(value:Float, expected:Float):Void {
    if (Math.abs(value-expected)>1e-7*Math.max(1.0,Math.abs(expected))) throw 'Stiffness $value differs from $expected';
  }
  public static function run():Void {
    // EA = 1000 N, two one-metre paths together; q redistributes length between them.
    var belt = new PositionStiffness([
      {compliance:0.1/1000,coordinates:[{joint:"x",coefficient:1.0/1000}]},
      {compliance:0.9/1000,coordinates:[{joint:"x",coefficient:-1.0/1000}]}]);
    near(belt.at(_->0.0),1000*(1.0/0.1+1.0/0.9));
    near(belt.at(_->0.4),4000.0);
    var range = belt.range(_->({lower:0.0,upper:0.8}));
    near(range.minimum,4000.0); near(range.maximum,1000*(1.0/0.1+1.0/0.9));
    if (range.minimum>4000 || range.maximum<1000*(1.0/0.1+1.0/0.9)) throw "Range must enclose the analytic extrema";
    // A skewed box: the minimum lies on a face, not at a sampled centre or vertex.
    var skew = new PositionStiffness([
      {compliance:0.0001,coordinates:[{joint:"x",coefficient:0.001},{joint:"y",coefficient:0.001}]},
      {compliance:0.0009,coordinates:[{joint:"x",coefficient:-0.001},{joint:"y",coefficient:-0.001}]}]);
    near(skew.range(id->id=="x" ? {lower:0.0,upper:0.3} : {lower:0.0,upper:0.5}).minimum,4000);
    // Conversion from millimetre coordinates and mm/N compliance to SI, including a placed joint origin.
    var millimetres = new PositionStiffness([{compliance:0.1,coordinates:[{joint:"nut",coefficient:0.001}]}]);
    var si = millimetres.scaled(0.001,_->0.001,_->20.0);
    near(si.at(_->0.03),1.0/0.00015);
    // Nut/bearing compliance remains in series as shaft length changes.
    var screw = new PositionStiffness([{compliance:1.0/3e8+1.0/2e8+0.1/1e7,coordinates:[{joint:"nut",coefficient:1.0/1e7}]}]);
    near(screw.at(_->0.2),1.0/(1.0/3e8+1.0/2e8+0.3/1e7));
    var refused = false;
    try belt.range(_->({lower:-0.1,upper:0.8})) catch (_:Dynamic) refused=true;
    if (!refused) throw "A span reaching its anchor must be rejected";
    var model = new robotkit.model.RobotModel("law round trip");
    var base = model.addLink(new robotkit.model.Link("base")), load = model.addLink(new robotkit.model.Link("load"));
    var rotor = model.addLink(new robotkit.model.Link("rotor"));
    var x = model.addJoint(new robotkit.model.Joint("x",robotkit.model.JointType.Prismatic,base,load));
    x.limits = new robotkit.model.JointLimits(0.0,0.8);
    var motor = model.addJoint(new robotkit.model.Joint("motor",robotkit.model.JointType.Revolute,base,rotor));
    motor.limits = new robotkit.model.JointLimits(-100.0,100.0);
    var coupling = model.addCoupling(new robotkit.model.JointCoupling("belt","x","motor",1.0,0.0));
    coupling.setStiffnessLaw(belt,model.joints);
    var copy = robotkit.model.RobotModelCodec.decode(robotkit.model.RobotModelCodec.encode(model));
    if (copy.couplings[0].stiffnessLaw == null) throw "Robot serialization dropped the constitutive law";
    near(copy.couplings[0].stiffnessLaw.at(_->0.4),4000);
    // A formerly short screw becomes softer if its permitted nut travel grows.
    var shaftLaw = new PositionStiffness([{compliance:0.001,coordinates:[{joint:"x",coefficient:0.001}]}]);
    coupling.setStiffnessLaw(shaftLaw,model.joints);
    coupling.validateStiffnessLaw(model.joints);
    x.limits.upper = 1.0;
    refused = false;
    try coupling.validateStiffnessLaw(model.joints) catch (_:Dynamic) refused=true;
    if (!refused) throw "Editing joint travel must invalidate an unsafe cached planning envelope";
    coupling.setStiffnessLaw(shaftLaw,model.joints);
    coupling.validateStiffnessLaw(model.joints);
    Sys.println("Position stiffness checks passed: analytic extrema, convex certificate, SI origins and screw series compliance");
  }
  public static function main():Void run();
}
