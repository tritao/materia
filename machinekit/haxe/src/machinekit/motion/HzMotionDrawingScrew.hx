package machinekit.motion;

import cadkit.modeling.Part;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Required SFU1605/BK12 and SFU2005/BK15 machining, HZMotion Motor_Bracket.pdf p10.
 * A procurement/design requirement, not a purchased Vallder screw designation.
 * Shoulder-to-thread-end E1=36, coupling F1=15, M12x1 over S1=12.
 * BK12 has a 51 mm fixed end; BK15 has 64 mm (E1=44 + F1=20),
 * unlike the Vallder standard ends. Race roots are explicit study assumptions.
 */
class HzMotionDrawingScrew extends LeadScrew {
 public static inline final FIXED_END:Float=51;
 public static inline final FLOAT_END:Float=11;
 static var recipe:Null<ComponentType>;
 public final endProfile:String;
 public final fixedEnd:Float;
 public final floatingEnd:Float;
 public function new(length:Float=400,endProfile:String="BK12") {
  if(["BK12","BK15"].indexOf(endProfile)<0)throw "Unknown HZMotion screw end";
  if(!Math.isFinite(length)||length<150)throw "HZMotion screw must contain machined ends and nut travel";
  super(endProfile=="BK15"?new LeadScrewThread(Ball,20,5,1,RightHand,16):BallScrew.race(),length,'HZMOTION-DRAWING-$endProfile-L${Dimension.format(length)}');
  this.endProfile=endProfile;fixedEnd=endProfile=="BK15"?64:FIXED_END;floatingEnd=endProfile=="BK15"?13:FLOAT_END;
  addConnector("fixedShoulder",Mount,Solids.axial(0,0,fixedEnd));
  addConnector("floatingSeat",Axis,Solids.axial(0,0,length-floatingEnd));
 }
 override public function geometry(detail:ComponentDetail=Preview):Part {
  if(endProfile=="BK15")return geometryBk15(detail);
  return Solids.building([],tracked->{
   tracked.push(Part.cylinderSpan(5,0,15));
   tracked.push(Part.cylinderSpan(5.4,15,27)); // Semantic M12x1 root; no helical thread.
   tracked.push(Part.cylinderSpan(6,27,51));
   tracked.push(Solids.named(Part.cylinderSpan(8,51,totalLength-11),"race"));
   tracked.push(Part.cylinderSpan(5,totalLength-11,totalLength-3));
   tracked.push(Part.cylinderSpan(4.8,totalLength-3,totalLength-1.85));
   tracked.push(Part.cylinderSpan(5,totalLength-1.85,totalLength));
   var body=Solids.union(tracked.copy());
   if(detail==Envelope)return body;
   try {
    var result=new Part(cadkit.threads.Threads.attach(body.shape,"race:cyl.side",thread.threadSpec(),AxisMinimum));
    body.close();return result;
   }catch(e:Dynamic){body.close();throw e;}
  });
 }
 function geometryBk15(detail:ComponentDetail):Part {
  // HZMotion p10: F1=20, E1=44, S1=12, bearing15, M15x1.
  // Floating end follows the explicit SFU20 standard 13 mm end requirement.
  var body=Solids.union([Part.cylinderSpan(6,0,20),Part.cylinderSpan(6.9,20,32),
   Part.cylinderSpan(7.5,32,64),Solids.named(Part.cylinderSpan(10,64,totalLength-13),"race"),
   Part.cylinderSpan(7.5,totalLength-13,totalLength-4),Part.cylinderSpan(7.15,totalLength-4,totalLength-2.85),Part.cylinderSpan(7.5,totalLength-2.85,totalLength)]);
  if(detail==Envelope)return body;
  try {var result=new Part(cadkit.threads.Threads.attach(body.shape,"race:cyl.side",thread.threadSpec(),AxisMinimum));body.close();return result;}catch(e:Dynamic){body.close();throw e;}
 }
 override public function axialRigidityAt(z:Float):Float {
  if(!Math.isFinite(z)||z<0||z>totalLength)throw "Invalid HZ screw station";
  var coupling=endProfile=="BK15"?20.0:15.0,journal=endProfile=="BK15"?32.0:27.0;
  var diameter=z<coupling?(endProfile=="BK15"?12.0:10.0):z<journal?(endProfile=="BK15"?13.8:10.8):z<fixedEnd?(endProfile=="BK15"?15.0:12.0):z<totalLength-floatingEnd?thread.rootDiameter():z>=totalLength-4 && z<=totalLength-2.85 && endProfile=="BK15"?14.3:endProfile=="BK15"?15.0:10.0;
  return YOUNGS_MODULUS/1e6*Math.PI*diameter*diameter/4;
 }
 override public function axialStiffness():Float return 1/axialComplianceBetween(0,totalLength);
 override public function axialComplianceBetween(from:Float,to:Float):Float {
  if(!Math.isFinite(from+to)||Math.min(from,to)<0||Math.max(from,to)>totalLength)throw "Invalid HZ screw compliance interval";
  var points=endProfile=="BK15"?[0.0,20,32,fixedEnd,totalLength-floatingEnd,totalLength-4,totalLength-2.85,totalLength]:[0.0,15,27,fixedEnd,totalLength-floatingEnd,totalLength-3,totalLength-1.85,totalLength],sum=0.0;
  for(i in 0...points.length-1){var a=Math.max(Math.min(from,to),points[i]),b=Math.min(Math.max(from,to),points[i+1]);if(b>a)sum+=(b-a)/axialRigidityAt((a+b)/2);}
  return sum;
 }
 public static function recipeType():ComponentType {
  if(recipe==null)recipe=new ComponentType("machinekit.motion.hzmotion-bk12-screw-end-study",[P.length("length",400),P.choice("endProfile",["BK12","BK15"],"BK12")],v->new HzMotionDrawingScrew(v.number("length"),v.token("endProfile")));
  return recipe;
 }
 override public function componentType():Null<ComponentType> return recipeType();
 override public function values():ComponentValues return new ComponentValues().setNumber("length",totalLength).setToken("endProfile",endProfile).setToken("material",materialSpec());
}
