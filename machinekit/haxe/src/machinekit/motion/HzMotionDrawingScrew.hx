package machinekit.motion;

import cadkit.modeling.Part;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Required SFU1605 machining profile, HZMotion Motor_Bracket.pdf p10, BK12-C.
 * A procurement/design requirement, not a purchased Vallder screw designation.
 * Shoulder-to-thread-end E1=36, coupling F1=15, M12x1 over S1=12.
 * This explicit profile has a 51 mm fixed end, unlike Vallder's 54 mm end.
 */
class HzMotionDrawingScrew extends LeadScrew {
 public static inline final FIXED_END:Float=51;
 public static inline final FLOAT_END:Float=11;
 static var recipe:Null<ComponentType>;
 public function new(length:Float=400) {
  if(!Math.isFinite(length)||length<150)throw "HZMotion screw must contain machined ends and nut travel";
  super(BallScrew.race(),length,'HZMOTION-DRAWING-BK12-SFU1605-L${Dimension.format(length)}-END51');
  addConnector("fixedShoulder",Mount,Solids.axial(0,0,FIXED_END));
  addConnector("floatingSeat",Axis,Solids.axial(0,0,length-FLOAT_END));
 }
 override public function geometry(detail:ComponentDetail=Preview):Part {
  return Solids.building([],tracked->{
   tracked.push(Part.cylinderSpan(5,0,15));
   tracked.push(Part.cylinderSpan(5.4,15,27)); // Semantic M12x1 root; no helical thread.
   tracked.push(Part.cylinderSpan(6,27,51));
   tracked.push(Part.cylinderSpan(8,51,totalLength-11));
   tracked.push(Part.cylinderSpan(5,totalLength-11,totalLength-3));
   tracked.push(Part.cylinderSpan(4.8,totalLength-3,totalLength-1.85));
   tracked.push(Part.cylinderSpan(5,totalLength-1.85,totalLength));
   return Solids.union(tracked.copy());
  });
 }
 public static function recipeType():ComponentType {
  if(recipe==null)recipe=new ComponentType("machinekit.motion.hzmotion-bk12-screw-end-study",[P.length("length",400)],v->new HzMotionDrawingScrew(v.number("length")));
  return recipe;
 }
 override public function componentType():Null<ComponentType> return recipeType();
 override public function values():ComponentValues return new ComponentValues().setNumber("length",totalLength).setToken("material",materialSpec());
}
