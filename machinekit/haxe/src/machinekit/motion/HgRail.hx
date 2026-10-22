package machinekit.motion;

import cadkit.modeling.*;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Exact per-size HG running sections, not scaled approximations. Offline geometry.
 * X across, Y=-railHeight..0, Z=0..length. The table's default end margin determines drilling.
 * Reference identity does not qualify supplier load/preload or material.
 */
class HgRail extends LinearRail {
 static var recipe:Null<ComponentType>;
 public final model:String;
 public function new(model:String="HGR15R",length:Float=400) {
  if(HgReferenceData.rails().indexOf(model)<0)throw "Unknown HG rail: "+model;
  if(!Math.isFinite(length))throw "HG rail requires a finite length";
  super(HgReferenceData.spec(model),length,true);
  this.model=model;
 }
 override public function mountingHole():Null<{diameter:Float,counterboreDiameter:Float,counterboreDepth:Float,reference:String}> {
  var holes=drilling(model);
  return {diameter:holes[0],counterboreDiameter:holes[1],counterboreDepth:holes[2],
   reference:"Pinned HG table/CAD "+model+" sha256:"+HgReferenceData.hash(model)};
 }
 /** Drawn from its own profile, whose shape sets its mass. */
 override public function massGeometry():Null<cadkit.modeling.Part> return null;

 override public function geometry(detail:ComponentDetail=Preview):Part {
  if(detail==Envelope)return super.geometry(detail);
  var edges:Array<Curve> = [],wire:Null<Curve> = null,face:Null<Sketch> = null,result:Null<Part> = null;
  try {
   var section=HgReferenceData.section(model),first=new Vector(section[0][0],section[0][1]),current=first;
   for(i in 0...section.length) {
    var s=section[i],b=i==section.length-1 ? first : new Vector(s[2],s[3]);
    // Supplier STEP endpoints differ by up to 0.00000024 mm (HGR45R).
    // Weld only within the import tolerance, never bridge a broken running section.
    if(Math.abs(current.x-s[0])>1e-6 || Math.abs(current.y-s[1])>1e-6)throw "Disconnected HG reference section";
    edges.push(s.length==4 ? Curve.line(current,b) : Curve.arc(current,new Vector(s[4],s[5]),b));current=b;
   }
   wire=Curve.wire(edges);face=Sketch.face(wire);result=face.extrude(length);
   var holes=drilling(model),tools:Array<Part> = [];
   try {
    for(z in holePositions) {
     if(z<holes[1]/2 || z>length-holes[1]/2)throw "HG rail counterbore crosses cut end";
     tools.push(Part.cylinderAlongY(holes[0]/2,-spec.railHeight-1,1,0,z));
     tools.push(Part.cylinderAlongY(holes[1]/2,-holes[2],1,0,z));
    }
    if(tools.length>0){var drilled=result.subtractAll(tools);result.close();result=drilled;}
    for(tool in tools)tool.close();
   }catch(e:Dynamic){for(tool in tools)tool.close();throw e;}
   face.close();wire.close();for(edge in edges)edge.close();return result;
  }catch(e:Dynamic){if(result!=null)result.close();if(face!=null)face.close();if(wire!=null)wire.close();for(edge in edges)edge.close();throw e;}
 }
 /** Through diameter, counterbore diameter/depth, from the rail table (HGR20 d=6.1). */
 public static function drilling(model:String):Array<Float> return switch model {
  case "HGR15R": [4.5,7.5,5.3]; case "HGR20R": [6.1,9.5,8.5];
  case "HGR25R": [7,11,9]; case "HGR30R","HGR35R": [9,14,12];
  case "HGR45R": [14,20,17];default:throw "Unknown HG rail: "+model;
 };
 public static function referenceRecipeType():ComponentType {
  if(recipe==null)recipe=new ComponentType("machinekit.motion.hg-rail",[P.choice("model",HgReferenceData.rails(),"HGR15R"),P.length("length",400)],v -> new HgRail(v.token("model"),v.number("length")));
  return recipe;
 }
 override public function componentType():Null<ComponentType> return referenceRecipeType();
 override public function values():ComponentValues return new ComponentValues().setToken("model",model).setNumber("length",length).setToken("material",materialSpec());
}
