package machinekit.milling;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Parametric reconstruction, not supplier CAD. RKE92 external dimensions are
 * published; internal slides, mounting accessories and recesses are nominal.
 */
class Rke92VisePart extends MachineComponent {
 static var recipe:Null<ComponentType>;
 static function parameters():Array<ComponentParameter> return [
  P.choice("role",["body","jaw-back","slide","jaw-plate","drive","stop","clamp","key"],"body"),
  P.length("depth",39)
 ];
 public function new(role:String="body",depth:Float=39) {
  var v=P.resolve(parameters(),new ComponentValues().setToken("role",role).setNumber("depth",depth));
  super("RKE92-REFERENCE-"+role,"RKE92 drawing reference: "+role,"steel",false,v);
  addConnector("mount",Mount,Solids.axial(0,0,0));
 }
 public static function recipeType():ComponentType {
  if(recipe==null)recipe=new ComponentType("machinekit.milling.rke92-reference-part",parameters(),v->new Rke92VisePart(v.token("role"),v.number("depth")));
  return recipe;
 }
 override public function componentType():Null<ComponentType> return recipeType();
 override public function hasGeometry():Bool return true;
 override public function geometry(detail:ComponentDetail=Preview):Part {
  var d=parameterNumber("depth"),role=values().token("role");
  return switch(role) {
   case "body":
    var body=Part.box(94,276,80).translated(new Vector(0,138,0));
    var cuts=[Part.box(52,197,57).translated(new Vector(0,178.5,24)),
     Part.box(20,278,5.2).translated(new Vector(0,138,-1))];
    for(side in [-1,1])cuts.push(Part.box(12,278,16).translated(new Vector(side*42,138,12)));
    Solids.cut(body,cuts);
   case "jaw-back", "slide":
    var pieces=[Part.box(92,d,32).translated(new Vector(0,-d/2,0))];
    if(role=="slide")pieces.push(Part.box(50,d,50).translated(new Vector(0,-d/2,-50)));
    var cuts:Array<Part> = [];
    for(x in [-31.5,31.5])cuts.push(Part.cylinderAlongY(2.5,-14,1,x,14));
    if(role=="slide")cuts.push(Part.cylinderAlongY(10.5,-d-1,1,0,-32));
    Solids.cut(Solids.union(pieces),cuts);
   case "jaw-plate":
    var plate=Part.box(92,25,32).translated(new Vector(0,-12.5,0));
    var cuts:Array<Part> = [];
    for(x in [-31.5,31.5]) {
     cuts.push(Part.cylinderAlongY(3.3,-26,1,x,14));
     cuts.push(Part.cylinderAlongY(5.5,-6.4,1,x,14));
    }
    Solids.cut(plate,cuts);
   case "drive":
    var raw=Solids.union([Part.cylinderAlongY(10,0,70),Part.cylinderAlongY(24,20,50)]);
    Solids.cut(raw,[Part.cylinderAlongY(4,62,71)]);
   case "stop":
    var raw=Solids.union([Part.box(16,16,6).translated(new Vector(-2,0,0)),Part.box(6,8,30)]);
    Solids.cut(raw,[Part.cylinderSpan(2.25,-1,7,-6,0)]);
   case "clamp":
    Solids.cut(Part.box(60,24,10),[Part.cylinderSpan(3.3,-1,11,10,0)]);
   case "key":
    Solids.union([Part.box(19.8,20,4),Part.box(9.6,20,4).translated(new Vector(0,0,-4))]);
   default:throw "Unknown vise part";
  };
 }
}
