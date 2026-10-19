package machinekit.assembly;
import cadkit.modeling.*;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import machinekit.standard.SocketHeadCapScrew;

/** Fabricated base, risers, carriage and motor bracket for a standard purchased SFU20 stack. */
class Sfu20AxisPart extends MachineComponent {
 public final kind:String;
 public final layout:Sfu20AxisLayout;
 static var recipe:Null<ComponentType>;
 public static function kinds():Array<String> return ["base","riserLeft","riserRight","carriage","motorBracket"];
 public function new(kind:String="base",?layout:Sfu20AxisLayout) {
  if(layout==null)layout=new Sfu20AxisLayout();if(kinds().indexOf(kind)<0)throw "Unknown SFU20 fabrication";
  super("SFU20-"+kind+"-v1-L"+Dimension.format(layout.screwLength)+"-R"+Dimension.format(layout.railLength)+"-S"+Dimension.format(layout.railSpacing),"SFU20 axis "+kind,"aluminium 6061");
  this.kind=kind;this.layout=layout;
 }
 static function box(w:Float,h:Float,l:Float,x:Float,y:Float,z:Float):Part {var raw=Part.box(w,h,l),p=raw.translated(new Vector(x,y,z));raw.close();return p;}
 static function tap(size:String,depth:Float,x:Float,y:Float,z:Float,normal:Vector):Part {
  var raw=SocketHeadCapScrew.metric(size,depth).tapHole(depth);
  try {var posed=raw.placed(new Location(new Plane(new Vector(x,y,z),Vector.X(),normal)));raw.close();return Solids.named(posed,'tap-$x-$y-$z');}catch(e:Dynamic){raw.close();throw e;}
 }
 override public function hasGeometry():Bool return true;
 override public function geometry(detail:ComponentDetail=Preview):Part return Solids.building([],owned -> {
  var l=layout,body:Part=null,tools:Array<Part> = [];
  switch kind {
   case "base":
    body=box(l.width(),16,l.baseLength(),0,-8,6);owned.push(body);
    for(z in [l.fixedCentreZ()-7.5,l.fixedCentreZ()+7.5,l.floatingCentreZ()])for(x in [-27.0,27])tools.push(tap("M6",14,x,0,z,Vector.Y()));
    for(x in [-38.0,38])tools.push(tap("M6",14,x,0,30,Vector.Y()));
    for(side in [-1,1])for(z in [150.0,140+l.railLength-10])tools.push(tap("M6",14,side*(l.railSpacing/2+16),0,z,Vector.Y()));
   case "riserLeft","riserRight":
    body=box(48,20,l.railLength,0,10,0);owned.push(body);
    var x=kind=="riserLeft" ? -16.0 : 16.0;
    for(z in [10.0,l.railLength-10]) {tools.push(Part.cylinderAlongY(3.3,-0.1,20.1,x,z));tools.push(Part.cylinderAlongY(5.5,13.5,20.1,x,z));}
    for(z in l.railHoles())tools.push(tap("M5",12,0,20,z,Vector.Y()));
   case "carriage":
    body=box(l.width(),14,100,0,7,-50);owned.push(body);
    for(x in [-24.0,24])for(z in [-12.0,12])tools.push(Part.cylinderAlongY(3.3,-0.1,14.1,x,z));
    for(side in [-1,1])for(x in [-16.0,16])for(z in [-18.0,18]) {
     tools.push(Part.cylinderAlongY(2.75,-0.1,14.1,side*l.railSpacing/2+x,z));
     tools.push(Part.cylinderAlongY(4.75,6,14.1,side*l.railSpacing/2+x,z));
    }
   case "motorBracket":
    body=Solids.union([box(84,65,6,0,32.5,0),box(100,8,40,0,4,6)]);owned.push(body);
    tools.push(Part.cylinderSpan(19.1,-0.1,6.1,0,28));
    for(x in [-23.57,23.57])for(y in [-23.57,23.57])tools.push(tap("M4",6,x,28+y,0,Vector.Z().scale(-1)));
    for(x in [-38.0,38])tools.push(Part.cylinderAlongY(3.3,-0.1,8.1,x,30));
   default:throw "Unknown SFU20 fabrication";
  }
  for(t in tools)owned.push(t);return Solids.cut(body,tools);
 });
 public static function recipeType():ComponentType {
  if(recipe==null)recipe=new ComponentType("machinekit.assembly.sfu20-axis-part",[P.choice("kind",kinds(),"base"),P.length("travel",300),P.length("screwLength",600),P.length("railLength",450),P.length("railSpacing",180)],v -> new Sfu20AxisPart(v.token("kind"),new Sfu20AxisLayout(v.number("travel"),v.number("screwLength"),v.number("railLength"),v.number("railSpacing"))),true,false,["travel"]);
  return recipe;
 }
 override public function componentType():Null<ComponentType> return recipeType();
 override public function values():ComponentValues return new ComponentValues().setToken("kind",kind).setNumber("travel",layout.travel).setNumber("screwLength",layout.screwLength).setNumber("railLength",layout.railLength).setNumber("railSpacing",layout.railSpacing).setToken("material",materialSpec());
}
