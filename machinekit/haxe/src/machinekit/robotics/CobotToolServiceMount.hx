package machinekit.robotics;

import cadkit.modeling.Part;
import cadkit.modeling.Curve;
import cadkit.modeling.Plane;
import cadkit.modeling.Vector;
import machinekit.component.MachineComponent;
import machinekit.component.ComponentDetail;
import machinekit.component.Solids;
import materia.assembly.AssemblyFrames;

/** Prototype split collar following the generic terminal housing taper.
    0.5 mm radial clearance reserves a liner; grip/preload and shell strength are unvalidated. */
class CobotToolServiceMount extends MachineComponent {
  public final housing:CobotToolLink;
  public final cap:Bool;
  public final centre:Float;
  public final radius:Float;
  public function new(housing:CobotToolLink,cap:Bool) {
    super('TOOL-SERVICE-COLLAR-${housing.designation}-$cap',"Prototype tool-service split collar "+(cap ? "socket half" : "lower half"),"aluminium 6061",true);
    this.housing=housing;this.cap=cap;centre=housing.length/2;
    if(housing.length<16) throw "Terminal housing too short for service collar";
    radius=Math.max(housing.radiusAt(centre-6),housing.radiusAt(centre+6))+0.5;
    addConnector("housing",Mount,AssemblyFrames.identity());
    addConnector("socketMount",Mount,AssemblyFrames.fromRotationMatrix(0,radius+8,centre,[1,0,0,0,0,1,0,-1,0]));
  }
  function box(x:Float,y:Float,z:Float,dx:Float,dy:Float,dz:Float):Part {
    var raw=Part.box(dx,dy,dz);var placed=raw.translated(new Vector(x,y,z));raw.close();return placed;
  }
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail = Preview):Part {
    var sections:Array<Curve> = [];
    for(i in 0...5) {
      var z=centre-6.1+12.2*i/4;
      sections.push(Curve.circle(housing.radiusAt(z)+0.5,Plane.XY().offset(z)));
    }
    var bore=Part.loft(sections);for(c in sections) c.close();
    var ring=Solids.cut(Part.cylinderSpan(radius+4,centre-6,centre+6),[bore]);
    // Half separation leaves 0.4 mm across the split before clamping.
    var half=box(0,cap ? (radius+12)/2+0.1 : -(radius+12)/2-0.1,centre-6,(radius+12)*2,radius+12-0.2,12);
    var cut=ring.shape.common(half.shape);ring.close();half.close();
    var pieces=[new Part(cut)];
    for(x in [-radius-5,radius+5]) pieces.push(box(x,cap ? 3.1 : -3.1,centre-6,10,5.8,12));
    if(cap) {
      pieces.push(box(0,radius+6,centre-14,32,4,28));
      var low=Math.sqrt(Math.pow(radius+4,2)-13*13)-1;
      for(x in [-10.0,10.0]) pieces.push(box(x,(low+radius+8)/2,centre-6,6,radius+8-low,12));
    }
    var tools:Array<Part> = [];
    for(x in [-radius-5,radius+5]) tools.push(Part.cylinderAlong(1.7,new Vector(x,-8,centre),Vector.Y(),16));
    if(cap) for(x in [-10.0,10.0]) for(z in [-9.0,9.0])
      tools.push(Part.cylinderAlong(1.25,new Vector(x,radius+3,centre+z),Vector.Y(),6));
    var result=Solids.cut(Solids.union(pieces),tools);
    var refined=result.refined();result.close();return refined;
  }
}
