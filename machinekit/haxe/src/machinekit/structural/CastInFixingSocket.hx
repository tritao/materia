package machinekit.structural;

import cadkit.modeling.Part;
import cadkit.modeling.Curve;
import cadkit.modeling.Vector;
import cadkit.threads.Threads;
import cadkit.threads.ThreadedFace.ThreadStart;
import machinekit.component.*;
import machinekit.standard.FastenerThreads;
import machinekit.component.ComponentRecipeSupport as P;

/** Nominal crimped-end fixing socket. Mouth is Z=0; embedded body extends toward -Z.
 * PFEIFER 05.250.064.040 supplies M6 x 40, a=25 and e=6 mm. OD and crimp
 * dimensions below are explicit study assumptions, not recovered supplier CAD.
 * Thread is semantic, consistent with other MachineKit fasteners.
 */
class CastInFixingSocket extends MachineComponent {
  static var recipe:Null<ComponentType>;
  public static inline final SOURCE="https://www.pfeifer.info/out/assets/PFEIFER_SOCKET-CRIMPED-END_PPEN.PDF";
  public static function parameters():Array<ComponentParameter> return [
    P.length("length",40),P.length("outsideDiameter",8),P.length("boreDepth",25),
    P.length("threadLength",6),P.length("tailWidth",11),P.length("tailThickness",2.2),
    P.length("tailWave",1.4)
  ];
  public static function recipeType():ComponentType {
    if(recipe==null)recipe=new ComponentType("machinekit.structural.cast-in-fixing-socket",parameters(),v->new CastInFixingSocket(v),true);
    return recipe;
  }
  public function new(?overrides:ComponentValues) {
    var v=P.resolve(parameters(),overrides);
    for(n in ["length","outsideDiameter","boreDepth","threadLength","tailWidth","tailThickness","tailWave"])
      if(!Math.isFinite(v.number(n))||v.number(n)<=0)throw "Socket dimensions must be finite and positive";
    if(v.number("outsideDiameter")<=6.2||v.number("threadLength")>=v.number("boreDepth")||v.number("boreDepth")>=v.number("length")||
      v.number("tailWidth")<v.number("outsideDiameter")||v.number("tailThickness")>=v.number("outsideDiameter"))throw "Inconsistent socket study dimensions";
    var key=[for(n in ["length","outsideDiameter","boreDepth","threadLength","tailWidth","tailThickness","tailWave"])Dimension.format(v.number(n))].join("-");
    super("CAST-IN-SOCKET-M6-NOMINAL-"+key,"Nominal M6 cast-in fixing socket, crimped tail; OD/tail unverified","Stainless sheet",false,v);
    addConnector("mouth",Mount,Solids.axial(0,0,0));
    addConnector("mount",Mount,Solids.axial(0,0,0));
    addConnector("tail",Mount,Solids.axial(0,0,-v.number("length")));
  }
  override public function componentType():Null<ComponentType> return recipeType();
  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail=Preview):Part {
    var length=parameterNumber("length"),boreDepth=parameterNumber("boreDepth"),threadLength=parameterNumber("threadLength"),r=parameterNumber("outsideDiameter")/2;
    var wide=parameterNumber("tailWidth")/2,thin=parameterNumber("tailThickness")/2,wave=parameterNumber("tailWave");
    var tailLength=length-boreDepth;
    // Equal polygon topology makes the closed, flattened/waved retention tail a stable loft.
    var sections:Array<Curve> = [];
    for(s in [[-length,wide,thin,0.0],[-length+tailLength*0.25,wide,thin,wave],
        [-length+tailLength*0.60,wide*0.92,thin,-wave],[-boreDepth-0.5,r,r,0.0],[-boreDepth+0.5,r,r,0.0]]) {
      sections.push(Curve.polyline([for(i in 0...24){var a=2*Math.PI*i/24;new Vector(s[1]*Math.cos(a),s[2]*Math.sin(a)+s[3],s[0]);}],true));
    }
    var tail:Part;
    try {tail=Part.loft(sections,true);}catch(error:Dynamic){for(s in sections)s.close();throw error;}
    for(s in sections)s.close();
    var body=Solids.union([tail,Part.cylinderSpan(r,-boreDepth,0)]);
    if(detail==Envelope)return body;
    // Blind insertion cavity: the closed tail remains below a=25. First e=6 is the catalogue thread region.
    body=Solids.cut(body,[Part.cylinderSpan(3.1,-boreDepth,-threadLength)]);
    var bore=Solids.named(Part.cylinderSpan(3,-threadLength,0.1),"thread-bore");
    var threaded=new Part(Threads.attach(bore.shape,"thread-bore:cyl.side",FastenerThreads.metric(6,1).mating(),AxisMaximum));
    bore.close();
    return Solids.cut(body,[threaded]);
  }
}
