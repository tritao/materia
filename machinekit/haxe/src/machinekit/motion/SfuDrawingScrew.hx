package machinekit.motion;
import cadkit.modeling.Part;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;

/** Vallder's published SFU20/SFU25 standard ends, shared by the 5/10 mm lead variants.
 * Semantic race; root envelope for the lock thread. Small chamfers/fillets omitted.
 * Root diameter is an explicit engineering assumption. Catalogue end option, not custom machining.
 */
class SfuDrawingScrew extends LeadScrew {
 static var recipe:Null<ComponentType>;
 public static function models():Array<String> return ["SFU2005","SFU2010","SFU2505","SFU2510"];
 public final model:String;
 public final fixedEnd:Float;
 public final floatingEnd:Float;
 public final couplingDiameter:Float;
 public final couplingLength:Float;
 public final journalDiameter:Float;
 public final journalStart:Float;
 public final grooveStart:Float;
 public final grooveDepth:Float;
 public function new(model:String="SFU2005",length:Float=500,raceRootDiameter:Float=0) {
  if(models().indexOf(model)<0)throw "Unknown drawing screw: "+model;
  var s=SfuFamilyData.nut(model),ends=SfuFamilyData.ends(model),end=ends.fixedEnd,floatEnd=ends.floatingEnd;
  if(!Math.isFinite(length) || length<=end+floatEnd+s.length || length>4000)throw "SFU length must contain its standard ends and selected nut";
  super(new LeadScrewThread(Ball,s.diameter,s.pitch,1,RightHand,raceRootDiameter),length,"SFU-DRAWING-"+model+"-L"+Dimension.format(length));
  this.model=model;fixedEnd=end;floatingEnd=floatEnd;couplingDiameter=ends.couplingDiameter;couplingLength=ends.couplingLength;
  journalDiameter=ends.journalDiameter;journalStart=ends.couplingLength+ends.threadReliefLength;
  grooveStart=length-floatEnd+ends.ringSeatLength;grooveDepth=ends.grooveDepth;
  addConnector("raceStart",Mount,Solids.axial(0,0,fixedEnd));
  addConnector("fixedJournalStart",Axis,Solids.axial(0,0,journalStart));
  addConnector("floatingSeat",Axis,Solids.axial(0,0,length-floatingEnd));
  addConnector("outputRing",Mount,Solids.axial(0,0,grooveStart+1.15/2));
 }
 override public function geometry(detail:ComponentDetail=Preview):Part {
  var d=journalDiameter,race=thread.screwDiameter/2,l=totalLength;
  var body=Solids.union([
   Solids.named(Part.cylinderSpan(couplingDiameter/2,0,couplingLength),"coupling-end"),
   Solids.named(Part.cylinderSpan((d-1.2)/2,couplingLength,journalStart-2),"lock-thread-root"),
   Solids.named(Part.cylinderSpan((d-2)/2,journalStart-2,journalStart),"relief"),
   Solids.named(Part.cylinderSpan(d/2,journalStart,fixedEnd),"fixed-journal"),
   Solids.named(Part.cylinderSpan(race,fixedEnd,l-floatingEnd),"race"),
   Solids.named(Part.cylinderSpan(d/2,l-floatingEnd,grooveStart),"floating-journal"),
   Solids.named(Part.cylinderSpan(d/2-grooveDepth,grooveStart,grooveStart+1.15),"ring-groove"),
   Solids.named(Part.cylinderSpan(d/2,grooveStart+1.15,l),"tip")]);
  try {var result=new Part(cadkit.threads.Threads.attach(body.shape,"race:cyl.side",thread.threadSpec(),AxisMinimum));body.close();return result;}catch(e:Dynamic){body.close();throw e;}
 }
 override public function axialStiffness():Float return 1/axialComplianceBetween(0,totalLength);
 override public function axialRigidityAt(z:Float):Float {
  if(!Math.isFinite(z) || z<0 || z>totalLength)throw "Invalid SFU shaft station";
  var d=z<couplingLength ? couplingDiameter : z<journalStart-2 ? journalDiameter-1.2 : z<journalStart ? journalDiameter-2 :
   z<fixedEnd ? journalDiameter : z<totalLength-floatingEnd ? thread.rootDiameter() : z>=grooveStart && z<=grooveStart+1.15 ? journalDiameter-2*grooveDepth : journalDiameter;
  return YOUNGS_MODULUS/1e6*Math.PI*d*d/4;
 }
 override public function axialComplianceBetween(from:Float,to:Float):Float {
  if(!Math.isFinite(from+to) || Math.min(from,to)<0 || Math.max(from,to)>totalLength)throw "Invalid SFU compliance interval";
  var points=[0.0,couplingLength,journalStart-2,journalStart,fixedEnd,totalLength-floatingEnd,grooveStart,grooveStart+1.15,totalLength],sum=0.0;
  for(i in 0...points.length-1){var a=Math.max(Math.min(from,to),points[i]),b=Math.min(Math.max(from,to),points[i+1]);if(b>a)sum+=(b-a)/axialRigidityAt((a+b)/2);}
  return sum;
 }
 public static function drawingRecipeType():ComponentType {
  if(recipe==null)recipe=new ComponentType("machinekit.motion.sfu-drawing-screw",[P.choice("model",models(),"SFU2005"),P.length("length",500),P.length("raceRootDiameter",16)],v -> new SfuDrawingScrew(v.token("model"),v.number("length"),v.number("raceRootDiameter")));
  return recipe;
 }
 override public function componentType():Null<ComponentType> return drawingRecipeType();
 override public function values():ComponentValues return new ComponentValues().setToken("model",model).setNumber("length",totalLength).setNumber("raceRootDiameter",thread.rootDiameter()).setToken("material",materialSpec());
}
