package toolpathkit.tool;

/** Router tooling dimensions in metres; the holder envelope includes the collet/nut.
    Gauge length is spindle gauge plane to tip, independent of exposed cutting length. */
class RouterBit {
 public final tool:Tool;
 public final flutes:Int;
 public final helix:String;
 public final collet:String;
 public final holder:String;
 public final holderDiameter:Float;
 public final holderLength:Float;
 public function new(number:Int, diameter:Float, cuttingLength:Float, stickOut:Float,
   gaugeLength:Float, flutes:Int, helix:String, collet:String, holder:String,
   holderDiameter:Float, holderLength:Float) {
  if(flutes<1||["up","down","compression","straight"].indexOf(helix)<0||collet.length==0||holder.length==0)
   throw "Invalid router flutes/helix/holder";
  for(v in [diameter,cuttingLength,stickOut,gaugeLength,holderDiameter,holderLength])
   if(!Math.isFinite(v)||v<=0)throw "Invalid router dimension";
  if(cuttingLength>stickOut||stickOut+holderLength>gaugeLength||holderDiameter<diameter)
   throw "Router tool exceeds its gauge/holder envelope";
  this.flutes=flutes;this.helix=helix;this.collet=collet;this.holder=holder;
  this.holderDiameter=holderDiameter;this.holderLength=holderLength;
  var radius=diameter/2,outer=holderDiameter/2;
  var profile=new CutterProfile([
   CutterSegment.Line(0,0,radius,0,CutterZone.Cutting),
   CutterSegment.Line(radius,0,radius,cuttingLength,CutterZone.Cutting),
   CutterSegment.Line(radius,cuttingLength,radius,stickOut,CutterZone.Shank),
   CutterSegment.Line(radius,stickOut,outer,stickOut,CutterZone.Holder),
   CutterSegment.Line(outer,stickOut,outer,gaugeLength,CutterZone.Holder)]);
  tool=new Tool(number,gaugeLength,diameter,profile,cuttingLength,stickOut);
 }
}
