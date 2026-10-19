package machinekit.motion;
import cadkit.modeling.*;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import machinekit.motion.SfuFamilyData.SfuNutReference;

/** Shared SFU flange family; published per-model dimensions, exact hole clearances and flats.
 * Root diameter is an explicit engineering input, not inferred from nominal size/lead.
 */
class SfuNut extends LeadScrewNut {
 static var recipe:Null<ComponentType>;
 public final spec:SfuNutReference;
 public function new(model:String="SFU1605",raceRootDiameter:Float=0) {
  var s=SfuFamilyData.nut(model);
  super(new LeadScrewThread(Ball,s.diameter,s.pitch,1,RightHand,raceRootDiameter),6,true,BallNut,
   {bodyDiameter:s.pilot,bodyLength:s.length,flangeDiameter:s.flange,flangeThickness:s.thickness,flangeOffset:s.offset,boltCircleDiameter:s.pcd,mountScrew:s.mount});
  spec=s;setMaterial("steel");
 }
 override public function boltPattern():Array<{x:Float,y:Float}> return [for(a in [45,90,135,225,270,315]){x:boltCircleDiameter/2*Math.cos(a*Math.PI/180),y:boltCircleDiameter/2*Math.sin(a*Math.PI/180)}];
 override public function geometry(detail:ComponentDetail=Preview):Part {
  var body=super.geometry(detail),s=spec;
  return Solids.building([body],owned -> {
   var tail=Solids.cut(Part.cylinderSpan(s.pilot/2+0.1,s.offset+s.thickness,s.length+0.1),[Part.cylinderSpan(s.barrel/2,s.offset+s.thickness-0.1,s.length+0.2)]);owned.push(tail);
   var tools=[tail];
   for(side in [-1,1]) {
    var raw=Part.box(s.flange,s.flange*2,s.thickness+0.2),flat=raw.translated(new Vector(side*(s.flats/2+s.flange/2),0,s.offset-0.1));raw.close();owned.push(flat);tools.push(flat);
   }
   if(detail!=Envelope)for(p in boltPattern()) {var hole=Part.cylinderSpan(s.hole/2,s.offset-0.1,s.offset+s.thickness+0.1,p.x,p.y);owned.push(hole);tools.push(hole);}
   return Solids.cut(body,tools);
  });
 }
 public static function familyRecipeType():ComponentType {
  if(recipe==null)recipe=new ComponentType("machinekit.motion.sfu-nut",[P.choice("model",SfuFamilyData.nutModels(),"SFU1605"),P.length("raceRootDiameter",13)],v -> new SfuNut(v.token("model"),v.number("raceRootDiameter")));
  return recipe;
 }
 override public function componentType():Null<ComponentType> return familyRecipeType();
 override public function values():ComponentValues return new ComponentValues().setToken("model",spec.model).setNumber("raceRootDiameter",thread.rootDiameter()).setToken("material",materialSpec());
}
