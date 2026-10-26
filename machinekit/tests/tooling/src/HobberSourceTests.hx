import machinekit.hobbing.FoundationLayout;
import materia.casting.BoxAlgebra;
import materia.casting.CastingCavities;
import materia.casting.CastingPatterns;
import materia.assembly.AssemblyFrames as F;

/** Approved source declarations, independent of artifact export. Native kit
 * review still proves reconstruction, fixture clearance and both motion graphs. */
@:access(machinekit.hobbing.HobberFoundationSizing)
class HobberSourceTests {
 static var assertions=0;
 static function check(value:Bool,message:String):Void {
  assertions++;
  if(!value)throw message;
 }

 public static function bodyIntent():Void {
  for(kind in ["bed","column"]) {
   var values=FoundationLayout.bodyValues(kind),cuts=CastingPatterns.decode(values.token("features"));
   var envelope=materia.casting.CastingEnvelope.build(values.number("width"),values.number("depth"),values.number("height"));
   var cavities=CastingCavities.derive(envelope,cuts);
   check(cavities.length==cuts.length,"Every declared source cut derives exactly: "+kind);
   var byFeature=new Map<String,Array<materia.casting.Box3>>(),voids:Array<materia.casting.Box3>=[];
   for(cavity in cavities) {
    check(CastingCavities.supportsBoxes(cavity),"Hobber design cuts retain exact box topology: "+cavity.featureId);
    var boxes=CastingCavities.boxes(cavity);byFeature.set(cavity.featureId,boxes);voids=voids.concat(boxes);
   }
   for(group in BoxAlgebra.groups(BoxAlgebra.subtract(voids,[]))) {
    var members=cuts.filter(feature->{
     var boxes=byFeature.get(feature.id);
     return BoxAlgebra.volume(BoxAlgebra.subtract(boxes,group))<BoxAlgebra.volume(boxes)-1e-7;
    });
    check(members.filter(feature->feature.openings.filter(opening->opening.kind==BodyFace).length>0).length>0,
     "Every source cavity network has a declared body opening: "+[for(feature in members)feature.id].join(", "));
   }
   if(kind=="column") {
    check(cuts.length==3 && cuts.filter(feature->feature.kind==ServiceOpening && StringTools.startsWith(feature.id,"head-recess-")).length==3,
     "The column is solid apart from its declared head recess");
   } else {
    var drive=cuts.filter(feature->feature.id=="pocket-driveRibRear-leftWebInside-floor")[0];
    check(drive.openings.filter(opening->opening.kind==BodyFace && opening.target=="body.z-min").length==1,
     "The drive chamber opens directly to the sole");
    var duct=cuts.filter(feature->feature.id=="cable-route")[0];
    check(duct.openings.length==2 && duct.openings.filter(opening->opening.kind==BodyFace).length==2,
     "The cable duct has two real end-face openings");
   }
  }
 }

 static function foundationIntent():Void {
  var selections=Sys.args().indexOf("--column-only")>=0 ? [HobberMoldSetup.HobberMoldCasting.Column]
   : Sys.args().indexOf("--plinth-only")>=0 ? [HobberMoldSetup.HobberMoldCasting.Plinth]
   : [HobberMoldSetup.HobberMoldCasting.Plinth,HobberMoldSetup.HobberMoldCasting.Column];
  for(kind in selections) {
   var foundation:machinekit.structural.MineralFoundation=kind==Plinth
    ? new machinekit.hobbing.MineralCastBed() : new machinekit.hobbing.MineralCastColumn();
   var document=HobberMoldSetup.document(foundation,kind),input=document.input;
   var axis=kind==Plinth ? F.transformVector(document.setup.sourceToMold,0,0,1)
    : F.transformVector(document.setup.sourceToMold,0,1,0);
   check(axis.z< -1+1e-8,"Declared mounting faces point toward the mold bed: "+kind);
   var lifting=input.inserts.filter(insert->insert.role==materia.casting.InsertSpec.InsertRole.LiftingMount);
   var original=lifting.filter(insert->insert.id=="liftLeft" || insert.id=="liftRight");
   var additional=lifting.filter(insert->StringTools.startsWith(insert.id,"liftAdditional"));
   check(original.length==2 && additional.length==2,"Preserve both original lifts and add exactly one symmetric pair");
   for(insert in original) {
    var expectedX=kind==Plinth ? -150-machinekit.hobbing.HobberXLayout.front().bedCentre
     : (insert.id=="liftLeft" ? -160.0 : 160.0);
    var expectedY=kind==Plinth ? (insert.id=="liftLeft" ? -310.0 : 310.0) : -40.0;
    check(Math.abs(insert.pose.x-expectedX)+Math.abs(insert.pose.y-expectedY)<1e-7,
     "Original lifting service centre does not move: "+insert.id);
   }
   check(kind==Plinth ? Math.abs(additional[0].pose.x-additional[1].pose.x)<1e-7
     && Math.abs(additional[0].pose.y+additional[1].pose.y)<1e-7
    : Math.abs(additional[0].pose.x+additional[1].pose.x)<1e-7
     && Math.abs(additional[0].pose.y-additional[1].pose.y)<1e-7,
    "The new lifting pair remains symmetric");
   for(insert in lifting) {
    var size=materia.casting.InsertSpec.InsertBodyCodec.box(insert.body,insert.id);
    Sys.println("Source lifting "+kind+"/"+insert.id+" at "+insert.pose.x+", "+insert.pose.y
     +"; blank "+size.width+" x "+size.depth+" x "+size.height);
   }
   var minimum=document.recipe.designReview.minimumLiftingPoints;
   check(minimum!=null && lifting.length>=minimum,"Source lifting inserts satisfy the unchanged shop point-count rule: "+kind);
   for(insert in input.inserts) {
    var stock=F.compose(document.setup.sourceToMold,insert.pose);
    var face=materia.tooling.casting.InsertFixturePlanner.mountingFace(insert,document.setup.sourceToMold,document.recipe.hardware);
    var holes=materia.tooling.casting.InsertFixturePlanner.availableHoles(insert);
    check(holes.filter(hole->materia.tooling.casting.InsertFixturePlanner.select(hole,stock,face,document.recipe.hardware)!=null).length>0,
     "Every source insert has its own usable fixing thread: "+kind+"/"+insert.id);
    switch insert.origin {
     case Fabricated(_):
      check(switch insert.anchorage {case Bolted(_,anchors):anchors.length>=2;default:false;},
       "Made inserts use the shared demand-sized bolted anchor rule: "+kind+"/"+insert.id);
     case Catalogue(_):
    }
   }
   if(kind==Column) {
    var rows=foundation.insertLocations(),site=rows.filter(row->row.id=="liftAdditional1")[0];
    var neighbours=rows.filter(row->row.id!=site.id),resin=new machinekit.structural.MineralCasting(FoundationLayout.bodyValues("column")).geometry();
    var cover=switch site.part.insertSpec().anchorage {case Bolted(rules,_):rules.minimumCoverMm;default:throw "Missing lifting anchors";};
    var bad:machinekit.structural.MineralFoundation.CastInsertPlacement={id:site.id,part:site.part,openings:site.openings,
     pose:F.fromRotationMatrix(site.pose.x,54,site.pose.z,F.toRotationMatrix(site.pose))};
    var message="";
    try machinekit.hobbing.HobberFoundationSizing.checkBlank(bad,neighbours,resin,FoundationLayout.dimensions("column"),cover)
     catch(error:Dynamic)message=Std.string(error);
    try machinekit.hobbing.HobberFoundationSizing.checkBlank(site,neighbours,resin,FoundationLayout.dimensions("column"),cover)
     catch(error:Dynamic){resin.close();throw error;}
    resin.close();
    check(message.indexOf("Lifting blank lacks casting cover to source anchor:")>=0 && message.indexOf("rail")>=0,
     "The former Y=54 layout is rejected against the real rail-anchor heads, not only the rail blanks");
    check(site.pose.y<54,"The nearest admitted lifting land preserves cover to neighbouring anchor hardware");
   }
   if(Sys.args().indexOf("--mold-check")>=0) {
    var planned=machinekit.tooling.CastingMoldCad.plan(document,HobberMoldSetup.options(document.input));
    for(fixture in planned.fixtures)check(fixture.fixings.length>0,
     "Every declared source fixture thread has a supported screw chase: "+fixture.insertId);
    var review=machinekit.tooling.CastingMoldCad.review(planned);
    HobberMoldChecks.run(planned,review);
    check(review.symmetricDifferenceMm3<1e-3,"Exact corrected hobber reconstruction: "+kind);
    for(rule in ["reconstruction","coverage","assembly-graph","demolding-graph"])
     check(review.validation.filter(row->row.rule=="tooling.cad."+rule && row.status=="pass").length==1,
      "Corrected hobber native "+rule+": "+kind);
   }
  }
 }
 static function main():Void {
  bodyIntent();Sys.println("Hobber source cavity intent passed");foundationIntent();
  Sys.println('HobberSourceTests: $assertions assertions passed');
 }
}
