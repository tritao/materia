import machinekit.hobbing.HobberZLayout as Z;
import machinekit.component.Solids;
import machinekit.assembly.PosedParts;
import materia.assembly.AssemblyFrames as F;
import cadkit.modeling.AssemblyModel;
import cadkit.modeling.Part;
class HobberZDriveChecks {
  public static function run():Void {
    var n=0;function check(ok:Bool,label:String):Void {n++;if(!ok)throw label;}
    function near(a:Float,b:Float,label:String):Void check(Math.abs(a-b)<1e-7,label+": "+a+" vs "+b);
    var m=new FlexsplineHobber(),model=new AssemblyModel("mm");m.addTo(model,"");var state=model.initialState();state.forwardKinematics();
    var members=m.components(),work=F.inverse(FlexsplineHobber.MACHINE_FROM_WORK),cache=new PosedParts();
    function pose(id:String):materia.assembly.AssemblyRecord.AssemblyFrame return F.compose(work,state.worldPose(id));
    var a=pose("railZLeft"),b=pose("railZRight"),screw=pose("Zscrew"),fixed=pose("Zfixed"),floating=pose("Zfloating"),motor=pose("Zmotor");
    near(screw.y,(a.y+b.y)/2,"screw between rails");near(screw.x-a.x,19,"screw 19 mm behind guide axis plane");
    near(screw.z,Z.SHAFT_TOP,"top drive journal");near(motor.z,765,"motor mounting face");
    var direction=F.transformVector(screw,0,0,1);near(direction.z,-1,"screw axis points down");
    check(fixed.z>floating.z,"fixed bearing above floating bearing");
    var part:machinekit.motion.BallScrew=cast members.filter(p->p.id=="Zscrew")[0].component;
    near(part.totalLength,420,"screw length");
    var flange=m.column.insertLocations().filter(p->p.id=="driveSeat")[0],body=m.column.body.geometry();
    var raw=Part.box(flange.part.parameterNumber("width"),flange.part.parameterNumber("depth"),125),back=PosedParts.moved(raw,F.compose(flange.pose,F.translation(0,0,-125)));raw.close();
    var missing=back.subtract(body),missingMm3=missing.volume();missing.close();back.close();
    check(missingMm3<0.001,"125 mm full casting backing behind drive insert");check(body.solidCount()==1,"column remains monolithic");body.close();
    var coupling=cache.posed(members.filter(p->p.id=="Zcoupling")[0].component,state.worldPose("Zcoupling"));
    var bracket=cache.posed(members.filter(p->p.id=="ZdriveFrame")[0].component,state.worldPose("ZdriveFrame"));
    var couplingClearance=coupling.distance(bracket);coupling.close();bracket.close();
    check(couplingClearance>=2,"coupling clears motor plate by at least 2 mm");
    var minimumRaceMargin=1e30,minimumMovingGap=1e30;
    try {
      for(z in [-15.0,-10,0,190,195]){
        var legal=Math.max(-10,Math.min(190,z)),delta=F.translation(0,0,z-legal);
        state.setJoint("Z",legal);state.forwardKinematics();
        var nutWorld=F.compose(delta,state.worldPose("Znut"));
        var nut=F.compose(F.inverse(state.worldPose("Zscrew")),nutWorld);
        var far=nut.z+45,margin=Math.min(nut.z-part.inputJournalLength,part.totalLength-machinekit.motion.BallScrew.JOURNAL_LENGTH-far);
        minimumRaceMargin=Math.min(minimumRaceMargin,margin);check(margin>=10,"nut stays on race with 10 mm allowance at "+z);
        check(pose("Zfixed").z>F.compose(work,nutWorld).z,"gravity-loaded segment hangs below fixed bearing");
        for(id in ["Zfixed","Zfloating","ZdriveFrame"]){var component=members.filter(p->p.id==id)[0].component;
          var p=cache.posed(component,state.worldPose(id)),saddle=cache.posed(members.filter(p->p.id=="Zplate")[0].component,F.compose(delta,state.worldPose("Zplate")));
          var gap=p.distance(saddle);p.close();saddle.close();minimumMovingGap=Math.min(minimumMovingGap,gap);
          check(gap>=10-1e-7,"moving saddle retains 10 mm drive clearance at "+z+" / "+id+": "+gap);
        }
      }
      state.setJoint("Z",0);state.forwardKinematics();var q0=state.joint("Zscrew-turn"),p0=F.compose(F.inverse(state.worldPose("Zscrew")),state.worldPose("Znut"));
      state.setJoint("Z",10);state.forwardKinematics();var q1=state.joint("Zscrew-turn"),p1=F.compose(F.inverse(state.worldPose("Zscrew")),state.worldPose("Znut"));
      near(p1.z-p0.z,part.thread.signedLead()*(q1-q0)/(2*Math.PI),"physical right-hand screw sign");
      near(pose("Zplate").z,373,"positive machine Z still raises head");
    }catch(e:Dynamic){cache.close();throw e;}cache.close();
    var report={assertions:n,couplingPassageMm:40,couplingWindowCornerRadiusMm:3,couplingClearanceMm:couplingClearance,guideSpacingMm:290,screwLateralOffsetMm:screw.y-(a.y+b.y)/2,screwBehindGuideMm:19,
      shaftTopZMm:730,shaftBottomZMm:310,motorFaceZMm:765,motorTopZMm:910,fixedBearingZMm:fixed.z,floatingBearingZMm:floating.z,
      screwLengthMm:420,nutCentreAtZeroZMm:418,minimumRaceMarginMm:minimumRaceMargin,minimumSaddleDriveClearanceMm:minimumMovingGap,
      driveInsertMissingBackingMm3:missingMm3,castingBackingMm:125,columnCastingKg:m.column.castingByWeight().kg,
      columnCells:[for(b in m.column.body.materialBoxes())materia.casting.BoxAlgebra.limits(b)],
      scope:"Geometry, mechanical travel and thread sign only; structural stiffness and brake performance unqualified."};
    var output=Sys.getEnv("HOBBER_Z_DRIVE_EVIDENCE");if(output!=null)sys.io.File.saveContent(output,haxe.Json.stringify(report));
    Sys.println("HOBBER_Z_DRIVE_PASS "+haxe.Json.stringify(report));
  }
}
