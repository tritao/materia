import machinekit.motion.*;
import machinekit.hobbing.*;
import machinekit.component.*;
import machinekit.assembly.MachineAssembly;
import materia.assembly.AssemblyFrames as F;
import stockkit.hobbing.*;

/** Interface/persistence, services and conditional analytic pitch screening; not a full HB11 cut. */
class HobberPurchasedTableChecks {
  static var count=0;
  static function check(ok:Bool,label:String):Void{count++;if(!ok)throw label;}
  static function near(a:Float,b:Float,tol:Float,label:String):Void check(Math.abs(a-b)<=tol,'$label: $a / $b');
  public static function screen(axis:PrecisionRotaryAxis,s:gearkit.GearSpec):Dynamic {
    var data=axis.specification(),r=s.pitchRadius(),angular:Array<{id:String,amplitude:Float,order:Float}> = [
      {id:"absoluteAccuracy",amplitude:data.number("absoluteAccuracy"),order:4},
      {id:"interpolationError",amplitude:data.number("interpolationError"),order:4096},
      {id:"speedRipple",amplitude:data.number("speedRipple"),order:1},
      {id:"coggingTorque",amplitude:data.number("coggingTorque")/data.number("servoTorsionalStiffness"),order:48},
      {id:"sealDragRipple",amplitude:data.number("sealDragRipple")/data.number("servoTorsionalStiffness"),order:2}];
    var sum=0.0,pitch=0.0;for(row in angular){sum+=row.amplitude;pitch+=2*r*row.amplitude*Math.abs(Math.sin(row.order*Math.PI/s.teeth))*1000;}
    var radial=data.number("radialRunout")+2*16.989670571379065/data.number("radialStiffness");
    var rows:Array<Dynamic> = [],worst=1;
    for(pair in [{key:"Fp",value:2*r*sum*1000+radial*1000},{key:"fp",value:pitch+radial*1000*Math.sin(Math.PI/s.teeth)},{key:"Fr",value:radial*1000}]){
      var grade=gearkit.Iso1328.grade(s,pair.key,pair.value);if(grade==null)worst=12;else worst=Std.int(Math.max(worst,grade));
      rows.push({parameter:pair.key,boundUm:pair.value,extrapolatedGrade:grade,class6LimitUm:gearkit.Iso1328.tolerance(s,pair.key,6)});
    }
    var available=gearkit.Iso1328.tolerance(s,"Fp",6)/(2*r*1000)-sum+data.number("speedRipple")-radial/(2*r);
    return {scope:"Conditional analytic table-only pitch/runout bound, not a cut-derived or overall gear grade. Harmonics/phases/compliance assumed. Full source/combined HB11 cut remains open.",
      conditionalWorstPitchGrade:worst>11?null:worst,isoClass:null,outsideIsoScope:gearkit.Iso1328.outsideScope(s),rows:rows,
      harmonicPhaseBounds:angular,sealMeanPhaseOffsetRad:data.number("sealDrag")/data.number("servoTorsionalStiffness"),
      maximumOncePerTurnSpeedRippleFractionForClass6Fp:Math.max(0,available),
      forceBasis:"HB10 measured radial gash-window peak 16.989670571 N; not tangential force or instantaneous peaks",fields:data.report()};
  }
  public static function run():Void {
    var bought=new PurchasedRotaryTable(),built=new RotaryTable(new TorqueMotor(new ComponentValues().setNumber("torqueConstant",2).setNumber("backEmfConstant",2/Math.sqrt(3))),new CrossedRollerBearing(new ComponentValues().setNumber("outside",220).setNumber("bore",180).setNumber("height",20)));
    var choices:Array<PrecisionRotaryAxis> = [bought,built],reports:Array<Dynamic> = [];
    for(axis in choices){var assembly=axis.assembly(),spec=axis.specification(),restored=MachineAssembly.decode(assembly.encode(),MachineKitComponents.standard());
      check(restored.components().length==assembly.components().length,"rotary form persists its mechanical occurrences");
      var def=assembly.definition();check(def.joints.filter(j->j.id=="C").length==1,"rotary form has one C joint");
      check(assembly.components().filter(m->m.id==spec.drivePart)[0].component.motorDrive()!=null,"rotary form drive resolves");
      check(Std.isOfType(assembly.components().filter(m->m.id==spec.encoderPart)[0].component,EncoderPart),"rotary form encoder resolves");
      for(row in spec.all())check(row.reason!=""&&row.basis!=null,"every rotary field retains provenance");
      var machine=new FlexsplineHobber(null,12,false,axis),poses=machine.solvedPoses();
      var face=F.compose(poses.get("table/platen"),F.translation(0,0,assembly.components().filter(m->m.id=="platen")[0].component.parameterNumber(axis==bought?"faceThickness":"height")));
      near(face.z,865,1e-7,"both table faces at floor865");near(machine.workOrigin.z,1000,1e-7,"both work bands at floor1000");
      reports.push({form:axis==bought?"purchased":"built",prediction:screen(axis,machine.spec)});
    }
    near(bought.massProperties().mass,52,1e-9,"catalogue unit mass shared across physical envelopes");
    var lines=bought.billOfMaterials().lines();check(lines.length==1&&lines[0].quantity==1,"one complete purchased table BOM line");
    var measured=new PurchasedRotaryTable(new ComponentValues().setNumber("radialRunout",0.001).setToken("radialRunoutBasis","Measured").setToken("radialRunoutReason","Explicit bench test fixture"));
    var saved=MachineAssembly.decode(measured.encode(),MachineKitComponents.standard()),part:PurchasedRotaryPart=cast saved.components().filter(m->m.id=="housing")[0].component;
    near(part.axis.number("radialRunout"),0.001,0,"measured value persists");check(part.axis.basis("radialRunout")==machinekit.transmission.ValueBasis.Measured,"Measured basis persists");
    var m=new FlexsplineHobber(),cache=new machinekit.assembly.PosedParts(),poses=m.solvedPoses();
    var raw=cadkit.modeling.Part.cylinderSpan(19.99,-590,FoundationLayout.TABLE_FACE+20),cylinder=machinekit.assembly.PosedParts.moved(raw,FlexsplineHobber.MACHINE_FROM_WORK);raw.close();
    try{for(member in m.components())if(StringTools.startsWith(member.id,"table/")||member.id=="bed/casting"||member.id=="bed/tablePlate"||member.id=="mandrel"){
      var shape=cache.posed(member.component,poses.get(member.id)),v=machinekit.assembly.PosedParts.sharedVolume(shape,cylinder);shape.close();check(v<0.001,'40 mm union access blocked by ${member.id}');
    }}catch(e:Dynamic){cylinder.close();cache.close();throw e;}cylinder.close();cache.close();
    var ring=new RingCompliance(m.spec.rootRadius()-0.3,0.6,m.spec.faceWidth,210000,1000,"explicit test support"),sync=new HobbingErrorSource("sync","test","zero test sync",(t,a)->a);
    var errors=FlexsplineHobberErrors.sources(m,t->16.989670571,ring,sync),seal=errors.filter(e->e.id=="table-seal-drag")[0];
    check(seal!=null,"explicit seal-drag source present");var q=new HobbingAxes(46,0,4,0,0,0);
    near(seal.apply(0,q).c,-0.6/200000,1e-15,"seal mean plus ripple uses torsional stiffness");
    check(errors.length==21,"all bought-table sources feed HB11");
    Sys.println("PURCHASED_TABLE_PASS "+haxe.Json.stringify({assertions:count,purchaseFields:bought.specification().report(),forms:reports,geometryInterface:"provisional until vendor drawing; 6 bar air and encoder/driver services declared, no hydraulic/clamp solver",unionClearDiameterMm:39.98}));
  }
}
