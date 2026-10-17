import machinekit.motion.*;
import machinekit.hobbing.*;
import machinekit.component.*;
import machinekit.assembly.MachineAssembly;
import materia.assembly.AssemblyDefinition.AssemblyJointType;

class HobberPartTests {
  static var assertions=0;
  static function check(ok:Bool,label:String):Void {assertions++;if(!ok)throw label;}
  static function near(a:Float,b:Float,tolerance:Float,label:String):Void check(Math.abs(a-b)<=tolerance,'$label: $a vs $b');
  static function rejects(fn:Void->Void,label:String):Void {var failed=false;try fn()catch(_:Dynamic)failed=true;check(failed,label);}
  static function members():Array<MachineComponent>return [new TorqueMotor(),new AngularContactBearing(),new CrossedRollerBearing(),new RotaryEncoderRing(),
    new Hob(),new HobArbor(),new OutboardSupport(),new ExpandingMandrel(),new RotaryClamp(),new TablePlaten(),new TableHousing(),new SpindleHousing(),new SwivelHousing(),new MineralCasting(),new CastInsert()];
  public static function run():Void {
    var registry=MachineKitComponents.standard(),parts=members();
    for(p in parts){
      var type=p.componentType();if(type==null)throw "missing recipe";check(registry.has(type.id),'registered ${p.designation}');
      var rebuilt=type.create(p.values());
      check(type.key(p.values())==type.key(rebuilt.values()),'recipe roundtrip ${p.designation}');
      check(p.bom.quantity==1&&p.bom.material!=null&&p.bom.typeId==type.id,'BOM ${p.designation}');
      var errors=PrecisionErrors.of(p),copyErrors=PrecisionErrors.of(rebuilt);
      check(errors!=null&&copyErrors!=null&&errors.describe()==copyErrors.describe(),'error bounds roundtrip ${p.designation}');
    }
    var motor=new TorqueMotor(),actuator=motor.actuator("table","C",48,1);
    near(motor.jointTorque(7),7,0,"direct drive has no reduction");
    near(actuator.maxEffort,30,1e-12,"peak torque limits joint effort");
    near(actuator.maxRate,120*2*Math.PI/60,1e-12,"motor max speed limits joint speed");
    check(actuator.ratedTorque==10&&actuator.assumed!=null,"continuous and assumed ratings survive motor export");
    check(motor.count("poleCount")==24&&motor.coggingTorque==0.03,"poles and cogging carried by motor");
    var encoder=new RotaryEncoderRing();
    near(encoder.quantum(),2*Math.PI/8388608,1e-16,"23-bit encoder quantum");
    near(encoder.angularAccuracy,2*Math.PI/(180*3600),1e-16,"encoder accuracy radians");
    var table=new RotaryTable(),spindle=new HobSpindle(),head=new SwivelHead();
    var assemblies:Array<MachineAssembly> = [table,spindle,head];
    for(assembly in assemblies){
      assembly.validate();
      var saved=assembly.encode(),restored=MachineAssembly.decode(saved,registry);
      check(assembly.components().length==restored.components().length,"all assembly members survive save/load");
      var originalErrors=PrecisionErrors.inAssembly(assembly),restoredErrors=PrecisionErrors.inAssembly(restored);
      check(originalErrors.length>0&&originalErrors.length==restoredErrors.length,"error sources carried into assembly description");
      for(i in 0...originalErrors.length){
        check(originalErrors[i].part==restoredErrors[i].part,"error source remains on its physical part");
        near(originalErrors[i].error.bound,restoredErrors[i].error.bound,0,"error bound preserves units/value");
        check(originalErrors[i].error.basis==restoredErrors[i].error.basis,"error provenance preserved");
      }
      check(assembly.billOfMaterials().lines().length==restored.billOfMaterials().lines().length,"subassembly BOM survives");
    }
    var editedBearing=new CrossedRollerBearing(new ComponentValues().setNumber("radialRunout",0.007).setToken("errorsBasis","Stated"));
    var editedErrors=MachineAssembly.decode(new RotaryTable(null,editedBearing).encode(),registry);
    var changed=PrecisionErrors.inAssembly(editedErrors).filter(e->e.part=="bearing"&&e.error.quantity=="radialRunout");
    check(changed.length==1&&changed[0].error.bound==0.007&&changed[0].error.basis==Stated,"edited part error and provenance survive assembly description");
    var c=table.definition().joints.filter(j->j.id=="C")[0];
    check(c.type==Continuous&&c.limits.velocity==table.motor.maxSpeed&&c.limits.effort==table.motor.peakTorque,"C limits from installed motor");
    var faster=new TorqueMotor(new ComponentValues().setNumber("peakTorque",42).setNumber("maxSpeed",20));
    var edited=new RotaryTable(faster);
    check(edited.definition().joints.filter(j->j.id=="C")[0].limits.effort==42,"edited motor changes derived C limit");
    near(spindle.motor.continuousTorque*spindle.motor.ratedSpeed,1500,1e-8,"spindle continuous mechanical power");
    near(spindle.motor.maxSpeed,8000*2*Math.PI/60,1e-9,"spindle 8000 rpm limit");
    var winding=spindle.motor.electricalRating();if(winding==null)throw "spindle winding absent";
    var current=spindle.motor.continuousTorque/winding.torqueConstant;
    check(3*current*current*winding.phaseResistance<0.05*1500,"spindle assumed winding has coherent rated copper loss");
    check(spindle.frontBearing!=spindle.rearBearing,"spindle has two bearing instances");
    check(spindle.billOfMaterials().quantity(spindle.frontBearing.bom.partNumber)==2,"opposed bearings are two of the same BOM part");
    check(spindle.interfaceName()==spindle.arbor.interfaceName(),"HSK-E gauge interface identity");
    var a=head.definition().joints.filter(j->j.id=="A")[0];
    check(a.limits.lower==-Math.PI/4&&a.limits.upper==Math.PI/4,"swivel range");
    near(head.clamp.holdingTorque,100,0,"stationary clamp rating separate from drive effort");
    var hob=new Hob(),rebuilt:Hob=cast hob.componentType().create(hob.values());
    var original=hob.design.rakeFaceRegion(3);
    var copied=rebuilt.design.rakeFaceRegion(3);
    check(original.length==copied.length,"hob recipe regenerates rake region");
    for(i in 0...original.length)near(Math.abs(original[i].x-copied[i].x)+Math.abs(original[i].y-copied[i].y)+Math.abs(original[i].z-copied[i].z),0,1e-12,"recovered hob rake points");
    var derived=Hob.fromDesign(hob.design,hob.target);
    near(derived.design.leadAngle,hob.design.leadAngle,1e-14,"part built from HobDesign");
    for(catalog in [TorqueMotor.catalog(),AngularContactBearing.catalog(),CrossedRollerBearing.catalog(),RotaryEncoderRing.catalog()]){
      var designation=catalog.designations()[0],row=catalog.get(designation),metadata=catalog.metadata(designation);
      check(row.token("ratingsBasis")=="Assumed"&&row.token("errorsBasis")=="Assumed","reference catalogue row explicitly assumed");
      var fields=metadata.verifiedFields;
      check(fields!=null&&fields.length==0,"no unverified fields called catalogue facts");
    }
    rejects(()->{new RotaryTable(new TorqueMotor(new ComponentValues().setNumber("outside",200)));},"incompatible motor envelope rejected");
    rejects(()->{new TorqueMotor(new ComponentValues().setNumber("peakTorque",1));},"peak below continuous torque rejected");
    rejects(()->{new TorqueMotor(new ComponentValues().setNumber("maxSpeed",Math.NaN));},"nonfinite motor speed rejected");
    rejects(()->{new AngularContactBearing(new ComponentValues().setNumber("bore",100));},"bearing walls must exist");
    rejects(()->{new RotaryEncoderRing(new ComponentValues().setInteger("countsPerRevolution",0));},"zero encoder counts rejected");
    rejects(()->{new ExpandingMandrel(new ComponentValues().setNumber("clampingPressure",20000000));},"mandrel overpressure rejected");
    rejects(()->{new CrossedRollerBearing(new ComponentValues().setNumber("radialRunout",-1));},"negative error bounds rejected");
    rejects(()->{new TorqueMotor(new ComponentValues().setToken("ratingsBasis","Catalog"));},"catalogue claim requires source");
    var foundations:Array<MineralFoundation> = [new MineralCastBed(),new MineralCastColumn()];
    for(foundation in foundations){
      foundation.validate();check(!FoundationIntegrity.check(foundation).hasErrors(),"insert positions and face planes conform");
      var weight=foundation.castingByWeight(),shape=foundation.body.geometry(),mass=foundation.body.massProperties();
      near(weight.kg,shape.volume()*2400e-9,1e-8,"casting purchased by native volume/density mass");
      var tensor=mass.inertia;if(tensor==null)throw "casting inertia absent";
      check(tensor.xx>0&&tensor.yy>0&&tensor.zz>0,"casting inertia is volumetric");shape.close();
      check(foundation.billOfMaterials().lines().length>4,"casting and steel inserts separately itemised");
      check(foundation.body.bom.description.indexOf(" kg")>=0,"casting BOM shows purchasing weight");
      var bad=FoundationIntegrity.checkMounts(foundation,[{name:"bad rail mount",parent:"casting"}]);
      check(bad.hasErrors()&&bad.items[0].subject=="bad rail mount","bad precision mount named");
      check(!FoundationIntegrity.checkMounts(foundation,[{name:"rail mount",parent:"railLeft"}]).hasErrors(),"precision rail mount lands on steel");
      var poses=foundation.solvedPoses(),pose=poses.get("railLeft");if(pose==null)throw "missing rail insert";
      poses.set("railLeft",materia.assembly.AssemblyFrames.compose(materia.assembly.AssemblyFrames.translation(0,0,0.03),pose));
      check(FoundationIntegrity.check(foundation,poses).hasErrors(),"insert out of position tolerance rejected");
      var rebuilt=MachineAssembly.decode(foundation.encode(),registry);
      check(rebuilt.components().length==foundation.components().length,"foundation child parts saved separately");
    }
    var physical=materia.project.MaterialLibrary.require("mineral-cast").physical;
    check(physical.youngsModulusPa==35e9&&physical.thermalExpansionPerK==14e-6&&physical.propertiesAssumed==true,"assumed bulk properties explicit");
    check(physical.dampingLossFactor==0.02&&physical.thermalConductivityWPerMK==2,"damping and conductivity explicit");
    Sys.println('HobberPartTests: $assertions assertions passed');
  }
  static function geometry():Void {
    for(p in members()){
      var shape=p.geometry(Preview);
      try {
        check(shape.volume()>0&&Math.isFinite(shape.volume()),'finite solid volume ${p.designation}');
        check(shape.solidCount()==1,'single valid envelope ${p.designation}');
        var bounds=shape.shape.bounds();
        check(bounds.get_max().get_z()>bounds.get_min().get_z(),'nondegenerate envelope ${p.designation}');
        if(Std.isOfType(p,MineralCasting))
          near(p.massProperties().mass,shape.volume()*1e-9*2400,1e-9,"mineral cast uses declared reference density");
        Sys.println('${p.designation}: volume_mm3=${shape.volume()}');
      }catch(e:Dynamic){shape.close();throw e;}
      shape.close();
    }
    var assemblies:Array<MachineAssembly> = [new RotaryTable(),new HobSpindle(),new SwivelHead()];
    for(assembly in assemblies){
      var poses=assembly.solvedPoses(),members=assembly.components(),cache=new machinekit.assembly.PosedParts();
      var housing=members.filter(m->m.id=="housing")[0];
      try {
        for(m in members)if(m.id!="housing")near(cache.overlap(housing.component,poses.get("housing"),m.component,poses.get(m.id)),0,1e-6,'housing seat clears ${m.id}');
      }catch(e:Dynamic){cache.close();throw e;}
      cache.close();
    }
    Sys.println('Hobber geometry: $assertions assertions passed');
  }
  public static function main():Void {if(Sys.args().length>0&&Sys.args()[0]=="geometry")geometry();else run();}
}
