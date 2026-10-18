import machinekit.robotics.OffsetGripperFinger;
import machinekit.robotics.OffsetFingerSpec;
import machinekit.component.ComponentRegistry;
import machinekit.component.ComponentValues;
import machinekit.component.Bom;
import machinekit.assembly.MachineAssembly;
import machinekit.assembly.MachineAssemblyCodec;

class ManufacturingVariantChecks {
  static function check(value:Bool,label:String):Void {if(!value) throw label;}
  static function near(a:Float,b:Float):Void check(Math.abs(a-b)<0.000001,"Interface coordinates differ");
  public static function main():Void {
    var family=OffsetGripperFinger.family(),registry=new ComponentRegistry();registry.registerFamily(family);
    check(registry.familyForType(family.recipe.id)==family,"Family discoverability");
    check(machinekit.component.MachineKitComponents.defaultRegistry().familyForType(family.recipe.id)==family,"Built-in family registration");
    var bom=new Bom(),assembly=new MachineAssembly();
    for(side in ["left","right"]) {
      var values=new ComponentValues().setToken("side",side);
      var cnc=family.create("cnc-aluminium",values),sls=family.create("sls-pa12",values);
      check(values.get("variant")==null,"Factory mutated caller's inputs");
      check(cnc.bom.partNumber!=sls.bom.partNumber && cnc.geometryKey()!=sls.geometryKey(),"Variant identities alias");
      for(name in ["mount","contact"]) {
        var a=cnc.connector(name).frame,b=sls.connector(name).frame;
        near(a.x,b.x);near(a.y,b.y);near(a.z,b.z);near(a.qw,b.qw);
      }
      var a=cnc.geometry(),b=sls.geometry();
      check(a.valid() && b.valid() && a.solidCount()==1 && b.solidCount()==1,"Variant geometry invalid");
      check(b.volume()>a.volume(),"SLS variant must reinforce geometry, not only recolor it");
      check(cnc.massProperties().mass>sls.massProperties().mass,"Distinct material mass properties");
      a.close();b.close();
      var sleeves=new OffsetGripperFinger(side=="left","sls-pa12").compressionSleeves();
      check(sleeves.length==2,"Missing SLS compression hardware");
      for(sleeve in sleeves) {check(sleeve.valid() && sleeve.solidCount()==1,"Invalid sleeve");sleeve.close();}
      check(family.variant("cnc-aluminium").hardware().length==0 && family.variant("sls-pa12").hardware()[0].quantity==2,"Hardware not variant specific");
      var rejected=false;try {sls.setMaterial("aluminium 6061");} catch(_:Dynamic) {rejected=true;}
      check(rejected,"Material-only change bypassed manufacturing variant");
      check(family.variant("sls-pa12").status=="prototype" && family.variant("sls-pa12").pendingChecks().length>0,"Unverified process presented as validated");
      bom.addComponent(cnc);bom.addComponent(sls);assembly.addComponent(side+"-cnc",cnc);assembly.addComponent(side+"-sls",sls);
    }
    check(bom.lines().length==4,"Manufacturing BOM collapsed alternatives");
    for(line in bom.lines()) check(line.familyId==family.id && line.manufacturingVariant!=null && line.manufacturingProcess!=null,"BOM metadata lost");
    assembly.addBomItem(bom.lines()[0],2);
    var saved=MachineAssemblyCodec.encode(MachineAssemblyCodec.describe(assembly));
    var restored=new MachineAssembly();MachineAssemblyCodec.read(MachineAssemblyCodec.decode(saved),restored,registry);
    for(entry in restored.components()) {
      var original=assembly.components().filter(item -> item.id==entry.id)[0].component;
      check(entry.component.bom.partNumber==original.bom.partNumber,"Saved variant did not roundtrip");
      check(entry.component.bom.manufacturingVariant==original.bom.manufacturingVariant,"Saved variant metadata lost");
    }
    var restoredBom=restored.billOfMaterials();
    for(line in restoredBom.lines()) check(line.familyId==family.id && line.manufacturingVariant!=null,"Saved BOM-extra metadata lost");
    check(restoredBom.quantity(bom.lines()[0].partNumber)==3,"Saved BOM-extra quantity lost");
    var rejected=false;try {family.create("unknown");} catch(_:Dynamic) {rejected=true;}
    check(rejected,"Unknown variant accepted");
    Sys.println("Manufacturing family checks passed: shared interfaces, distinct geometry/material/hardware/BOM, material lock, registry and saved-assembly roundtrip");
  }
}
