import cadbridge.AssemblyPhysicalPartView;
import cadbridge.AssemblySimulationBridge.AssemblyPhysicalPart;

/** Analytic unit/override checks for the free-stock mass data used by physical tending. */
class TendingPhysicalMassChecks {
  static function near(actual:Float,expected:Float):Void {
    if(!Math.isFinite(actual) || Math.abs(actual-expected)>1e-12)throw 'Mass-property mismatch: $actual versus $expected';
  }
  public static function main():Void {
    // A 60 x 60 x 20 mm solid; CAD tensor is centroidal at unit density, in mm^5.
    var part:AssemblyPhysicalPart={id:"blank",materialId:"aluminium",density:2700,volume:72000,
      centerOfMass:[0,0,10],inertia:[24000000,0,0,0,24000000,0,0,0,43200000]};
    var resolved=AssemblyPhysicalPartView.massProperties(part,0.001),tensor:robotkit.spatial.Inertia3=cast resolved.inertia;
    near(resolved.massKg,0.1944);near(resolved.centerOfMass.z,0.01);
    near(tensor.xx,0.0000648);near(tensor.yy,0.0000648);near(tensor.zz,0.00011664);
    part.physicalMassKg=0.3888;
    var declared=AssemblyPhysicalPartView.massProperties(part,0.001),scaled:robotkit.spatial.Inertia3=cast declared.inertia;
    near(declared.massKg,0.3888);near(scaled.xx,0.0001296);
    part.physicalInertiaKgM2=[0.0002,0,0,0,0.0003,0,0,0,0.0004];
    part.centerOfMass=[2,3,10];
    var explicit=AssemblyPhysicalPartView.massProperties(part,0.001),chosen:robotkit.spatial.Inertia3=cast explicit.inertia;
    near(chosen.xx,0.0002);near(chosen.zz,0.0004);near(explicit.centerOfMass.x,0.002);near(explicit.centerOfMass.y,0.003);
    var rejected=false;try AssemblyPhysicalPartView.massProperties(part,0) catch(_:Dynamic)rejected=true;
    if(!rejected)throw "Invalid length scale accepted";
    part.physicalMassKg=-1;rejected=false;try AssemblyPhysicalPartView.massProperties(part,0.001) catch(_:Dynamic)rejected=true;
    if(!rejected)throw "Invalid physical mass accepted";
    var report={passed:true,analyticBoxUnits:true,declaredMassPreserved:true,explicitSIInertiaPreserved:true,
      centerOfMassFramePreserved:true,invalidInputsRejected:true};
    sys.io.File.saveContent("build/electric-mass-properties-checks.json",haxe.Json.stringify(report,null,"  "));
    Sys.println(haxe.Json.stringify(report));
  }
}
