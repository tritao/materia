package machinekit.motion;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import machinekit.transmission.ValueBasis;

typedef RotaryAxisField={name:String,value:Float,unit:String,basis:ValueBasis,reason:String};
/** RA-D8 machine-independent rotary-axis data. Every numeric input retains its own provenance. */
class RotaryAxisSpec {
  public final mountPart:String;public final workPart:String;
  public final drivePart:String;public final encoderPart:String;public final errorPart:String;
  final data:ComponentValues;
  public function new(data:ComponentValues,drivePart:String,encoderPart:String,errorPart:String,mountPart:String="housing",workPart:String="platen"){this.mountPart=mountPart;this.workPart=workPart;this.data=data.copy();this.drivePart=drivePart;this.encoderPart=encoderPart;this.errorPart=errorPart;
    for(row in all())if(!Math.isFinite(row.value)||row.value<0||StringTools.trim(row.reason)=="")throw 'invalid rotary value ${row.name}';}
  public function number(name:String):Float return data.number(name);
  public function basis(name:String):ValueBasis return switch data.token(name+"Basis") {case "Catalog":Catalog;case "Measured":Measured;case "Derived":Derived;case "Stated":Stated;case "Assumed":Assumed;default:throw "invalid rotary value basis";};
  public function reason(name:String):String return data.token(name+"Reason");
  public function all():Array<RotaryAxisField> return [for(row in defaults()) {name:row.name,value:number(row.name),unit:row.unit,basis:basis(row.name),reason:reason(row.name)}];
  public var continuousTorque(get,never):Float;function get_continuousTorque():Float return number("continuousTorque");
  public var peakTorque(get,never):Float;function get_peakTorque():Float return number("peakTorque");
  public var ratedSpeed(get,never):Float;function get_ratedSpeed():Float return number("ratedSpeed");
  public var maxSpeed(get,never):Float;function get_maxSpeed():Float return number("maxSpeed");
  public function signature():String return haxe.crypto.Sha256.encode(haxe.Json.stringify(report()));
  public function report():Array<{name:String,value:Float,unit:String,basis:String,reason:String}> return [for(row in all()){name:row.name,value:row.value,unit:row.unit,basis:Std.string(row.basis),reason:row.reason}];
  public function values():ComponentValues return data.copy();
  public static function parameters():Array<ComponentParameter>{var out:Array<ComponentParameter> = [];
    for(row in defaults()){out.push(P.quantity(row.name,row.value,row.unit));out.push(P.choice(row.name+"Basis",["Catalog","Measured","Assumed","Derived","Stated"],Std.string(row.basis)));out.push(P.text(row.name+"Reason",row.reason));}return out;}
  public static function defaults():Array<RotaryAxisField>{var arc=Math.PI/648000;
    var c="Published purchase data; source: machinekit/STRAIN_WAVE_RESEARCH.md, Suppliers and prices. Confirm unit certificate.";
    return [
      {name:"faceDiameter",value:200,unit:"mm",basis:Catalog,reason:c},{name:"height",value:210,unit:"mm",basis:Catalog,reason:c},
      {name:"bore",value:40,unit:"mm",basis:Catalog,reason:c},{name:"mass",value:52,unit:"kg",basis:Catalog,reason:c},
      {name:"continuousTorque",value:54.9,unit:"N m",basis:Catalog,reason:c},{name:"peakTorque",value:223,unit:"N m",basis:Catalog,reason:c},
      {name:"continuousCurrent",value:3.43,unit:"A rms",basis:Catalog,reason:c},{name:"peakCurrent",value:17.9,unit:"A rms",basis:Catalog,reason:c},
      {name:"ratedSpeed",value:200*Math.PI/30,unit:"rad/s",basis:Catalog,reason:c},{name:"maxSpeed",value:234*Math.PI/30,unit:"rad/s",basis:Catalog,reason:c},
      {name:"loadCapacity",value:50,unit:"kg",basis:Catalog,reason:c},{name:"absoluteAccuracy",value:5*arc,unit:"rad",basis:Catalog,reason:c+" Published positioning accuracy; not NRRO or interpolation error."},
      {name:"repeatability",value:2*arc,unit:"rad",basis:Catalog,reason:c},{name:"clampPressure",value:600000,unit:"Pa",basis:Catalog,reason:c},
      {name:"driverAcLow",value:220,unit:"V rms",basis:Catalog,reason:c},{name:"driverAcHigh",value:380,unit:"V rms",basis:Catalog,reason:c},
      {name:"ingressRating",value:67,unit:"IP code",basis:Catalog,reason:c},{name:"naturalCooling",value:1,unit:"boolean",basis:Catalog,reason:c},
      {name:"housingDiameter",value:260,unit:"mm",basis:Assumed,reason:"Conservative clearance envelope; only the face diameter is published. Vendor outline required."},
      {name:"barrelDiameter",value:200,unit:"mm",basis:Assumed,reason:"Provisional cylindrical housing under the face; outline drawing required."},
      {name:"flangeThickness",value:15,unit:"mm",basis:Assumed,reason:"Provisional accessible flange outside the barrel; vendor drawing required."},
      {name:"faceThickness",value:20,unit:"mm",basis:Assumed,reason:"Envelope split for kinematic presentation, not an internal construction drawing."},
      {name:"rotatingMass",value:8,unit:"kg",basis:Assumed,reason:"Unpublished rotor/platen mass allocation within the catalog 52 kg total."},
      {name:"rotorInertia",value:0.04,unit:"kg m2",basis:Assumed,reason:"8 kg rotating allocation at an effective 71 mm radius; bench identification required."},
      {name:"radialRunout",value:0.002,unit:"mm TIR",basis:Assumed,reason:"Purchase target, unpublished; not inferred from positioning accuracy."},
      {name:"axialRunout",value:0.002,unit:"mm TIR",basis:Assumed,reason:"Purchase target, unpublished; retain earlier HB11 2 um TIR screen."},
      {name:"radialNrro",value:0.0003,unit:"mm peak",basis:Assumed,reason:"Unpublished target; non-repeatable sequence modeled explicitly, no vendor qualification."},
      {name:"axialNrro",value:0.0003,unit:"mm peak",basis:Assumed,reason:"Unpublished target; non-repeatable sequence modeled explicitly."},
      {name:"interpolationError",value:0.5*arc,unit:"rad peak",basis:Assumed,reason:"Unpublished half-arcsecond target; assumed 4096-fold harmonic."},
      {name:"encoderCounts",value:8388608,unit:"counts/revolution",basis:Assumed,reason:"23-bit absolute target; encoder interface and actual resolution require confirmation."},
      {name:"coggingTorque",value:0.1,unit:"N m peak",basis:Assumed,reason:"Low-speed disturbance allowance; 48th harmonic, bench measurement required."},
      {name:"speedRipple",value:0.001,unit:"fraction peak",basis:Assumed,reason:"0.1 percent once-per-turn ripple at 10-20 rpm; measurement required."},
      {name:"sealDrag",value:0.5,unit:"N m mean",basis:Assumed,reason:"IP67 does not establish seal type; conservative contact-seal allowance."},
      {name:"sealDragRipple",value:0.1,unit:"N m peak",basis:Assumed,reason:"20 percent drag variation, twice per turn; not measured."},
      {name:"servoTorsionalStiffness",value:200000,unit:"N m/rad",basis:Assumed,reason:"Quasistatic closed-loop disturbance rejection used only by explicit seal/cogging error sources."},
      {name:"momentStiffness",value:1000000,unit:"N m/rad",basis:Assumed,reason:"Retain former HB11 moment-stiffness screen; purchased bearing type is not claimed."},
      {name:"radialStiffness",value:100000,unit:"N/mm",basis:Assumed,reason:"Unpublished bearing/housing target; conservative static compliance screen."},
      {name:"axialStiffness",value:100000,unit:"N/mm",basis:Assumed,reason:"Unpublished bearing/housing target; axial load envelope must be stated separately."},
      {name:"pivotBelowFace",value:60,unit:"mm",basis:Assumed,reason:"Unpublished bearing centre, 60 mm below table face; sensitivity required."},
      {name:"torqueConstant",value:54.9/3.43,unit:"N m/A rms",basis:Assumed,reason:"Continuous torque/current quotient only; peak quotient differs, not a measured motor constant."},
      {name:"backEmfConstant",value:54.9/3.43/Math.sqrt(3),unit:"V/(rad/s)",basis:Assumed,reason:"Power-consistent sinusoidal motor convention derived from assumed Kt."},
      {name:"phaseResistance",value:0.3,unit:"ohm",basis:Assumed,reason:"Unpublished winding resistance, retained simulation reference."},
      {name:"clampHoldingTorque",value:300,unit:"N m",basis:Assumed,reason:"Procurement target above peak motor torque; air pressure alone does not establish holding torque."},
      {name:"thermalExpansion",value:12e-6,unit:"1/K",basis:Assumed,reason:"Effective steel-rich axial expansion; housing/material stack unknown, HB14 thermal model input."},
      {name:"heatAt15Rpm",value:10,unit:"W",basis:Assumed,reason:"Lumped low-speed winding/iron/drive heat allowance; seal power is separate; duty measurement required."},
      {name:"thermalResistance",value:0.2,unit:"K/W",basis:Assumed,reason:"Natural-cooling lumped thermal estimate, requires duty/temperature measurement."},
      {name:"thermalCapacity",value:23400,unit:"J/K",basis:Assumed,reason:"52 kg x 450 J/kg/K effective heat capacity, mixed materials unconfirmed."},
      {name:"facePcd",value:160,unit:"mm",basis:Assumed,reason:"Provisional six M8 holes on this PCD, clocked 30 degrees; vendor drawing required."},
      {name:"centeringDiameter",value:100,unit:"mm",basis:Assumed,reason:"Provisional 3 mm-deep female face pilot with matching mandrel spigot; vendor drawing required."},
      {name:"flangePcd",value:230,unit:"mm",basis:Assumed,reason:"Provisional six M10 flange holes, 30 degree clocking, 15 mm flange; vendor drawing required."},
      {name:"driverAcTolerance",value:0.1,unit:"fraction",basis:Assumed,reason:"380 VAC +/-10 percent includes site's 400 VAC; actual driver approval required."}
    ];}
}
