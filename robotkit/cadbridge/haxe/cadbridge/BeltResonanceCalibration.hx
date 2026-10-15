package cadbridge;

import haxe.crypto.Sha256;
import haxeon.wire.JsonWire;
import machinekit.assembly.MachineAssembly;
import machinekit.transmission.TimingBelt;
import materia.assembly.BeltCalibration;
import materia.assembly.BeltCalibration.BeltCalibrations;
import materia.assembly.BeltCalibration.ResonanceMeasurement;
import robotkit.model.DriveLoads;
import robotkit.model.RobotModel;

/** Reviewable, stale-checked update. One measurement identifies one common rigidity correction for an explicit belt group;
 * it cannot identify the individual CoreXY belts. Mass/inertia and mode attribution remain model inputs. */
class BeltResonanceCalibration {
  public final beforeFrequency:Float;
  public final afterFrequency:Float;
  public final rigidityFactor:Float;
  public final fitted:RobotModel;
  public var values(get,never):Map<String,BeltCalibration>;
  final updates:Map<String,BeltCalibration>;
  final original:MachineAssembly;
  final originalModel:RobotModel;
  final fingerprint:String;
  final modelHash:String;

  static function identity(assembly:MachineAssembly):String return Sha256.encode(JsonWire.encode(assembly.describe()));

  public function new(assembly:MachineAssembly,model:RobotModel,belts:Array<String>,measurement:ResonanceMeasurement,
      pose:Map<String,Float>) {
    original=assembly; originalModel=model; fingerprint=identity(assembly);
    modelHash=Sha256.encode(robotkit.model.RobotModelCodec.encode(model).toString());
    if (belts.length == 0 || model.elasticNetworks.length>0) throw "Calibration needs a nonempty scalar belt group without elastic networks";
    var axis=[for (joint in model.joints) if (joint.id == measurement.axis) joint];
    if (axis.length!=1 || axis[0].type!=robotkit.model.JointType.Prismatic) throw "Calibration needs a linear model axis";
    function position(id:String):Float {
      var value=pose.get(id), found=false;
      if (value == null || !Math.isFinite(value)) throw 'Missing physical calibration coordinate "$id"';
      for (joint in model.joints) if (joint.id == id) {
        found=true;
        if (value<joint.limits.lower || value>joint.limits.upper) throw 'Calibration pose exceeds "$id" travel';
      }
      if (!found) throw 'Unknown calibration coordinate "$id"';
      return value;
    }
    position(measurement.axis);
    var selected=new Map<String,String>(), parts=new Map<String,TimingBelt>();
    for (belt in belts) {
      if (parts.exists(belt)) throw "Calibration belt group contains duplicates";
      for (member in assembly.components()) if (member.id == belt && Std.isOfType(member.component,TimingBelt))
        parts.set(belt,cast member.component);
      if (!parts.exists(belt)) throw 'Unknown local timing belt "$belt"';
      for (coupling in model.couplings) {
        var relation=assembly.transmissionFor(coupling.id);
        if (relation != null) switch relation.source {
          case TimingBelt(id,_) if (id == belt):
            if (coupling.stiffnessLaw == null || relation.stiffness != null || relation.dampingRatio != null)
              throw "Measured belt group cannot contain static or overridden stiffness/damping";
            selected.set(coupling.id,belt);
          case _:
        }
      }
    }
    // Every spring in this connected mechanism must scale together; otherwise f^2 is not proportional to the fitted EA.
    var connected=[measurement.axis], changed=true;
    while (changed) {
      changed=false;
      for (c in model.couplings) if (connected.indexOf(c.leader)>=0 || connected.indexOf(c.follower)>=0) {
        for (id in [c.leader,c.follower]) if (connected.indexOf(id)<0) { connected.push(id); changed=true; }
      }
    }
    var count=0;
    for (c in model.couplings) if (c.stiffness>0 && connected.indexOf(c.leader)>=0) {
      if (!selected.exists(c.id)) throw "A single resonance cannot identify a partial group of compliant transmissions";
      count++;
      var law=c.stiffnessLaw;
      if (law == null) throw "Measured belt has no position-dependent law";
      for (spring in law.springs) for (coordinate in spring.coordinates) position(coordinate.joint);
    }
    if (count == 0) throw "Selected belts do not support the measured axis";
    for (id in selected.keys()) {
      var c=[for (c in model.couplings) if (c.id == id) c][0];
      if (connected.indexOf(c.leader)<0) throw "Calibration group includes an unrelated mechanism";
    }
    var load=DriveLoads.atPosition(model,position,null,[measurement.axis])[0];
    if (!(load.stiffness>0 && load.mass>0)) throw "Measured axis has no modelled elastic mode";
    beforeFrequency=Math.sqrt(load.stiffness/load.mass)/(2*Math.PI);
    rigidityFactor=Math.pow(measurement.frequencyHz/beforeFrequency,2);
    updates=new Map<String,BeltCalibration>();
    for (belt in belts) {
      var reference=parts.get(belt).cordRigidityPerWidth(), found=false;
      for (id in selected.keys()) if (selected.get(id) == belt) {
        var relation=assembly.transmissionFor(id);
        if (relation == null) throw "Measured coupling lost its transmission";
        var old=relation.calibration;
        var current=old == null ? parts.get(belt).cordRigidityPerWidth() : old.rigidityPerWidth;
        if (found && current!=reference) throw "One physical belt has inconsistent previous calibration";
        reference=current; found=true;
      }
      if (!found) throw 'Selected belt "$belt" has no elastic motion relation';
      var value:BeltCalibration={rigidityPerWidth:reference*rigidityFactor,dampingRatio:measurement.dampingRatio,
        axis:measurement.axis,pose:[for (id in pose.keys()) {joint:id,value:position(id)}],frequencyHz:measurement.frequencyHz,
        massKg:load.mass,spectrumSource:measurement.spectrumSource,decaySource:measurement.decaySource,
        fitError:measurement.fitError,method:BeltCalibrations.METHOD,decayStart:measurement.decayStart,decayEnd:measurement.decayEnd};
      value.pose.sort((a,b)->Reflect.compare(a.joint,b.joint));
      BeltCalibrations.validate(value); updates.set(belt,value);
    }
    fitted=robotkit.model.RobotModelCodec.decode(robotkit.model.RobotModelCodec.encode(model));
    for (c in fitted.couplings) if (selected.exists(c.id)) {
      var law=c.stiffnessLaw;
      if (law == null) throw "Measured belt has no position-dependent law";
      c.setStiffnessLaw(law.scaled(1.0/rigidityFactor,_ -> 1.0,_ -> 0.0),fitted.joints);
      c.dampingRatio=measurement.dampingRatio;
      c.calibration=BeltCalibrations.copy(updates.get(selected.get(c.id)),id -> id);
      var removed=[for (a in c.assumptions) if (a.quantity == "stiffness" || a.quantity == "dampingRatio") a.label];
      c.assumptions=[for (a in c.assumptions) if (a.quantity!="stiffness" && a.quantity!="dampingRatio") a];
      c.assumed=[for (label in c.assumed) if (removed.indexOf(label)<0) label];
    }
    var result=DriveLoads.atPosition(fitted,position,null,[measurement.axis])[0];
    afterFrequency=Math.sqrt(result.stiffness/result.mass)/(2*Math.PI);
    if (Math.abs(afterFrequency/measurement.frequencyHz-1)>1e-8) throw "Belt model does not satisfy the common-rigidity scaling assumption";
  }

  function get_values():Map<String,BeltCalibration> {
    var result=new Map<String,BeltCalibration>();
    for (id in updates.keys()) result.set(id,BeltCalibrations.copy(updates.get(id),joint -> joint));
    return result;
  }

  /** Standalone overlay also works for source-defined printers with code-only, non-rebuildable CAD members. */
  public function encode():String {
    var ids=[for (id in updates.keys()) id]; ids.sort(Reflect.compare);
    var data:materia.assembly.BeltCalibration.BeltCalibrationSet={schemaVersion:1,assemblyHash:fingerprint,modelHash:modelHash,
      belts:[for (id in ids) {belt:id,calibration:BeltCalibrations.copy(updates.get(id),joint -> joint)}]};
    return JsonWire.encode(data);
  }

  /** Replay only against the assembly AND physical model used for the fit, including its mass assumptions. */
  public static function applySaved(assembly:MachineAssembly,model:RobotModel,text:String):Void {
    var data:materia.assembly.BeltCalibration.BeltCalibrationSet=JsonWire.decode(text);
    if (data.schemaVersion!=1 || data.assemblyHash!=identity(assembly) ||
        data.modelHash!=Sha256.encode(robotkit.model.RobotModelCodec.encode(model).toString()))
      throw "Saved calibration does not match this source assembly and physical model";
    var updates=new Map<String,BeltCalibration>();
    for (entry in data.belts) {
      if (updates.exists(entry.belt)) throw "Duplicate belt in saved calibration";
      BeltCalibrations.validate(entry.calibration);
      updates.set(entry.belt,BeltCalibrations.copy(entry.calibration,id -> id));
    }
    assembly.calibrateBelts(updates);
  }

  /** Explicit local model update, with no firmware writes. */
  public function apply():Void {
    if (identity(original)!=fingerprint ||
        Sha256.encode(robotkit.model.RobotModelCodec.encode(originalModel).toString())!=modelHash)
      throw "Assembly or physical model changed after resonance fitting; rebuild the calibration proposal";
    original.calibrateBelts(updates);
  }
}
