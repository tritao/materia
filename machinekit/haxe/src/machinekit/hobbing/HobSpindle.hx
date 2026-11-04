package machinekit.hobbing;

import machinekit.assembly.MachineAssembly;
import machinekit.assembly.Transmission;
import machinekit.component.*;
import machinekit.motion.*;
import machinekit.standard.DeepGrooveBearing;
import machinekit.standard.SocketHeadCapScrew;
import machinekit.transmission.TimingBelt;
import machinekit.transmission.TimingPulley;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyDefinition.AssemblyJointType;

/** B spindle cartridge driven by a servo beside it through a timing belt (FRAME_PLAN head redesign).
    Coordinates are along the spindle from the housing rear datum (z = 0), the tool face at 180: the
    belt-drive cartridge has the 19-bit absolute spindle encoder on its rear face (z 0..-14), the cartridge
    pulley behind it (z -16..-34) and the opposed angular-contact pair at 129..163, nose 170..180. The servo
    stands parallel to the cartridge above it (`centres()` from the spindle axis, shaft toward the rear), its flange on a
    milled cradle bolted to the spindle seat block (the flange bolts run in vertical slots, a jack screw sets the belt tension), and drives a
    pulley in the cartridge pulley's plane: 40T on the motor and on the spindle (HTD5M, 15 mm belt, 1:1: the reference cut's 2984 rpm stays
    inside the servo's 3000 rpm rated speed). The spindle encoder is the B master (electronic-gearbox) encoder;
    the motor keeps its own absolute encoder for the drive's loop. The arbor's flange is held by four M4 screws on a 32 mm PCD (no
    drawbar); two pins carry torque. Fits, preload, balance and sealing remain manufacturing design. */
class HobSpindle extends MachineAssembly {
  /**
   * The Leadshine ELM2H-1000LA80F being quoted (catalogue drawing: 80 flange, 103.9 body, connectors to LH 55 from the axis), with
   * the generic high-voltage winding assumed so a driver can be wired (`LeadshineServo.driven`).
   */
  public static inline final MOTOR = "ELM2H-1000LA80F";
  /**
   * Quarter turns of the servo about its own axis (the flange's 90 mm square pattern allows four): which side its connectors point. At
   * 0 they point up, away from the cradle, the saddle and the bed (the servo stands above the cartridge); the others would point across or
   * down at the seat block. Not `inline`, so that a change reaches every caller in an incremental build.
   */
  public static final MOTOR_CLOCKING = 0;
  /** Motor flange face, on the cradle wall's front face. */
  public static inline final MOTOR_FACE_Z = 1.0;
  /** Cartridge shaft: rear end of the stub, behind the pulley. */
  public static inline final SHAFT_REAR = -36.0;
  /** Pulley width and the belt's, the belt centred on the pulley (HTD5M, 15 mm). */
  public static inline final PULLEY_WIDTH = 18.0;
  public static inline final BELT_WIDTH = 15.0;
  /** The spindle encoder, on the housing rear face toward -z. */
  public static inline final ENCODER_LENGTH = 14.0;
  public static inline final ENCODER_DIAMETER = 80.0;
  /** Hollow bore: the Ø20 stub with 2 mm round it. */
  public static inline final ENCODER_BORE = 24.0;
  /** 19-bit absolute BiSS-C ring encoder on the rear shaft: assumed (class of a Ø52-80 ring encoder), no catalogue row yet. */
  public static inline final ENCODER_COUNTS = 524288.0;
  public static inline final MOTOR_TEETH = 40;
  public static inline final SPINDLE_TEETH = 40;
  /**
   * Nominal centre distance before the whole-tooth belt: the 80 flange clears the cradle foot (top 78) and the seat block (top 58) from above.
   * Two 40T HTD5M pulleys (pitch Ø63.66) on 125 mm centres take the catalogue 450-5M belt (90 teeth) exactly.
   */
  public static inline final NOMINAL_CENTRES = 125.0;
  /** The servo slides this far each way along the belt's line in the cradle's slotted flange holes: slack to fit the belt, travel to tension it. */
  public static inline final TENSION_TRAVEL = 3.0;
  /** Assumed mass of the 15 mm HTD5M belt per metre of length, kg/m (belt maker to confirm): converts a span's natural frequency to its tension. */
  public static inline final BELT_MASS_PER_M = 0.07;
  /** Assumed belt cord stiffness per mm of width (N) that `belt` is built with: HTD5M has no recorded family value. */
  public static inline final BELT_CORD_RIGIDITY = 5000.0;
  /**
   * The servo stands this far toward work -x of the cartridge's vertical (spindle +x), so its 90 mm flange clears the Y drive's motor and
   * bracket (work x from 35) on the swivel plate's top edge; the belt's centre line tilts by `asin(MOTOR_OFFSET / centres())`.
   */
  public static inline final MOTOR_OFFSET = 8.0;
  /** Seat of the front angular-contact bearing on the housing datum. */
  public static inline final FRONT_BEARING_Z = 146.0;
  public static inline final ARBOR_SCREW_PCD = 32.0;
  /** Cartridge speed rating, below the servo's 5000 rpm / ratio; the reference cut runs near 3000 rpm. */
  public static inline final MAX_RPM = 4000.0;
  public final motor:ServoMotor;
  public final frontBearing:AngularContactBearing;
  public final rearBearing:AngularContactBearing;
  public final tailBearing:DeepGrooveBearing;
  public final arbor:HobArbor;
  /** The spindle encoder: B master. */
  public final encoder:ShaftEncoder;
  /** The servo's own absolute encoder (its drive loop). */
  public final motorEncoder:ShaftEncoder;
  public final motorPulley:TimingPulley;
  public final spindlePulley:TimingPulley;
  public final belt:TimingBelt;
  /** Motor turns per spindle turn (the belt's reduction as the transmission solves it). */
  public final ratio:Float;
  static var centresCache:Null<Float>;

  /** Whole-tooth belt centre distance, nearest the nominal, mm. */
  public static function centres():Float {
    if (centresCache == null) {
      var nominal = NOMINAL_CENTRES, pitch = 5.0;
      var target = Math.round(TimingBelt.twoPulley(HTD5M, SPINDLE_TEETH, MOTOR_TEETH, nominal, BELT_WIDTH).length / pitch) * pitch;
      var lo = nominal - 5, hi = nominal + 5;
      for (_ in 0...40) {
        var mid = (lo + hi) / 2;
        if (TimingBelt.twoPulley(HTD5M, SPINDLE_TEETH, MOTOR_TEETH, mid, BELT_WIDTH).length < target) lo = mid;
        else hi = mid;
      }
      centresCache = (lo + hi) / 2;
    }
    return centresCache;
  }

  /** The servo axis' height above the cartridge axis (the tilted centre line's vertical part), mm. */
  public static function motorHeight():Float return Math.sqrt(centres() * centres() - MOTOR_OFFSET * MOTOR_OFFSET);

  /** Front face of both pulleys (toward the housing): the motor shaft's end is flush with the pulleys' rear face. */
  public static function pulleyFront(shaftLength:Float):Float return MOTOR_FACE_Z - shaftLength + PULLEY_WIDTH;

  public function new(?motor:ServoMotor, ?arbor:HobArbor, clocking:Int = MOTOR_CLOCKING) {
    super();
    this.motor = motor == null ? LeadshineServo.driven(MOTOR) : motor;
    this.arbor = arbor == null ? new HobArbor() : arbor;
    var face = this.motor.rating, c = centres(), h = motorHeight(), front = pulleyFront(face.shaftLength);
    // The motor shaft must carry its pulley and the cartridge stub reach past it.
    if (front > -ENCODER_LENGTH - 1 || front - PULLEY_WIDTH < SHAFT_REAR || face.bodyDiameter > 80 || face.shaftDiameter > 19
      || this.arbor.parameterNumber("outside") != 16 || this.arbor.parameterNumber("bore") != 0) throw "spindle reference seats do not fit installed parts";
    frontBearing = new AngularContactBearing();
    rearBearing = new AngularContactBearing();
    tailBearing = DeepGrooveBearing.metric("6205", false);
    encoder = new ShaftEncoder(ENCODER_COUNTS, true, false, ENCODER_DIAMETER, ENCODER_LENGTH, ENCODER_BORE);
    motorEncoder = new ShaftEncoder(this.motor.rating.encoderCounts, true);
    spindlePulley = new TimingPulley(HTD5M, SPINDLE_TEETH, 20, PULLEY_WIDTH);
    motorPulley = new TimingPulley(HTD5M, MOTOR_TEETH, face.shaftDiameter, PULLEY_WIDTH);
    belt = TimingBelt.twoPulley(HTD5M, SPINDLE_TEETH, MOTOR_TEETH, c, BELT_WIDTH, BELT_CORD_RIGIDITY);
    addComponent("housing", new SpindleHousing());
    attach("frontBearing", frontBearing, FRONT_BEARING_Z);
    addComponent("rearBearing", rearBearing);
    addMemberConnector("housing", "rearBearingSeat", materia.assembly.AssemblyFrames.alongY(0, 0, 146, 0, 0, -1));
    addMate("rearBearing-mount", Fixed, "housing", "rearBearingSeat", "rearBearing", "mount");
    addComponent("tailBearing", tailBearing);
    addMemberConnector("housing", "tailBearingSeat", Solids.axial(0, 0, 0));
    addMate("tailBearing-mount", Fixed, "housing", "tailBearingSeat", "tailBearing", "front");
    addComponent("shaft", new HobSpindlePart("shaft"));
    addMemberConnector("shaft", "toolFace", Solids.axial(0, 0, 180));
    // The shaft turns in the opposed angular-contact pair, so B joins the front bearing (fixed to the housing) to the shaft:
    // the pair's inner rings sit on the Ø25 journal, which is the load path into the housing.
    addMemberConnector("frontBearing", "gaugeLine", Solids.axial(0, 0, 180 - FRONT_BEARING_Z));
    addMateOnAxis(
      "B",
      Continuous,
      "frontBearing",
      "gaugeLine",
      "shaft",
      "toolFace",
      {x: 0, y: 1, z: 0},
      0,
      {
        lower: null,
        upper: null,
        velocity: maxSpeed(),
        effort: jointTorque(this.motor.rating.peakTorque),
        assumptions: [{quantity : "speed limit", label : "assumed 4000 rpm cartridge rating"}].concat(
          this.motor.ratingsBasis == Assumed ? [{quantity : "effort limit", label : "assumed peak servo torque"}] : []
        )
      }
    );
    // The spindle encoder stands on the housing rear face (stator), its housing toward -z, and reads the shaft through its bore.
    var back = F.fromRotationMatrix(0, 0, 0, [1.0, 0, 0, 0, -1, 0, 0, 0, -1]);
    fixed("encoder", encoder, back, "housing");
    // The cartridge pulley on the shaft stub behind the encoder; its rear face is 2 mm short of the stub's end.
    fixed("spindlePulley", spindlePulley, F.fromRotationMatrix(0, 0, front, [1.0, 0, 0, 0, -1, 0, 0, 0, -1]), "shaft");
    // The servo stands above the cartridge (spindle -y is up), shaft toward the rear, its flange face on the cradle wall.
    var spin = clocking * Math.PI / 2, cs = Math.cos(spin), sn = Math.sin(spin);
    var motorPose = F.compose(F.fromRotationMatrix(MOTOR_OFFSET, -h, MOTOR_FACE_Z, [1.0, 0, 0, 0, -1, 0, 0, 0, -1]),
      F.fromRotationMatrix(0, 0, 0, [cs, -sn, 0, sn, cs, 0, 0, 0, 1]));
    addComponent("motor", this.motor, motorPose);
    rotating("motorShaft", this.motor.shaft(), motorPose, "motor", "motor-turn", this.motor.rating.maxSpeed);
    var motorFrame = F.fromRotationMatrix(MOTOR_OFFSET, -h, front, [1.0, 0, 0, 0, -1, 0, 0, 0, -1]);
    fixed("motorPulley", motorPulley, motorFrame, "motorShaft");
    // The motor's own encoder sits on its rear face.
    fixed("motorEncoder", motorEncoder, F.compose(motorPose, F.fromRotationMatrix(0, 0, -face.bodyLength, [1.0, 0, 0, 0, -1, 0, 0, 0, -1])), "motor");
    // The belt's x runs from the cartridge pulley to the servo's, tilted from straight up by the offset; its z is toward the rear.
    var ux = MOTOR_OFFSET / c, uy = -h / c;
    fixed("belt", belt, F.fromRotationMatrix(0, 0, front - 1.5, [ux, uy, 0, uy, -ux, 0, 0, 0, -1]), "spindlePulley");
    addMemberConnector("spindlePulley", "beltPlane", Solids.axial(0, 0, 1.5));
    addMemberConnector("motorPulley", "beltPlane", Solids.axial(0, 0, 1.5));
    addBeltPath({
      belt: "belt",
      wraps: [
        {instanceId: "spindlePulley", connectorName: "beltPlane"},
        {instanceId: "motorPulley", connectorName: "beltPlane"}
      ]
    });
    ratio = addTransmission("hob-belt", "B", "motor-turn", Transmission.BeltReduction("belt", "spindlePulley", "motorPulley"), Same);
    addComponent("arbor", this.arbor);
    addMate("arbor-mount", Fixed, "shaft", "toolFace", "arbor", "gaugeLine");
    for (i in 0...4) {
      var angle = Math.PI / 4 + i * Math.PI / 2, r = ARBOR_SCREW_PCD / 2, id = "arborScrew" + (i + 1);
      addComponent(id, SocketHeadCapScrew.metric("M4", 10));
      addMemberConnector("arbor", id + "Seat", Solids.axial(r * Math.cos(angle), r * Math.sin(angle), 4));
      addMate(id + "-mount", Fixed, "arbor", id + "Seat", id, "head");
    }
    addComponent("bearingLocknut", new HobSpindlePart("bearingLocknut"));
    addMemberConnector("shaft", "bearingLocknutDatum", Solids.axial(0, 0, 0));
    addMate("bearingLocknut-mount", Fixed, "shaft", "bearingLocknutDatum", "bearingLocknut", "mount");
    attach("bearingRetainer", new HobSpindlePart("bearingRetainer"), 0);
    // Four M4 x 12 for the gland (heads on its face at 186), on Ø72.
    for (i in 0...4) {
      var angle = Math.PI / 4 + i * Math.PI / 2, co = Math.cos(angle), si = Math.sin(angle);
      screw("glandScrew" + (i + 1), "M4", 12, 36 * co, 36 * si, 186, 1);
    }
    // The servo's flange holes are on the 80-frame 90 mm PCD; the motor's own screws are not modelled (the generic motor body is an
    // envelope that hides the real recessed corners).
    var flange = this.motor.mounting();
    if (flange == null) flange = ServoMotor.assumedMounting(this.motor.rating.designation);
    for (hole in flange.holes) if (Math.abs(Math.sqrt(hole.x * hole.x + hole.y * hole.y) - 45) > 1e-6
      || Math.abs(Math.abs(hole.x) - Math.abs(hole.y)) > 1e-6) throw "servo flange holes do not match the cradle pattern";
    addEncoder("hob-angle", "B", "encoder");
    exposeConnector("mount", "housing", "mount");
    exposeConnector("gaugeLine", "arbor", "gaugeLine");
    exposeConnector("hobCentre", "arbor", "hobCentre");
  }
  function attach(id:String, part:MachineComponent, z:Float):Void {
    addComponent(id, part);
    addMemberConnector("housing", id + "Seat", Solids.axial(0, 0, z));
    addMate(id + "-mount", Fixed, "housing", id + "Seat", id, "mount");
  }
  /** `part` at `pose` (spindle frame), fixed to `parent`. */
  function fixed(id:String, part:MachineComponent, pose:AssemblyFrame, parent:String):Void {
    addComponent(id, part, pose);
    var positions = solvedPoses();
    addMemberConnector(parent, 'to-$id', F.compose(F.inverse(positions.get(parent)), pose));
    addMemberConnector(id, "seat", F.identity());
    addMate('attach-$id', Fixed, parent, 'to-$id', id, "seat");
  }
  /** `part` at `pose` turning about its own z against `parent` as joint `joint`. */
  function rotating(id:String, part:MachineComponent, pose:AssemblyFrame, parent:String, joint:String, speed:Float):Void {
    addComponent(id, part, pose);
    var positions = solvedPoses(), frame = F.translation(pose.x, pose.y, pose.z);
    addMemberConnector(parent, 'to-$id', F.compose(F.inverse(positions.get(parent)), frame));
    addMemberConnector(id, "seat", F.compose(F.inverse(pose), frame));
    addMateOnAxis(joint, Continuous, parent, 'to-$id', id, "seat", {x: 0, y: 0, z: 1}, 0, {lower: null, upper: null, velocity: speed, effort: null});
  }
  /** A socket screw fixed to the housing, head bearing face at `z`, shank toward `direction` * -z. */
  function screw(id:String, size:String, length:Float, x:Float, y:Float, z:Float, direction:Float):Void {
    addComponent(id, SocketHeadCapScrew.metric(size, length));
    addMemberConnector("housing", id + "Seat", materia.assembly.AssemblyFrames.alongY(x, y, z, 0, 0, direction));
    addMate(id + "-mount", Fixed, "housing", id + "Seat", id, "head");
  }
  /** B speed limit: the lower of the servo's maximum seen through the belt and the cartridge rating. */
  public function maxSpeed():Float return Math.min(motor.rating.maxSpeed * MOTOR_TEETH / SPINDLE_TEETH, MAX_RPM * 2 * Math.PI / 60);
  public function interfaceName():String return "ASSUMED-TF20";
  /**
   * Belt pretension per strand (N) the drive is set to: the slack side is kept at a quarter of the motor's peak tangential force
   * (assumed rule; belt maker to confirm), i.e. 0.75 of the peak force for the mean of the two spans.
   */
  public static function pretension(peakTorque:Float, motorPulleyPitchDiameter:Float):Float return 0.75 * peakTorque / (motorPulleyPitchDiameter / 2000);

  /** Natural frequency (Hz) of the free span (the two pulleys are equal, so the span is the centre distance) at tension `t` newtons, as a taut string; `BELT_MASS_PER_M` assumed. */
  public static function spanFrequency(t:Float):Float return Math.sqrt(t / BELT_MASS_PER_M) / (2 * centres() / 1000);

  /** Tension (N) of the span from the frequency read with a sonic meter. */
  public static function tensionAt(frequency:Float):Float return 4 * BELT_MASS_PER_M * Math.pow(centres() / 1000 * frequency, 2);

  /** Centre-distance increase (mm) that stretches the belt to tension `t`: two strands carry `t` over the belt length, cord stiffness `BELT_CORD_RIGIDITY` x width. */
  public static function tensioningTravel(t:Float, beltLength:Float):Float return t * beltLength / (2 * BELT_CORD_RIGIDITY * BELT_WIDTH);

  /** The belt reduces nothing at 1:1: joint torque is the motor torque times the spindle-to-motor tooth ratio (belt efficiency not applied). */
  public function jointTorque(motorTorque:Float):Float return motorTorque * SPINDLE_TEETH / MOTOR_TEETH;
}
