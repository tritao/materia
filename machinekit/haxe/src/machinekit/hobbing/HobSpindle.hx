package machinekit.hobbing;

import machinekit.assembly.MachineAssembly;
import machinekit.assembly.Transmission;
import machinekit.component.*;
import machinekit.motion.*;
import machinekit.standard.FinePitchNut;
import machinekit.transmission.TimingBelt;
import machinekit.transmission.TimingPulley;
import materia.assembly.AssemblyFrames as F;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyDefinition.AssemblyJointType;

/** Purchased D80 BT30 belt cartridge with a made hob arbor. The servo and its
 * 1:1 HTD5M belt remain on the separate cradle; the spindle encoder measures
 * the cartridge rotor. Unpublished packaging is identified in Bt30SpindleData. */
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
  /** Deliberate 4000 rpm operating cap, below catalogue 6000 rpm and the servo maximum. */
  public static inline final MAX_RPM = 4000.0;
  public final motor:ServoMotor;
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
    encoder = new ShaftEncoder(ENCODER_COUNTS, true, false, ENCODER_DIAMETER, ENCODER_LENGTH, ENCODER_BORE);
    motorEncoder = new ShaftEncoder(this.motor.rating.encoderCounts, true);
    spindlePulley = new TimingPulley(HTD5M, SPINDLE_TEETH, 20, PULLEY_WIDTH);
    motorPulley = new TimingPulley(HTD5M, MOTOR_TEETH, face.shaftDiameter, PULLEY_WIDTH);
    belt = TimingBelt.twoPulley(HTD5M, SPINDLE_TEETH, MOTOR_TEETH, c, BELT_WIDTH, BELT_CORD_RIGIDITY);
    addComponent("housing", new SpindleHousing());
    addComponent("shaft", new HobSpindlePart());
    addMemberConnector("shaft", "toolFace", Solids.axial(0, 0, 180));
    // Internal bearings and drawbar are included in the purchase; the journal
    // joint exposes its rotor without inventing a manufacturer bearing design.
    addMemberConnector("housing", "gaugeLine", Solids.axial(0, 0, Bt30SpindleData.GAUGE_Z));
    addMateOnAxis(
      "B",
      Continuous,
      "housing",
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
        assumptions: [{quantity : "speed limit", label : "4000 rpm operating cap (design choice), below catalogue 6000 rpm"}].concat(
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
    fixed("pullStud", new HobSpindlePart(new ComponentValues().setToken("role", "pullStud")), F.translation(0, 0, 180), "arbor");
    // The hob is clamped against the arbor shoulder by an M16x1 fine nut.
    // Nut thickness and wrench size are explicit project inputs, not vendor data.
    fixed("hobLocknut", new FinePitchNut(16, 1, 24, 8), F.translation(0, 0, 180 + this.arbor.hobEnd()), "arbor");
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
  /** B speed limit: the lower of the servo's maximum seen through the belt and the cartridge rating. */
  public function maxSpeed():Float return Math.min(motor.rating.maxSpeed * MOTOR_TEETH / SPINDLE_TEETH, MAX_RPM * 2 * Math.PI / 60);
  public function interfaceName():String return "BT30";
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
