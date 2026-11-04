package machinekit.hobbing;

import machinekit.assembly.MachineAssembly;
import machinekit.component.*;
import machinekit.motion.*;
import machinekit.standard.DeepGrooveBearing;
import machinekit.standard.SocketHeadCapScrew;
import materia.assembly.AssemblyDefinition.AssemblyJointType;

/** B spindle cartridge driven by a standard flanged servo through a coupling.
    Coordinates are along the spindle from the housing rear datum (z = 0), the tool face at 180:
    motor face at -70 on a bell adapter, coupling -55..-10, floating rear 6205 at 0..15, opposed
    angular-contact pair 129..163, nose 170..180. The servo's own absolute encoder is the B master
    (electronic-gearbox) encoder, drawn as a shaft-encoder envelope on the motor's rear. The arbor's
    flange is held by four M4 screws on a 32 mm PCD (no drawbar); two pins carry torque.
    The coupling is a sleeve envelope standing in for a balanced torsionally stiff
    bellows/disc coupling. Fits, preload, balance and sealing remain manufacturing design. */
class HobSpindle extends MachineAssembly {
  /** Assumed 1 kW 400 V servo in the envelope of the Leadshine ELM2H-1000LA80F being quoted. */
  public static inline final MOTOR = "GENERIC-HV-SERVO-1000W";
  public static inline final MOTOR_FACE = -70.0;
  public static inline final COUPLING_START = -55.0;
  public static inline final COUPLING_LENGTH = 45.0;
  public static inline final SHAFT_STUB = 20.0;
  /** Seat of the front angular-contact bearing on the housing datum. */
  public static inline final FRONT_BEARING_Z = 146.0;
  public static inline final SHAFT_REAR = -30.0;
  public static inline final ARBOR_SCREW_PCD = 32.0;
  /** Cartridge speed rating, below the servo's 5000 rpm; the reference cut runs near 3000 rpm. */
  public static inline final MAX_RPM = 4000.0;
  public final motor:ServoMotor;
  public final coupling:ShaftCoupling;
  public final frontBearing:AngularContactBearing;
  public final rearBearing:AngularContactBearing;
  public final tailBearing:DeepGrooveBearing;
  public final arbor:HobArbor;
  public final encoder:ShaftEncoder;
  public function new(?motor:ServoMotor, ?arbor:HobArbor) {
    super();
    this.motor = motor == null ? ServoMotor.model(MOTOR) : motor;
    this.arbor = arbor == null ? new HobArbor() : arbor;
    var face = this.motor.rating;
    // The motor shaft must reach into its half of the coupling, and the pilot and body fit the adapter.
    if (MOTOR_FACE + face.shaftLength < COUPLING_START + 15 || MOTOR_FACE + face.shaftLength > SHAFT_REAR - 2
      || face.bodyDiameter > 80 || face.shaftDiameter > 19 || this.arbor.parameterNumber("outside") != 16
      || this.arbor.parameterNumber("bore") != 0) throw "spindle reference seats do not fit installed parts";
    coupling = new ShaftCoupling(face.shaftDiameter, SHAFT_STUB, 36, COUPLING_LENGTH);
    frontBearing = new AngularContactBearing();
    rearBearing = new AngularContactBearing();
    tailBearing = DeepGrooveBearing.metric("6205", false);
    encoder = new ShaftEncoder(this.motor.rating.encoderCounts, true);
    addComponent("housing", new SpindleHousing());
    attach("motorAdapter", new HobSpindlePart("motorAdapter"), 0);
    addComponent("motor", this.motor);
    // The servo bolts to the adapter's rear flange (the adapter is on the housing), not to the housing itself.
    addMemberConnector("motorAdapter", "motorSeat", Solids.axial(0, 0, MOTOR_FACE));
    addMate("motor-mount", Fixed, "motorAdapter", "motorSeat", "motor", "mountFace");
    attach("frontBearing", frontBearing, FRONT_BEARING_Z);
    addComponent("encoder", encoder);
    // The encoder sits on the motor's rear face.
    addMemberConnector("motor", "encoderSeat", materia.assembly.AssemblyFrames.alongY(0, 0, -face.bodyLength, 0, 0, -1));
    addMate("encoder-mount", Fixed, "motor", "encoderSeat", "encoder", "mount");
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
        effort: this.motor.rating.peakTorque,
        assumptions: [{quantity : "speed limit", label : "assumed 4000 rpm cartridge rating"}].concat(
          this.motor.ratingsBasis == Assumed ? [{quantity : "effort limit", label : "assumed peak servo torque"}] : []
        )
      }
    );
    // Rotating with the shaft: the coupling clamps the stub, and the motor's own shaft (which the servo's bearings carry) reaches into
    // the coupling's other half, so it is held by the coupling.
    addComponent("coupling", coupling);
    addMemberConnector("shaft", "couplingDatum", Solids.axial(0, 0, COUPLING_START));
    addMate("coupling-mount", Fixed, "shaft", "couplingDatum", "coupling", "sideA");
    addComponent("motorShaft", this.motor.shaft());
    addMemberConnector("coupling", "motorShaftDatum", Solids.axial(0, 0, MOTOR_FACE - COUPLING_START));
    addMate("motorShaft-mount", Fixed, "coupling", "motorShaftDatum", "motorShaft", "input");
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
    // Four M4 x 12 each for the gland (heads on its face at 186) and the adapter (heads at -8), on Ø72.
    for (i in 0...4) {
      var angle = Math.PI / 4 + i * Math.PI / 2, c = Math.cos(angle), s = Math.sin(angle);
      screw("glandScrew" + (i + 1), "M4", 12, 36 * c, 36 * s, 186, 1);
      screw("adapterScrew" + (i + 1), "M4", 12, 36 * c, 36 * s, -8, -1);
    }
    // The adapter's four M6 holes are on the 80-frame 90 mm PCD. The motor's own screws are
    // not modelled: the generic motor body is an envelope that hides the real recessed corners.
    var flange = this.motor.mounting();
    if (flange == null) flange = ServoMotor.assumedMounting(this.motor.rating.designation);
    for (hole in flange.holes) if (Math.abs(Math.sqrt(hole.x * hole.x + hole.y * hole.y) - 45) > 1e-6
      || Math.abs(Math.abs(hole.x) - Math.abs(hole.y)) > 1e-6) throw "servo flange holes do not match the motor adapter";
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
  /** A socket screw fixed to the housing, head bearing face at `z`, shank toward `direction` * -z. */
  function screw(id:String, size:String, length:Float, x:Float, y:Float, z:Float, direction:Float):Void {
    addComponent(id, SocketHeadCapScrew.metric(size, length));
    addMemberConnector("housing", id + "Seat", materia.assembly.AssemblyFrames.alongY(x, y, z, 0, 0, direction));
    addMate(id + "-mount", Fixed, "housing", id + "Seat", id, "head");
  }
  /** B speed limit: the lower of the servo's maximum and the cartridge rating. */
  public function maxSpeed():Float return Math.min(motor.rating.maxSpeed, MAX_RPM * 2 * Math.PI / 60);
  public function interfaceName():String return "ASSUMED-TF20";
  /** Direct coupling: joint torque equals motor torque. */
  public function jointTorque(motorTorque:Float):Float return motorTorque;
}
