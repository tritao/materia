package machinekit.motion;

import machinekit.assembly.MachineAssembly;
import machinekit.component.ComponentValues;
import machinekit.component.Solids;
import machinekit.motion.ErobActuatorData as D;

/**
 * An eRob 110 integrated actuator: fixed `housing` and turning `output` joined by one revolute joint `A` (named by
 * `joint`) about the hollow axis. The joint is continuous, as the flange turns without end stops, its effort limit
 * is the ratio's start-stop peak torque and its speed limit the rated output speed on a 48 V bus. The output
 * encoder is registered as the joint's feedback. Exposed connectors: `seat` (the housing's front face) and
 * `output` (the output mounting face); exposed ports: `power` (48 V DC) and `bus`.
 */
class ErobActuator extends MachineAssembly {
  public final housing:ErobActuatorPart;
  public final output:ErobActuatorPart;
  public final jointId:String;

  public function new(ratio:Int = 100, encoderBits:Int = 20, brake:Bool = true, joint:String = "A", ?overrides:ComponentValues) {
    super();
    jointId = joint;
    var values = overrides == null ? new ComponentValues() : overrides.copy();
    values.setToken("ratio", Std.string(ratio)).setToken("encoderBits", Std.string(encoderBits)).setBoolean("brake", brake);
    housing = new ErobActuatorPart(values.copy().setToken("variant", ErobActuatorPart.HOUSING));
    output = new ErobActuatorPart(values.copy().setToken("variant", ErobActuatorPart.OUTPUT));
    addComponent("housing", housing);
    addComponent("output", output);
    addMemberConnector("housing", "rotation", Solids.axial(0, 0, 0));
    var row = housing.ratioRow;
    addMateOnAxis(
      joint,
      Continuous,
      "housing",
      "rotation",
      "output",
      "axis",
      {x: 0, y: 1, z: 0},
      0,
      {
        lower: null,
        upper: null,
        velocity: housing.ratedSpeed(),
        effort: row.peakTorque,
        acceleration: null,
        assumptions: [{quantity: "acceleration limit", label: "not published; set by the controller profile"}]
      }
    );
    addEncoder(joint + "-encoder", joint, "housing");
    exposeConnector("seat", "housing", "seat");
    exposeConnector("output", "output", "face");
    exposePort("power", "housing", "power");
    exposePort("bus", "housing", "bus");
  }

  /** The actuator record for this joint at `volts` on the bus. */
  public function actuator(id:String, volts:Float = D.BUS_VOLTS):materia.assembly.AssemblyDefinition.AssemblyActuator return housing.actuator(id, jointId, volts, 1.0);
}
