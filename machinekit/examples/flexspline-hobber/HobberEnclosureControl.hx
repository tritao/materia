import cadkit.modeling.AssemblyState;
import machinekit.enclosure.SlidingDoor;
import processkit.guard.EnclosureGuard;
import processkit.guard.EnclosureGuard.GuardInput;
import processkit.guard.EnclosureGuard.GuardOutput;

/** Kinematic controller adapter. The host supplies real lock/standstill feedback and applies motion permissions. */
class HobberEnclosureControl {
  public static inline final JOINT = "enclosure/door/slide";
  public final guard:EnclosureGuard;
  final state:AssemblyState;
  final stroke:Float;
  final applySafety:GuardOutput->Void;

  public function new(state:AssemblyState, stroke:Float, applySafety:GuardOutput->Void,
      setupSpeedLimit:Float = 0.025, timeoutSeconds:Float = 10) {
    if(state==null||applySafety==null||!Math.isFinite(stroke)||stroke<=2*SlidingDoor.SWITCH_TOLERANCE_MM)
      throw "Hobber guard needs an assembly, door stroke and safety-output consumer";
    this.state=state;this.stroke=stroke;this.applySafety=applySafety;
    guard=new EnclosureGuard(setupSpeedLimit,timeoutSeconds);
  }

  public function update(input:GuardInput, dt:Float):GuardOutput {
    if(input==null)throw "Hobber guard needs sampled feedback";
    var position=state.joint(JOINT);
    if(!Math.isFinite(position)||position<0||position>stroke)throw "Door is outside mechanical travel";
    input.closed=position<=SlidingDoor.SWITCH_TOLERANCE_MM;
    input.open=position>=stroke-SlidingDoor.SWITCH_TOLERANCE_MM;
    var output=guard.update(input,dt);
    // Apply stop/enable outputs before moving the access guard.
    applySafety(output);
    var next=switch output.door {
      case OpenDoor: Math.min(stroke,position+SlidingDoor.SPEED_MM_PER_SECOND*dt);
      case CloseDoor: Math.max(0,position-SlidingDoor.SPEED_MM_PER_SECOND*dt);
      case Hold: position;
    };
    state.setJoint(JOINT,next);state.forwardKinematics();
    return output;
  }
}
