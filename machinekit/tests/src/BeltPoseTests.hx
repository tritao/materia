import cadkit.modeling.AssemblyState;
import haxeon.wire.JsonWire;
import machinekit.transmission.BeltPoseContext;
import materia.assembly.AssemblyDefinition;

/** `BeltPoseContext` copies only what it changes: its definition and state equal the full-copy version's, and the source is untouched. */
class BeltPoseTests {
  static var passed = 0;

  static function check(ok:Bool, message:String):Void {
    if (!ok) throw message;
    passed++;
  }

  /** The constructor as it was: a full JSON copy, then the same changes. The reference the new copy must equal. */
  static function reference(mechanical:AssemblyDefinition): {definition: AssemblyDefinition, state: AssemblyState} {
    var definition:AssemblyDefinition = JsonWire.decode(JsonWire.encode(mechanical));
    definition.couplings = [];
    definition.actuators = [];
    definition.encoders = [];
    definition.elasticNetworks = null;
    for (joint in definition.joints) {
      joint.limits.lower = null;
      joint.limits.upper = null;
      joint.stop = null;
    }
    return {definition: definition, state: new AssemblyState(definition)};
  }

  public static function main():Void {
    var plotter = new CoreXyPlotter();
    var mechanical = plotter.definition();
    var before = JsonWire.encode(mechanical);
    check(mechanical.couplings != null && mechanical.couplings.length > 0, "the plotter has couplings to drop");
    var bounded = 0;
    for (joint in mechanical.joints) if (joint.limits.lower != null || joint.limits.upper != null) bounded++;
    check(bounded > 0, "the plotter has bounded joints to unbound");
    var expected = reference(mechanical);
    var actual = new BeltPoseContext(mechanical);
    check(
      JsonWire.encode(actual.definition) == JsonWire.encode(expected.definition),
      "the probe definition equals the full copy's"
    );
    check(
      JsonWire.encode(actual.state.definition) == JsonWire.encode(expected.state.definition),
      "the state's own definition equals the full copy's"
    );
    check(JsonWire.encode(mechanical) == before, "building the context leaves the source definition untouched");
    for (joint in actual.definition.joints) check(
      joint.limits.lower == null && joint.limits.upper == null && joint.stop == null,
      "joint " + joint.id + " is unbounded"
    );
    actual.reset();
    expected.state.setJoint("x", 0);
    check(JsonWire.encode(mechanical) == before, "resetting the context leaves the source definition untouched");
    // Two contexts from one source do not see each other's changes.
    var other = new BeltPoseContext(mechanical);
    other.definition.joints[0].limits.lower = 5.0;
    other.definition.couplings = null;
    check(
      JsonWire.encode(actual.definition) == JsonWire.encode(expected.definition),
      "another context's changes do not reach this one"
    );
    check(JsonWire.encode(mechanical) == before, "changes to a context's joints do not reach the source");
    Sys.println("belt pose tests: " + passed + " checks passed");
  }
}
