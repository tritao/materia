package robotkit.runtime;

typedef PowerUpSidePlacement = {
  final offsets:Array<Float>;
  /** Explicit motor followers exempted from shared-axis displacement propagation. */
  final drives:Array<Int>;
};

/** Resolve startup racking in axis SI units, identified by physical home switch. */
class PowerUpSideOffsets {
  public static function resolve(blueprint:RobotRuntimeBlueprint, base:Array<Float>,
      named:Map<String, Float>):PowerUpSidePlacement {
    if (blueprint == null || base == null || base.length != blueprint.jointCount || named == null)
      throw "Side startup offsets require a full machine offset vector";
    var offsets = base.copy(), drives:Array<Int> = [];
    for (value in offsets) if (!Math.isFinite(value)) throw "Startup offsets must be finite";
    for (id in named.keys()) {
      var contact:Null<robotkit.model.JointSwitch> = null;
      for (candidate in blueprint.switches) if (candidate.id == id) contact = candidate;
      if (contact == null || contact.role != "home" || contact.driveJoint == null)
        throw 'Startup side "$id" requires a home switch with an explicit motor';
      var binding = SwitchDriveBinding.resolve(blueprint, contact);
      if (contact.driveJoint == contact.joint || drives.indexOf(binding.joint) >= 0)
        throw "Startup racking requires distinct motor followers";
      var value = named.get(id);
      if (value == null || !Math.isFinite(value)) throw "Startup side displacement must be finite";
      var position = offsets[binding.joint] + binding.ratio * value;
      if (!Math.isFinite(position)) throw "Startup motor displacement overflow";
      offsets[binding.joint] = position;
      drives.push(binding.joint);
    }
    drives.sort((a, b) -> a - b);
    return {offsets: offsets, drives: drives};
  }
}
