package app.editor;

import app.MachineControlModel.MachineControlAxis;

/** The model's joint positions in display units (degrees or millimetres), as a live display's driver reads its controls. */
class LiveControls implements materia.project.LiveInputs {
  final axes:Map<String, MachineControlAxis> = [];

  public function new(list:Array<MachineControlAxis>) {
    for (axis in list) axes.set(axis.id, axis);
  }

  /** The joint's position, or 0 when the model has no such joint. */
  public function position(id:String):Float {
    var axis = axes.get(id);
    return axis == null ? 0.0 : axis.value;
  }

  /** True when a momentary button's joint is in the lower half of its travel; false for a joint the model lacks. */
  public function pressed(id:String):Bool {
    var axis = axes.get(id);
    if (axis == null || axis.minimum == null || axis.maximum == null) return false;
    return axis.value < (axis.minimum + axis.maximum) / 2;
  }

  /** Which of a clicking joint's positions it is at (0 for the first); 0 for a joint without detents. */
  public function detentIndex(id:String):Int {
    var axis = axes.get(id);
    if (axis == null || axis.detents.length == 0) return 0;
    var best = 0;
    for (i in 1...axis.detents.length) if (Math.abs(axis.detents[i] - axis.value)
      < Math.abs(axis.detents[best] - axis.value)) best = i;
    return best;
  }
}
