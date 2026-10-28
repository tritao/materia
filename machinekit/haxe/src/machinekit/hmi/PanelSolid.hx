package machinekit.hmi;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;

/** A fixed solid of a panel (its plate, a shell) as a component, so a panel can be checked as an assembly. */
class PanelSolid extends MachineComponent {
  final build:() -> Part;

  /** `build` makes a new solid each time it is called; the caller of `geometry` owns and closes it. */
  public function new(designation:String, description:String, build:() -> Part, material:String = "plastic") {
    super(designation, description, material, true);
    this.build = build;
  }

  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail = Preview):Part return build();
}
