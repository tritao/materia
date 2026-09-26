package robotkit.manipulation;

/** The ordered patch plan; `fullyPlanned` is false if any patch could not reach 100% from any searched candidate. */
class WorkPatchPlanResult {
  public final patches:Array<WorkPatch>;
  public final fullyPlanned:Bool;

  public function new(patches:Array<WorkPatch>, fullyPlanned:Bool) {
    this.patches = patches;
    this.fullyPlanned = fullyPlanned;
  }
}
