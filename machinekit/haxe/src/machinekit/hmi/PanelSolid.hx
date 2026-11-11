package machinekit.hmi;

import cadkit.parametric.Feature;
import cadkit.parametric.recording.DocumentBuilder;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;

/** A panel part's geometry as recorded steps: write them with the builder, return the body. */
typedef PanelRecording = DocumentBuilder -> Feature;

/** A fixed solid of a panel (its plate, a shell) as a component, so a panel can be checked as an assembly. */
class PanelSolid extends MachineComponent {
  final build:Null<() -> Part>;
  final recorder:Null<PanelRecording>;

  /**
   * `build` makes a new solid each time it is called (the caller of `geometry` owns and closes it); or the part records its
   * geometry with `recorder` (`recorded`), and its solid and history come from that.
   */
  public function new(designation:String, description:String, build:Null<() -> Part>, material:String = "plastic",
      ?recorder:PanelRecording) {
    super(designation, description, material, true);
    if ((build == null) == (recorder == null)) throw 'Panel solid "$designation" needs a build or a recorder, not both';
    this.build = build;
    this.recorder = recorder;
  }

  /** A panel solid that records its geometry. */
  public static function recorded(designation:String, description:String, recorder:PanelRecording,
      material:String = "plastic"):PanelSolid return new PanelSolid(designation, description, null, material, recorder);

  override public function record(b:DocumentBuilder, detail:ComponentDetail):Null<Feature> {
    var steps = recorder;
    if (steps == null) return null;
    return steps(b);
  }

  override public function hasGeometry():Bool return true;
  override public function geometry(detail:ComponentDetail = Preview):Part {
    var made = build;
    return made == null ? super.geometry(detail) : made();
  }
}
