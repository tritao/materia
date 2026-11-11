import cadkit.parametric.Feature;
import cadkit.parametric.recording.DocumentBuilder;
import machinekit.assembly.AssemblyPreview;
import machinekit.assembly.MachineAssembly;
import machinekit.component.ComponentRecipeSupport as P;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.MachineComponent;
import materia.assembly.AssemblyFrames as F;

/** A plate with a window, recorded by overriding `recordGeometry`: length and width are recipe inputs, thickness is ground stock. */
class RecordedTestPlate extends MachineComponent {
  public static final recipe = new machinekit.cache.Once<ComponentType>();

  static function parameters():Array<machinekit.component.ComponentParameter> return [P.length("length", 120), P.length("width", 80)];

  public static function recipeType():ComponentType return recipe.get(() -> new ComponentType("test.recorded-plate",
    parameters(), v -> new RecordedTestPlate(v)).numbered(_ -> "TEST-RECORDED-PLATE"));

  /** What the subclass records; `mistake` breaks the parameter rules on purpose. */
  public final mistake:String;

  public function new(?values:ComponentValues, mistake:String = "") {
    super("TEST-RECORDED-PLATE", "Recorded test plate", "steel", false, P.resolve(parameters(), values));
    this.mistake = mistake;
  }

  override public function componentType():Null<ComponentType> return recipeType();

  override public function recordGeometry(b:DocumentBuilder, detail:machinekit.component.ComponentDetail):Null<Feature> {
    var length = b.length("length", values().number("length"));
    var width = b.length("width", mistake == "width derived" ? 80.0 : values().number("width"));
    if (mistake == "width derived") width.derivedBy("not a rule");
    var thickness = b.length("thickness", 10).fixedBy("10 mm ground flat stock");
    if (mistake == "stray input") b.length("stray", 3);
    var plate = b.label(b.boxBetween(0, length, 0, width, 0, thickness), "Stock plate");
    var window = b.boxBetween(20, 50, 20, 60, -1, 11);
    return b.label(b.cutAll(plate, [window], ["front window"]), "Front window");
  }
}

class PartRecordingTests {
  static function check(value:Bool, message:String):Void {
    if (!value) throw "PartRecordingTests: " + message;
  }

  static function throws(action:() -> Void, expected:String, what:String):Void {
    var message:Null<String> = null;
    try action() catch (error:Dynamic) message = Std.string(error);
    check(message != null && message.indexOf(expected) >= 0, '$what: got "$message", expected "$expected"');
  }

  static function volume(part:MachineComponent):Float {
    var solid = part.geometry();
    var v = solid.volume();
    solid.close();
    return v;
  }

  static function recordingOf(part:MachineComponent):Void {
    var document = part.recording();
    if (document == null) throw "PartRecordingTests: no recording";
    document.close();
  }

  public static function run():Void {
    var plate = new RecordedTestPlate();
    check(Math.abs(volume(plate) - (120 * 80 - 30 * 40) * 10) < 1e-6, "solid comes from the recording");

    // A recipe override is a different dimension of the same recording.
    var longer = cast(RecordedTestPlate.recipeType().create(new ComponentValues().setNumber("length", 150)), RecordedTestPlate);
    check(Math.abs(volume(longer) - (150 * 80 - 30 * 40) * 10) < 1e-6, "an override changes the recorded length");

    // The parameter rules.
    throws(() -> recordingOf(new RecordedTestPlate(null, "stray input")), 'editable parameter "stray" is not a recipe input', "a stray input");
    throws(() -> recordingOf(new RecordedTestPlate(null, "width derived")), 'recipe input "width" is recorded as derived', "an input recorded as derived");

    // Derived holes are the last step, under Fasteners.
    var drilled = new RecordedTestPlate();
    drilled.machine([{frame: F.identity(), x: 90, y: 40, diameter: 9, depth: 10, through: true, kind: "clearance", thread: "M8", name: "J1.h1.clearance"}]);
    check(Math.abs(volume(drilled) - ((120 * 80 - 30 * 40) * 10 - Math.PI * 4.5 * 4.5 * 10)) < 1e-6, "the derived hole is cut");
    var document = drilled.recording();
    if (document == null) throw "PartRecordingTests: no recording";
    var text = machinekit.component.PartRecordings.encoded(document);
    check(text.indexOf('"group":"Fasteners"') >= 0 && text.indexOf('"type":"holes"') >= 0, "holes recorded as the Fasteners step");
    check(drilled.derivedRouteHoles().length == 1, "the hole table reads the same hole");

    // The scene carries the history and one source row per top-level step, with its faces.
    var assembly = new MachineAssembly();
    assembly.addComponent("plate", drilled, F.identity());
    var scene = AssemblyPreview.scene(assembly, "recorded");
    var project = scene.project;
    if (project == null) throw "PartRecordingTests: the scene has no project";
    var histories = project.histories;
    var sources = project.sources;
    if (histories == null || sources == null) throw "PartRecordingTests: the scene has no histories or sources";
    check(histories.length == 1 && histories[0].occurrence == "plate", "one history, for the plate occurrence");
    var labels = [for (row in sources.features) row.label];
    for (label in ["Stock plate", "Front window", "Fasteners · Derived holes"])
      check(labels.indexOf(label) >= 0, 'source row "$label" in ${labels.join(", ")}');
    for (row in sources.features) check(row.faces.length > 0, 'step "${row.label}" names the faces it made');
    Sys.println("PartRecordingTests: recorded geometry, parameter rules, derived holes and scene histories pass");
  }

  public static function main():Void run();
}
