import machinekit.assembly.MachineAssembly;
import machinekit.assembly.AssemblyPreview;

/** Export, motion, navigation and inspection for the installed physical cell. */
class HobberScenePresentation {
  public static function build(
    assembly:MachineAssembly,
    machine:FlexsplineHobber,
    stroke:Float,
    scenarios:Bool = false
  ):materia.project.SceneArtifact.SceneArtifactData {
    var scene = AssemblyPreview.scene(assembly, "hobber-enclosed");
    var pressureAngle = machine.spec.form.pressureAngle();
    if (scene.machineMotion == null) scene.machineMotion = {belts: []};
    var axes = ["machine/X", "machine/"+FlexsplineHobber.Y, "machine/Z", "machine/swivel/A", "machine/spindle/B", "machine/table/C"];
    scene.machineMotion.hobbing = {
      module: machine.spec.module,
      teeth: machine.spec.teeth,
      width: machine.spec.faceWidth,
      pressureAngle: pressureAngle,
      starts: machine.hob.design.starts,
      gashes: machine.hob.design.gashes,
      outside: 32,
      length: 40,
      wall: 0.6,
      radialStep: 0.1,
      axialStep: 0.25,
      tolerance: 0.001,
      feed: 5,
      cuttingSpeed: 300,
      axes: axes,
      stock: "machine/blank",
      controllerHz: 4000,
      cupLength: 40,
      cupEndThickness: 2
    };
    scene.machineMotion.enclosureGuard = {
      doorJoint: "enclosure/door/slide",
      closedSensor: "enclosure/door/closed",
      openSensor: "enclosure/door/open",
      spindleJoints: [axes[4], axes[5]],
      axisJoints: axes.slice(0, 4),
      stroke: stroke * 0.001,
      setupSpeedLimit: 0.025,
      timeoutSeconds: 10,
      lockTravelSeconds: 0.05,
      linearStandstillTolerance: 1e-5,
      rotaryStandstillTolerance: 1e-4,
      standstillConfirmationSeconds: 0.25
    };
    var definition = scene.assemblyDefinition;
    if (definition == null) throw "Unified scene needs its assembly";
    if (scenarios) {
      var config:materia.project.SceneArtifact.SceneArtifactHobbing = scene.machineMotion.hobbing;
      config.scenario = "cutting";
      config.handling = {
        sleeve: "machine/mandrelSleeve",
        outside: machine.mandrel.parameterNumber("outside"),
        wall: machine.mandrel.parameterNumber("sleeveWall"),
        length: machine.mandrel.parameterNumber("sleeveLength"),
        radialTravel: machine.mandrel.parameterNumber("radialFitGap"),
        retractX: hobber.HobberLayout.get().xRetract,
        frontApproachMm: hobber.HobberLayout.get().frontApproach,
        liftMm: hobber.HobberLayout.get().frontLift
      };
    }
    if (scene.project == null) scene.project = {dynamicParts: [], motions: []};
    var project:materia.project.SceneArtifact.SceneArtifactProject = cast scene.project;
    project.navigation = [
      {
        occurrence: "machine/bed/casting",
        label: "Open mold",
        variant: "plinth-mold",
        target: machinekit.tooling.CastingMoldView.REFERENCE_ID
      },
      {
        occurrence: "machine/column/casting",
        label: "Open mold",
        variant: "column-mold",
        target: machinekit.tooling.CastingMoldView.REFERENCE_ID
      }
    ];
    HobberInspectionViews.apply(scene, machine, "machine/");
    return scene;
  }
}
