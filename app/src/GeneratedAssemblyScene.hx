package app;

import app.CncProgramPlayer.CncJob;
import nativekit.scene.GeometryData;
import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinition.AssemblyStateRecord;
import cadbridge.AssemblySimulationBridge.AssemblyPhysicalData;

typedef GeneratedAssemblyScene = {
  @:optional var projectJob:String;
  var objects:Array<SceneObjectData>;
  var geometryBySnapshot:Map<String, GeometryData>;
  var assemblyDefinition:Null<AssemblyDefinition>;
  var assemblyState:Null<AssemblyStateRecord>;
  /** Immutable topology and compiled kinematics reused when installing or reconfiguring the scene. */
  @:optional var assemblyModel:cadkit.modeling.CompiledAssembly;
  var localCentersByDefinition:Map<String, Array<Float>>;
  /** What each definition's faces offer a mate (`GeometricConnectors.describeFaces`), when the project wrote it. */
  @:optional var faceDescriptorsByDefinition:Map<String, String>;
  var metresPerUnit:Float;
  var physical:AssemblyPhysicalData;
  var recipeDocument:Null<String>;
  @:optional var recipeDiagnostics:Array<String>;
  /** Joint motion the project ships with, applied to its own assembly in simulation. */
  @:optional var robotMotions:Array<RobotMotionTrack>;
  /** The machining job the project's generator made for its machine, if any. */
  @:optional var cncJob:CncJob;
  /** The assembly is a wheeled robot driving on the floor, when the generator says so. */
  @:optional var mobileBase:materia.project.SceneArtifact.SceneArtifactMobileBase;
  /** Work the assembly's robot does on its own, when the generator ships some. */
  @:optional var mission:materia.project.SceneArtifact.SceneArtifactMission;
  /** The tools the assembly's robot works with, as its parts declare them. */
  @:optional var robotTools:Array<materia.project.SceneArtifact.SceneArtifactRobotTool>;
  /** The sensors on the assembly's robot, as its parts declare them. */
  @:optional var robotSensors:Array<materia.project.SceneArtifact.SceneArtifactRobotSensor>;
  @:optional var machineMotion:materia.project.SceneArtifact.SceneArtifactMachineMotion;
}
