package materia.presentation;

import materia.assembly.AssemblyDefinition.AssemblyVector;
import materia.assembly.AssemblyDefinition.AssemblyJointCoordinate;

@:wire typedef AssemblyPresentation = {
  @:id(1) var schemaVersion:Int;
  @:id(2) var definition:String;
  @:id(3) var defaultView:String;
  @:id(4) var views:Array<PresentationView>;
  @:id(5) var explode:Array<ExplodeOffset>;
  @:id(6) var procedures:Array<Procedure>;
}
@:wire typedef PartSelector = {
  @:id(1) var occurrence:String;
  @:id(2) var subtree:Bool;
}
@:wire enum ViewCamera {
  @:id(1) Frame(select:Array<PartSelector>, yaw:Float, pitch:Float, margin:Float);
  @:id(2) Anchored(anchor:PartSelector, offset:AssemblyVector, span:Float, yaw:Float, pitch:Float);
}
@:wire enum VisibilityMode { @:id(1) Shown; @:id(2) Transparent; @:id(3) Hidden; }
@:wire enum PresentationSource { @:id(1) Generated(producer:String); @:id(2) Authored; }
@:wire typedef VisibilityRule = {
  @:id(1) var select:PartSelector;
  @:id(2) var mode:VisibilityMode;
}
@:wire typedef SectionPlane = {
  /** Null names the world frame. */
  @:id(1) var occurrence:Null<String>;
  @:id(2) var point:AssemblyVector;
  @:id(3) var normal:AssemblyVector;
}
@:wire typedef PresentationView = {
  @:id(1) var id:String;
  @:id(2) var label:String;
  @:id(3) var camera:ViewCamera;
  @:id(4) var visibility:Array<VisibilityRule>;
  @:id(5) var sections:Array<SectionPlane>;
  @:id(6) var explode:Float;
  @:id(7) @:optional var source:PresentationSource;
}
@:wire typedef ExplodeOffset = {
  @:id(1) var select:PartSelector;
  @:id(2) var direction:AssemblyVector;
  @:id(3) var distance:Float;
  @:id(4) var stage:Int;
  @:id(5) @:optional var source:PresentationSource;
}
@:wire enum ProcedureKind { @:id(1) Assembly; @:id(2) Disassembly; @:id(3) Service; }
@:wire enum ProcedureStart { @:id(1) Assembled; @:id(2) Disassembled; }
@:wire enum CheckStatus { @:id(1) Passed; @:id(2) Failed; }
@:wire enum MotionEvidence {
  @:id(1) Illustrative;
  @:id(2) Checked(checker:String, status:CheckStatus, geometryFingerprint:String, report:String);
}
@:wire typedef PresentationMotion = {
  @:id(1) var path:Array<AssemblyVector>;
  @:id(2) var evidence:MotionEvidence;
}
/** A rigid checked group may span unrelated occurrences with different parent frames. */
@:wire typedef PartMotion = {
  @:id(1) var select:PartSelector;
  @:id(2) var motion:PresentationMotion;
}
@:wire enum StepAction {
  @:id(1) Remove(select:PartSelector, motion:PresentationMotion);
  @:id(2) Install(select:PartSelector, motion:PresentationMotion);
  @:id(3) SetJoints(coordinates:Array<AssemblyJointCoordinate>);
  @:id(4) Note;
  @:id(5) RemoveGroup(members:Array<PartMotion>);
  @:id(6) InstallGroup(members:Array<PartMotion>);
}
@:wire typedef ProcedureStep = {
  @:id(1) var id:String;
  @:id(2) var title:String;
  @:id(3) var instruction:String;
  @:id(4) @:optional var view:String;
  @:id(5) var highlight:Array<PartSelector>;
  @:id(6) var action:StepAction;
}
@:wire typedef Procedure = {
  @:id(1) var id:String;
  @:id(2) var label:String;
  @:id(3) var kind:ProcedureKind;
  @:id(4) var start:ProcedureStart;
  @:id(5) var steps:Array<ProcedureStep>;
  @:id(6) @:optional var source:PresentationSource;
}
