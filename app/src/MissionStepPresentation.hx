package app;

import materia.project.SceneArtifact.SceneArtifactMissionStep;

/** Shared user-facing labels and scene targets for mission views. */
class MissionStepPresentation {
  public static function targetOf(step:SceneArtifactMissionStep):Null<String> {
    if (step.at != null) return step.at.occurrence;
    if (step.weld != null) return step.weld.frame;
    if (step.contactWork != null) return step.contactWork.frame;
    return null;
  }

  public static function label(step:SceneArtifactMissionStep):String {
    var action = switch step.kind {
      case "goTo": "Move base";
      case "pick": "Pick";
      case "place": "Place";
      case "moveJoints": "Move joints";
      case "findWork": "Locate workpiece";
      case "weld": "Weld seam";
      case "stow": "Return arm";
      case "screw": step.screw == null ? "Drive screw" : switch step.screw {
        case Single(_): "Drive screw";
        case Snug(_,_): "Snug screw";
        case Final(_): "Final tightening";
      };
      default: step.kind;
    };
    var target = targetOf(step);
    return action + (target == null ? "" : " · " + target);
  }

}
