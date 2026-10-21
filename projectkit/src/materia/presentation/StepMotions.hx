package materia.presentation;

import materia.presentation.AssemblyPresentation;

/** Common movement traversal for single parts and atomic rigid groups. */
class StepMotions {
  public static function members(action:StepAction):Array<PartMotion> return switch action {
    case Remove(select,motion)|Install(select,motion):[{select:select,motion:motion}];
    case RemoveGroup(members)|InstallGroup(members):members;
    case _:[];
  }
  public static function installing(action:StepAction):Bool return switch action {
    case Install(_,_)|InstallGroup(_):true;
    case _:false;
  }
  public static function failed(action:StepAction):Bool {
    for(member in members(action))switch member.motion.evidence {
      case Checked(_,Failed,_,_):return true;
      case _:
    }
    return false;
  }
}
