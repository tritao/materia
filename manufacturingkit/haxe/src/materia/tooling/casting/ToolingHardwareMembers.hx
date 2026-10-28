package materia.tooling.casting;

import materia.tooling.ToolingIds;
import materia.tooling.casting.CoreSupport.SupportHardwareKind;

/** Shared occurrence contract for the planner and the CAD fastener producer. */
class ToolingHardwareMembers {
  public static function fixtureScrew(id:String):Array<String>
    return [id,ToolingIds.derive("",id,"-washer")];
  public static function throughBolt(id:String):Array<String>
    return [id,ToolingIds.derive("",id,"-top-washer"),ToolingIds.derive("",id,"-bottom-washer"),ToolingIds.derive("",id,"-nut")];
  public static function of(id:String,kind:SupportHardwareKind):Array<String> return switch kind {
    case FixtureScrew:fixtureScrew(id);
    case ClampBolt:throughBolt(id);
    case LocatingPin,FixturePin,BoreMask,BorePlug:[id];
    default:throw "Unsupported tooling hardware kind: "+kind;
  };
}
