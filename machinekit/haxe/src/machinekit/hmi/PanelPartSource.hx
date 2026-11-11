package machinekit.hmi;

import cadkit.modeling.Part;

/** How a panel's plate or shell is made: built as a solid each time (`Built`), or recorded (`Recorded`, so it has a history). */
enum PanelPartSource {
  Built(build:() -> Part);
  Recorded(recorder:machinekit.hmi.PanelSolid.PanelRecording);
}

class PanelPartSources {
  public static function solid(designation:String, description:String, source:PanelPartSource, material:String):PanelSolid return switch source {
    case Built(build): new PanelSolid(designation, description, build, material);
    case Recorded(recorder): PanelSolid.recorded(designation, description, recorder, material);
  };
}
