package app;

class CadEdgeModes {
  public static function label(mode:CadEdgeMode):String return switch(mode) {
    case All:"All edges";
    case Shaded:"Shaded";
    default:"Feature edges";
  };
  public static function next(mode:CadEdgeMode):CadEdgeMode return switch(mode) {
    case Feature:All;
    case All:Shaded;
    default:Feature;
  };
}
