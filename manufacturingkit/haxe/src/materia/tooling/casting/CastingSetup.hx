package materia.tooling.casting;

import materia.assembly.AssemblyRecord.AssemblyFrame;

enum abstract CoreStrategy(String) to String {
  var SacrificialEPS = "sacrificial-eps";
  var ReusableFormer = "reusable-former";
  var Unsupported = "unsupported";
}

class CoreStrategies {
  public static function values():Array<CoreStrategy> return [
    CoreStrategy.SacrificialEPS,
    CoreStrategy.ReusableFormer,
    CoreStrategy.Unsupported
  ];
  public static function tokens():Array<String> return [for (strategy in values())(strategy:String)];
}

/** Mold +Z is upward, gravity is -Z and the open fill face faces +Z.
 * The setup is tied to the exact input fingerprint, not merely its revision number.
 */
typedef CastingSetup = {
  var id:String;
  var revision:Int;
  var sourceFingerprint:String;
  var sourceToMold:AssemblyFrame;

  /** Absolute mold Z of the final mix surface, and extra containment above it. */
  var fillHeightMm:Float;

  var freeboardMm:Float;

}
