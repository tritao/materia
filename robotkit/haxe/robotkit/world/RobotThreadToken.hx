package robotkit.world;

/** Internal owner-thread identity used by RobotWorld's confinement check. */
#if hl
extern abstract RobotThreadToken(hl.Abstract<"hl_thread">) {
  @:hlNative("std", "thread_current")
  public static function current():RobotThreadToken;
}
#else
/** Targets without HashLink threads run on one thread, so every caller shares its token. */
class RobotThreadToken {
  static final only = new RobotThreadToken();

  public static function current():RobotThreadToken return only;

  function new() {}
}
#end
