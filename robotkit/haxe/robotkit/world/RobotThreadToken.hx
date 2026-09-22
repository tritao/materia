package robotkit.world;

/** Internal owner-thread identity used by RobotWorld's confinement check. */
#if hl
extern abstract RobotThreadToken(hl.Abstract<"hl_thread">) {
  @:hlNative("std", "thread_current")
  public static function current():RobotThreadToken;
}
#else
class RobotThreadToken {
  public static function current():RobotThreadToken return new RobotThreadToken();
}
#end
