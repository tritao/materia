package robotkit.skill;

import robotkit.world.RobotSnapshot;
import robotkit.work.WorkSurface;
import robotkit.perception.PointCloud;
import robotkit.perception.SurfaceRegistration;
import robotkit.perception.SurfaceRegistrationResult;

/**
 * Registers a design `WorkSurface` against an already-captured `PointCloud`
 * (typically `ScanSurface.cloud`) via `SurfaceRegistration.register`.
 * Registration is a closed-form computation, not a physical action, so this
 * skill completes within `start()` like `GoTo` completes immediately when
 * already at its goal; it never needs `update()` ticks.
 */
class RegisterSurface implements Skill {
  public final design:WorkSurface;
  public final cloud:PointCloud;
  public final seed:Int;
  public final maxIterations:Int;
  public final inlierThreshold:Float;
  public final maxTranslationCorrection:Float;
  public final maxRotationCorrection:Float;
  /** The registration outcome once this skill reaches a terminal status. */
  public var registration(default, null):Null<SurfaceRegistrationResult> = null;

  final lifecycle:SkillLifecycle = new SkillLifecycle();

  public function new(design:WorkSurface, cloud:PointCloud, seed:Int,
      ?maxIterations:Int = 500, ?inlierThreshold:Float = 0.01,
      ?maxTranslationCorrection:Float = 0.05, ?maxRotationCorrection:Float = 0.0872665) {
    if (design == null || cloud == null) throw "RegisterSurface requires a design surface and observed cloud";
    this.design = design;
    this.cloud = cloud;
    this.seed = seed;
    this.maxIterations = maxIterations;
    this.inlierThreshold = inlierThreshold;
    this.maxTranslationCorrection = maxTranslationCorrection;
    this.maxRotationCorrection = maxRotationCorrection;
  }

  public function start():Void {
    lifecycle.begin();
    try {
      var value = SurfaceRegistration.register(design, cloud, seed, maxIterations, inlierThreshold,
        maxTranslationCorrection, maxRotationCorrection);
      registration = value;
      if (value.accepted) lifecycle.succeed("registration accepted");
      else lifecycle.fail(value.rejectionReason == null ? "registration rejected" : value.rejectionReason);
    } catch (error:Dynamic) {
      lifecycle.fail(Std.string(error));
    }
  }

  public function update(snapshot:RobotSnapshot, durationSeconds:Float):SkillStatus return lifecycle.status();

  public function cancel():Void {
    if (!lifecycle.isRunning()) return;
    lifecycle.cancel();
  }

  public function status():SkillStatus return lifecycle.status();
  public function result():Null<SkillResult> return lifecycle.result();
}
