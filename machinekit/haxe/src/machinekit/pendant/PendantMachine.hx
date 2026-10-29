package machinekit.pendant;

import cadkit.threads.SurfaceDetail.DisplayBinding;
import machinekit.operator.PendantDisplaySignals;

/**
 * The machine a pendant belongs to: the operator profile its display serves and the joint each of the profile's axes moves when the
 * wheel jogs it. The pendant is the same on every machine; this is what makes it that machine's.
 */
class PendantMachine {
  /** The name of the operator profile (see `ReferenceOperatorProfiles.named`). */
  public final profile:String;
  /** Which assembly joint a jog of each axis moves, as display bindings named by `PendantDisplaySignals.jog`. */
  public final jogs:Array<DisplayBinding>;

  public function new(profile:String, jogs:Array<DisplayBinding>) {
    this.profile = profile;
    this.jogs = jogs;
  }

  /** The reference pendant on its own: the hobber's profile and no machine joints to move. */
  public static function reference():PendantMachine return new PendantMachine("hobber", []);

  /** A machine whose axis `axisId` moves joint `jointId`, for each pair given. */
  public static function moving(profile:String, axes:Map<String, String>):PendantMachine {
    var ids = [for (axis in axes.keys()) axis];
    ids.sort(Reflect.compare);
    return new PendantMachine(profile, [for (axis in ids) new DisplayBinding(PendantDisplaySignals.jog(axis), axes.get(axis))]);
  }
}
