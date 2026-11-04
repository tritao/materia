import machinekit.structural.CastingVolumeAgreement;
import machinekit.structural.MineralCasting;
import materia.casting.InsertStages.InsertStage;

/**
 * The hobber base and column take the analytic casting volume (no B-rep) for their mass, as cast and (with post-cure
 * receivers) in service, and it agrees with integrating the real solid.
 */
class HobberCastingMassChecks {
  public static function run(machine:FlexsplineHobber):Int {
    var count = 0;
    for (entry in [{label: "bed", casting: machine.bed.body}, {label: "column", casting: machine.column.body}]) {
      // Service differs from the pour only by post-cure receivers; integrating the base twice costs half a minute.
      var stages = entry.casting.postCureBonds().length > 0 ? [InsertStage.BeforeCasting, InsertStage.Service] : [InsertStage.BeforeCasting];
      for (stage in stages) {
        CastingVolumeAgreement.check(
          new MineralCasting(entry.casting.values().copy().setToken("buildStage", stage)),
          entry.label + " " + stage
        );
        count++;
      }
    }
    return count;
  }
}
