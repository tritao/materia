import materia.tooling.casting.CastingMoldPlanner;
import machinekit.tooling.CastingMoldCad;

/** Phase 4 acceptance is a test suite, independent of full artifact export. */
class MillMoldSuite {
	public static function main():Void {
		var frame = MillMoldSetup.frame();
		var p = CastingMoldPlanner.plan(MillMoldSetup.document(frame), MillMoldSetup.options(frame));
		MillMoldChecks.run(frame, p, CastingMoldCad.review(p));
	}
}
