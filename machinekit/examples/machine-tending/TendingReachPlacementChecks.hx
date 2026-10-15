import materia.assembly.AssemblyFrames;

/** The saved placement is a search hint, not permission to extend the design's candidate set. */
@:access(TendingReach)
class TendingReachPlacementChecks {
	public static function run():Void {
		var candidates = [for (y in [0.0, -75.0, 75.0, -150.0])
			{height: 550.0, baseXOffset: 0.0, baseYOffset: y}];
		var original = candidates.copy();
		var saved = AssemblyFrames.fromRotationMatrix(174.15, -1023, 550, [0.0, 1, 0, -1, 0, 0, 0, 0, 1]);
		function require(ok:Bool, message:String):Void if (!ok) throw message;
		require(TendingReach.prioritizePlacement(candidates, saved, 174.15, -873), "saved design placement is recognized");
		require(candidates.length == original.length && candidates[0] == original[3], "only the existing candidate is moved");
		for (i in 0...3) require(candidates[i + 1] == original[i], "fallback candidate order is preserved");
		require(original[3].baseYOffset == -150 && saved.y == -1023, "candidate and saved frame values are unchanged");
		var nonfinite = AssemblyFrames.compose(saved, AssemblyFrames.identity());
		nonfinite.x = Math.NaN;
		for (invalid in [AssemblyFrames.translation(174.15, -1023, 550),
			AssemblyFrames.fromRotationMatrix(175.15, -1023, 550, [0.0, 1, 0, -1, 0, 0, 0, 0, 1]),
			nonfinite, null]) {
			var copy = original.copy();
			require(!TendingReach.prioritizePlacement(copy, invalid, 174.15, -873), "invalid or absent placement is ignored");
			for (i in 0...copy.length) require(copy[i] == original[i], "rejected hint leaves every fallback intact");
		}
		Sys.println("Tending placement hint checks passed (26 assertions)");
	}
}
