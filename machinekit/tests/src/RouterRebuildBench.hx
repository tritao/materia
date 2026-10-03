/** Stable geometry rebuild benchmark; run the same entry before and after the pose-context change. */
class RouterRebuildBench {
	public static function main():Void {
		for (belts in [false, true]) {
			var router = new CncRouter(belts);
			var warm = router.describe();
			var started = Sys.time();
			for (_ in 0...3) { var result = router.describe(); }
			Sys.println((belts ? "belt" : "screw") + " router: " + (Sys.time() - started) / 3 + " s/rebuild");
		}
	}
}
