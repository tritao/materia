package robotkit.model;

/** Explicit policy for deriving runtime collision from authored geometry. */
enum abstract CollisionApproximation(String) from String to String {
	var None = "none";
	var BoundsBox = "bounds-box";
}
