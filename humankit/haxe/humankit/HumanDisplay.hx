package humankit;

/** How a human is drawn: the skinned mesh, the collision capsules, or the skeleton. */
enum abstract HumanDisplay(Int) from Int to Int {
	var Mesh = 0;
	var Capsules = 1;
	var Skeleton = 2;
}
