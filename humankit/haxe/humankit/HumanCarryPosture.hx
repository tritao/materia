package humankit;

/** Hands held near the chest while the legs keep their walking gait. */
enum abstract HumanCarryPosture(Int) from Int to Int {
	var None = 0;
	var LeftHand = 1;
	var RightHand = 2;
	var BothHands = 3;
}
