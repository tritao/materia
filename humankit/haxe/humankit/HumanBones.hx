package humankit;

class HumanBones {
	/** Bones every humanoid rig must provide. */
	public static final REQUIRED:Array<HumanBone> = [Pelvis, Chest, Neck, Head, UpperArmL, ForearmL, HandL,
		UpperArmR, ForearmR, HandR, ThighL, ShinL, FootL, ThighR, ShinR, FootR];

	/** Bones used when present: spine segments, shoulders, toes, and knuckles for hand frames. */
	public static final OPTIONAL:Array<HumanBone> = [Spine, Spine2, ShoulderL, ShoulderR, IndexL, MiddleL, PinkyL,
		IndexR, MiddleR, PinkyR, ToeL, ToeR];

	public static function all():Array<HumanBone>
		return REQUIRED.concat(OPTIONAL);
}
