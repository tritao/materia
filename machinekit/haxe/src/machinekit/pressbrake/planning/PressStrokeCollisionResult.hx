package machinekit.pressbrake.planning;

/** Physical interference and insufficient geometric resolution remain distinct search outcomes. */
enum PressStrokeCollisionResult {
	Clear;
	Collision(material:PressCollisionOwner, obstacle:PressCollisionOwner, sample:Int, angle:Float);
	Unresolved(material:PressCollisionOwner, obstacle:PressCollisionOwner, sample:Int, angle:Float);
}
