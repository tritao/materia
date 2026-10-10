package machinekit.standard;

/** The drive and its nominal bit or wrench size, stated by the fastener. */
@:wire
enum ScrewDrive {
	@:id(1) HexSocket(acrossFlatsMm:Float);
	@:id(2) Hexalobular(size:String);
	@:id(3) Cross(size:Int);
	@:id(4) HexHead(acrossFlatsMm:Float);
	@:id(5) Slot(widthMm:Float);
}
