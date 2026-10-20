package machinekit.milling;

/** Pedestal layout intent in mm; openings and the cap engagement are explicit. */
class MillPedestalDetails {
	public var minimumDimension:Float = 600;
	public var wall:Float = 90;
	public var sole:Float = 100;
	public var roof:Float = 90;
	public var rib:Float = 25;
	public var chamberColumns:Int = 1;
	public var chamberRows:Int = 2;
	public var serviceWidth:Float = 360;
	public var serviceDepth:Float = 100;
	public var serviceHeight:Float = 380;
	public var serviceCentreInset:Float = 40;
	public var serviceFloor:Float = 150;
	public var ribPassageDepth:Float = 27;
	public var cornerChamfer:Float = 30;
	public var capThickness:Float = 20;
	public var capEmbedment:Float = 12;
	public function new() {}
}
