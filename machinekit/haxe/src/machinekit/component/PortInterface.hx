package machinekit.component;

@:wire enum PortInterface {
	@:id(1) PushIn(tubeOd:Float);
	@:id(2) Thread(designation:String);
	@:id(3) Plug(designation:String, pins:Int);
	/** Integrated changer feed-through; the key and channel must match across halves. */
	@:id(4) Coupling(key:String, channel:Int);
	@:id(5) Unspecified;
}
