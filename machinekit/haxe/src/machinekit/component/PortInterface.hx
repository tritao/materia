package machinekit.component;

enum PortInterface {
	PushIn(tubeOd:Float);
	Thread(designation:String);
	Plug(designation:String, pins:Int);
	/** Integrated changer feed-through; the key and channel must match across halves. */
	Coupling(key:String, channel:Int);
	Unspecified;
}
