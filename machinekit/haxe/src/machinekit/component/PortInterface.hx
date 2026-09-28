package machinekit.component;

enum PortInterface {
	PushIn(tubeOd:Float);
	Thread(designation:String);
	Plug(designation:String, pins:Int);
	Unspecified;
}
