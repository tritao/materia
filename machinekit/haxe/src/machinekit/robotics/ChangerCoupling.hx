package machinekit.robotics;

/** Mechanical identity and mating connector of a tool changer half. */
interface ChangerCoupling {
	public function couplingKey():String;
	public function couplingConnector():String;
}
