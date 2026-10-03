package machinekit.transmission;

/** Parts are valid individually but cannot work together in this transmission. */
class TransmissionDesignError extends haxe.Exception {
	public function new(message:String) super(message);
}
