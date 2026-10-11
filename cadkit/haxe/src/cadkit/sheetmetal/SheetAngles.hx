package cadkit.sheetmetal;

/** Explicit bend angles shared by design and manufactured geometry. */
interface SheetAngles {
	public function angle(bend:Int):Float;
	public function ids():Array<Int>;
}
