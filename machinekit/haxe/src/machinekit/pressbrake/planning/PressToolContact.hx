package machinekit.pressbrake.planning;

import cadkit.sheetmetal.SheetDiagnosis;
import machinekit.pressbrake.ToolContactPrism;

/** Exact sections remain attached to a typed installed tooling owner. */
class PressToolContact {
	public final owner:PressCollisionOwner;
	final sections:Array<ToolContactPrism>;
	public function new(owner:PressCollisionOwner, prisms:Array<ToolContactPrism>) {
		switch owner { case Punch(_, _), Die(_, _): case _: throw new SheetDiagnosis("press.contact-owner", "Contact sections need a punch or die owner"); }
		this.owner = owner; sections = prisms.copy();
	}
	public function prisms():Array<ToolContactPrism> return sections.copy();
}
