package machinekit.robotics;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.PortInterface;
import machinekit.component.PortKind;
import machinekit.component.PortRole;
import machinekit.component.Solids;

/** Generic robot-side changer half with bridged air and signal channels. */
class ToolChangerMaster extends MachineComponent {
	public final airChannels:Int;
	public final diameter:Float;
	public final thickness:Float;


	public function new(airChannels:Int, diameter:Float = 60, thickness:Float = 15) {
		if (airChannels < 1 || !Math.isFinite(diameter) || diameter <= 0 ||
			!Math.isFinite(thickness) || thickness <= 0)
			throw "Tool changer master needs positive channels and dimensions";
		super('CHANGER-MASTER-$airChannels-${Dimension.format(diameter)}',
			'Generic $airChannels-channel robot-side tool changer', "aluminium 6061", true);
		this.airChannels = airChannels;
		this.diameter = diameter;
		this.thickness = thickness;
		addConnector("robot", Mount, Solids.axial(0, 0, 0));
		addConnector("tool", Mount, Solids.axial(0, 0, thickness));
		for (i in 1...airChannels + 1) {
			addPort({name: 'airIn$i', kind: Pneumatic, role: Consumer, iface: PushIn(6), required: false});
			addPort({name: 'airOut$i', kind: Pneumatic, role: Supply, iface: PushIn(6), required: false});
			addBridge('airIn$i', 'airOut$i');
		}
		addPort({name: "signalIn", kind: Signal, role: Consumer, iface: Plug("generic", 4), required: false});
		addPort({name: "signalOut", kind: Signal, role: Supply, iface: Plug("generic", 4), required: false});
		addBridge("signalIn", "signalOut");
		addPort({name: "lock", kind: Pneumatic, role: Consumer, iface: PushIn(6), required: true});
		addCapability(ChangerLock("lock"));
		addCapability(Coupling('generic:$airChannels:${Dimension.format(diameter)}', "tool"));
	}

	public static function recipeType():machinekit.component.ComponentType
		return machinekit.component.MachineKitAdditionalRecipes.byId("machinekit.robotics.tool-changer-master");

	/** Subclasses must declare their own recipe and saved values. */
	override public function componentType():Null<machinekit.component.ComponentType>
		return Std.isExactType(this, ToolChangerMaster) ? recipeType() : null;

	override public function values():machinekit.component.ComponentValues return new machinekit.component.ComponentValues().setInteger("airChannels", airChannels).setNumber("diameter", diameter).setNumber("thickness", thickness).setToken("material", materialSpec());

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part
		return Part.cylinderSpan(diameter / 2, 0, thickness);
}
