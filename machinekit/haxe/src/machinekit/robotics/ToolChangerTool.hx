package machinekit.robotics;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.PortInterface;
import machinekit.component.PortKind;
import machinekit.component.PortRole;
import machinekit.component.Solids;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.ComponentType;

/** Generic tool-side changer half with bridged air and signal channels. */
class ToolChangerTool extends MachineComponent {
	public final airChannels:Int;
	public final diameter:Float;
	public final thickness:Float;


	public function new(airChannels:Int, diameter:Float = 60, thickness:Float = 12) {
		if (airChannels < 1 || !Math.isFinite(diameter) || diameter <= 0 ||
			!Math.isFinite(thickness) || thickness <= 0)
			throw "Tool changer tool needs positive channels and dimensions";
		super('CHANGER-TOOL-$airChannels-${Dimension.format(diameter)}',
			'Generic $airChannels-channel tool-side changer', "aluminium 6061", true);
		this.airChannels = airChannels;
		this.diameter = diameter;
		this.thickness = thickness;
		addConnector("master", Mount, Solids.axial(0, 0, 0));
		addConnector("payload", Mount, Solids.axial(0, 0, thickness));
		for (i in 1...airChannels + 1) {
			addPort({name: 'airIn$i', kind: Pneumatic, role: Consumer, iface: PushIn(6), required: true});
			addPort({name: 'airOut$i', kind: Pneumatic, role: Supply, iface: PushIn(6), required: false});
			addBridge('airIn$i', 'airOut$i');
		}
		addPort({name: "signalIn", kind: Signal, role: Consumer, iface: Plug("generic", 4), required: true});
		addPort({name: "signalOut", kind: Signal, role: Supply, iface: Plug("generic", 4), required: false});
		addBridge("signalIn", "signalOut");
		addFacet(new machinekit.component.CouplingFacet('generic:$airChannels:${Dimension.format(diameter)}', "master"));
	}

	static var recipeTypeCache:Null<ComponentType>;

	public static function recipeType():ComponentType {
		if (recipeTypeCache == null) recipeTypeCache = new ComponentType("machinekit.robotics.tool-changer-tool", [ComponentRecipeSupport.count("airChannels", 2),
			ComponentRecipeSupport.length("diameter", 60), ComponentRecipeSupport.length("thickness", 12)],
			v -> new ToolChangerTool(v.integer("airChannels"), v.number("diameter"), v.number("thickness")), true);
		return recipeTypeCache;
	}

	/** Subclasses must declare their own recipe and saved values. */
	override public function componentType():Null<ComponentType>
		return Std.isExactType(this, ToolChangerTool) ? recipeType() : null;

	override public function values():machinekit.component.ComponentValues return new machinekit.component.ComponentValues().setInteger("airChannels", airChannels).setNumber("diameter", diameter).setNumber("thickness", thickness).setToken("material", materialSpec());

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part
		return Part.cylinderSpan(diameter / 2, 0, thickness);
}
