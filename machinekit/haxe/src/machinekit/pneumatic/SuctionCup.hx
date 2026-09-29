package machinekit.pneumatic;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.Dimension;
import machinekit.component.MachineComponent;
import machinekit.component.PortInterface;
import machinekit.component.PortKind;
import machinekit.component.PortRole;
import machinekit.component.Solids;

/** Generic suction cup with a vacuum input and contact frame. */
class SuctionCup extends MachineComponent {
	public final diameter:Float;
	public final height:Float;
	/** Measured or vendor-rated sealed area in mm²; null until specified. */
	public final effectiveAreaMm2:Null<Float>;
	/** Rated overturning moment at the chosen vacuum, in N m; null if unknown. */
	public final ratedMomentNm:Null<Float>;

	public function new(diameter:Float, height:Float,
			?effectiveAreaMm2:Float, ?ratedMomentNm:Float,
			?catalogDesignation:String, ?vacuumInterface:PortInterface,
			?catalogDescription:String) {
		if (!Math.isFinite(diameter) || diameter <= 0 || !Math.isFinite(height) || height <= 0)
			throw "Suction cup needs positive dimensions";
		var nominalArea = Math.PI * diameter * diameter / 4;
		if (effectiveAreaMm2 != null && (!Math.isFinite(effectiveAreaMm2) ||
				effectiveAreaMm2 <= 0 || effectiveAreaMm2 > nominalArea + 1e-9))
			throw "Suction cup effective area must fit within its nominal diameter";
		if (ratedMomentNm != null && (!Math.isFinite(ratedMomentNm) || ratedMomentNm <= 0))
			throw "Suction cup moment rating must be positive and finite";
		var ratingId = effectiveAreaMm2 == null ? "" : '-A${Dimension.format(effectiveAreaMm2)}';
		if (ratedMomentNm != null) ratingId += '-M${Dimension.format(ratedMomentNm)}';
		var interfaceId = switch vacuumInterface {
			case null: "";
			case PushIn(size): size == 6 ? "" : '-PI${Dimension.format(size)}';
			case Thread(name): '-TH$name';
			case Plug(name, pins): '-PL$name-$pins';
			case Coupling(key, channel): '-CO$key-$channel';
			case Unspecified: "-UNSPECIFIED";
		};
		super(catalogDesignation == null
			? 'SUCTION-CUP-${Dimension.format(diameter)}-${Dimension.format(height)}$ratingId$interfaceId'
			: catalogDesignation,
			catalogDesignation == null ? "Generic suction cup" :
				(catalogDescription == null ? 'Catalog suction cup $catalogDesignation' : catalogDescription),
			"rubber", catalogDesignation == null);
		this.diameter = diameter;
		this.height = height;
		this.effectiveAreaMm2 = effectiveAreaMm2;
		this.ratedMomentNm = ratedMomentNm;
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
		addConnector("contact", Face, Solids.axial(0, 0, height));
		addPort({name: "vacuum", kind: Vacuum, role: Consumer,
			iface: vacuumInterface == null ? PushIn(6) : vacuumInterface, required: true});
		addCapability(Suction(effectiveAreaMm2, ratedMomentNm, "vacuum", "contact"));
	}

	public static function recipeType():machinekit.component.ComponentType
		return machinekit.component.MachineKitAdditionalRecipes.byId("machinekit.pneumatic.suction-cup");

	/** Subclasses must declare their own recipe and saved values. */
	override public function componentType():Null<machinekit.component.ComponentType>
		return Std.isExactType(this, SuctionCup) ? recipeType() : null;

	override public function values():machinekit.component.ComponentValues {
		var values = new machinekit.component.ComponentValues().setNumber("diameter", diameter).setNumber("height", height)
			.set("effectiveAreaMm2", effectiveAreaMm2 == null ? machinekit.component.ComponentValue.Unset : machinekit.component.ComponentValue.Number(effectiveAreaMm2))
			.set("ratedMomentNm", ratedMomentNm == null ? machinekit.component.ComponentValue.Unset : machinekit.component.ComponentValue.Number(ratedMomentNm))
			.set("catalogDesignation", codeOnly ? machinekit.component.ComponentValue.Unset : machinekit.component.ComponentValue.Token(designation))
			.set("catalogDescription", codeOnly ? machinekit.component.ComponentValue.Unset : machinekit.component.ComponentValue.Token(description));
		machinekit.component.MachineKitAdditionalRecipes.interfaceValues(values, "vacuumInterface", port("vacuum").iface);
		return values.setToken("material", materialSpec());
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part
		return Part.cylinderSpan(diameter / 2, 0, height);
}
