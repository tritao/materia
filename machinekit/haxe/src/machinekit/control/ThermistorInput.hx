package machinekit.control;

import cadkit.modeling.Part;
import machinekit.component.ComponentDetail;
import machinekit.component.ComponentParameter;
import machinekit.component.ComponentParameterType;
import machinekit.component.ComponentValue;
import machinekit.component.ComponentRecipeSupport;
import machinekit.component.ComponentType;
import machinekit.component.ComponentValues;
import machinekit.component.MachineComponent;
import machinekit.component.Solids;
import machinekit.power.DcVoltage;

/** Controller ADC front end: NTC to ground, pull-up to an assumed ideal reference.
 * The envelope is authored, not a verified board layout; ADC quantization and pin
 * assignments belong to a concrete controller. All electrical values are explicit.
 */
class ThermistorInput extends MachineComponent {
	static var recipe:Null<ComponentType>;
	public final pullupOhm:Float;
	public final reference:DcVoltage;
	public final channels:Int;
	public final width:Float;
	public final depth:Float;
	public final height:Float;

	public function new(pullupOhm:Float, reference:DcVoltage, channels:Int, width:Float, depth:Float, height:Float) {
		for (value in [pullupOhm,width,depth,height])
			if (!Math.isFinite(value) || value<=0) throw "Thermistor front-end resistance and dimensions must be finite and positive";
		if (channels<1 || channels>64) throw "Thermistor front end needs 1 to 64 channels";
		super('THERMISTOR-INPUT-$channels', "Controller thermistor ADC front end (assumed ideal reference and envelope)", "plastic");
		this.pullupOhm=pullupOhm;this.reference=reference;this.channels=channels;
		this.width=width;this.depth=depth;this.height=height;
		addConnector("mount",Mount,Solids.axial(0,0,0));
		for (slot in 0...channels) addPort({name:inputName(slot),kind:Signal,role:Consumer,iface:Unspecified,required:true});
	}

	public function inputName(slot:Int):String {
		if (slot<0 || slot>=channels) throw "Thermistor channel is outside the controller front end";
		return 'ntc$slot';
	}
	/** Divider voltage, including a short (zero ohms) and open circuit (positive infinity). */
	public function voltage(ohms:Float):Float {
		if (Math.isNaN(ohms) || ohms<0) throw "NTC resistance must be nonnegative";
		return ohms==0 ? 0 : reference.volts()/(1+pullupOhm/ohms);
	}
	public function resistance(volts:Float):Float {
		if (!Math.isFinite(volts) || volts<=0 || volts>=reference.volts()) throw "NTC voltage must lie strictly between the reference rails";
		return pullupOhm*volts/(reference.volts()-volts);
	}
	public static function recipeType():ComponentType {
		if (recipe==null) recipe=new ComponentType("machinekit.control.thermistor-input",[
			new ComponentParameter("pullupOhm",Scalar,Number(4700),"ohm",0),
			new ComponentParameter("referenceVolts",Scalar,Number(3.3),"V",0),
			ComponentRecipeSupport.count("channels",2),ComponentRecipeSupport.length("width",30),
			ComponentRecipeSupport.length("depth",20),ComponentRecipeSupport.length("height",4)
		],v -> new ThermistorInput(v.number("pullupOhm"),new DcVoltage(v.number("referenceVolts")),
			v.integer("channels"),v.number("width"),v.number("depth"),v.number("height")));
		return recipe;
	}
	override public function componentType():Null<ComponentType> return Std.isExactType(this,ThermistorInput) ? recipeType() : null;
	override public function values():ComponentValues return new ComponentValues().setNumber("pullupOhm",pullupOhm)
		.setNumber("referenceVolts",reference.volts()).setInteger("channels",channels).setNumber("width",width)
		.setNumber("depth",depth).setNumber("height",height).setToken("material",materialSpec());
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail=Preview):Part return Solids.named(Part.box(width,depth,height),"thermistor-front-end");
}
