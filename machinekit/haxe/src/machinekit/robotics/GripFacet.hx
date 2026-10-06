package machinekit.robotics;

import machinekit.component.ComponentFacet;
import machinekit.component.MachineComponent;

/** Gripper jaws with `strokeMm` of travel, opened and closed through two ports. */
class GripFacet implements ComponentFacet {
	public final strokeMm:Float;
	/** Grip force in N, when known. */
	public final forceN:Null<Float>;
	public final openPort:String;
	public final closePort:String;
	/** Joint-switch id within the assembly that owns this gripper. */
	public final graspSensor:String;
	/** Connector at the centre of the finger working zone. */
	public final contactConnector:String;
	/** Assumed blank-width range for which the physical jaw travel can close. */
	public final minimumPartWidthMm:Float;
	public final nominalPartWidthMm:Float;
	public final maximumPartWidthMm:Float;
	/** Usable finger length measured from the TCP toward the tool flange. */
	public final fingerLengthMm:Float;
	/** Assumed sliding friction coefficient of the replaceable finger pads. */
	public final fingerPadFrictionCoefficient:Float;
	/** Whether fingers have a physical support toe beneath the fingertip plane. */
	public final supportStep:Bool;

	public function new(strokeMm:Float, forceN:Null<Float>, openPort:String, closePort:String,
			graspSensor:String, contactConnector:String, minimumPartWidthMm:Float,
			nominalPartWidthMm:Float, maximumPartWidthMm:Float, fingerLengthMm:Float,
			fingerPadFrictionCoefficient:Float, supportStep:Bool = false) {
		this.strokeMm = strokeMm;
		this.forceN = forceN;
		this.openPort = openPort;
		this.closePort = closePort;
		this.graspSensor = graspSensor;
		this.contactConnector = contactConnector;
		this.minimumPartWidthMm = minimumPartWidthMm;
		this.nominalPartWidthMm = nominalPartWidthMm;
		this.maximumPartWidthMm = maximumPartWidthMm;
		this.fingerLengthMm = fingerLengthMm;
		this.fingerPadFrictionCoefficient = fingerPadFrictionCoefficient;
		this.supportStep = supportStep;
	}

	public function check(component:MachineComponent):Void {
		component.port(openPort);
		component.port(closePort);
		if (graspSensor == null || graspSensor.length == 0 || contactConnector == null || contactConnector.length == 0 ||
			!(minimumPartWidthMm > 0) || !(minimumPartWidthMm <= nominalPartWidthMm) ||
			!(nominalPartWidthMm <= maximumPartWidthMm) || !(fingerLengthMm > 0) ||
			!(fingerPadFrictionCoefficient > 0) ||
			!Math.isFinite(minimumPartWidthMm + nominalPartWidthMm + maximumPartWidthMm + fingerLengthMm +
				fingerPadFrictionCoefficient))
			throw "Gripper needs a grasp sensor and contact frame";
		component.connector(contactConnector);
	}

	public function describe():String return 'grip $strokeMm mm, $minimumPartWidthMm–$maximumPartWidthMm mm parts, ' +
		'$forceN N, $openPort/$closePort, $graspSensor, ' +
		(supportStep ? 'with fingertip support toes' : 'flat fingers');

	/** The facet of this kind on `component`, or null. */
	public static function of(component:MachineComponent):Null<GripFacet> {
		for (facet in component.facets()) if (Std.isOfType(facet, GripFacet)) return cast facet;
		return null;
	}
}
