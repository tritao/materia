package machinekit.pneumatic;

import cadkit.InertiaTensor;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.ComponentDetail;
import machinekit.component.MachineComponent;
import machinekit.component.PortKind;
import machinekit.component.Solids;

/** One cut hose with an authored centreline in its member frame (millimetres).
 * The solid envelope follows the route; mass follows centreline length. */
class RoutedHose extends MachineComponent {
	public final route:Array<Vector>;
	public final outerDiameterMm:Float;
	public final innerDiameterMm:Float;
	public final lengthMm:Float;
	public final massPerMetreKg:Float;
	public final serviceKind:PortKind;

	public function new(designation:String, route:Array<Vector>, outerDiameterMm:Float,
			innerDiameterMm:Float, massPerMetreKg:Float, serviceKind:PortKind = Vacuum,
			material:String = "polyurethane PU", codeOnly:Bool = true) {
		if (route == null || route.length < 2 || !Math.isFinite(outerDiameterMm) ||
			outerDiameterMm <= 0 || !Math.isFinite(innerDiameterMm) || innerDiameterMm <= 0 ||
			innerDiameterMm >= outerDiameterMm || !Math.isFinite(massPerMetreKg) ||
			massPerMetreKg <= 0 ||
			(serviceKind != Vacuum && serviceKind != Pneumatic))
			throw "Routed hose needs a route, wall thickness, mass per metre and fluid service";
		var copied:Array<Vector> = [];
		for (point in route) {
			if (point == null || !Math.isFinite(point.x) || !Math.isFinite(point.y) ||
				!Math.isFinite(point.z)) throw "Routed hose points must be finite";
			copied.push(new Vector(point.x, point.y, point.z));
		}
		var length = 0.0;
		for (i in 0...copied.length - 1) {
			var segment = copied[i + 1].subtract(copied[i]);
			var extent = segment.length();
			if (!(extent > 0)) throw "Routed hose has a zero-length segment";
			length += extent;
		}
		super(designation, 'Routed ${serviceKind == Vacuum ? "vacuum" : "air"} hose, ${Math.round(length)} mm cut',
			material, codeOnly);
		this.route = copied;
		this.outerDiameterMm = outerDiameterMm;
		this.innerDiameterMm = innerDiameterMm;
		this.lengthMm = length;
		this.massPerMetreKg = massPerMetreKg;
		this.serviceKind = serviceKind;
		addConnector("mount", Mount, Solids.axial(0, 0, 0));
		addConnector("start", Mount, Solids.axial(copied[0].x, copied[0].y, copied[0].z));
		var end = copied[copied.length - 1];
		addConnector("end", Mount, Solids.axial(end.x, end.y, end.z));
		addPort({name: "input", kind: serviceKind, role: Consumer,
			iface: PushIn(outerDiameterMm), required: true, connector: "start"});
		addPort({name: "output", kind: serviceKind, role: Supply,
			iface: PushIn(outerDiameterMm), required: false, connector: "end"});
		addBridge("input", "output");
		var weightedX = 0.0, weightedY = 0.0, weightedZ = 0.0;
		for (i in 0...copied.length - 1) {
			var a = copied[i], b = copied[i + 1];
			var fraction = b.subtract(a).length() / length;
			weightedX += fraction * (a.x + b.x) / 2;
			weightedY += fraction * (a.y + b.y) / 2;
			weightedZ += fraction * (a.z + b.z) / 2;
		}
		var mass = massPerMetreKg * length / 1000;
		var tensor = InertiaTensor.zero();
		for (i in 0...copied.length - 1) {
			var a = copied[i], b = copied[i + 1], delta = b.subtract(a);
			var segmentLength = delta.length(), segmentMass = mass * segmentLength / length;
			var ux = delta.x / segmentLength, uy = delta.y / segmentLength, uz = delta.z / segmentLength;
			var radius = outerDiameterMm / 2;
			var perpendicular = segmentMass * (segmentLength * segmentLength / 12 + radius * radius / 4);
			var parallel = segmentMass * radius * radius / 2;
			var difference = parallel - perpendicular;
			var centreX = (a.x + b.x) / 2 - weightedX;
			var centreY = (a.y + b.y) / 2 - weightedY;
			var centreZ = (a.z + b.z) / 2 - weightedZ;
			tensor = tensor.add(new InertiaTensor(perpendicular + difference * ux * ux,
				difference * ux * uy, difference * ux * uz,
				perpendicular + difference * uy * uy, difference * uy * uz,
				perpendicular + difference * uz * uz)
				.shifted(segmentMass, centreX, centreY, centreZ));
		}
		declareMass(mass, new Vector(weightedX, weightedY, weightedZ), tensor);
	}

	override public function hasGeometry():Bool return true;

	override public function geometry(detail:ComponentDetail = Preview):Part {
		var pieces:Array<Part> = [];
		return Solids.building(pieces, tracked -> {
			var radius = outerDiameterMm / 2;
			for (i in 0...route.length - 1) {
				var delta = route[i + 1].subtract(route[i]);
				var direction = delta.normalized();
				tracked.push(Part.cylinderAlong(radius, route[i].subtract(direction.scale(radius)),
					direction, delta.length() + 2 * radius));
			}
			return Solids.union(tracked);
		});
	}
}
