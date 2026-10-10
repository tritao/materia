package machinekit.assembly;

import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import cadkit.threads.ThreadSpec;
import cadkit.threads.Threads;
import machinekit.component.MachineComponent;
import machinekit.component.ComponentDetail;
import machinekit.component.Dimension;
import machinekit.component.Solids;
import machinekit.motion.FlangeBearingHousing;
import machinekit.standard.SocketHeadCapScrew;

/** Aluminium fixture plate with four independently named taps on the housing's bolt square. */
class FlangeTappedPlate extends MachineComponent {
	public final housing:FlangeBearingHousing;
	public final screw:SocketHeadCapScrew;
	public final thickness:Float;
	public final tapDepth:Float;
	public final holeShift:Float;
	public final thread:ThreadSpec;

	public function new(housing:FlangeBearingHousing, screw:SocketHeadCapScrew, thickness:Float = 20,
		tapDepth:Float = 16, holeShift:Float = 0, thread:Null<ThreadSpec> = null) {
		if (!Math.isFinite(thickness) || !Math.isFinite(tapDepth) || !Math.isFinite(holeShift)
			|| thickness <= 0 || tapDepth <= 0 || tapDepth > thickness) throw "Fixture tap depth must fit its plate";
		var nominal = thread == null ? screw.thread.mating() : cast thread;
		if (!nominal.internal) throw "Fixture plate requires an internal thread";
		super('FLANGEPLATE-${housing.designation}-T${Dimension.format(thickness)}-D${Dimension.format(tapDepth)}-X${Dimension.format(holeShift)}'
			+ '-${nominal.designation}-${nominal.majorDiameter}-${nominal.pitch}-${nominal.starts}-${nominal.hand}-${nominal.profile}-${nominal.toleranceClass}-DRILL${screw.spec.tapDrill}',
			"Tapped flange fixture plate", "aluminium 6061", true);
		this.housing = housing; this.screw = screw; this.thickness = thickness;
		this.tapDepth = tapDepth; this.holeShift = holeShift;
		this.thread = nominal;
		addConnector("mount", Mount, Solids.axial(0, 0, thickness));
	}

	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail = Preview):Part {
		var body = Part.box(housing.face + 20, housing.face + 20, thickness);
		if (detail == Envelope) return body;
		var tools:Array<Part> = [], index = 1;
		return Solids.building([body], tracked -> {
			for (point in housing.boltPattern()) {
				var tap = Threads.tappedBore(thread, screw.spec.tapDrill, tapDepth);
				tracked.push(tap);
				var placed = tap.translated(new Vector(point.x + holeShift, point.y, thickness));
				tap.close(); tracked.push(placed);
				var named = Solids.named(placed, "bolt" + index++);
				tracked.push(named); tools.push(named);
			}
			return Solids.cut(body, tools);
		});
	}
}
