package machinekit.pressbrake.planning;

import cadkit.modeling.Location;
import cadkit.modeling.Vector;
import cadkit.sheetmetal.SheetDiagnosis;

/** Machine material at joint zero with an explicit, typed stroke motion. */
class PressMachineMember {
	public final id:String;
	public final motion:PressMemberMotion;
	final solids:Array<ConvexPrism>;
	public function new(id:String, motion:PressMemberMotion, pieces:Array<ConvexPrism>) {
		if (id == null || id.length == 0 || motion == null || pieces == null || pieces.length == 0)
			throw new SheetDiagnosis("press.machine-volume", "Machine material needs an identity, motion and nonempty pieces");
		for (piece in pieces) if (piece == null)
			throw new SheetDiagnosis("press.machine-volume", "Machine material has a null piece", [id]);
		this.id = id; this.motion = motion; solids = pieces.copy();
	}
	public function at(ramTravel:Float, gaugeX:Float, gaugeR:Float):PressCollisionBody {
		var shift = switch motion {
			case Fixed: new Vector();
			case Ram: new Vector(0, 0, -ramTravel);
			case BackgaugeX: new Vector(0, gaugeX, 0);
			case BackgaugeXR: new Vector(0, gaugeX, gaugeR);
		};
		var pose = Location.translation(shift);
		return new PressCollisionBody(MachineMember(id), [for (piece in solids) piece.placed(pose)], 0);
	}
}
