package machinekit.pressbrake;

import cadkit.modeling.Axis;
import cadkit.modeling.Location;
import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import machinekit.component.Solids;

/** Four independent grooves on a square block. The active face is placed at z=0. */
class MultiVDieProfile {
	public final id:String;
	public final size:Float;
	final faces:Array<DieGroove>;

	public function new(id:String, size:Float, grooves:Array<DieGroove>) {
		if (id == null || id.length == 0 || !Math.isFinite(size) || !(size > 0))
			throw "Four-V die needs an identity and a positive block size";
		if (grooves == null || grooves.length != 4) throw 'Four-V die "$id" needs four grooves';
		faces = grooves.copy();
		for (groove in faces) {
			if (groove == null || !(groove.depth() + groove.vWidth / 2 < size / 2))
				throw 'Four-V die "$id" grooves must leave separate shoulders and a central web';
		}
		this.id = id;
		this.size = size;
	}

	public function groove(face:Int):DieGroove {
		if (face < 0 || face >= 4) throw 'Four-V die "$id" has no face $face';
		return faces[face];
	}

	/** New owned physical block including every groove; selecting a face rotates the whole block. */
	public function geometry(start:Float, end:Float, activeFace:Int = 0):Part {
		var active = groove(activeFace);
		var rectangle = new ToolProfilePolygon([new ToolProfilePoint(-size / 2, -size),
			new ToolProfilePoint(size / 2, -size), new ToolProfilePoint(size / 2, 0), new ToolProfilePoint(-size / 2, 0)]);
		var block = new ToolProfilePrism(rectangle, start, end).geometry();
		try {
			for (face in 0...4) {
				var blank = new ToolProfilePrism(rectangle, start, end).geometry();
				var die:Null<Part> = null;
				var cutter:Null<Part> = null;
				var placed:Null<Part> = null;
				try {
					die = new DieProfile(id + '-face-$face', faces[face], size, size).geometry(start, end);
					cutter = blank.subtract(die);
					var placement = Location.translation(new Vector(0, 0, -size / 2))
						.compose(Location.rotation(Axis.X(), (face - activeFace) * Math.PI / 2))
						.compose(Location.translation(new Vector(0, 0, size / 2)));
					placed = Solids.named(cutter.placed(placement), 'groove.$face');
					var next = block.subtract(placed);
					block.close();
					block = next;
					placed.close(); cutter.close(); die.close(); blank.close();
				} catch (error:Dynamic) {
					if (placed != null) placed.close();
					if (cutter != null) cutter.close();
					if (die != null) die.close();
					blank.close();
					throw error;
				}
			}
			return block;
		} catch (error:Dynamic) {
			block.close();
			throw error;
		}
	}

	public function collisionSection(maximumError:Float, activeFace:Int = 0):ToolProfilePolygon {
		var active = groove(activeFace);
		var result:Array<ToolProfilePoint> = [];
		for (side in 0...4) {
			var face = (side + activeFace) % 4;
			var points = new DieProfile(id + '-face-$face', faces[face], size, size).collisionSection(maximumError).points();
			var angle = side * Math.PI / 2;
			for (i in 2...points.length - 1) {
				var p = points[i], z = p.z + size / 2;
				result.push(new ToolProfilePoint(p.y * Math.cos(angle) - z * Math.sin(angle),
					p.y * Math.sin(angle) + z * Math.cos(angle) - size / 2));
			}
		}
		return new ToolProfilePolygon(result);
	}
}
