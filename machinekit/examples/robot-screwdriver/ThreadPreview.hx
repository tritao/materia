import cadkit.modeling.Part;
import cadkit.modeling.Vector;
import cadkit.threads.ThreadSpec;
import cadkit.threads.Threads;
import cadkit.threads.ThreadedFace.ThreadStart;
import materia.project.ThreadSpecData.ThreadHand;
import materia.project.SceneArtifact;
import machinekit.assembly.AssemblyPreview;
import machinekit.component.Solids;
import machinekit.standard.SocketHeadCapScrew;
import machinekit.standard.PropertyClass;
import haxe.io.Bytes;

/** Sectioned tap drills expose two independent thread depths beside a plain hole and a screw. */
class ThreadPreview {
	public static function preview():Bytes {
		var tools:Array<Part> = [];
		for (index in 0...3) {
			var hole = Part.cylinderSpan(2.5, -0.1, 30.1, (index - 1) * 15);
			if (index < 2) {
				var spec = new ThreadSpec("M6", 6, 1, 1, index == 1 ? LeftHand : RightHand, true);
				var threaded = new Part(Threads.attach(hole.shape, "cyl.side", spec, AxisMaximum,
					(index == 0 ? 20.0 : 12.0) + 0.1));
				hole.close(); hole = threaded;
			}
			tools.push(Solids.named(hole, "hole" + index));
		}
		var plate = Solids.cut(Solids.named(Part.box(55, 20, 30), "plate"), tools);
		// Remove the front half: the thread keeps its named cylindrical face and manufacturing interval.
		var sectionBase = Part.box(60, 20, 32);
		var section = sectionBase.translated(new Vector(0, -10, -1)); sectionBase.close();
		plate = Solids.cut(plate, [section]);
		var screw = SocketHeadCapScrew.metric("M6", 40, Steel88);
		var shank = screw.geometry();
		var placed = shank.translated(new Vector(43, 0, 40)); shank.close();
		return SceneArtifact.encode({lengthUnit: "mm", metresPerUnit: 0.001, parts: [
			AssemblyPreview.part("plate", "M6 taps: RH 20 mm / LH 12 mm / plain", plate, "aluminium"),
			AssemblyPreview.part("screw", screw.designation, placed, screw.materialId)
		]});
	}
}

function main():Void {
	var scene = SceneArtifact.decode(ThreadPreview.preview());
	var plateThreads = scene.parts[0].threads, screwThreads = scene.parts[1].threads;
	if (plateThreads == null || plateThreads.length != 2 || screwThreads == null || screwThreads.length != 1)
		throw "Thread preview lost a named thread";
	trace("Thread preview passed");
}
