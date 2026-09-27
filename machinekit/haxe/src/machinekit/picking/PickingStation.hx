package machinekit.picking;

import materia.assembly.AssemblyFrames;
import machinekit.component.Bom;
import machinekit.standard.SocketHeadCapScrew;

/** One rack plus one adjacent operator bench, derived from station parameters. */
class PickingStation {
	public final config:PickingStationConfig;
	public final rack:StorageRack;
	public final bench:OperatorBench;
	final benchX:Float;

	public function new(?config:PickingStationConfig) {
		this.config = config == null ? PickingStationConfig.defaults() : config;
		rack = new StorageRack(this.config);
		bench = new OperatorBench(this.config.benchWidth, this.config.benchDepth, this.config.benchHeight);
		benchX = this.config.rackWidth / 2 + this.config.benchWidth / 2 + 180;
	}

	public function storagePositions():Array<StoragePosition> return rack.positions();

	public function instances():Array<StationInstance> {
		var result = rack.instances();
		result.push({id: "operator-bench", label: "Operator bench", component: bench,
			pose: AssemblyFrames.translation(benchX, 0, 0)});
		return result;
	}

	public function billOfMaterials():Bom {
		var result = rack.billOfMaterials();
		result.addComponent(bench);
		result.addComponent(SocketHeadCapScrew.metric("M5", 12), 4);
		return result;
	}

	public function componentByInstanceId(id:String):StationInstance {
		for (entry in instances()) if (entry.id == id) return entry;
		throw 'Unknown picking-station instance "$id"';
	}
}
