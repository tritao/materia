package pickingstation;

import materia.assembly.AssemblyFrames;
import machinekit.component.Bom;
import machinekit.component.MachineComponent;
import machinekit.assembly.MachineAssembly;
import machinekit.standard.SocketHeadCapScrew;

/** Composes one rack, its shelves, bins, indicators, frame, and feet. */
class StorageRack extends MachineAssembly {
	public final config:PickingStationConfig;
	public final frame:RackFrame;
	public final shelf:ShelfAssembly;
	public final bin:StorageBin;
	public final indicator:PickIndicator;
	public final foot:AdjustableFoot;
	final positionsList:Array<StoragePosition>;

	public function new(config:PickingStationConfig) {
		super();
		if (config == null) throw "Storage rack requires a configuration";
		this.config = config;
		frame = new RackFrame(config.rackWidth, config.rackDepth, config.rackHeight,
			PickingStationConfig.FOOT_HEIGHT, config.shelfCount, config.shelfSpacing);
		shelf = new ShelfAssembly(config.shelfWidth, config.shelfDepth, 18,
			config.shelfInclinationDegrees);
		bin = new StorageBin(config.binWidth, config.binDepth, config.binHeight,
			config.binWallThickness);
		indicator = new PickIndicator();
		foot = new AdjustableFoot();
		positionsList = [];
		for (shelfIndex in 1...config.shelfCount + 1)
			for (binIndex in 1...config.binsPerShelf + 1)
				positionsList.push(new StoragePosition(config, shelfIndex, binIndex));
		buildAssemblyMembers();
	}

	public function positions(prefix:String = "rack-01"):Array<StoragePosition> {
		if (prefix == "rack-01") return positionsList.copy();
		return [for (position in positionsList) new StoragePosition(config,
			position.shelfIndex, position.binIndex, prefix)];
	}

	public function instances(prefix:String = "rack-01"):Array<StationInstance> {
		var result:Array<StationInstance> = [{id: MachineAssembly.join(prefix, "frame"), label: "Rack extrusion frame",
			component: frame, pose: AssemblyFrames.translation(0, 0, 0)}];
		for (shelfIndex in 1...config.shelfCount + 1) {
			var shelfId = MachineAssembly.join(prefix, 'shelf-${twoDigits(shelfIndex)}');
			result.push({id: shelfId, label: 'Shelf ${twoDigits(shelfIndex)}', component: shelf,
				pose: AssemblyFrames.translation(0, 0, config.shelfZ(shelfIndex))});
		}
		for (position in positionsList) {
			var localId = positionLocalId(position);
			var binId = MachineAssembly.join(prefix, localId);
			result.push({id: binId, label: 'Storage bin ${binId}', component: bin,
				pose: position.placement});
			result.push({id: '$binId/indicator', label: 'Pick indicator ${binId}', component: indicator,
				pose: position.indicatorPlacement});
		}
		var x = config.rackWidth / 2 - PickingStationConfig.FRAME_SIZE / 2;
		var y = config.rackDepth / 2 - PickingStationConfig.FRAME_SIZE / 2;
		for (corner in [
			{name: "front-left", x: -x, y: -y}, {name: "front-right", x: x, y: -y},
			{name: "back-left", x: -x, y: y}, {name: "back-right", x: x, y: y}])
			result.push({id: MachineAssembly.join(prefix, 'feet/${corner.name}'), label: 'Adjustable foot ${corner.name}',
				component: foot, pose: AssemblyFrames.translation(corner.x, corner.y, 0)});
		return result;
	}

	function buildAssemblyMembers():Void {
		addComponent("frame", frame);
		for (shelfIndex in 1...config.shelfCount + 1)
			addComponent('shelf-${twoDigits(shelfIndex)}', shelf,
				AssemblyFrames.translation(0, 0, config.shelfZ(shelfIndex)));
		for (position in positionsList) {
			var id = positionLocalId(position);
			addComponent(id, bin, position.placement);
			addComponent('$id/indicator', indicator, position.indicatorPlacement);
		}
		var x = config.rackWidth / 2 - PickingStationConfig.FRAME_SIZE / 2;
		var y = config.rackDepth / 2 - PickingStationConfig.FRAME_SIZE / 2;
		for (corner in [
			{name: "front-left", x: -x, y: -y}, {name: "front-right", x: x, y: -y},
			{name: "back-left", x: -x, y: y}, {name: "back-right", x: x, y: y}])
			addComponent('feet/${corner.name}', foot, AssemblyFrames.translation(corner.x, corner.y, 0));
		addBomItem(SocketHeadCapScrew.metric("M5", 12).bom, config.shelfCount * 4);
	}

	static function positionLocalId(position:StoragePosition):String
		return 'shelf-${twoDigits(position.shelfIndex)}/bin-${twoDigits(position.binIndex)}';

	static function twoDigits(value:Int):String return value < 10 ? "0" + value : Std.string(value);
}

/** Addressed storage location with geometry and container clearance data. */
class StoragePosition {
	public final id:String;
	public final shelfIndex:Int;
	public final binIndex:Int;
	public final placement:materia.assembly.AssemblyRecord.AssemblyFrame;
	public final indicatorId:String;
	public final indicatorPlacement:materia.assembly.AssemblyRecord.AssemblyFrame;
	public final allowedContainerWidth:Float;
	public final allowedContainerDepth:Float;
	public final allowedContainerHeight:Float;

	public function new(config:PickingStationConfig, shelfIndex:Int, binIndex:Int, prefix:String = "rack-01") {
		id = config.positionId(shelfIndex, binIndex, prefix);
		this.shelfIndex = shelfIndex;
		this.binIndex = binIndex;
		var radians = config.shelfInclinationDegrees * Math.PI / 180;
		placement = materia.assembly.AssemblyFrames.translation(config.binX(binIndex),
			config.binY() * Math.cos(radians) - 18 * Math.sin(radians),
			config.shelfZ(shelfIndex) + config.binY() * Math.sin(radians) + 18 * Math.cos(radians));
		placement.qx = Math.sin(radians / 2);
		placement.qw = Math.cos(radians / 2);
		indicatorId = config.indicatorId(shelfIndex, binIndex, prefix);
		indicatorPlacement = materia.assembly.AssemblyFrames.translation(config.binX(binIndex),
			config.indicatorY(), config.shelfZ(shelfIndex) + 5);
		allowedContainerWidth = config.binWidth;
		allowedContainerDepth = config.binDepth;
		allowedContainerHeight = config.binHeight;
	}
}
