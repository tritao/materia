import pickingstation.PickingScenario;
import pickingstation.PickingScenario.PickingActionResult;
import pickingstation.PickingScenario.PickingScenarioStatus;

/** Project-owned implementation of Materia's project-ui v1 action protocol. */
class PickingStationUiExtension {
	public static function render(request:Dynamic):Dynamic {
		if (Reflect.field(request, "protocol") != "materia.project-ui.v1")
			throw "Unsupported project UI request";
		var scenario = new PickingScenario();
		var events:Array<Dynamic> = Reflect.field(request, "actions");
		if (events == null) throw "Project UI request needs actions";
		var requestedSelection:Null<String> = null;
		for (event in events) {
			var kind:String = Reflect.field(event, "kind");
			var id:String = Reflect.field(event, "id");
			switch (kind) {
				case "action":
					var result:PickingActionResult = switch (id) {
						case "start-order": scenario.start();
						case "confirm-pick": scenario.confirmPick();
						case "reset-fixture": scenario.reset();
						default: throw 'Unknown picking action "$id"';
					};
					requestedSelection = result.accepted ? "scene" : null;
				case "select":
					requestedSelection = null;
					if (id != null && StringTools.startsWith(id, "project:")) {
						var positionId = id.substr("project:".length);
						if (StringTools.endsWith(positionId, "/indicator"))
							positionId = positionId.substr(0, positionId.length - "/indicator".length);
						for (position in scenario.station.storagePositions())
							if (position.id == positionId) {
								scenario.selectBin(positionId);
								requestedSelection = "scene";
							}
					}
				default: throw 'Unknown project UI event "$kind"';
			}
		}
		var state = scenario.state();
		var rows:Array<Dynamic> = [
			{key: "status", text: "Status: " + Std.string(state.status)},
			{key: "sku", text: state.currentSku == null ? "No active pick" : 'SKU: ${state.currentSku}'},
			{key: "quantity", text: state.currentSku == null ? "" : 'Pick quantity: ${state.requestedQuantity}'},
			{key: "source", text: state.sourcePositionId == null ? "" : 'Source: ${state.sourcePositionId}'},
			{key: "progress", text: 'Progress: ${state.completedLines}/${state.totalLines} lines · ${scenario.totalPickedQuantity()} units'},
			{key: "feedback", text: shortFeedback(state.status, state.feedback)},
			{key: "selected", text: state.selectedPositionId == null ? "Select the highlighted bin." :
				"Selected: " + compactPosition(state.selectedPositionId)}
		];
		for (position in scenario.station.storagePositions())
			rows.push({key: "stock-" + position.id,
				text: '${compactPosition(position.id)} · ${scenario.skuAt(position.id)} · ${scenario.quantityAt(position.id)} available'});
		rows.push({key: "manufacturing", text: "Cut planning below: sheet blanks and plan."});
		var colours:Array<Dynamic> = [];
		var target = scenario.highlightedPositionId(), shortage = scenario.status == Shortage;
		for (position in scenario.station.storagePositions()) {
			var active = position.id == target;
			colours.push({id: "project:" + position.id, r: active ? 1.0 : 0.24,
				g: active ? 0.70 : 0.34, b: active ? 0.16 : 0.43});
			colours.push({id: "project:" + position.indicatorId,
				r: active ? (shortage ? 0.96 : 0.16) : 0.24,
				g: active ? (shortage ? 0.20 : 0.92) : 0.34,
				b: active ? (shortage ? 0.15 : 0.26) : 0.43});
		}
		return {protocol: "materia.project-ui.v1", panel: {
			title: "VIRTUAL PICKING STATION", rows: rows, actionAfter: 6,
			actions: [
				{id: "start-order", label: "Start order", enabled: state.status == Idle},
				{id: "confirm-pick", label: "Confirm pick", enabled: state.status == Running},
				{id: "reset-fixture", label: "Reset fixture", enabled: true}
			]}, colours: colours, selectScene: requestedSelection};
	}

	static function compactPosition(id:String):String {
		var parts = id.split("/");
		return parts.length < 3 ? id :
			"S" + Std.string(Std.parseInt(parts[1].substr(6))) + "/B" +
			Std.string(Std.parseInt(parts[2].substr(4)));
	}

	static function shortFeedback(status:PickingScenarioStatus, feedback:String):String {
		if (status == Shortage || status == Complete || status == Idle) return feedback;
		if (StringTools.startsWith(feedback, "Wrong bin")) return "Wrong bin. Quantities unchanged.";
		if (StringTools.startsWith(feedback, "Correct bin")) return "Correct bin. Press Confirm pick.";
		if (StringTools.startsWith(feedback, "Select")) return feedback;
		return "Select highlighted bin, then confirm.";
	}
}
