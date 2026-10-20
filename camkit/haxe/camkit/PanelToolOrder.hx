package camkit;

import materia.tooling.panel.PanelPlan.PanelOperation;

/** Breadth-first tool-block scheduling. Saturating a tool's ready operations
 * cannot worsen a precedence-only schedule; every level is exactly one load.
 */
class PanelToolOrder {
	public static function order(operations:Array<PanelOperation>, number:PanelOperation->Int):Array<PanelOperation> {
		var tools = [for (op in operations) number(op)],
			unique:Array<Int> = [];
		for (tool in tools)
			if (unique.indexOf(tool) < 0)
				unique.push(tool);
		unique.sort((a, b) -> a - b);
		var states:Array<Array<PanelOperation>> = [[]], seen = new Map<String, Bool>(), cursor = 0;
		while (cursor < states.length) {
			var done = states[cursor++];
			if (done.length == operations.length)
				return done;
			for (tool in unique) {
				var next = done.copy(), changed = true;
				while (changed) {
					changed = false;
					for (op in operations)
						if (next.indexOf(op) < 0
							&& number(op) == tool
							&& (op.accessVia == null || next.filter(p -> p.id == op.accessVia).length == 1)
							&& (op.kind != OutsideProfile || next.length == operations.length - 1)) {
							next.push(op);
							changed = true;
						}
				}
				if (next.length == done.length)
					continue;
				var ids = [for (op in next) op.id];
				ids.sort(Reflect.compare);
				var key = ids.join("|");
				if (!seen.exists(key)) {
					seen.set(key, true);
					states.push(next);
				}
				if (states.length > 50000)
					throw "Panel tool scheduling exceeds exact search limit";
			}
		}
		throw "Cyclic panel CAM operation prerequisites";
	}
}
