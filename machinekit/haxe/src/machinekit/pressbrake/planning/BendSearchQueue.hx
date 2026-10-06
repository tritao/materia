package machinekit.pressbrake.planning;

import cadkit.sheetmetal.SheetDiagnosis;

/** Stable best-first heap keeps frontier growth from repeatedly sorting every queued state. */
class BendSearchQueue {
	final nodes:Array<BendSearchNode>;
	public function new() { nodes = []; }
	public function empty():Bool return nodes.length == 0;
	public function push(node:BendSearchNode):Void {
		nodes.push(node); var i = nodes.length-1;
		while (i > 0) {
			var parent = Std.int((i-1)/2);
			if (BendSearchNode.compare(nodes[parent],nodes[i]) <= 0) return;
			var swap = nodes[parent]; nodes[parent] = nodes[i]; nodes[i] = swap; i = parent;
		}
	}
	public function take():Null<BendSearchNode> {
		if (nodes.length == 0) return null;
		var result = nodes[0], last = nodes.pop();
		if (last == null) throw new SheetDiagnosis("press.sequence-queue","Nonempty heap lost its last state");
		if (nodes.length == 0) return result;
		nodes[0] = last; var i = 0;
		while (2*i+1 < nodes.length) {
			var child = 2*i+1;
			if (child+1 < nodes.length && BendSearchNode.compare(nodes[child+1],nodes[child]) < 0) child++;
			if (BendSearchNode.compare(nodes[i],nodes[child]) <= 0) break;
			var swap = nodes[i]; nodes[i] = nodes[child]; nodes[child] = swap; i = child;
		}
		return result;
	}
}
