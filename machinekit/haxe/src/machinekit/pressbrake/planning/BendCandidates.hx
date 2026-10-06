package machinekit.pressbrake.planning;

import cadkit.sheetmetal.Bend;
import cadkit.sheetmetal.BendSeam;
import cadkit.sheetmetal.Sheet;
import cadkit.sheetmetal.WallEdge;
import machinekit.pressbrake.ToolStation;

/** Finite setup candidates come from authored seams, installed material endpoints and finger datums. */
class BendCandidates {
	public static function centers(sheet:Sheet, bend:Bend, station:ToolStation, press:PressGeometry, faceA:Bool, reverse:Bool):Array<Float> {
		var width = bend.edgeA.span.length(), direction = faceA == reverse ? 1.0 : -1.0;
		var intervals = BendSeam.intervals(sheet.wall(bend.wallA).outline,bend.edgeA.span,true);
		var events:Array<Float> = [], offset = station.offset-press.bedLength/2;
		var low = Math.POSITIVE_INFINITY, high = Math.NEGATIVE_INFINITY;
		for (interval in intervals) for (fraction in [interval.first,interval.last]) {
			var relative = direction*width*(fraction-0.5); low = Math.min(low,relative); high = Math.max(high,relative);
		}
		if (intervals.length == 0) return [];
		for (row in [station.punches,station.dies]) for (piece in row.placements(offset)) {
			var ends = [piece.start,piece.end], horn = piece.segment.horn;
			if (horn != null) ends.push(switch horn.end {
				case Left: piece.start+horn.reliefLength;
				case Right: piece.end-horn.reliefLength;
			});
			for (interval in intervals) for (fraction in [interval.first,interval.last])
				for (end in ends) events.push(end-direction*width*(fraction-0.5));
		}
		for (finger in press.fingerContacts()) for (fraction in [0.0,1.0])
			events.push(finger.x-direction*width*(fraction-0.5));
		events.push(-press.bedLength/2-low); events.push(press.bedLength/2-high);
		events.sort(compare); var unique:Array<Float> = [];
		for (event in events) if (unique.length == 0 || event-unique[unique.length-1] > 1e-8) unique.push(event);
		var result:Array<Float> = [];
		for (i in 0...unique.length) {
			add(result,unique[i],low,high,station,press);
			if (i+1 < unique.length) add(result,(unique[i]+unique[i+1])/2,low,high,station,press);
		}
		return result;
	}
	static function add(result:Array<Float>, center:Float, first:Float, last:Float, station:ToolStation, press:PressGeometry):Void {
		var low = station.offset-press.bedLength/2, high = low+Math.min(station.punches.span,station.dies.span);
		if (center+first >= low-1e-8 && center+last <= high+1e-8
			&& center+first >= -press.bedLength/2 && center+last <= press.bedLength/2) result.push(center);
	}
	public static function gauges(sheet:Sheet):Array<GaugeEdge> {
		var result:Array<GaugeEdge> = [];
		for (wall in sheet.walls()) {
			var loops = wall.outline.loops();
			for (i in 0...loops.length) for (j in 0...loops[i].length) result.push(new GaugeEdge(wall.id,new WallEdge(j,i)));
		}
		return result;
	}
	static function compare(a:Float,b:Float):Int return a < b ? -1 : a > b ? 1 : 0;
}
