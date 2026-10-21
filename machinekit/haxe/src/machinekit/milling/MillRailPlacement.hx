package machinekit.milling;

import machinekit.assembly.AxisBuilder;
import machinekit.motion.LinearRail;
import materia.assembly.AssemblyRecord.AssemblyFrame;
import materia.assembly.AssemblyFrames as F;

typedef MillPlacedRail = {
	var memberId:String;
	var insertId:String;
	var rail:LinearRail;
	var pose:AssemblyFrame;
}

/** Shared placement declarations for the axes and their casting-seat holes. */
class MillRailPlacement {
	public static function fingerprint(rail:LinearRail,memberId:String):String
		return haxe.crypto.Sha256.encode(materia.tooling.ToolingCodec.canonical({
			definition:machinekit.assembly.MachineAssembly.definitionKey(memberId,rail),
			specification:rail.spec,length:rail.length,mountingHole:rail.mountingHole(),
			connectors:rail.mountingConnectors()}));

	public static function y(l:MillGuidedXYStage.MillGuidedXYLayout):Array<MillPlacedRail>
		return [for (side in [-1, 1]) {
			var id = side < 0 ? "railYLeft" : "railYRight";
			{
				memberId: id,
				insertId: id,
				rail: l.yRail,
				pose: AxisBuilder.orient(side * l.yRailSpacing / 2, l.yRailStart,
					l.yRailPlane + l.block.spec.railHeight, [0, 0, 1], [0, 1, 0])
			};
		}];

	public static function z(l:MillZAxisLayout):Array<MillPlacedRail> {
		var f = l.frame;
		var rotation = F.fromRotationMatrix(0, 0, 0, [-1, 0, 0, 0, -1, 0, 0, 0, 1]);
		return [for (side in [-1, 1]) {
			var name = side < 0 ? "Left" : "Right";
			{
				memberId: 'rail$name',
				insertId: 'railZ$name',
				rail: l.rail,
				pose: F.compose(F.translation(side * f.zRailSpacing / 2,
					f.columnFront - l.block.spec.railHeight, f.railPlane + f.zRailStart), rotation)
			};
		}];
	}
}
