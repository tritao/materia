package machinekit.milling;

import machinekit.assembly.MachineAssembly;
import machinekit.assembly.Diagnostics;
import machinekit.design.AssemblyDesignChecks;
import machinekit.design.DesignRequirement;
import materia.assembly.AssemblyFrames as F;

/** Mill-specific assembly intent only. All evaluation is shared with other machines. */
class MillYNutDesignChecks {
	public static function check(axis:MillXYStage,frame:MillFrame):Diagnostics {
		var machine=new MachineAssembly();
		machine.include("stage",axis);machine.include("frame",frame);
		if(frame.zAxis!=null)machine.include("head",new MillZAxis(frame.zAxis));
		return AssemblyDesignChecks.check(machine);
	}
	public static function declare(axis:MillXYStage):Void {
		var l=axis.layout, i=0;
		for(p in l.nut.boltPattern()) {
			var feature='flange$i';
			axis.addDesignRequirement(FastenerJoint('nut-flange$i','nutYBolt$i',{member:"nutBracketY",feature:feature},[{member:"nutY",feature:feature}]));
			axis.addDesignRequirement(AccessPath('bench-nut-key$i',{member:"nutBracketY",feature:feature},
				[{id:"key approach",frame:F.identity(),volume:CylinderVolume(2.5,-65,-l.nut.flangeThickness)}],[],InstalledMembers(["nutY","nutBracketY"])));
			i++;
		}
		for(p in MillYNutMount.anchors()) {
			var name='anchor${p.x}-${p.y}';
			axis.addDesignRequirement(FastenerJoint('saddle-$name','nutBracketYBolt${p.x}-${p.y}',{member:"saddle",feature:name},[{member:"nutBracketY",feature:name}]));
			axis.addDesignRequirement(AccessPath('installed-key-$name',{member:"nutBracketY",feature:name},
				[{id:"key approach",frame:F.identity(),volume:CylinderVolume(2.5,-68,3)}],[],InstalledMembers(["nutBracketY","nutY","saddle"])));
		}
		axis.addDesignRequirement(AccessPath("cartridge-insertion",{member:"nutBracketY",feature:"mount"},
			[{id:"nut flange and screw heads",frame:F.identity(),volume:CylinderVolume(27.5,-l.nut.bodyLength-l.nut.flangeThickness,80)}],[],InstalledMembers(["saddle"])));
		for(p in MillYNutMount.pilots())axis.addDesignRequirement(AccessPath('pilot-insertion${p.x}',{member:"nutBracketY",feature:'pilot${p.x}'},
			[{id:"pilot",frame:F.identity(),volume:CylinderVolume(3,-8,80)}],[],InstalledMembers(["saddle"])));
		// Assumed 8 mm extension, 10 mm elbow, 40 mm handling room, 20 mm
		// hose centreline bend radius. Envelopes add the documented clearance.
		// Port-local +Z is the connection direction, not a world axis.
		var envelopes:Array<DesignEnvelope> = [
			{id:"grease extension front approach",frame:F.translation(0,-30,6.5),volume:BoxVolume(10,65,69.5)},
			{id:"grease connection and 5 mm disconnect stroke",frame:F.identity(),volume:SocketVolume(5,1.5,76,1.7,3.2)},
			{id:"grease elbow/hose approach",frame:F.translation(0,-30,65),volume:BoxVolume(12,72,17)},
			{id:"grease connector handling",frame:F.translation(0,-55,50),volume:BoxVolume(42,42,47)},
			{id:"grease hose minimum bend radius",frame:F.translation(0,-65,46),volume:BoxVolume(52,52,52)}];
		axis.addDesignRequirement(AccessPath("grease-service",{member:"nutY",feature:"grease"},envelopes,[{joint:"x",value:l.xHalfTravel},{joint:"y",value:0}],WholeAssembly));
	}
}
