import machinekit.design.DesignRequirement;
import machinekit.design.DesignRequirements;

class CoaxialRecordTests {
	public static function main():Void {
		var original=Coaxial("rail-seat",{member:"screw",feature:"head"},{member:"steel",feature:"mount1"},1e-6);
		var nested=DesignRequirements.copy(original,value -> "machine/"+value);
		switch nested {
			case Coaxial(id,first,second,tolerance):
				if (id!="machine/rail-seat" || first.member!="machine/screw" || second.member!="machine/steel"
					|| first.feature!="head" || second.feature!="mount1" || tolerance!=1e-6)
					throw "Portable coaxial references were not qualified exactly";
			default:throw "Coaxial requirement changed kind";
		}
		if (DesignRequirements.id(original)!="rail-seat" || DesignRequirements.id(nested)!="machine/rail-seat")
			throw "Coaxial requirement identity changed";
		Sys.println("CoaxialRecordTests: 2 assertions passed");
	}
}
