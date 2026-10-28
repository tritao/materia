import materia.assembly.AssemblyDefinition;
import materia.assembly.AssemblyDefinitionCodec;
import materia.assembly.AssemblyFrames;

/** Regression: physical hardware takes a complete router past 1,000 parts. */
class AssemblyLargePartsTests {
    static function check(v:Bool,s:String):Void {if(!v)throw s;}
    static function fixture(count:Int):AssemblyDefinition return {schemaVersion:AssemblyDefinitionCodec.VERSION,
        id:"hardware",lengthUnit:"mm",definitions:[{id:"bolt",connectors:[]}],
        occurrences:[for(i in 0...count){id:"bolt"+i,definition:"bolt",initialPose:AssemblyFrames.identity()}],joints:[]};
    public static function main():Void {
        var model=fixture(1001),roundtrip=AssemblyDefinitionCodec.decode(AssemblyDefinitionCodec.encode(model));
        check(roundtrip.occurrences.length==1001,"complete hardware survives codec round trip");
        AssemblyDefinitionCodec.validate(fixture(AssemblyDefinitionCodec.MAX_PARTS));
        var rejected=false;
        try AssemblyDefinitionCodec.validate(fixture(AssemblyDefinitionCodec.MAX_PARTS+1))catch(_:Dynamic)rejected=true;
        check(rejected,"oversize occurrence payload remains bounded");
        var many=fixture(1);many.definitions=[for(i in 0...AssemblyDefinitionCodec.MAX_PARTS){id:"definition"+i,connectors:[]}];
        many.occurrences[0].definition="definition0";AssemblyDefinitionCodec.validate(many);
        many.definitions.push({id:"overflow",connectors:[]});rejected=false;
        try AssemblyDefinitionCodec.validate(many)catch(_:Dynamic)rejected=true;
        check(rejected,"oversize definition payload remains bounded");
        var connected=fixture(1);
        connected.definitions[0].connectors=[for(i in 0...AssemblyDefinitionCodec.MAX_CONNECTORS){name:"mount"+i,frame:AssemblyFrames.identity()}];
        AssemblyDefinitionCodec.validate(connected);
        connected.definitions[0].connectors.push({name:"overflow",frame:AssemblyFrames.identity()});rejected=false;
        try AssemblyDefinitionCodec.validate(connected)catch(_:Dynamic)rejected=true;
        check(rejected,"connector payload remains bounded after admitting real long hosts");
        Sys.println("AssemblyLargePartsTests: physical hardware round trip and decode bounds pass");
    }
}
