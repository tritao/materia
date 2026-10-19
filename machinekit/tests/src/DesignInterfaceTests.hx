import cadkit.modeling.Part;
import machinekit.assembly.MachineAssembly;
import machinekit.component.*;
import machinekit.component.ComponentRecipeSupport as P;
import machinekit.design.DesignInterfaces;
import machinekit.design.DesignRequirement;
import machinekit.design.AssemblyDesignChecks;
import machinekit.standard.SocketHeadCapScrew;
import materia.assembly.AssemblyFrames as F;

/** A pump bearing housing, not a mill: the evaluator must not know member names,
 * global orientation, hole sizes, phase membership or supplier geometry.
 */
class DesignInterfaceTests {
	static var assertions=0;
	static function check(ok:Bool,message:String):Void {assertions++;if(!ok)throw message;}
	public static function main():Void {run();Sys.println('Generic design interfaces: $assertions assertions passed');}
	public static function run():Int {
		assertions=0;
		var registry=MachineKitComponents.standard();registry.register(DesignFixturePart.recipeType());
		var pump=fixture(), audit=AssemblyDesignChecks.check(pump); audit.throwIfErrors();check(!audit.hasErrors(),"pump assembly passes shared checker");
		var physical=new MachineAssembly();physical.include("pump",fixture());physical.addComponent("signal",new DesignSignalFixture());
		AssemblyDesignChecks.check(physical).throwIfErrors();check(true,"logical components do not obstruct service tools");
		var hexPump=fixture();hexPump.replaceComponent("clamp",machinekit.standard.HexBolt.metric("M6",16));
		AssemblyDesignChecks.check(hexPump).throwIfErrors();check(true,"hex bolt uses same generic fastener binding");
		var rebuilt=MachineAssembly.decode(pump.encode(),registry);
		check(rebuilt.designRequirements().length==pump.designRequirements().length,"requirements survive recipe roundtrip");
		AssemblyDesignChecks.check(rebuilt).throwIfErrors();check(true,"rebuilt pump passes same checks");
		var pose=F.compose(F.translation(100,80,60),F.turnY(0.7)), nested=new MachineAssembly();nested.include("pump",rebuilt,pose);
		var restored=MachineAssembly.decode(nested.encode(),registry);
		AssemblyDesignChecks.check(restored).throwIfErrors();check(true,"rotated nested pump passes");
		var screwReq=restored.designRequirements()[0];
		check(switch(screwReq){case FastenerJoint(id,s,t,through):s=="pump/clamp" && t.member=="pump/housing" && through[0].member=="pump/cover";case _:false;},"nested interface references remapped");
		var snapshot=restored.snapshot();check(snapshot.designRequirements().length==2,"snapshot retains requirements");
		// The machine adds a guard after the pump was built. WholeAssembly must
		// see it even though the pump itself knows nothing about the guard.
		var guard=F.compose(pose,F.translation(12,0,25));
		nested.addComponent("guard",new DesignFixturePart(new ComponentValues().setBoolean("plate",true)),guard);
		var blocked=AssemblyDesignChecks.check(nested);
		check(blocked.items.filter(e->e.code=="design.access-blocked" && e.subject.indexOf("guard")>=0).length>0,"parent guard obstructs nested lubrication port");
		for(mode in ["short","long","size","direction","offset","thin","undrilled","missing","stack","missing-through","bad-frame","pitch"]) {
			var bad=fixture();
			switch(mode) {
				case "pitch":bad.replaceComponent("clamp",machinekit.standard.HexBolt.custom({size:"M6",diameter:6,pitch:0.75,acrossFlats:10,headHeight:4,tapDrill:5.2,clearanceFine:6.4,clearanceMedium:6.6,clearanceCoarse:7},16));
				case "short":bad.replaceComponent("clamp",SocketHeadCapScrew.metric("M6",8));
				case "long":bad.replaceComponent("clamp",SocketHeadCapScrew.metric("M6",25));
				case "size":bad.replaceComponent("clamp",SocketHeadCapScrew.metric("M5",16));
				case "thin":bad.replaceComponent("housing",new DesignFixturePart(new ComponentValues().setBoolean("thin",true)));
				case "undrilled":bad.replaceComponent("housing",new DesignFixturePart(new ComponentValues().setBoolean("undrilled",true)));
				case "missing-through":bad.addDesignRequirement(FastenerJoint("empty-stack","clamp",{member:"housing",feature:"fixing"},[]));
				case "bad-frame":bad.addDesignRequirement(AccessPath("bad-frame",{member:"housing",feature:"lubrication"},[{id:"tool",frame:{x:0,y:0,z:0,qx:0,qy:0,qz:0,qw:2},volume:CylinderVolume(4,1,16)}],[],WholeAssembly));
				case "missing":bad.addDesignRequirement(FastenerJoint("missing","clamp",{member:"housing",feature:"absent"},[]));
				case "direction","offset","stack":
					var altered=new MachineAssembly();
					altered.addComponent("housing",new DesignFixturePart());
					altered.addComponent("cover",new DesignFixturePart(new ComponentValues().setBoolean("plate",true)),F.translation(0,0,mode=="stack"?-5:-4));
					altered.addComponent("clamp",SocketHeadCapScrew.metric("M6",16),mode=="direction"?F.translation(0,0,-4):F.fromRotationMatrix(1,0,-4,[1,0,0,0,-1,0,0,0,-1]));
					for(r in bad.designRequirements())altered.addDesignRequirement(r);bad=altered;
				case _:
			}
			check(AssemblyDesignChecks.check(bad).hasErrors(),'shared checker rejects $mode');
		}
		// A bench phase excludes the guard explicitly, unlike a service pose.
		var bench=new MachineAssembly();bench.include("pump",fixture());
		bench.addComponent("guard",new DesignFixturePart(),F.translation(12,0,25));
		bench.addDesignRequirement(AccessPath("bench",{member:"pump/housing",feature:"lubrication"},[{id:"tool",frame:F.identity(),volume:CylinderVolume(4,1,16)}],[],InstalledMembers(["pump/housing"])));
		var benchAudit=AssemblyDesignChecks.check(bench);
		check(benchAudit.items.filter(e->e.subject.indexOf("bench/")==0).length==0,"declared bench phase uses only installed parts");
		var moving=fixture([{joint:"guard-slide",value:100}]), posed=new MachineAssembly();posed.include("unit",moving,pose);
		var savedPose=MachineAssembly.decode(posed.encode(),registry);
		AssemblyDesignChecks.check(savedPose).throwIfErrors();check(true,"nested saved service pose moves cover clear of port");
		var badPose=fixture([{joint:"guard-slide",value:0}]);check(AssemblyDesignChecks.check(badPose).hasErrors(),"closed service cover blocks port");
		var invalidPose=fixture([{joint:"not-a-joint",value:100}]);check(AssemblyDesignChecks.check(invalidPose).hasErrors(),"unknown service joint rejected");
		var old=new MachineAssembly();old.addComponent("housing",new DesignFixturePart());
		var legacy=old.describe();legacy.machine.designRequirements=null;
		var loadedLegacy=MachineAssembly.decode(haxeon.wire.JsonWire.encode(legacy),registry);
		check(loadedLegacy.designRequirements().length==0,"description without optional requirements still loads");
		var refs=rebuilt.designRequirements();refs.pop();check(rebuilt.designRequirements().length==2,"requirements accessor is a defensive copy");
		return assertions;
	}
	static function fixture(?servicePose:Array<DesignJointValue>):MachineAssembly {
		var a=new MachineAssembly();a.addComponent("housing",new DesignFixturePart());
		a.addComponent("cover",new DesignFixturePart(new ComponentValues().setBoolean("plate",true)),F.translation(0,0,-4));
		a.addComponent("clamp",SocketHeadCapScrew.metric("M6",16),F.fromRotationMatrix(0,0,-4,[1,0,0,0,-1,0,0,0,-1]));
		a.addDesignRequirement(FastenerJoint("cover-clamp","clamp",{member:"housing",feature:"fixing"},[{member:"cover",feature:"clearance"}]));
		if(servicePose!=null) {
			a.addComponent("service-cover",new DesignFixturePart(new ComponentValues().setBoolean("plate",true)));
			a.addMemberConnector("housing","cover-slide",F.translation(12,0,25));
			a.addMateOnAxis("guard-slide","prismatic","housing","cover-slide","service-cover","mount",{x:0,y:1,z:0},0,{lower:0,upper:100,effort:null,velocity:null});
		}
		a.addDesignRequirement(AccessPath("lubrication",{member:"housing",feature:"lubrication"},[{id:"connector",frame:F.identity(),volume:CylinderVolume(4,1,16)}],servicePose==null?[]:servicePose,WholeAssembly));return a;
	}
}

private class DesignFixturePart extends MachineComponent {
	static var recipe:Null<ComponentType>;
	static function parameters():Array<ComponentParameter> return [P.flag("plate",false),P.flag("thin",false),P.flag("undrilled",false)];
	public static function recipeType():ComponentType {if(recipe==null)recipe=new ComponentType("test.design.pump-part",parameters(),v->new DesignFixturePart(v));return recipe;}
	public function new(?v:ComponentValues) {
		v=P.resolve(parameters(),v);super("PUMP-PART","Pump bearing housing/cover","steel",false,v);
		addConnector("mount",Mount,F.identity());
		var plate=v.boolean("plate"), thread:Null<BlindThreadInterface> = plate?null:{spec:machinekit.standard.FastenerThreads.coarse(6).mating(),tapDrill:5,usableDepth:14,cylindricalDepth:15,point:1.5,stock:20,minEngagement:6,minTipGap:1,minBack:3};
		addFacet(new DesignInterfaces([{id:plate?"clearance":"fixing",frame:F.identity(),radius:plate?3.3:2.5,depth:plate?4:16.5,minWall:4,minClearance:0.4,thread:thread,counterbore:null,approach:null}],plate?[]:[{id:"lubrication",frame:F.translation(12,0,20),kind:LubricationPort}],[]));
	}
	override public function componentType():Null<ComponentType> return recipeType();
	override public function hasGeometry():Bool return true;
	override public function geometry(detail:ComponentDetail=Preview):Part {
		var v=values(), plate=v.boolean("plate"), body=Part.box(v.boolean("thin")?12:40,40,plate?4:20);
		return v.boolean("undrilled")?body:Solids.cut(body,DesignInterfaces.of(this).drillingTools());
	}
}

private class DesignSignalFixture extends MachineComponent {
	public function new() {super("SIGNAL","Logical pump service signal");}
	override public function hasGeometry():Bool return false;
}
