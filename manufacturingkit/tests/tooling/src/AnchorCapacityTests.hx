import materia.tooling.ToolingDocument;
import materia.tooling.ToolingCodec;
import materia.tooling.casting.InsertAnchorageChecks as Checks;
import materia.casting.InsertAnchorage;
import materia.casting.InsertAnchorage.InsertAnchorageCodec as C;
import materia.assembly.AssemblyFrames as F;

class AnchorCapacityTests {
	static var assertions=0;
	static function check(ok:Bool,message:String):Void {assertions++;if (!ok) throw message;}
	static function near(a:Float,b:Float,message:String):Void check(Math.abs(a-b)<1e-7,message+": "+a+" vs "+b);
	static function rejects(fn:Void->Void,expected:String):Void {
		var error="";try fn() catch (value:Dynamic) error=Std.string(value);
		check(error.indexOf(expected)>=0,"Expected "+expected+", got "+error);
	}
	public static function run(document:ToolingDocument):Void {
		var pi=Math.PI;
		near(Checks.projectedConeUnionArea([{x:0,y:0,radius:1},{x:3,y:0,radius:1}]),2*pi,"disjoint cone projections add");
		near(Checks.projectedConeUnionArea([{x:0,y:0,radius:1},{x:1,y:0,radius:1}]),4*pi/3+Math.sqrt(3)/2,
			"two overlapping cones agree with the independent circular-lens formula");
		near(Checks.projectedConeUnionArea([{x:0,y:0,radius:1},{x:1,y:0,radius:1},{x:0.5,y:Math.sqrt(3)/2,radius:1}]),
			1.5*pi+Math.sqrt(3),"triple overlap is not subtracted three times");
		near(Checks.projectedConeUnionArea([{x:0,y:0,radius:2},{x:0,y:0,radius:1},{x:0,y:0,radius:2}]),4*pi,
			"nested and coincident cones cannot multiply their credited area");
		near(Checks.projectedConeUnionArea([{x:1e8,y:-1e8,radius:1},{x:1e8+1,y:-1e8,radius:1}]),4*pi/3+Math.sqrt(3)/2,
			"projection translation does not destroy small shared areas");
		var insert=ToolingCodec.inputSnapshot(document.input).inserts[0];
		insert.body=Primitive(Box(100,100,30));insert.interfaces=[];
		insert.role=materia.casting.InsertSpec.InsertRole.LiftingMount;
		insert.holes=[];
		var anchors:Array<InsertBoltAnchor> = [];
		for (x in [-25.0,25.0]) {
			var id=x<0 ? "anchor-left" : "anchor-right",hole=TestInsertHoles.through(5,20)[0];
			hole.id=id;hole.frame=F.translation(x,0,0);hole.extent=Blind;
			hole.thread={designation:"M6",majorDiameter:6,pitch:1,starts:1,internal:true,toleranceClass:"6H",hand:Right,profile:Metric};
			hole.tapDrill=5;hole.threadDepth=18;hole.clearanceDiameter=6.6;
			hole.limits={policyId:"c45-class8.8-vdi2230-v3",threaded:{minEngagement:7.8,minTipGap:1},
				minBack:1.5,minWall:6,minClearance:0.6,overrides:[]};
			insert.holes.push(hole);
			anchors.push({id:id,holeId:id,insertionMm:8,locking:RetainingCompound,lockingReference:null,
				screw:{partNumber:"ISO4162-F-full-head-M6x20-8.8",thread:{designation:"M6",majorDiameter:6,pitch:1,starts:1,
					internal:false,toleranceClass:"6g",hand:Right,profile:Metric},lengthMm:20,threadedLengthMm:16,
					headHeightMm:6.8,headAcrossFlatsMm:8,flangeDiameterMm:13.6,flangeThicknessMm:1.1,
					bearingInnerDiameterMm:6.8,bearingOuterDiameterMm:11.6,assemblyPreloadN:9290,
					preloadReference:"https://optimas.com/technical-resources/pre-load-and-tightening-torques-of-metric-thread-steel-fasteners/"}});
		}
		TestInsertHoles.beforeCasting(insert);
		var rules:InsertAnchorRules={demand:{ruleId:"capacity-control-v1",factor:1.5,liftingShareN:10000,
			servicePreloads:[],servicePreloadReference:anchors[0].screw.preloadReference},minimumEmbedmentMm:6,
			minimumBearingAreaMm2:20,minimumSpacingMm:3,minimumCoverMm:3};
		insert.anchorage=Bolted(rules,anchors);
		// Validate the controlled source fixture before exercising its capacity inputs.
		C.references(C.snapshot(C.record(insert.anchorage)),insert.body,insert.holes,insert.origin,insert.interfaces);
		near(C.demandN(rules),15000,"named design factor applies to the resolved demand");
		var facts=[for (anchor in anchors) {anchorId:anchor.id,bearingAreaMm2:80.0,embedmentMm:20.0}];
		var unknown=Checks.materialChecks(insert,facts,null,null);
		check(unknown.length==4 && unknown.filter(row -> row.status==Pending).length==4,"unknown mix strengths keep exactly the two checks per anchor Pending");
		var pass=Checks.materialChecks(insert,facts,200,6.3);
		check(pass.length==4 && pass.filter(row -> row.status!=Pass).length==0,"both material checks pass only with sufficient supplied values");
		var fail=Checks.materialChecks(insert,facts,1,1);
		check(fail.filter(row -> row.status==Fail).length==4,"insufficient supplied strengths fail both checks");
		for (hole in insert.holes) hole.frame.x=hole.frame.x<0 ? -10 : 10;
		var overlap=Checks.materialChecks(insert,facts,200,6.3);
		check(overlap.filter(row -> StringTools.endsWith(row.rule,".cone") && row.status==Fail).length==2,
			"simple projected cone overlap prevents counting the same mix area twice");
		check(overlap.filter(row -> StringTools.endsWith(row.rule,".bearing") && row.status==Pass).length==2,
			"cone overlap does not invent a flange-bearing penalty");
		for (row in overlap) check(row.affectedIds.indexOf(insert.id)>=0 && row.affectedIds.length==2,
			"capacity finding names the source insert and anchor");
		var saved=ToolingCodec.canonical(insert);
		anchors.reverse();facts.reverse();
		var reversed=Checks.materialChecks(insert,facts,200,6.3);
		check(ToolingCodec.canonical(reversed)==ToolingCodec.canonical(overlap),"anchor and fact order cannot change capacities or rule order");
		anchors.reverse();
		check(ToolingCodec.canonical(insert)==saved,"capacity analysis does not mutate source design");
		rejects(() -> Checks.materialChecks(insert,facts,0,null),"positive");
		rejects(() -> Checks.materialChecks(insert,facts,null,Math.NaN),"positive");
		rejects(() -> Checks.materialChecks(insert,[],null,null),"differs from source");
		// Decimal catalogue dimensions can land a few ulps below exact contact.
		insert.holes[0].frame.x=0;
		insert.holes[1].frame.x=16.6-1e-14;
		var boundary=Checks.check(insert).filter(row -> row.rule.indexOf(".spacing.")>=0);
		check(boundary.length==1 && boundary[0].status==Pass,
			"exact minimum flange spacing admits floating-point roundoff");
		insert.holes[1].frame.x-=0.0001;
		var below=Checks.check(insert).filter(row -> row.rule.indexOf(".spacing.")>=0);
		check(below.length==1 && below[0].status==Fail,
			"a real flange-spacing shortfall still fails");
		Sys.println('AnchorCapacityTests: $assertions assertions passed');
	}
}
