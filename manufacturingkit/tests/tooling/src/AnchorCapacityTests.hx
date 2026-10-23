import materia.tooling.ToolingDocument;
import materia.tooling.ToolingCodec;
import materia.casting.InsertAnchorage.InsertAnchorageChecks as Checks;
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
	static function screenDistances():Void {
		function face(points:Array<materia.casting.InsertAnchorage.AnchorPoint>,normal:materia.casting.InsertAnchorage.AnchorPoint)
			:materia.casting.InsertAnchorage.AnchorFace
			return {normal:normal,edges:[for (i in 0...points.length) {a:points[i],b:points[(i+1)%points.length]}]};
		var origin={x:0.0,y:0.0,z:0.0},axis={x:0.0,y:0.0,z:1.0},normal={x:0.0,y:0.0,z:1.0};
		function rectangle(x0:Float,y0:Float,x1:Float,y1:Float,z:Float)
			return face([{x:x0,y:y0,z:z},{x:x1,y:y0,z:z},{x:x1,y:y1,z:z},{x:x0,y:y1,z:z}],normal);
		var horizontal=rectangle(-5,-5,5,5,3);
		near(InsertAnchorScreen.cylinderFace(origin,axis,1,1,horizontal),2,"cylinder cap to finite plane");
		near(InsertAnchorScreen.cylinderFace(origin,axis,1,1,rectangle(2,2,3,3,0.5)),Math.sqrt(8)-1,
			"finite corner distance differs from intersecting infinite planes");
		near(InsertAnchorScreen.cylinderFace(origin,axis,1,1,rectangle(0.4,-0.1,0.6,0.1,3)),2,
			"end-disc projection overlaps a face excluding its centre");
		var generator=face([{x:3.0,y:-0.2,z:0.3},{x:3.0,y:0.2,z:0.3},{x:3.0,y:0.2,z:0.7},{x:3.0,y:-0.2,z:0.7}],{x:1.0,y:0.0,z:0.0});
		near(InsertAnchorScreen.cylinderFace(origin,axis,1,1,generator),2,"side generator overlaps only inside the cylinder height");
		var diagonal=face([{x:-5.0,y:-5.0,z:10.0},{x:10.0,y:-5.0,z:-5.0},{x:10.0,y:5.0,z:-5.0},{x:-5.0,y:5.0,z:10.0}],
			{x:Math.sqrt(0.5),y:0.0,z:Math.sqrt(0.5)});
		near(InsertAnchorScreen.cylinderFace(origin,axis,1,2,diagonal),Math.sqrt(2),"oblique plane uses exact cylindrical support");
		var holed=rectangle(-5,-5,5,5,3);
		holed.edges=holed.edges.concat(rectangle(-2,-2,2,2,3).edges);
		near(InsertAnchorScreen.cylinderFace(origin,axis,1,1,holed),Math.sqrt(5),"inner trim stays an opening in a planar face");
		near(InsertAnchorScreen.hexFace(F.identity(),4,0,1,generator),1,"hex face uses actual flat rather than enclosing cylinder");
		near(InsertAnchorScreen.cylinderFace({x:2.0,y:0.0,z:0.0},axis,1,1,generator),0,"exact radial contact has zero distance");
	}
	public static function run(document:ToolingDocument):Void {
		screenDistances();
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
		var insert=ToolingCodec.copyInput(document.input).inserts[0];
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
			anchors.push({id:id,holeId:id,insertionMm:8,locking:RetainingCompound(materia.casting.CastingInput.CastingTextInput.Unresolved("Anchor retaining compound and cure/torque evidence")),
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
		var lockingRaw:Dynamic=C.record(insert.anchorage);
		var lockingRows:Array<Dynamic>=cast Reflect.field(lockingRaw,"anchors");
		Reflect.setField(lockingRows[0],"locking",{kind:"retaining-compound",reference:{kind:"known",value:"Declared process trial"}});
		var knownLock=C.snapshot(lockingRaw);
		check(switch knownLock {case Bolted(_,rows):switch rows[0].locking {
			case RetainingCompound(Known(reference)):reference=="Declared process trial";case _:false;};case _:false;},
			"retaining reference is an explicit typed input");
		Reflect.setField(lockingRows[0],"locking",{kind:"unknown-stop"});
		rejects(() -> C.snapshot(lockingRaw),"Unknown anchor locking");
		Reflect.setField(lockingRows[0],"locking","retaining-compound");
		rejects(() -> C.snapshot(lockingRaw),"Invalid casting text: kind");
		var facts=[for (anchor in anchors) {anchorId:anchor.id,bearingAreaMm2:80.0,embedmentMm:20.0}];
		var unknown=Checks.materialChecks(insert,facts,Unresolved("Bearing test"),Unresolved("Tensile test"));
		check(unknown.length==4 && unknown.filter(row -> row.status==Pending).length==4,"unknown mix strengths keep exactly the two checks per anchor Pending");
		check(unknown[0].message.indexOf("Bearing test")>=0 || unknown[0].message.indexOf("Tensile test")>=0,
			"typed unresolved strength retains its missing-input reason");
		var pass=Checks.materialChecks(insert,facts,Known(200),Known(6.3));
		check(pass.length==4 && pass.filter(row -> row.status!=Pass).length==0,"both material checks pass only with sufficient supplied values");
		var fail=Checks.materialChecks(insert,facts,Known(1),Known(1));
		check(fail.filter(row -> row.status==Fail).length==4,"insufficient supplied strengths fail both checks");
		for (hole in insert.holes) hole.frame.x=hole.frame.x<0 ? -10 : 10;
		var overlap=Checks.materialChecks(insert,facts,Known(200),Known(6.3));
		check(overlap.filter(row -> StringTools.endsWith(row.rule,".cone") && row.status==Fail).length==2,
			"simple projected cone overlap prevents counting the same mix area twice");
		check(overlap.filter(row -> StringTools.endsWith(row.rule,".bearing") && row.status==Pass).length==2,
			"cone overlap does not invent a flange-bearing penalty");
		for (row in overlap) check(row.affectedIds.indexOf(insert.id)>=0 && row.affectedIds.length==2,
			"capacity finding names the source insert and anchor");
		var saved=ToolingCodec.canonical(insert);
		anchors.reverse();facts.reverse();
		var reversed=Checks.materialChecks(insert,facts,Known(200),Known(6.3));
		check(ToolingCodec.canonical(reversed)==ToolingCodec.canonical(overlap),"anchor and fact order cannot change capacities or rule order");
		anchors.reverse();
		check(ToolingCodec.canonical(insert)==saved,"capacity analysis does not mutate source design");
		rejects(() -> Checks.materialChecks(insert,facts,Known(0),Unresolved("Tensile test")),"positive");
		rejects(() -> Checks.materialChecks(insert,facts,Unresolved("Bearing test"),Known(Math.NaN)),"positive");
		rejects(() -> Checks.materialChecks(insert,facts,Unresolved(""),Known(1)),"reason");
		rejects(() -> Checks.materialChecks(insert,[],Unresolved("Bearing test"),Unresolved("Tensile test")),"differs from source");
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
