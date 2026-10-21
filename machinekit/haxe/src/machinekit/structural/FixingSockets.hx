package machinekit.structural;

import machinekit.component.ComponentValues;
import machinekit.standard.FastenerThreads;
import materia.assembly.AssemblyFrames as F;
import materia.casting.CastingSolid;
import materia.casting.InsertSpec.InsertBody;
import materia.casting.InsertSpec.InsertBodyCodec;
import materia.casting.InsertHole;
import materia.casting.InsertMaterial;
import materia.casting.InsertOrigin;
import materia.casting.InsertSpec;

/** Undimensioned exterior choices belong to the source setup, never the supplier data. */
typedef SocketExteriorStudy = {
	var outsideDiameter:Float;
	var reliefDiameter:Float;
	var tailWidth:Float;
	var tailThickness:Float;
	var tailWave:Float;
	var tailBlend:Float;
}

/** Catalogue variants create the ordinary insert component, not another component family. */
class FixingSockets {
	public static inline var PFEIFER_REFERENCE = "05.250.064.040";
	public static inline var PFEIFER_REVISION = "sha256:8cc485b6908661ee4b21f9079ec052cff06361a94b47bb6994a5d57ba6cd1e98";
	public static inline var PFEIFER_DOCUMENT = "https://www.pfeifer.info/out/assets/PFEIFER_SOCKET-CRIMPED-END_PPEN.PDF";
	/** PFEIFER fixing socket, crimped end: M6 x 40, a=25, e=6; grade 1.4571.
	 * Catalogue concrete capacity is not mineral-cast anchorage evidence.
	 */
	public static inline var LENGTH = 40.0;
	public static inline var BORE_DEPTH = 25.0;
	public static inline var THREAD_LENGTH = 6.0;
	public static inline var POLICY = "pfeifer-05.250.064.040-fixing-study-v1";
	/** In-house study rules: one pitch at the blind bottom; >=0.8 mm sleeve
	 * ligament and >=0.5 mm closed-end material to avoid a resin path. They screen
	 * geometry only, not strength. The unverified exterior cannot qualify them.
	 */
	public static inline var TIP_GAP_PITCHES = 1.0;
	public static inline var MINIMUM_SLEEVE_WALL = 0.8;
	public static inline var MINIMUM_CLOSED_END = 0.5;
	/** In-house machining separation, independently of ISO 273 bolt clearance. */
	public static inline var FEATURE_SEPARATION_DIAMETERS = 0.1;

	public static function pfeiferStudy(exterior:SocketExteriorStudy, positionToleranceMm:Float):CastInsert {
		var choice = InsertHoles.metricThread(FastenerThreads.coarse(6).mating());
		var thread = choice.thread;
		var body = CrimpedSocket(Cylinder(exterior.outsideDiameter, BORE_DEPTH), {length: LENGTH - BORE_DEPTH, width: exterior.tailWidth,
			thickness: exterior.tailThickness, wave: exterior.tailWave, blend: exterior.tailBlend});
		InsertBodyCodec.validate(body);
		var hole:InsertHole = {
			id: "mounting", frame: F.fromRotationMatrix(0, 0, LENGTH, [1, 0, 0, 0, -1, 0, 0, 0, -1]),
			diameter: choice.tapDrill, depth: BORE_DEPTH, extent: Blind, thread: InsertHoles.threadRecord(thread),
			tapDrill: choice.tapDrill, threadDepth: THREAD_LENGTH, clearanceDiameter: choice.clearanceDiameter,
			counterbore: null, relief: {diameter: exterior.reliefDiameter, start: THREAD_LENGTH},
			limits: {
				policyId: POLICY, threaded: {minEngagement: THREAD_LENGTH, minTipGap: thread.pitch * TIP_GAP_PITCHES},
				minBack: MINIMUM_CLOSED_END, minWall: MINIMUM_SLEEVE_WALL,
				minClearance: thread.majorDiameter * FEATURE_SEPARATION_DIAMETERS, overrides: []
			}
		};
		var origin:InsertOrigin = Catalogue({manufacturer: "PFEIFER", reference: PFEIFER_REFERENCE, revision: PFEIFER_REVISION, holePolicyId: POLICY, document: PFEIFER_DOCUMENT,
			geometryBasis: NominalStudy, geometryFingerprint: null, evidence: []});
		return new CastInsert(new ComponentValues().setToken("body", InsertBodyCodec.encode(body))
			.setToken("holes", InsertHoles.fromNeutral([hole])).setToken("origin", haxe.Json.stringify(InsertOrigin.InsertOriginCodec.record(origin)))
			.setToken("anchorage",haxe.Json.stringify(materia.casting.InsertAnchorage.InsertAnchorageCodec.record(CatalogueRetention)))
			.setToken("stockMaterial", Stainless14571).setToken("role", ThreadedAnchor).setToken("finish", AsSupplied)
			.setToken("label", "catalogue fixing socket").setNumber("positionTolerance", positionToleranceMm));
	}
}
