package machinekit.motion;

private typedef LargeFrame = {
	var prefix:String;
	var suffix:String;
	var windingFrame:String;
	var voltage:Float;
	var face:Float;
	var boltCircle:Float;
	var holeDiameter:Float;
	var flangeThickness:Float;
	var pilotDiameter:Float;
	var pilotHeight:Float;
	var connectorHeight:Float;
	var ratedRpm:Float;
}

private typedef LargeVariant = {
	var watts:Int;
	var length:Float;
	var brakeLength:Float;
	var shaftDiameter:Float;
	var shaftLength:Float;
	var keyLength:Float;
	var keyWidth:Float;
	var keyHeight:Float;
	var keyFloor:Float;
	var ratedTorque:Float;
	var peakTorque:Float;
	var ratedCurrent:Float;
	var peakCurrent:Float;
	var maxRpm:Float;
	/** Source units kg m² × 10^-4. */
	var inertia:Float;
	var brakeInertia:Float;
	var mass:Float;
	var brakeMass:Float;
}

/** Larger ELM2 rows transcribed from LeadshineServo.SOURCE pp. 7–16.
 * Keep winding, inertia class and connector suffix in each designation.
 */
class LeadshineServoLargeCatalog {
	public static function rows():Array<LeadshineServoSpec> {
		var result:Array<LeadshineServoSpec> = [];
		function family(frame:LeadshineServoLargeCatalog.LargeFrame, variants:Array<LeadshineServoLargeCatalog.LargeVariant>):Void {
			for (v in variants) for (brake in [false, true]) result.push({
				designation: '${frame.prefix}-${StringTools.lpad(Std.string(v.watts), "0", 4)}${frame.windingFrame}${brake ? "E" : "F"}${frame.suffix}',
				series: "ELM2", brake: brake, watts: v.watts, voltage: frame.voltage,
				ratedRpm: frame.ratedRpm, maxRpm: v.maxRpm, face: frame.face,
				bodyLength: brake ? v.brakeLength : v.length, shaftDiameter: v.shaftDiameter, shaftLength: v.shaftLength,
				boltCircle: frame.boltCircle, holeDiameter: frame.holeDiameter, flangeThickness: frame.flangeThickness,
				pilotDiameter: frame.pilotDiameter, pilotHeight: frame.pilotHeight, connectorHeight: frame.connectorHeight,
				keyLength: v.keyLength, keyWidth: v.keyWidth, keyHeight: v.keyHeight, keyFloor: v.keyFloor,
				ratedTorque: v.ratedTorque, peakTorque: v.peakTorque, ratedCurrent: v.ratedCurrent, peakCurrent: v.peakCurrent,
				rotorInertia: (brake ? v.brakeInertia : v.inertia)*1e-4, mass: brake ? v.brakeMass : v.mass
			});
		}
		// Dimension and rating tables distinguish voltage and connector variants.
		family({prefix:"ELM2H", suffix:"-HD", windingFrame:"LD130", voltage:220, face:130, boltCircle:145, holeDiameter:8.5,
			flangeThickness:17.5, pilotDiameter:110, pilotHeight:6, connectorHeight:106, ratedRpm:1500}, [
			{watts:850,length:147,brakeLength:172,shaftDiameter:19,shaftLength:57,keyLength:30,keyWidth:5,keyHeight:5,keyFloor:16,
				ratedTorque:5.4,peakTorque:16.2,ratedCurrent:6.5,peakCurrent:19.5,maxRpm:3000,inertia:13.88,brakeInertia:15.78,mass:5.6,brakeMass:6.9},
			{watts:1300,length:168,brakeLength:192,shaftDiameter:22,shaftLength:57,keyLength:30,keyWidth:6,keyHeight:6,keyFloor:18.5,
				ratedTorque:8.4,peakTorque:25.2,ratedCurrent:9.5,peakCurrent:28.5,maxRpm:3000,inertia:20.59,brakeInertia:22.26,mass:7.5,brakeMass:8.8},
			{watts:1800,length:195,brakeLength:219,shaftDiameter:24,shaftLength:57,keyLength:30,keyWidth:8,keyHeight:8,keyFloor:19,
				ratedTorque:11.5,peakTorque:34.5,ratedCurrent:9,peakCurrent:28.4,maxRpm:2000,inertia:23.69,brakeInertia:25.36,mass:8.1,brakeMass:9.4}
		]);
		family({prefix:"ELM2M", suffix:"-H", windingFrame:"LB130", voltage:220, face:131, boltCircle:145, holeDiameter:9,
			flangeThickness:14, pilotDiameter:110, pilotHeight:5, connectorHeight:111, ratedRpm:2500}, [
			{watts:1000,length:166,brakeLength:223,shaftDiameter:22,shaftLength:57,keyLength:40,keyWidth:6,keyHeight:6,keyFloor:18.5,
				ratedTorque:4,peakTorque:12,ratedCurrent:4,peakCurrent:12,maxRpm:2600,inertia:8.5,brakeInertia:8.95,mass:6.2,brakeMass:8.3},
			{watts:1500,length:179,brakeLength:236,shaftDiameter:22,shaftLength:57,keyLength:40,keyWidth:6,keyHeight:6,keyFloor:18.5,
				ratedTorque:6,peakTorque:18,ratedCurrent:6,peakCurrent:18,maxRpm:2600,inertia:12.6,brakeInertia:12.6,mass:7.4,brakeMass:9.5},
			{watts:2000,length:192,brakeLength:270,shaftDiameter:22,shaftLength:57,keyLength:40,keyWidth:6,keyHeight:6,keyFloor:18.5,
				ratedTorque:7.7,peakTorque:22,ratedCurrent:7.5,peakCurrent:22,maxRpm:2800,inertia:15.3,brakeInertia:16.8,mass:8.3,brakeMass:11}
		]);
		family({prefix:"ELM2L", suffix:"T-H", windingFrame:"LA100", voltage:380, face:100, boltCircle:115, holeDiameter:9,
			flangeThickness:12, pilotDiameter:95, pilotHeight:5, connectorHeight:111, ratedRpm:3000}, [
			{watts:1000,length:154,brakeLength:194,shaftDiameter:24,shaftLength:45,keyLength:36,keyWidth:8,keyHeight:7,keyFloor:20,
				ratedTorque:3.2,peakTorque:9.6,ratedCurrent:3.9,peakCurrent:11.7,maxRpm:6000,inertia:2.43,brakeInertia:2.63,mass:4.6,brakeMass:5.9},
			{watts:1500,length:178,brakeLength:218,shaftDiameter:24,shaftLength:45,keyLength:36,keyWidth:8,keyHeight:7,keyFloor:20,
				ratedTorque:4.9,peakTorque:14.7,ratedCurrent:5.1,peakCurrent:14.4,maxRpm:5000,inertia:3.503,brakeInertia:3.503,mass:5.8,brakeMass:7.1}
		]);
		family({prefix:"ELM2H", suffix:"T-H", windingFrame:"LD130", voltage:380, face:130, boltCircle:145, holeDiameter:8.5,
			flangeThickness:17.5, pilotDiameter:110, pilotHeight:4, connectorHeight:106, ratedRpm:1500}, [
			{watts:850,length:147,brakeLength:172,shaftDiameter:19,shaftLength:55,keyLength:30,keyWidth:5,keyHeight:5,keyFloor:16,
				ratedTorque:5.4,peakTorque:16.2,ratedCurrent:3.5,peakCurrent:10.5,maxRpm:3000,inertia:13.88,brakeInertia:15.78,mass:5.6,brakeMass:6.9},
			{watts:1300,length:168,brakeLength:192,shaftDiameter:22,shaftLength:55,keyLength:30,keyWidth:6,keyHeight:6,keyFloor:18.5,
				ratedTorque:8.4,peakTorque:25.2,ratedCurrent:4.8,peakCurrent:14.4,maxRpm:3000,inertia:20.59,brakeInertia:22.26,mass:7.5,brakeMass:8.8},
			{watts:1800,length:195,brakeLength:219,shaftDiameter:24,shaftLength:55,keyLength:30,keyWidth:8,keyHeight:8,keyFloor:19,
				ratedTorque:11.5,peakTorque:34.5,ratedCurrent:6.5,peakCurrent:19.5,maxRpm:3000,inertia:23.69,brakeInertia:25.36,mass:8.1,brakeMass:9.4}
		]);
		family({prefix:"ELM2M", suffix:"T-H", windingFrame:"LD180", voltage:380, face:180, boltCircle:200, holeDiameter:13.5,
			flangeThickness:18, pilotDiameter:114, pilotHeight:3.2, connectorHeight:134, ratedRpm:1500}, [
			{watts:2900,length:201,brakeLength:249,shaftDiameter:35,shaftLength:79,keyLength:65,keyWidth:10,keyHeight:8,keyFloor:30,
				ratedTorque:18.6,peakTorque:55.8,ratedCurrent:11.8,peakCurrent:35.4,maxRpm:3000,inertia:56.8,brakeInertia:65.3,mass:16.3,brakeMass:21.8},
			{watts:4400,length:230,brakeLength:278,shaftDiameter:35,shaftLength:79,keyLength:65,keyWidth:10,keyHeight:8,keyFloor:30,
				ratedTorque:28.4,peakTorque:85,ratedCurrent:15.7,peakCurrent:47.1,maxRpm:3000,inertia:78.2,brakeInertia:86,mass:21.4,brakeMass:26.5},
			{watts:5500,length:257,brakeLength:305,shaftDiameter:42,shaftLength:113,keyLength:96,keyWidth:12,keyHeight:8,keyFloor:37,
				ratedTorque:35,peakTorque:87.5,ratedCurrent:20.6,peakCurrent:51.5,maxRpm:3000,inertia:109,brakeInertia:118,mass:25.8,brakeMass:31.1},
			{watts:7500,length:297,brakeLength:345,shaftDiameter:42,shaftLength:113,keyLength:96,keyWidth:12,keyHeight:8,keyFloor:37,
				ratedTorque:48,peakTorque:119,ratedCurrent:25.7,peakCurrent:64.5,maxRpm:3000,inertia:130,brakeInertia:140,mass:32.3,brakeMass:37.6}
		]);
		return result;
	}
}
