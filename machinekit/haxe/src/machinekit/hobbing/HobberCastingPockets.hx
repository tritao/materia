package machinekit.hobbing;

import materia.casting.CastingFeature;
import materia.casting.CastingPatterns;
import machinekit.structural.CastingFeaturePatterns;

/** Direct pocket spans between named ribs, rail beams, bosses and struts.
 * These declarations preserve the reviewed plinth before its approved filling
 * correction. There is no source grid, retained-box subtraction or hashed ID.
 */
class HobberCastingPockets {
	static inline final DRIVE_CAP_TOP = 720.0;
	static inline final SOCKET_BOSS_DEPTH = 60.0;

	public static function plinth(width:Float, depth:Float, height:Float):Array<CastingFeature> {
		var layout = HobberXLayout.front();
		var wall = 25.0, socketBossHalf = 20.0, socketEdgeInset = 35.0;
		var footInset = 120.0, footBossHalf = FoundationLayout.FOOT_BOSS_WIDTH / 2;
		var tableHalf = 140.0, ribHalf = FoundationLayout.PLINTH_CORE_WEB / 2;
		var driveRibHalf = FoundationLayout.DRIVE_RIB_HALF_WIDTH;
		var enclosureInset = 100.0, enclosureBossHalf = 60.0, enclosureBossDepth = 70.0;
		var railCentre = FoundationLayout.BED_RAIL_CENTRE, railBeamHalf = FoundationLayout.BED_RAIL_BEAM_WIDTH / 2;
		var capHalf = 150.0, webCentre = 112.5;
		var frontStrutX = 100.0, middleStrutX = 160.0, rearStrutX = 220.0, strutY = 80.0;
		var strutHalf = FoundationLayout.PRECAST_STRUT_SECTION / 2;
		var xFrontWall = -width / 2 + wall;
		var xFrontSocketStart = -width / 2 + enclosureInset - enclosureBossHalf;
		var xFrontSocketEnd = -width / 2 + socketEdgeInset + socketBossHalf;
		var xTableFront = -tableHalf - layout.bedCentre;
		var xFrontFootRear = -width / 2 + footInset + footBossHalf;
		var xTableRear = tableHalf - layout.bedCentre;
		var xCentreSocketLeft = -socketBossHalf;
		var xCentreWebLeft = -ribHalf;
		var xCentreWebRight = ribHalf;
		var xCentreSocketRight = socketBossHalf;
		var xDriveRibFront = layout.floatingCentre - layout.bedCentre - driveRibHalf;
		var xDriveRibRear = layout.floatingCentre - layout.bedCentre + driveRibHalf;
		var xFrontStrutLeft = frontStrutX - strutHalf;
		var xFrontStrutRight = frontStrutX + strutHalf;
		var xMiddleStrutLeft = middleStrutX - strutHalf;
		var xMiddleStrutRight = middleStrutX + strutHalf;
		var xRearStrutLeft = rearStrutX - strutHalf;
		var xRearStrutRight = rearStrutX + strutHalf;
		var xRearFootFront = width / 2 - footInset - footBossHalf;
		var xMotorRibFront = layout.screwStart - layout.bedCentre - driveRibHalf;
		var xMotorRibRear = layout.screwStart - layout.bedCentre + driveRibHalf;
		var xRearEnclosureStart = width / 2 - enclosureInset - enclosureBossHalf;
		var xRearSocketStart = width / 2 - socketEdgeInset - socketBossHalf;
		var xRearSocketEnd = width / 2 - enclosureInset + enclosureBossHalf;
		var xRearWall = width / 2 - wall;
		var xWellFront = FoundationLayout.WORK_WELL_CENTRE_X - layout.bedCentre - FoundationLayout.WORK_WELL_WIDTH / 2;
		var xWellRear = FoundationLayout.WORK_WELL_CENTRE_X - layout.bedCentre + FoundationLayout.WORK_WELL_WIDTH / 2;
		var xChannelFront = layout.deckStart - FoundationLayout.DRIVE_CHANNEL_FRONT_CLEARANCE - layout.bedCentre;
		var yLeftWall = -depth / 2 + wall;
		var yLeftSocketInside = -depth / 2 + socketEdgeInset + socketBossHalf;
		var yLeftEnclosureInside = -depth / 2 + enclosureBossDepth;
		var yLeftRailOutside = -railCentre - railBeamHalf;
		var yLeftRailInside = -railCentre + railBeamHalf;
		var yLeftCap = -capHalf;
		var yLeftTable = -tableHalf;
		var yLeftWebOutside = -webCentre - ribHalf;
		var yLeftStrutOutside = -strutY - strutHalf;
		var yLeftWebInside = -webCentre + ribHalf;
		var yLeftStrutInside = -strutY + strutHalf;
		var yRightStrutInside = strutY - strutHalf;
		var yRightWebInside = webCentre - ribHalf;
		var yRightStrutOutside = strutY + strutHalf;
		var yRightWebOutside = webCentre + ribHalf;
		var yRightTable = tableHalf;
		var yRightCap = capHalf;
		var yRightRailInside = railCentre - railBeamHalf;
		var yRightRailOutside = railCentre + railBeamHalf;
		var yRightEnclosureInside = depth / 2 - enclosureBossDepth;
		var yRightSocketInside = depth / 2 - socketEdgeInset - socketBossHalf;
		var yRightWall = depth / 2 - wall;
		var zFloor = FoundationLayout.PLINTH_CORE_FLOOR;
		var zFootRoof = FoundationLayout.FOOT_SUPPORT_TOP;
		var zCapSole = FoundationLayout.DRIVE_CAP_UNDERSIDE;
		var zCapTop = DRIVE_CAP_TOP;
		var zSocketSole = height - SOCKET_BOSS_DEPTH;
		var zRoof = height - wall;
		var zWellFloor = FoundationLayout.TABLE_WELL_FLOOR_HEIGHT;
		return [
			pocket("pocket-centreSocketLeft-leftSocketInside-floor",
				xCentreSocketLeft, xCentreWebLeft, yLeftSocketInside, yLeftRailOutside, zFloor, zRoof,
				[
					"pocket-centreSocketLeft-leftWall-floor",
					"pocket-frontFootRear-leftWall-floor"
				]),
			pocket("pocket-centreSocketLeft-leftWall-floor",
				xCentreSocketLeft, xCentreWebLeft, yLeftWall, yLeftSocketInside, zFloor, zSocketSole,
				[
					"pocket-centreSocketLeft-leftSocketInside-floor",
					"pocket-frontFootRear-leftWall-floor"
				]),
			pocket("pocket-centreSocketLeft-rightRailOutside-floor",
				xCentreSocketLeft, xCentreWebLeft, yRightRailOutside, yRightSocketInside, zFloor, zRoof,
				[
					"cable-route",
					"pocket-centreSocketLeft-rightSocketInside-floor",
					"pocket-frontFootRear-rightRailOutside-floor"
				]),
			pocket("pocket-centreSocketLeft-rightSocketInside-floor",
				xCentreSocketLeft, xCentreWebLeft, yRightSocketInside, yRightWall, zFloor, zSocketSole,
				[
					"pocket-centreSocketLeft-rightRailOutside-floor",
					"pocket-frontFootRear-rightRailOutside-floor"
				]),
			pocket("pocket-centreSocketRight-leftWall-floor",
				xCentreSocketRight, xRearEnclosureStart, yLeftWall, yLeftRailOutside, zFloor, zRoof,
				[
					"pocket-centreWebRight-leftSocketInside-floor",
					"pocket-centreWebRight-leftWall-floor",
					"pocket-rearEnclosureStart-leftEnclosureInside-floor",
					"pocket-rearEnclosureStart-leftWall-floor"
				]),
			pocket("pocket-centreSocketRight-rightRailOutside-floor",
				xCentreSocketRight, xRearEnclosureStart, yRightRailOutside, yRightWall, zFloor, zRoof,
				[
					"pocket-centreWebRight-rightRailOutside-floor",
					"pocket-centreWebRight-rightSocketInside-floor",
					"pocket-rearEnclosureStart-rightEnclosureInside-floor",
					"pocket-rearEnclosureStart-rightRailOutside-floor"
				]),
			pocket("pocket-centreWebRight-leftCap-capTop",
				xCentreWebRight, xFrontStrutLeft, yLeftCap, yLeftStrutInside, zCapTop, zRoof,
				[
					"X-drive-channel",
					"pocket-centreWebRight-leftRailInside-floor",
					"pocket-driveRibFront-leftRailInside-capTop",
					"pocket-driveRibRear-leftRailInside-floor",
					"pocket-frontStrutLeft-leftCap-capTop"
				]),
			pocket("pocket-centreWebRight-leftCap-floor",
				xCentreWebRight, xDriveRibFront, yLeftCap, yLeftWebOutside, zFloor, zCapSole,
				[
					"pocket-centreWebRight-leftRailInside-floor"
				]),
			pocket("pocket-centreWebRight-leftRailInside-floor",
				xCentreWebRight, xDriveRibFront, yLeftRailInside, yLeftCap, zFloor, zRoof,
				[
					"pocket-centreWebRight-leftCap-capTop",
					"pocket-centreWebRight-leftCap-floor",
					"pocket-driveRibFront-leftRailInside-capTop"
				]),
			pocket("pocket-centreWebRight-leftSocketInside-floor",
				xCentreWebRight, xCentreSocketRight, yLeftSocketInside, yLeftRailOutside, zFloor, zRoof,
				[
					"pocket-centreSocketRight-leftWall-floor",
					"pocket-centreWebRight-leftWall-floor"
				]),
			pocket("pocket-centreWebRight-leftWall-floor",
				xCentreWebRight, xCentreSocketRight, yLeftWall, yLeftSocketInside, zFloor, zSocketSole,
				[
					"pocket-centreSocketRight-leftWall-floor",
					"pocket-centreWebRight-leftSocketInside-floor"
				]),
			pocket("pocket-centreWebRight-rightCap-floor",
				xCentreWebRight, xDriveRibFront, yRightCap, yRightRailInside, zFloor, zRoof,
				[
					"pocket-centreWebRight-rightStrutInside-capTop",
					"pocket-centreWebRight-rightWebOutside-floor",
					"pocket-driveRibFront-rightCap-capTop"
				]),
			pocket("pocket-centreWebRight-rightRailOutside-floor",
				xCentreWebRight, xCentreSocketRight, yRightRailOutside, yRightSocketInside, zFloor, zRoof,
				[
					"cable-route",
					"pocket-centreSocketRight-rightRailOutside-floor",
					"pocket-centreWebRight-rightSocketInside-floor"
				]),
			pocket("pocket-centreWebRight-rightSocketInside-floor",
				xCentreWebRight, xCentreSocketRight, yRightSocketInside, yRightWall, zFloor, zSocketSole,
				[
					"pocket-centreSocketRight-rightRailOutside-floor",
					"pocket-centreWebRight-rightRailOutside-floor"
				]),
			pocket("pocket-centreWebRight-rightStrutInside-capTop",
				xCentreWebRight, xFrontStrutLeft, yRightStrutInside, yRightCap, zCapTop, zRoof,
				[
					"X-drive-channel",
					"pocket-centreWebRight-rightCap-floor",
					"pocket-driveRibFront-rightCap-capTop",
					"pocket-driveRibRear-rightCap-floor",
					"pocket-frontStrutLeft-rightStrutInside-capTop"
				]),
			pocket("pocket-centreWebRight-rightWebOutside-floor",
				xCentreWebRight, xDriveRibFront, yRightWebOutside, yRightCap, zFloor, zCapSole,
				[
					"pocket-centreWebRight-rightCap-floor"
				]),
			pocket("pocket-driveRibFront-leftRailInside-capTop",
				xDriveRibFront, xDriveRibRear, yLeftRailInside, yLeftCap, zCapTop, zRoof,
				[
					"pocket-centreWebRight-leftCap-capTop",
					"pocket-centreWebRight-leftRailInside-floor",
					"pocket-driveRibRear-leftRailInside-floor"
				]),
			pocket("pocket-driveRibFront-rightCap-capTop",
				xDriveRibFront, xDriveRibRear, yRightCap, yRightRailInside, zCapTop, zRoof,
				[
					"pocket-centreWebRight-rightCap-floor",
					"pocket-centreWebRight-rightStrutInside-capTop",
					"pocket-driveRibRear-rightCap-floor"
				]),
			pocket("pocket-driveRibRear-leftCap-floor",
				xDriveRibRear, xMotorRibFront, yLeftCap, yLeftWebOutside, zFloor, zCapSole,
				[
					"pocket-driveRibRear-leftRailInside-floor"
				]),
			pocket("pocket-driveRibRear-leftRailInside-floor",
				xDriveRibRear, xMotorRibFront, yLeftRailInside, yLeftCap, zFloor, zRoof,
				[
					"pocket-centreWebRight-leftCap-capTop",
					"pocket-driveRibFront-leftRailInside-capTop",
					"pocket-driveRibRear-leftCap-floor",
					"pocket-frontStrutLeft-leftCap-capTop",
					"pocket-frontStrutRight-leftCap-capTop",
					"pocket-middleStrutLeft-leftCap-capTop",
					"pocket-middleStrutRight-leftCap-capTop",
					"pocket-motorRibFront-leftRailInside-capTop",
					"pocket-rearStrutLeft-leftCap-capTop",
					"pocket-rearStrutRight-leftCap-capTop"
				]),
			pocket("pocket-driveRibRear-leftWebInside-floor",
				xDriveRibRear, xRearFootFront, yLeftWebInside, yRightWebInside, zFloor, zCapSole,
				[
					"pocket-rearFootFront-leftWebInside-footRoof"
				]),
			pocket("pocket-driveRibRear-rightCap-floor",
				xDriveRibRear, xMotorRibFront, yRightCap, yRightRailInside, zFloor, zRoof,
				[
					"pocket-centreWebRight-rightStrutInside-capTop",
					"pocket-driveRibFront-rightCap-capTop",
					"pocket-driveRibRear-rightWebOutside-floor",
					"pocket-frontStrutLeft-rightStrutInside-capTop",
					"pocket-frontStrutRight-rightStrutInside-capTop",
					"pocket-middleStrutLeft-rightStrutOutside-capTop",
					"pocket-middleStrutRight-rightStrutInside-capTop",
					"pocket-motorRibFront-rightCap-capTop",
					"pocket-rearStrutLeft-rightStrutInside-capTop",
					"pocket-rearStrutRight-rightStrutInside-capTop"
				]),
			pocket("pocket-driveRibRear-rightWebOutside-floor",
				xDriveRibRear, xMotorRibFront, yRightWebOutside, yRightCap, zFloor, zCapSole,
				[
					"pocket-driveRibRear-rightCap-floor"
				]),
			pocket("pocket-frontFootRear-leftWall-floor",
				xFrontFootRear, xCentreSocketLeft, yLeftWall, yLeftRailOutside, zFloor, zRoof,
				[
					"pocket-centreSocketLeft-leftSocketInside-floor",
					"pocket-centreSocketLeft-leftWall-floor",
					"pocket-tableFront-leftWall-footRoof"
				]),
			pocket("pocket-frontFootRear-rightRailOutside-floor",
				xFrontFootRear, xCentreSocketLeft, yRightRailOutside, yRightWall, zFloor, zRoof,
				[
					"pocket-centreSocketLeft-rightRailOutside-floor",
					"pocket-centreSocketLeft-rightSocketInside-floor",
					"pocket-tableFront-rightRailOutside-footRoof"
				]),
			pocket("pocket-frontSocketEnd-leftRailInside-floor",
				xFrontSocketEnd, xTableFront, yLeftRailInside, yRightRailInside, zFloor, zWellFloor,
				[
					"chip-drain",
					"pocket-tableFront-leftRailInside-floor",
					"pocket-tableFront-rightTable-floor",
					"pocket-wellFront-leftRailInside-floor",
					"pocket-wellFront-leftWebOutside-floor",
					"pocket-wellFront-rightWebOutside-floor",
					"work-well"
				]),
			pocket("pocket-frontSocketStart-leftEnclosureInside-footRoof",
				xFrontSocketStart, xTableFront, yLeftEnclosureInside, yLeftRailOutside, zFootRoof, zRoof,
				[
					"foot0-stem-upper",
					"pocket-frontSocketStart-leftWall-footRoof",
					"pocket-frontWall-leftWall-footRoof",
					"pocket-tableFront-leftWall-footRoof"
				]),
			pocket("pocket-frontSocketStart-leftWall-footRoof",
				xFrontSocketStart, xTableFront, yLeftWall, yLeftEnclosureInside, zFootRoof, zCapSole,
				[
					"foot0-stem-upper",
					"pocket-frontSocketStart-leftEnclosureInside-footRoof",
					"pocket-frontWall-leftWall-footRoof",
					"pocket-tableFront-leftWall-footRoof"
				]),
			pocket("pocket-frontSocketStart-rightEnclosureInside-footRoof",
				xFrontSocketStart, xTableFront, yRightEnclosureInside, yRightWall, zFootRoof, zCapSole,
				[
					"foot1-stem-upper",
					"pocket-frontSocketStart-rightRailOutside-footRoof",
					"pocket-frontWall-rightRailOutside-footRoof",
					"pocket-tableFront-rightRailOutside-footRoof"
				]),
			pocket("pocket-frontSocketStart-rightRailOutside-footRoof",
				xFrontSocketStart, xTableFront, yRightRailOutside, yRightEnclosureInside, zFootRoof, zRoof,
				[
					"foot1-stem-upper",
					"pocket-frontSocketStart-rightEnclosureInside-footRoof",
					"pocket-frontWall-rightRailOutside-footRoof",
					"pocket-tableFront-rightRailOutside-footRoof"
				]),
			pocket("pocket-frontStrutLeft-leftCap-capTop",
				xFrontStrutLeft, xFrontStrutRight, yLeftCap, yLeftStrutOutside, zCapTop, zRoof,
				[
					"pocket-centreWebRight-leftCap-capTop",
					"pocket-driveRibRear-leftRailInside-floor",
					"pocket-frontStrutRight-leftCap-capTop"
				]),
			pocket("pocket-frontStrutLeft-rightStrutInside-capTop",
				xFrontStrutLeft, xFrontStrutRight, yRightStrutInside, yRightCap, zCapTop, zRoof,
				[
					"X-drive-channel",
					"pocket-centreWebRight-rightStrutInside-capTop",
					"pocket-driveRibRear-rightCap-floor",
					"pocket-frontStrutRight-rightStrutInside-capTop"
				]),
			pocket("pocket-frontStrutRight-leftCap-capTop",
				xFrontStrutRight, xMiddleStrutLeft, yLeftCap, yLeftStrutInside, zCapTop, zRoof,
				[
					"X-drive-channel",
					"pocket-driveRibRear-leftRailInside-floor",
					"pocket-frontStrutLeft-leftCap-capTop",
					"pocket-middleStrutLeft-leftCap-capTop"
				]),
			pocket("pocket-frontStrutRight-rightStrutInside-capTop",
				xFrontStrutRight, xMiddleStrutLeft, yRightStrutInside, yRightCap, zCapTop, zRoof,
				[
					"X-drive-channel",
					"pocket-driveRibRear-rightCap-floor",
					"pocket-frontStrutLeft-rightStrutInside-capTop",
					"pocket-middleStrutLeft-rightStrutOutside-capTop"
				]),
			pocket("pocket-frontWall-leftRailInside-floor",
				xFrontWall, xWellFront, yLeftRailInside, yLeftWebOutside, zFloor, zSocketSole,
				[
					"pocket-frontWall-leftWebOutside-floor",
					"pocket-wellFront-leftRailInside-floor",
					"work-well"
				]),
			pocket("pocket-frontWall-leftWall-footRoof",
				xFrontWall, xFrontSocketStart, yLeftWall, yLeftRailOutside, zFootRoof, zRoof,
				[
					"pocket-frontSocketStart-leftEnclosureInside-footRoof",
					"pocket-frontSocketStart-leftWall-footRoof"
				]),
			pocket("pocket-frontWall-leftWebOutside-floor",
				xFrontWall, xWellFront, yLeftWebOutside, yRightWebOutside, zFloor, zRoof,
				[
					"coolant-front-access",
					"pocket-frontWall-leftRailInside-floor",
					"pocket-frontWall-rightWebOutside-floor",
					"pocket-wellFront-leftWebOutside-floor",
					"work-well"
				]),
			pocket("pocket-frontWall-rightRailOutside-footRoof",
				xFrontWall, xFrontSocketStart, yRightRailOutside, yRightWall, zFootRoof, zRoof,
				[
					"pocket-frontSocketStart-rightEnclosureInside-footRoof",
					"pocket-frontSocketStart-rightRailOutside-footRoof"
				]),
			pocket("pocket-frontWall-rightWebOutside-floor",
				xFrontWall, xWellFront, yRightWebOutside, yRightRailInside, zFloor, zSocketSole,
				[
					"pocket-frontWall-leftWebOutside-floor",
					"pocket-wellFront-rightWebOutside-floor",
					"work-well"
				]),
			pocket("pocket-middleStrutLeft-leftCap-capTop",
				xMiddleStrutLeft, xMiddleStrutRight, yLeftCap, yLeftStrutInside, zCapTop, zRoof,
				[
					"X-drive-channel",
					"pocket-driveRibRear-leftRailInside-floor",
					"pocket-frontStrutRight-leftCap-capTop",
					"pocket-middleStrutRight-leftCap-capTop"
				]),
			pocket("pocket-middleStrutLeft-rightStrutOutside-capTop",
				xMiddleStrutLeft, xMiddleStrutRight, yRightStrutOutside, yRightCap, zCapTop, zRoof,
				[
					"pocket-driveRibRear-rightCap-floor",
					"pocket-frontStrutRight-rightStrutInside-capTop",
					"pocket-middleStrutRight-rightStrutInside-capTop"
				]),
			pocket("pocket-middleStrutRight-leftCap-capTop",
				xMiddleStrutRight, xRearStrutLeft, yLeftCap, yLeftStrutInside, zCapTop, zRoof,
				[
					"X-drive-channel",
					"pocket-driveRibRear-leftRailInside-floor",
					"pocket-middleStrutLeft-leftCap-capTop",
					"pocket-rearStrutLeft-leftCap-capTop"
				]),
			pocket("pocket-middleStrutRight-rightStrutInside-capTop",
				xMiddleStrutRight, xRearStrutLeft, yRightStrutInside, yRightCap, zCapTop, zRoof,
				[
					"X-drive-channel",
					"pocket-driveRibRear-rightCap-floor",
					"pocket-middleStrutLeft-rightStrutOutside-capTop",
					"pocket-rearStrutLeft-rightStrutInside-capTop"
				]),
			pocket("pocket-motorRibFront-leftRailInside-capTop",
				xMotorRibFront, xMotorRibRear, yLeftRailInside, yLeftCap, zCapTop, zRoof,
				[
					"pocket-driveRibRear-leftRailInside-floor",
					"pocket-motorRibRear-leftRailInside-floor",
					"pocket-rearStrutRight-leftCap-capTop"
				]),
			pocket("pocket-motorRibFront-rightCap-capTop",
				xMotorRibFront, xMotorRibRear, yRightCap, yRightRailInside, zCapTop, zRoof,
				[
					"pocket-driveRibRear-rightCap-floor",
					"pocket-motorRibRear-rightCap-floor",
					"pocket-rearStrutRight-rightStrutInside-capTop"
				]),
			pocket("pocket-motorRibRear-leftCap-floor",
				xMotorRibRear, xRearWall, yLeftCap, yLeftWebOutside, zFloor, zCapSole,
				[
					"pocket-motorRibRear-leftRailInside-floor",
					"pocket-rearSocketStart-leftRailInside-floor"
				]),
			pocket("pocket-motorRibRear-leftRailInside-floor",
				xMotorRibRear, xRearSocketStart, yLeftRailInside, yLeftCap, zFloor, zRoof,
				[
					"pocket-motorRibFront-leftRailInside-capTop",
					"pocket-motorRibRear-leftCap-floor",
					"pocket-rearSocketStart-leftRailInside-floor",
					"pocket-rearStrutRight-leftCap-capTop"
				]),
			pocket("pocket-motorRibRear-leftWebInside-footRoof",
				xMotorRibRear, xRearWall, yLeftWebInside, yRightWebInside, zFootRoof, zCapSole,
				[
					"foot2-stem-upper"
				]),
			pocket("pocket-motorRibRear-rightCap-floor",
				xMotorRibRear, xRearSocketStart, yRightCap, yRightRailInside, zFloor, zRoof,
				[
					"pocket-motorRibFront-rightCap-capTop",
					"pocket-motorRibRear-rightWebOutside-floor",
					"pocket-rearSocketStart-rightCap-floor",
					"pocket-rearStrutRight-rightStrutInside-capTop"
				]),
			pocket("pocket-motorRibRear-rightWebOutside-floor",
				xMotorRibRear, xRearWall, yRightWebOutside, yRightCap, zFloor, zCapSole,
				[
					"pocket-motorRibRear-rightCap-floor",
					"pocket-rearSocketStart-rightCap-floor"
				]),
			pocket("pocket-rearEnclosureStart-leftEnclosureInside-floor",
				xRearEnclosureStart, xRearSocketEnd, yLeftEnclosureInside, yLeftRailOutside, zFloor, zRoof,
				[
					"pocket-centreSocketRight-leftWall-floor",
					"pocket-rearEnclosureStart-leftWall-floor",
					"pocket-rearSocketEnd-leftWall-floor"
				]),
			pocket("pocket-rearEnclosureStart-leftWall-floor",
				xRearEnclosureStart, xRearSocketEnd, yLeftWall, yLeftEnclosureInside, zFloor, zCapSole,
				[
					"pocket-centreSocketRight-leftWall-floor",
					"pocket-rearEnclosureStart-leftEnclosureInside-floor",
					"pocket-rearSocketEnd-leftWall-floor"
				]),
			pocket("pocket-rearEnclosureStart-rightEnclosureInside-floor",
				xRearEnclosureStart, xRearSocketEnd, yRightEnclosureInside, yRightWall, zFloor, zCapSole,
				[
					"pocket-centreSocketRight-rightRailOutside-floor",
					"pocket-rearEnclosureStart-rightRailOutside-floor",
					"pocket-rearSocketEnd-rightRailOutside-floor"
				]),
			pocket("pocket-rearEnclosureStart-rightRailOutside-floor",
				xRearEnclosureStart, xRearSocketEnd, yRightRailOutside, yRightEnclosureInside, zFloor, zRoof,
				[
					"pocket-centreSocketRight-rightRailOutside-floor",
					"pocket-rearEnclosureStart-rightEnclosureInside-floor",
					"pocket-rearSocketEnd-rightRailOutside-floor"
				]),
			pocket("pocket-rearFootFront-leftWebInside-footRoof",
				xRearFootFront, xMotorRibFront, yLeftWebInside, yRightWebInside, zFootRoof, zCapSole,
				[
					"pocket-driveRibRear-leftWebInside-floor"
				]),
			pocket("pocket-rearSocketEnd-leftWall-floor",
				xRearSocketEnd, xRearWall, yLeftWall, yLeftRailOutside, zFloor, zRoof,
				[
					"pocket-rearEnclosureStart-leftEnclosureInside-floor",
					"pocket-rearEnclosureStart-leftWall-floor"
				]),
			pocket("pocket-rearSocketEnd-rightRailOutside-floor",
				xRearSocketEnd, xRearWall, yRightRailOutside, yRightWall, zFloor, zRoof,
				[
					"pocket-rearEnclosureStart-rightEnclosureInside-floor",
					"pocket-rearEnclosureStart-rightRailOutside-floor"
				]),
			pocket("pocket-rearSocketStart-leftCap-capTop",
				xRearSocketStart, xRearWall, yLeftCap, yLeftWebOutside, zCapTop, zSocketSole,
				[
					"pocket-rearSocketStart-leftRailInside-floor",
					"pocket-rearSocketStart-leftWebOutside-capTop",
					"pocket-rearStrutRight-leftCap-capTop"
				]),
			pocket("pocket-rearSocketStart-leftRailInside-floor",
				xRearSocketStart, xRearWall, yLeftRailInside, yLeftCap, zFloor, zSocketSole,
				[
					"pocket-motorRibRear-leftCap-floor",
					"pocket-motorRibRear-leftRailInside-floor",
					"pocket-rearSocketStart-leftCap-capTop"
				]),
			pocket("pocket-rearSocketStart-leftWebInside-capTop",
				xRearSocketStart, xRearWall, yLeftWebInside, yLeftStrutInside, zCapTop, zSocketSole,
				[
					"X-drive-channel",
					"pocket-rearSocketStart-leftWebOutside-capTop",
					"pocket-rearStrutRight-leftCap-capTop"
				]),
			pocket("pocket-rearSocketStart-leftWebOutside-capTop",
				xRearSocketStart, xRearWall, yLeftWebOutside, yLeftWebInside, zCapTop, zRoof,
				[
					"pocket-rearSocketStart-leftCap-capTop",
					"pocket-rearSocketStart-leftWebInside-capTop",
					"pocket-rearStrutRight-leftCap-capTop"
				]),
			pocket("pocket-rearSocketStart-rightCap-floor",
				xRearSocketStart, xRearWall, yRightCap, yRightRailInside, zFloor, zSocketSole,
				[
					"pocket-motorRibRear-rightCap-floor",
					"pocket-motorRibRear-rightWebOutside-floor",
					"pocket-rearSocketStart-rightWebOutside-capTop"
				]),
			pocket("pocket-rearSocketStart-rightStrutInside-capTop",
				xRearSocketStart, xRearWall, yRightStrutInside, yRightWebInside, zCapTop, zSocketSole,
				[
					"X-drive-channel",
					"pocket-rearSocketStart-rightWebInside-capTop",
					"pocket-rearStrutRight-rightStrutInside-capTop"
				]),
			pocket("pocket-rearSocketStart-rightWebInside-capTop",
				xRearSocketStart, xRearWall, yRightWebInside, yRightWebOutside, zCapTop, zRoof,
				[
					"pocket-rearSocketStart-rightStrutInside-capTop",
					"pocket-rearSocketStart-rightWebOutside-capTop",
					"pocket-rearStrutRight-rightStrutInside-capTop"
				]),
			pocket("pocket-rearSocketStart-rightWebOutside-capTop",
				xRearSocketStart, xRearWall, yRightWebOutside, yRightCap, zCapTop, zSocketSole,
				[
					"pocket-rearSocketStart-rightCap-floor",
					"pocket-rearSocketStart-rightWebInside-capTop",
					"pocket-rearStrutRight-rightStrutInside-capTop"
				]),
			pocket("pocket-rearStrutLeft-leftCap-capTop",
				xRearStrutLeft, xRearStrutRight, yLeftCap, yLeftStrutOutside, zCapTop, zRoof,
				[
					"pocket-driveRibRear-leftRailInside-floor",
					"pocket-middleStrutRight-leftCap-capTop",
					"pocket-rearStrutRight-leftCap-capTop"
				]),
			pocket("pocket-rearStrutLeft-rightStrutInside-capTop",
				xRearStrutLeft, xRearStrutRight, yRightStrutInside, yRightCap, zCapTop, zRoof,
				[
					"X-drive-channel",
					"pocket-driveRibRear-rightCap-floor",
					"pocket-middleStrutRight-rightStrutInside-capTop",
					"pocket-rearStrutRight-rightStrutInside-capTop"
				]),
			pocket("pocket-rearStrutRight-leftCap-capTop",
				xRearStrutRight, xRearSocketStart, yLeftCap, yLeftStrutInside, zCapTop, zRoof,
				[
					"X-drive-channel",
					"pocket-driveRibRear-leftRailInside-floor",
					"pocket-motorRibFront-leftRailInside-capTop",
					"pocket-motorRibRear-leftRailInside-floor",
					"pocket-rearSocketStart-leftCap-capTop",
					"pocket-rearSocketStart-leftWebInside-capTop",
					"pocket-rearSocketStart-leftWebOutside-capTop",
					"pocket-rearStrutLeft-leftCap-capTop"
				]),
			pocket("pocket-rearStrutRight-rightStrutInside-capTop",
				xRearStrutRight, xRearSocketStart, yRightStrutInside, yRightCap, zCapTop, zRoof,
				[
					"X-drive-channel",
					"pocket-driveRibRear-rightCap-floor",
					"pocket-motorRibFront-rightCap-capTop",
					"pocket-motorRibRear-rightCap-floor",
					"pocket-rearSocketStart-rightStrutInside-capTop",
					"pocket-rearSocketStart-rightWebInside-capTop",
					"pocket-rearSocketStart-rightWebOutside-capTop",
					"pocket-rearStrutLeft-rightStrutInside-capTop"
				]),
			pocket("pocket-tableFront-leftRailInside-floor",
				xTableFront, xTableRear, yLeftRailInside, yLeftTable, zFloor, zWellFloor,
				[
					"pocket-frontSocketEnd-leftRailInside-floor",
					"pocket-tableRear-leftCap-floor",
					"pocket-tableRear-leftRailInside-floor",
					"work-well"
				]),
			pocket("pocket-tableFront-leftWall-footRoof",
				xTableFront, xFrontFootRear, yLeftWall, yLeftRailOutside, zFootRoof, zRoof,
				[
					"pocket-frontFootRear-leftWall-floor",
					"pocket-frontSocketStart-leftEnclosureInside-footRoof",
					"pocket-frontSocketStart-leftWall-footRoof"
				]),
			pocket("pocket-tableFront-rightRailOutside-footRoof",
				xTableFront, xFrontFootRear, yRightRailOutside, yRightWall, zFootRoof, zRoof,
				[
					"pocket-frontFootRear-rightRailOutside-floor",
					"pocket-frontSocketStart-rightEnclosureInside-footRoof",
					"pocket-frontSocketStart-rightRailOutside-footRoof"
				]),
			pocket("pocket-tableFront-rightTable-floor",
				xTableFront, xTableRear, yRightTable, yRightRailInside, zFloor, zWellFloor,
				[
					"pocket-frontSocketEnd-leftRailInside-floor",
					"pocket-tableRear-rightCap-floor",
					"pocket-tableRear-rightWebOutside-floor",
					"work-well"
				]),
			pocket("pocket-tableRear-leftCap-floor",
				xTableRear, xCentreWebLeft, yLeftCap, yLeftWebOutside, zFloor, zWellFloor,
				[
					"pocket-tableFront-leftRailInside-floor",
					"pocket-tableRear-leftRailInside-floor",
					"work-well"
				]),
			pocket("pocket-tableRear-leftRailInside-floor",
				xTableRear, xCentreWebLeft, yLeftRailInside, yLeftCap, zFloor, zWellFloor,
				[
					"pocket-tableFront-leftRailInside-floor",
					"pocket-tableRear-leftCap-floor",
					"work-well"
				]),
			pocket("pocket-tableRear-leftWebInside-floor",
				xTableRear, xCentreWebLeft, yLeftWebInside, yRightWebInside, zFloor, zWellFloor,
				[
					"work-well"
				]),
			pocket("pocket-tableRear-rightCap-floor",
				xTableRear, xCentreWebLeft, yRightCap, yRightRailInside, zFloor, zWellFloor,
				[
					"pocket-tableFront-rightTable-floor",
					"pocket-tableRear-rightWebOutside-floor",
					"work-well"
				]),
			pocket("pocket-tableRear-rightWebOutside-floor",
				xTableRear, xCentreWebLeft, yRightWebOutside, yRightCap, zFloor, zWellFloor,
				[
					"pocket-tableFront-rightTable-floor",
					"pocket-tableRear-rightCap-floor",
					"work-well"
				]),
			pocket("pocket-wellFront-leftRailInside-floor",
				xWellFront, xFrontSocketEnd, yLeftRailInside, yLeftWebOutside, zFloor, zWellFloor,
				[
					"pocket-frontSocketEnd-leftRailInside-floor",
					"pocket-frontWall-leftRailInside-floor",
					"pocket-wellFront-leftWebOutside-floor",
					"work-well"
				]),
			pocket("pocket-wellFront-leftWebOutside-floor",
				xWellFront, xFrontSocketEnd, yLeftWebOutside, yRightWebOutside, zFloor, zWellFloor,
				[
					"pocket-frontSocketEnd-leftRailInside-floor",
					"pocket-frontWall-leftWebOutside-floor",
					"pocket-wellFront-leftRailInside-floor",
					"pocket-wellFront-rightWebOutside-floor",
					"work-well"
				]),
			pocket("pocket-wellFront-rightWebOutside-floor",
				xWellFront, xFrontSocketEnd, yRightWebOutside, yRightRailInside, zFloor, zWellFloor,
				[
					"pocket-frontSocketEnd-leftRailInside-floor",
					"pocket-frontWall-rightWebOutside-floor",
					"pocket-wellFront-leftWebOutside-floor",
					"work-well"
				])
		];
	}

	static function pocket(id:String, x0:Float, x1:Float, y0:Float, y1:Float, z0:Float, z1:Float,
		connections:Array<String>):CastingFeature
		return CastingFeaturePatterns.box(id, Lightening, [for (connection in connections) CastingPatterns.feature(connection)],
			x1 - x0, y1 - y0, z1 - z0, (x0 + x1) / 2, (y0 + y1) / 2, z0);
}
