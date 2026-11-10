package hobber;

import machinekit.hobbing.*;

import materia.casting.CastingFeature;
import materia.casting.CastingPatterns;
import machinekit.structural.CastingFeaturePatterns;

/** Positive pocket spans between the source ribs and rail beams. Outer bands
 * and drive-web slots are solid; the drive chamber opens directly to the sole.
 * There is no source grid, retained-box subtraction or hashed ID.
 */
class HobberCastingPockets {
  /** Wall and roof thickness the cores leave, which every cut near a core must keep, mm. */
  public static inline final WALL = 25.0;
  static inline final DRIVE_CAP_TOP = 720.0;
  static inline final SOCKET_BOSS_DEPTH = 60.0;
  static inline final DECK_RIB_WIDTH = 50.0;

  public static function plinth(width:Float, depth:Float, height:Float):Array<CastingFeature> {
    var layout = HobberLayout.get().xNominal();
    var wall = WALL;
    var footInset = 120.0, footBossHalf = MineralCastBed.FOOT_BOSS_WIDTH / 2;
    var tableHalf = MineralCastBed.TABLE_INSERT_WIDTH / 2 + MineralCastBed.SOURCE_STEEL_COVER_MM,
      ribHalf = MineralCastBed.PLINTH_CORE_WEB / 2;
    var driveRibHalf = MineralCastBed.DRIVE_RIB_HALF_WIDTH;
    var railCentre = HobberLayout.get().bedRailCentre, railBeamHalf = MineralCastBed.BED_RAIL_BEAM_WIDTH / 2;
    var capHalf = 150.0, webCentre = 112.5;
    var frontDeckRibX = 100.0, middleDeckRibX = 160.0, rearDeckRibX = 220.0, deckRibY = 80.0;
    var deckRibHalf = DECK_RIB_WIDTH / 2;
    var xFrontWall = -width / 2 + wall;
    var xTableFront = -tableHalf - layout.bedCentre;
    var xTableRear = tableHalf - layout.bedCentre;
    var xCentreWebLeft = -ribHalf;
    var xCentreWebRight = ribHalf;
    var xDriveRibRear = layout.floatingCentre - layout.bedCentre + driveRibHalf;
    var xFrontDeckRibLeft = frontDeckRibX - deckRibHalf;
    var xFrontDeckRibRight = frontDeckRibX + deckRibHalf;
    var xMiddleDeckRibLeft = middleDeckRibX - deckRibHalf;
    var xMiddleDeckRibRight = middleDeckRibX + deckRibHalf;
    var xRearDeckRibLeft = rearDeckRibX - deckRibHalf;
    var xRearDeckRibRight = rearDeckRibX + deckRibHalf;
    var xRearFootFront = width / 2 - footInset - footBossHalf;
    var xMotorRibFront = layout.screwStart - layout.bedCentre - driveRibHalf;
    // The pocket stops a full cover land before the actual relocated socket tail.
    var socketHalf = Math.max(MineralCastBed.TRAY_SOCKET_OUTSIDE_DIAMETER,
      MineralCastBed.TRAY_SOCKET_TAIL_WIDTH) / 2;
    var xRearSocketStart = width
      / 2 - MineralCastBed.TRAY_SOCKET_END_INSET - socketHalf - MineralCastBed.SOURCE_STEEL_COVER_MM;
    var xRearWall = width / 2 - wall;
    var xWellFront = MineralCastBed.WORK_WELL_CENTRE_X - layout.bedCentre - MineralCastBed.WORK_WELL_WIDTH / 2;
    var xFrontSocketEnd = xWellFront;
    var yLeftRailInside = -railCentre + railBeamHalf;
    var yLeftCap = -capHalf;
    var yLeftTable = -tableHalf;
    var yLeftWebOutside = -webCentre - ribHalf;
    var yLeftDeckRibOutside = -deckRibY - deckRibHalf;
    var yLeftWebInside = -webCentre + ribHalf;
    var yLeftDeckRibInside = -deckRibY + deckRibHalf;
    var yRightDeckRibInside = deckRibY - deckRibHalf;
    var yRightWebInside = webCentre - ribHalf;
    var yRightDeckRibOutside = deckRibY + deckRibHalf;
    var yRightWebOutside = webCentre + ribHalf;
    var yRightTable = tableHalf;
    var yRightCap = capHalf;
    var yRightRailInside = railCentre - railBeamHalf;
    var zFloor = MineralCastBed.PLINTH_CORE_FLOOR;
    var zFootRoof = MineralCastBed.FOOT_SUPPORT_TOP;
    var zCapSole = MineralCastBed.DRIVE_CAP_UNDERSIDE;
    var zCapTop = DRIVE_CAP_TOP;
    var zSocketSole = height - SOCKET_BOSS_DEPTH;
    // Retain the full wall below the recessed tray seats, not only below the top.
    var zRoof = height - wall - machinekit.enclosure.TraySeat.DEPTH;
    var zWellFloor = MineralCastBed.TABLE_WELL_FLOOR_HEIGHT;
    return [
      pocket(
        "pocket-centreWebRight-leftCap-capTop",
        xCentreWebRight,
        xFrontDeckRibLeft,
        yLeftWebOutside,
        yLeftDeckRibInside,
        zCapTop,
        zRoof,
        ["X-drive-channel", "pocket-frontDeckRibLeft-leftCap-capTop"]
      ),
      pocket(
        "pocket-centreWebRight-rightDeckRibInside-capTop",
        xCentreWebRight,
        xFrontDeckRibLeft,
        yRightDeckRibInside,
        yRightWebOutside,
        zCapTop,
        zRoof,
        ["X-drive-channel", "pocket-frontDeckRibLeft-rightDeckRibInside-capTop"]
      ),
      pocket(
        "pocket-driveRibRear-leftWebInside-floor",
        xDriveRibRear,
        xRearFootFront,
        yLeftWebInside,
        yRightWebInside,
        0.0,
        zCapSole,
        ["pocket-rearFootFront-leftWebInside-footRoof"],
        true
      ),
      pocket(
        "pocket-frontSocketEnd-leftRailInside-floor",
        xFrontSocketEnd,
        xTableFront,
        yLeftRailInside,
        yRightRailInside,
        zFloor,
        zWellFloor,
        [
          "pocket-frontWall-leftRailInside-floor",
          "pocket-frontWall-leftWebOutside-floor",
          "pocket-frontWall-rightWebOutside-floor",
          "chip-drain",
          "pocket-tableFront-leftRailInside-floor",
          "pocket-tableFront-rightTable-floor",
          "work-well"
        ]
      ),
      pocket(
        "pocket-frontDeckRibLeft-leftCap-capTop",
        xFrontDeckRibLeft,
        xFrontDeckRibRight,
        yLeftWebOutside,
        yLeftDeckRibOutside,
        zCapTop,
        zRoof,
        ["pocket-centreWebRight-leftCap-capTop", "pocket-frontDeckRibRight-leftCap-capTop"]
      ),
      pocket(
        "pocket-frontDeckRibLeft-rightDeckRibInside-capTop",
        xFrontDeckRibLeft,
        xFrontDeckRibRight,
        yRightDeckRibInside,
        yRightWebOutside,
        zCapTop,
        zRoof,
        [
          "X-drive-channel",
          "pocket-centreWebRight-rightDeckRibInside-capTop",
          "pocket-frontDeckRibRight-rightDeckRibInside-capTop"
        ]
      ),
      pocket(
        "pocket-frontDeckRibRight-leftCap-capTop",
        xFrontDeckRibRight,
        xMiddleDeckRibLeft,
        yLeftWebOutside,
        yLeftDeckRibInside,
        zCapTop,
        zRoof,
        ["X-drive-channel", "pocket-frontDeckRibLeft-leftCap-capTop", "pocket-middleDeckRibLeft-leftCap-capTop"]
      ),
      pocket(
        "pocket-frontDeckRibRight-rightDeckRibInside-capTop",
        xFrontDeckRibRight,
        xMiddleDeckRibLeft,
        yRightDeckRibInside,
        yRightWebOutside,
        zCapTop,
        zRoof,
        [
          "X-drive-channel",
          "pocket-frontDeckRibLeft-rightDeckRibInside-capTop",
          "pocket-middleDeckRibLeft-rightDeckRibOutside-capTop"
        ]
      ),
      pocket(
        "pocket-frontWall-leftRailInside-floor",
        xFrontWall,
        xWellFront,
        yLeftRailInside,
        yLeftWebOutside,
        zFloor,
        zSocketSole,
        ["pocket-frontSocketEnd-leftRailInside-floor", "pocket-frontWall-leftWebOutside-floor", "work-well"]
      ),
      pocket(
        "pocket-frontWall-leftWebOutside-floor",
        xFrontWall,
        xWellFront,
        yLeftWebOutside,
        yRightWebOutside,
        zFloor,
        zRoof,
        [
          "pocket-frontSocketEnd-leftRailInside-floor",
          "coolant-front-access",
          "pocket-frontWall-leftRailInside-floor",
          "pocket-frontWall-rightWebOutside-floor",
          "work-well"
        ]
      ),
      pocket(
        "pocket-frontWall-rightWebOutside-floor",
        xFrontWall,
        xWellFront,
        yRightWebOutside,
        yRightRailInside,
        zFloor,
        zSocketSole,
        ["pocket-frontSocketEnd-leftRailInside-floor", "pocket-frontWall-leftWebOutside-floor", "work-well"]
      ),
      pocket(
        "pocket-middleDeckRibLeft-leftCap-capTop",
        xMiddleDeckRibLeft,
        xMiddleDeckRibRight,
        yLeftWebOutside,
        yLeftDeckRibInside,
        zCapTop,
        zRoof,
        ["X-drive-channel", "pocket-frontDeckRibRight-leftCap-capTop", "pocket-middleDeckRibRight-leftCap-capTop"]
      ),
      pocket(
        "pocket-middleDeckRibLeft-rightDeckRibOutside-capTop",
        xMiddleDeckRibLeft,
        xMiddleDeckRibRight,
        yRightDeckRibOutside,
        yRightWebOutside,
        zCapTop,
        zRoof,
        ["pocket-frontDeckRibRight-rightDeckRibInside-capTop", "pocket-middleDeckRibRight-rightDeckRibInside-capTop"]
      ),
      pocket(
        "pocket-middleDeckRibRight-leftCap-capTop",
        xMiddleDeckRibRight,
        xRearDeckRibLeft,
        yLeftWebOutside,
        yLeftDeckRibInside,
        zCapTop,
        zRoof,
        ["X-drive-channel", "pocket-middleDeckRibLeft-leftCap-capTop", "pocket-rearDeckRibLeft-leftCap-capTop"]
      ),
      pocket(
        "pocket-middleDeckRibRight-rightDeckRibInside-capTop",
        xMiddleDeckRibRight,
        xRearDeckRibLeft,
        yRightDeckRibInside,
        yRightWebOutside,
        zCapTop,
        zRoof,
        [
          "X-drive-channel",
          "pocket-middleDeckRibLeft-rightDeckRibOutside-capTop",
          "pocket-rearDeckRibLeft-rightDeckRibInside-capTop"
        ]
      ),
      pocket(
        "pocket-rearFootFront-leftWebInside-footRoof",
        xRearFootFront,
        xMotorRibFront,
        yLeftWebInside,
        yRightWebInside,
        zFootRoof,
        zCapSole,
        ["pocket-driveRibRear-leftWebInside-floor"]
      ),
      pocket(
        "pocket-rearSocketStart-leftWebInside-capTop",
        xRearSocketStart,
        xRearWall,
        yLeftWebInside,
        yLeftDeckRibInside,
        zCapTop,
        zSocketSole,
        ["X-drive-channel", "pocket-rearSocketStart-leftWebOutside-capTop", "pocket-rearDeckRibRight-leftCap-capTop"]
      ),
      pocket(
        "pocket-rearSocketStart-leftWebOutside-capTop",
        xRearSocketStart,
        xRearWall,
        yLeftWebOutside,
        yLeftWebInside,
        zCapTop,
        zRoof,
        ["pocket-rearSocketStart-leftWebInside-capTop", "pocket-rearDeckRibRight-leftCap-capTop"]
      ),
      pocket(
        "pocket-rearSocketStart-rightDeckRibInside-capTop",
        xRearSocketStart,
        xRearWall,
        yRightDeckRibInside,
        yRightWebInside,
        zCapTop,
        zSocketSole,
        [
          "X-drive-channel",
          "pocket-rearSocketStart-rightWebInside-capTop",
          "pocket-rearDeckRibRight-rightDeckRibInside-capTop"
        ]
      ),
      pocket(
        "pocket-rearSocketStart-rightWebInside-capTop",
        xRearSocketStart,
        xRearWall,
        yRightWebInside,
        yRightWebOutside,
        zCapTop,
        zRoof,
        ["pocket-rearSocketStart-rightDeckRibInside-capTop", "pocket-rearDeckRibRight-rightDeckRibInside-capTop"]
      ),
      pocket(
        "pocket-rearDeckRibLeft-leftCap-capTop",
        xRearDeckRibLeft,
        xRearDeckRibRight,
        yLeftWebOutside,
        yLeftDeckRibOutside,
        zCapTop,
        zRoof,
        ["pocket-middleDeckRibRight-leftCap-capTop", "pocket-rearDeckRibRight-leftCap-capTop"]
      ),
      pocket(
        "pocket-rearDeckRibLeft-rightDeckRibInside-capTop",
        xRearDeckRibLeft,
        xRearDeckRibRight,
        yRightDeckRibInside,
        yRightWebOutside,
        zCapTop,
        zRoof,
        [
          "X-drive-channel",
          "pocket-middleDeckRibRight-rightDeckRibInside-capTop",
          "pocket-rearDeckRibRight-rightDeckRibInside-capTop"
        ]
      ),
      pocket(
        "pocket-rearDeckRibRight-leftCap-capTop",
        xRearDeckRibRight,
        xRearSocketStart,
        yLeftWebOutside,
        yLeftDeckRibInside,
        zCapTop,
        zRoof,
        [
          "X-drive-channel",
          "pocket-rearSocketStart-leftWebInside-capTop",
          "pocket-rearSocketStart-leftWebOutside-capTop",
          "pocket-rearDeckRibLeft-leftCap-capTop"
        ]
      ),
      pocket(
        "pocket-rearDeckRibRight-rightDeckRibInside-capTop",
        xRearDeckRibRight,
        xRearSocketStart,
        yRightDeckRibInside,
        yRightWebOutside,
        zCapTop,
        zRoof,
        [
          "X-drive-channel",
          "pocket-rearSocketStart-rightDeckRibInside-capTop",
          "pocket-rearSocketStart-rightWebInside-capTop",
          "pocket-rearDeckRibLeft-rightDeckRibInside-capTop"
        ]
      ),
      pocket(
        "pocket-tableFront-leftRailInside-floor",
        xTableFront,
        xTableRear,
        yLeftRailInside,
        yLeftTable,
        zFloor,
        zWellFloor,
        ["pocket-frontSocketEnd-leftRailInside-floor", "work-well"]
      ),
      pocket(
        "pocket-tableFront-rightTable-floor",
        xTableFront,
        xTableRear,
        yRightTable,
        yRightRailInside,
        zFloor,
        zWellFloor,
        ["pocket-frontSocketEnd-leftRailInside-floor", "work-well"]
      )
    ];
  }

  static function pocket(
    id:String,
    x0:Float,
    x1:Float,
    y0:Float,
    y1:Float,
    z0:Float,
    z1:Float,
    connections:Array<String>,
    soleOpening:Bool = false
  ):CastingFeature return CastingFeaturePatterns.box(
    id,
    Lightening,
    [for (connection in connections) CastingPatterns.feature(connection)].concat(soleOpening
    ? [CastingPatterns.face("body.z-min")] : []),
    x1 - x0,
    y1 - y0,
    z1 - z0,
    (x0 + x1) / 2,
    (y0 + y1) / 2,
    z0
  );
}
