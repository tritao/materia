import fixtures.ChainFixtures;
import fixtures.OracleFixtures;
import cnckit.tool.CutterProfile;
import cnckit.tool.CutterSegment;
import cnckit.tool.CutterZone;

class StockKitTests {
  public static function main():Void {
    profiles();
    OracleFixtures.run();
    ChainFixtures.run();
    Sys.println('StockKit tests passed (${Assert.count} assertions)');
  }

  static function profiles():Void {
    var flat = CutterProfile.flat(0.006, 0.02);
    Assert.near(flat.height(), 0.02, "flat mill height is its flute length");
    Assert.near(flat.topRadius(), 0.003, "flat mill radius");
    Assert.near(flat.fluteLength(), 0.02, "flat mill cutting zone");

    Assert.near(flat.halfSectionArea(), 0.003 * 0.02, "flat half-section is a rectangle", 1e-18);
    Assert.near(flat.volume(), Math.PI * 0.003 * 0.003 * 0.02, "flat mill volume", 1e-18);
    var ball = CutterProfile.ball(0.006, 0.003);
    Assert.near(ball.halfSectionArea(), Math.PI * 0.003 * 0.003 / 4, "ball tip is a quarter disc", 1e-18);
    Assert.near(ball.volume(), 2 / 3 * Math.PI * Math.pow(0.003, 3), "ball tip is a hemisphere", 1e-18);
    var bull = CutterProfile.bullNose(0.006, 0.001, 0.001);
    // Pappus: a quarter disc of the corner radius at distance r - rc + 4rc/3pi.
    Assert.near(bull.volume(), Math.PI * 0.002 * 0.002 * 0.001
      + 2 * Math.PI * (0.002 + 4 * 0.001 / (3 * Math.PI)) * Math.PI * 0.001 * 0.001 / 4,
      "bull-nose corner volume by Pappus", 1e-18);

    var ballTip = CutterProfile.ball(0.006, 0.02).below(0.001);
    // Spherical cap of height h on a sphere of radius R: pi h^2 (3R - h) / 3.
    Assert.near(ballTip.volume(), Math.PI * 0.001 * 0.001 * (3 * 0.003 - 0.001) / 3,
      "ball tip clipped below its equator is a spherical cap", 1e-18);
    Assert.near(ballTip.topRadius(), Math.sqrt(0.003 * 0.003 - 0.002 * 0.002),
      "clipped ball radius at the cut height", 1e-15);
    var veeTip = CutterProfile.vee(0.006, Math.PI / 2, 0.02).below(0.002);
    Assert.near(veeTip.volume(), Math.PI * 0.002 * 0.002 * 0.002 / 3,
      "clipped 90 degree V-bit is a cone", 1e-18);
    Assert.near(CutterProfile.flat(0.006, 0.02).below(0.005).height(), 0.005,
      "clipped flat mill keeps only the cut height", 1e-15);

    var held = CutterProfile.ball(0.006, 0.015).withShank(0.006, 0.01)
      .withHolder(0.02, 0.03);
    Assert.near(held.fluteLength(), 0.015, "shank and holder do not cut");
    Assert.near(held.height(), 0.055, "holder closes the tool");
    Assert.near(held.topRadius(), 0.01, "holder radius");
    Assert.check(held.isRadiallyMonotonic(), "stepped holder still widens upwards");

    var necked = CutterProfile.flat(0.006, 0.01).withShank(0.004, 0.01);
    Assert.check(!necked.isRadiallyMonotonic(), "a neck narrower than the flutes is detected");

    var tapered = CutterProfile.taperedBall(0.002, 5 * Math.PI / 180, 0.006, 0.04);
    var cone = tapered.segments[1];
    switch cone {
      case Line(r0, z0, r1, z1, _):
        // The cone continues the ball tangentially: its direction is
        // perpendicular to the ball radius at the tangent point.
        var dr = r1 - r0, dz = z1 - z0;
        Assert.near(dr * r0 + dz * (z0 - 0.001), 0.0, "taper is tangent to the ball", 1e-15);
        Assert.near(r1, 0.003, "taper reaches the shank radius");
      case _: Assert.check(false, "tapered ball has a conical second segment");
    }

    var rejected = false;
    try new CutterProfile([Line(0, 0, 0.003, 0, Cutting),
      Line(0.003, 0.001, 0.003, 0.01, Cutting)]) catch (_:Dynamic) rejected = true;
    Assert.check(rejected, "a gap in the profile is rejected");
    rejected = false;
    try new CutterProfile([Line(0, 0, 0.003, 0.002, Cutting),
      Line(0.003, 0.002, 0.004, 0.001, Cutting)]) catch (_:Dynamic) rejected = true;
    Assert.check(rejected, "an overhanging profile is rejected");
  }
}
