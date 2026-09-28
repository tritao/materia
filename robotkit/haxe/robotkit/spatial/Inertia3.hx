package robotkit.spatial;

/** Symmetric centroidal inertia tensor in kg m². */
class Inertia3 {
  public final xx:Float;
  public final xy:Float;
  public final xz:Float;
  public final yy:Float;
  public final yz:Float;
  public final zz:Float;

  public function new(xx:Float, xy:Float, xz:Float, yy:Float, yz:Float, zz:Float) {
    for (value in [xx, xy, xz, yy, yz, zz]) if (!Math.isFinite(value))
      throw "Inertia tensor must be finite";
    if (xx < 0 || yy < 0 || zz < 0) throw "Inertia diagonal must be non-negative";
    this.xx = xx; this.xy = xy; this.xz = xz;
    this.yy = yy; this.yz = yz; this.zz = zz;
  }

  public static function zero():Inertia3 return new Inertia3(0, 0, 0, 0, 0, 0);

  public function add(other:Inertia3):Inertia3
    return new Inertia3(xx + other.xx, xy + other.xy, xz + other.xz,
      yy + other.yy, yz + other.yz, zz + other.zz);

  public function rotated(rotation:Quat):Inertia3 {
    var basis = [rotation.rotate(new Vec3(1, 0, 0)),
      rotation.rotate(new Vec3(0, 1, 0)), rotation.rotate(new Vec3(0, 0, 1))];
    var r = [
      [basis[0].x, basis[1].x, basis[2].x],
      [basis[0].y, basis[1].y, basis[2].y],
      [basis[0].z, basis[1].z, basis[2].z]
    ];
    var tensor = [[xx, xy, xz], [xy, yy, yz], [xz, yz, zz]];
    var result = [for (_ in 0...3) [0.0, 0.0, 0.0]];
    for (i in 0...3) for (j in 0...3) for (k in 0...3) for (l in 0...3)
      result[i][j] += r[i][k] * tensor[k][l] * r[j][l];
    return new Inertia3(result[0][0], (result[0][1] + result[1][0]) * 0.5,
      (result[0][2] + result[2][0]) * 0.5, result[1][1],
      (result[1][2] + result[2][1]) * 0.5, result[2][2]);
  }

  /** Shift centroidal inertia to a parallel axis offset by `delta`. */
  public function shifted(massKg:Float, delta:Vec3):Inertia3 {
    if (!Math.isFinite(massKg) || massKg < 0 || delta == null)
      throw "Inertia shift requires non-negative mass and an offset";
    return new Inertia3(xx + massKg * (delta.y * delta.y + delta.z * delta.z),
      xy - massKg * delta.x * delta.y, xz - massKg * delta.x * delta.z,
      yy + massKg * (delta.x * delta.x + delta.z * delta.z),
      yz - massKg * delta.y * delta.z,
      zz + massKg * (delta.x * delta.x + delta.y * delta.y));
  }
}
