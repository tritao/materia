package gearkit;

/** Designer supplies the active profile control radius; a mating gear is needed to infer it.
    Defaults for tip form, 95% roll evaluation and end exclusions follow ISO 1328-1:2013.
    Gaussian cutoff lengths are mm; zero selects unfiltered diagnostic measurement. */
class GearInspection {
  public final spec:GearSpec;
  public final controlRadius:Float;
  public final tipFormRadius:Float;
  public final measurementRadius:Float;
  public final pitchZ:Float;
  public final profileZ:Float;
  public final rollStart:Float;
  public final rollEnd:Float;
  public final rollTip:Float;
  public final axialStart:Float;
  public final axialEnd:Float;
  public final profileCutoff:Float;
  public final helixCutoff:Float;
  public function new(s:GearSpec, controlRadius:Float, ?measurementRadius:Float,
      ?pitchZ:Float, ?profileZ:Float, ?tipFormRadius:Float,
      ?profileCutoff:Float, ?helixCutoff:Float) {
    spec=s;
    var p = new ToothProfile(s), rb = s.baseRadius();
    this.controlRadius = controlRadius;
    this.tipFormRadius = tipFormRadius == null ? s.tipRadius()-s.sign()*s.tipFilletRadius : tipFormRadius;
    this.measurementRadius = measurementRadius == null ? s.pitchRadius() : measurementRadius;
    this.pitchZ = pitchZ == null ? s.faceWidth/2 : pitchZ;
    this.profileZ = profileZ == null ? s.faceWidth/2 : profileZ;
    var lo = Math.min(p.rootJoinRadius,p.tipJoinRadius), hi = Math.max(p.rootJoinRadius,p.tipJoinRadius);
    for (r in [controlRadius,this.tipFormRadius,this.measurementRadius])
      if (!Math.isFinite(r) || r < lo || r > hi) throw "inspection radius is outside the unrounded involute";
    if (s.sign()*(this.tipFormRadius-controlRadius) <= 0) throw "profile control must precede tip form";
    for (z in [this.pitchZ,this.profileZ])
      if (!Math.isFinite(z) || z < 0 || z > s.faceWidth) throw "inspection Z outside face";
    rollStart = roll(controlRadius,rb);
    rollTip = roll(s.tipRadius(),rb);
    rollEnd = rollStart+0.95*(roll(this.tipFormRadius,rb)-rollStart);
    var trim = Math.min(0.05*s.faceWidth,s.module);
    axialStart = trim; axialEnd = s.faceWidth-trim;
    var pc = Math.max(Math.abs(rollEnd-rollStart)/30,0.25);
    this.profileCutoff = profileCutoff == null ? pc : profileCutoff;
    this.helixCutoff = helixCutoff == null ? Math.max(s.faceWidth/30,pc) : helixCutoff;
    for (c in [this.profileCutoff,this.helixCutoff])
      if (!Math.isFinite(c) || c < 0) throw "filter cutoff must be finite and non-negative";
    if (this.profileCutoff > pc || this.helixCutoff > Math.max(s.faceWidth/30,pc))
      throw "filter cutoff exceeds ISO recommended maximum";
  }
  public static function roll(r:Float, rb:Float):Float return Math.sqrt(Math.max(0,r*r-rb*rb));
}
