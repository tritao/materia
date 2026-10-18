package machinekit.hobbing;

/** Assumed small-displacement n=2 support estimate, N/mm. A simply supported
    sleeve mode w=d*cos(pi*z/L)*cos(2 theta) models end rings at +/-L/2.
    Bending energy of an isotropic cylindrical shell gives the stated modal k.
    Contact is a distributed tangent foundation in series with the sleeve.
    Oil resists uniform volume change, but contributes zero to n=2 at first order.
    End restraint, surface conformity and contact tangent require calibration. */
class MandrelRadialSupport {
  public final sleeveStiffness:Float;
  public final contactStiffness:Float;
  public final stiffness:Float;
  public final uniformOilStiffness:Float;
  public final seatingPressure:Float;
  public final contactPressure:Float;
  public final basis:String;
  public final supportsRim:Bool;
  public final sleeveStart:Float;
  public final sleeveEnd:Float;
  public final bandStart:Float;
  public final bandEnd:Float;
  public final radialGap:Float;
  public function new(mandrel:ExpandingMandrel,band:Float,?rimStart:Float,?rimBoreRadius:Float) {
    sleeveStart=mandrel.parameterNumber("sleeveStart");sleeveEnd=sleeveStart+mandrel.parameterNumber("sleeveLength");
    bandStart=rimStart==null?mandrel.parameterNumber("workSeat"):rimStart;bandEnd=bandStart+band;
    radialGap=rimBoreRadius==null?mandrel.parameterNumber("radialFitGap"):rimBoreRadius-mandrel.parameterNumber("outside")/2;
    supportsRim=bandStart>=sleeveStart&&bandEnd<=sleeveEnd&&bandStart>=0&&bandEnd<=mandrel.parameterNumber("height")
      &&radialGap>=0&&radialGap<=mandrel.parameterNumber("expansion")/2+1e-9;
    var t=mandrel.parameterNumber("sleeveWall"),length=mandrel.parameterNumber("sleeveLength"),r=mandrel.parameterNumber("outside")/2-t/2;
    var e=mandrel.parameterNumber("sleeveYoungs"),nu=mandrel.parameterNumber("sleevePoisson"),cavity=mandrel.parameterNumber("oilCavity"),bulk=mandrel.parameterNumber("oilBulkModulus");
    // Two conforming steel half spaces under a harmonic normal traction:
    // combined displacement = 4*p/(E_prime*q), E_prime=E/(1-nu²).
    // Finite sleeve geometry and surface roughness make this an estimate.
    var wave=Math.sqrt(4/(r*r)+Math.PI*Math.PI/(length*length));
    var contact=e*wave/(4*(1-nu*nu))*mandrel.parameterNumber("contactStiffnessFactor"),gap=radialGap;
    if(band<=0||r<=t||nu<=0||nu>=0.5)throw "invalid mandrel support geometry";
    var rigidity=e*t*t*t/(12*(1-nu*nu));
    sleeveStiffness=rigidity*Math.PI*r*(Math.pow(Math.PI,4)/(2*Math.pow(length,3))
      +9*length/(2*Math.pow(r,4))+(4-nu)*Math.PI*Math.PI/(r*r*length));
    var lo=bandStart-(sleeveStart+sleeveEnd)/2,hi=bandEnd-(sleeveStart+sleeveEnd)/2;
    var bandIntegral=band/2+length/(4*Math.PI)*(Math.sin(2*Math.PI*hi/length)-Math.sin(2*Math.PI*lo/length));
    contactStiffness=contact*Math.PI*r*bandIntegral;
    stiffness=supportsRim?1/(1/sleeveStiffness+1/contactStiffness):0;
    // Axisymmetric oil compression diagnostic, never added to n=2 stiffness.
    uniformOilStiffness=2*Math.PI*r*length*bulk/cavity;
    seatingPressure=gap*e*t/(r*r); // hoop estimate, excludes end restraint
    contactPressure=mandrel.clampingPressure/1e6-seatingPressure;
    basis='Assumed: sleeve span [$sleeveStart,$sleeveEnd] mm; toothed rim [$bandStart,$bandEnd] mm, radial gap=$radialGap mm, rim supported=$supportsRim; sleeve t=$t mm, end span L=$length mm, band=$band mm, mid-radius R=$r mm; E=$e N/mm², nu=$nu; '
      +'D=E*t^3/[12(1-nu²)]; k_s=D*pi*R*[pi^4/(2L^3)+9L/(2R^4)+(4-nu)*pi²/(R²L)]; '
      +'conforming steel harmonic half-space contact c=E*q/[4(1-nu²)]*factor, q=sqrt(4/R²+pi²/L²), factor=${mandrel.parameterNumber("contactStiffnessFactor")}, c=$contact N/mm³; '
      +'k_c=c*pi*R*integral_band cos²(pi*z/L) dz; 1/k=1/k_s+1/k_c; '
      +'oil K=$bulk N/mm², cavity=$cavity mm: n=2 volume change and oil stiffness zero at first order; '
      +'uniform oil k=$uniformOilStiffness N/mm only; radial fit gap=$gap mm, hoop seating pressure=$seatingPressure MPa, '
      +'remaining pressure=$contactPressure MPa (end restraint omitted from seating check); full seated contact required; '
      +'modal estimate, not measured stiffness or FEA; expansion sleeve/cavity are assumed design dimensions, envelope CAD does not resolve them';
  }
}
