import machinekit.assembly.MachineAssembly;
import machinekit.enclosure.GeneratedEnclosure;
import machinekit.enclosure.EnclosureRules;

typedef HobberInstalledCell = {
  assembly:MachineAssembly, shell:GeneratedEnclosure, machine:FlexsplineHobber, report:Dynamic
};

/** Physical cell: machine, enclosure and their installed interfaces. */
class HobberCell {
  /** The scene group the shell is (`includeAsGroup`): its own groups sit directly under it. */
  public static inline final ENCLOSURE_GROUP = "Enclosure";
  public static inline final SHELL_ID = "enclosure";

  /** A group inside the shell's, so host-side parts (mounts, clamps) sit beside the shell parts they serve. */
  public static function shellGroup(path:String):String return ENCLOSURE_GROUP + "/" + path;

  public static function build(machine:FlexsplineHobber):HobberInstalledCell {
    var fit = HobberEnclosureFit.fit(machine), shell = new GeneratedEnclosure(fit.layout, {
        halfWidth: hobber.HobberChipCover.OUTER_HALF_WIDTH + machinekit.enclosure.TraySeat.SEAM,
        rearY: hobber.HobberChipCover.REAR_END
      }), assembly = new MachineAssembly();
    assembly.includeAsGroup("Machine", "machine", machine);
    assembly.includeAsGroup(ENCLOSURE_GROUP, SHELL_ID, shell);
    HobberEnclosureMounts.install(assembly, shell);
    HobberTrayMounts.install(assembly, shell);
    for (name in machine.portNames()) {
      var port = machine.port(name, "machine");
      assembly.exposePort(name, port.instanceId, port.portName);
    }
    var uses:Array<PanelUse> = [];
    for (m in shell.components()) if (Std.isOfType(m.component, machinekit.enclosure.EnclosurePanel)) {
      var panel:machinekit.enclosure.EnclosurePanel = cast m.component;
      var zones:Array<EnclosureZone> = (m.id == "door/leaf" || m.id == "door/leftLeaf") ? [Window,
        Impact, Splash] : StringTools.startsWith(
          m.id,
          "tray"
        ) ? [Chute] : [Splash];
      uses.push({panelId: m.id,
        construction: panel.construction, zones: zones, edgesSealed: false, seamsSealed: false});
    }
    var qualificationIssues = EnclosureRules.check(
      uses,
      {
        process: WetMachining,
        maximumTemperatureC: 60,
        oilMist: true,
        extractionProvided: false,
        drainageProvided: false,
        chamberControlProvided: false,
        boundaryGuardProvided: false
      }
    );
    return {
      assembly: assembly,
      shell: shell,
      machine: machine,
      report: {
        samples: fit.samples,
        angularPaddingMm: fit.paddingMm,
        excludedElectricalRack: fit.excluded,
        input: fit.layout.input,
        dimensionsMm: [fit.layout.width(),
      fit.layout.depth(), fit.layout.height()],
      doorStrokeMm: fit.layout.stroke,
      doorLeaves: 2,
      doorDrive: "Proposed bought rod-style electric actuator, at least 210 mm usable stroke, plus passive opposite-run belt; ideal coupling only, hardware not sized",
      doorGuide: {
        track: "Captured steel roller channel, nominal assumed envelope",
        trackWidthMm: 44,
        slotWidthMm: 8,
        rollerDiameterMm: 22,
        rollersPerLeaf: 4,
        hangersPerLeaf: 2,
        lowerGuide: "Two stationary UHMW shoes per side with overlapping channel engagement",
        cover: "Removable steel splash hood",
        wheelSpinModeled: false,
        mountingDetail: "Bracket bores, fasteners, axle retention and frame attachments remain fabrication work"
      },
      baseMounts: {
        count: 4,
        loadPath: "Frame and doors -> dry-side outriggers -> steel side inserts -> solid casting bosses",
        trayIndependent: true,
        traySupports: "Four separate 25 mm shelf / 6 mm angle envelopes inside the lower frame",
        detail: "Geometric mounting envelopes; bracket bores, bolts, adjustment and insert retention remain unsized"
      },
      tray: {
        construction: machinekit.enclosure.TraySeat.CONSTRUCTION,
        pieces: 6,
        thicknessMm: 13,
        ledgeDepthMm: 4,
        compressedGasketMm: 2,
        topAboveCastingMm: 11,
        clamps: 10,
        fasteners: "Ten M6 x 30; nominal threads, torque and retention unqualified",
        motorBay: "Single machine/XrearCover lid; top-access screws, 3 mm lift, derived rearward slide, then lift clear; no tray parts removed",
        seams: "2 mm panel joints on gasketed backing; removable wet sealant required",
        materialQualification: false
      },
      servicePanels: shell.servicePanelIds,
      serviceOpenings: shell.serviceAccess,
      seams: shell.seamMethod,
      qualificationIssues: qualificationIssues,
      evidence: "X/Y/Z endpoints, A one-degree grid with radius*half-step padding; B/C descendants bounded by rotation-centred spheres",
      assumptions: "75 mm clearance; 400 mm opening; opposed belt-coupled leaves; OD140 side-jaw reservation; 100 mm mist opening; 60 C process and oil-mist rule assessment assumed. Robot, seals, retention and flow not qualified."
    }
    };
  }
}
