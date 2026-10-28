package machinekit.tooling;

import materia.tooling.export.ToolingFabrication.FabricationPlan;
import materia.tooling.ToolingValidation;
import materia.tooling.ToolingValidation.ToolingReadiness;
import materia.tooling.ToolingCodec;
import materia.tooling.export.ToolingManifest;
import materia.tooling.casting.CastingMoldPlan;
import materia.tooling.process.CastingProcessSpec;
import materia.casting.CastingInput;
import materia.casting.CastingInput.CastingTextInput;
import materia.tooling.ToolingDiagnostic.ToolingCheckStatus;
import materia.casting.InsertOrigin.InsertGeometryBasis;

typedef MissingToolingInput = {
  var reason:String;
  var checks:Array<String>;
  var artifacts:Array<String>;
}


/** Deterministic inventory of a freshly generated directory, excluding this manifest itself. */
class ToolingPackageManifest {
  static function fingerprint(manifest:Dynamic):String {
    return haxe.crypto.Sha256.encode(ToolingCodec.canonical({
      format: manifest.format,
      version: manifest.version,
      phase: manifest.phase,
      lengthUnit: manifest.lengthUnit,
      sourceFingerprint: manifest.sourceFingerprint,
      fabricationFingerprint: manifest.fabricationFingerprint,
      files: manifest.files,
      fabrication: manifest.fabrication,
      validation: manifest.validation,
      replacementGroups: manifest.replacementGroups
    }));
  }

  static function digest(bytes:haxe.io.Bytes):String {
    var value = haxe.crypto.Sha256.make(bytes), hex = new StringBuf();
    for (i in 0...value.length) hex.add(StringTools.hex(value.get(i), 2).toLowerCase());
    return hex.toString();
  }

  public static function write(
    dir:String,
    f:FabricationPlan,
    checks:Array<ToolingValidation>,
    plan:CastingMoldPlan,
    ?process:CastingProcessSpec
  ):Void {
    var records:Array<ToolingArtifact> = [];
    function visit(relative:String):Void {
      var names = sys.FileSystem.readDirectory(dir + (relative == "" ? "" : "/" + relative));
      names.sort(Reflect.compare);
      for (name in names) {
        var path = relative == "" ? name : relative + "/" + name;
        if (path == "manifest.json" || path == "README.md" || path == "START-HERE.md") continue;
        if (sys.FileSystem.isDirectory(dir + "/" + path)) visit(path);
        else {
          if (StringTools.endsWith(path, ".step")) reproducibleStep(dir + "/" + path);
          var bytes = sys.io.File.getBytes(dir + "/" + path);
          records.push({path: path, bytes: bytes.length, sha256: digest(bytes)});
        }
      }
    }
    var all:Array<ToolingValidation> = [], byRule:Map<String, ToolingValidation> = [];
    for (check in f.validation.concat(checks)) {
      var existing = byRule.get(check.rule);
      if (existing != null) {
        if (ToolingCodec.canonical(existing)
          != ToolingCodec.canonical(check)) throw "Conflicting fabrication validation: " + check.rule;
        continue;
      }
      byRule.set(check.rule, check);
      all.push(check);
    }
    all.sort((a, b) -> Reflect.compare(a.rule, b.rule));
    var seen = new Map<String, Bool>();
    for (check in all) {
      if (seen.exists(check.rule)) throw "Duplicate fabrication rule";
      seen.set(check.rule, true);
    }
    sys.io.File.saveContent(dir + "/fabrication-validation.json", ToolingCodec.canonical(all) + "\n");
    // The detailed report participates in the single hash inventory.
    records = [];
    visit("");
    var missing = missingInputs(plan, f, all, process);
    var status = statusText(all, missing, [for (record in records) record.path]);
    for (name in ["README.md", "START-HERE.md"]) {
      var text = (name == "README.md" ? "# Mold kit status\n\n" : "# Start here\n\n") + status;
      sys.io.File.saveContent(dir + "/" + name, text);
      var bytes = sys.io.File.getBytes(dir + "/" + name);
      records.push({path: name, bytes: bytes.length, sha256: digest(bytes)});
    }
    records.sort((a, b) -> Reflect.compare(a.path, b.path));
    var manifest:ToolingManifest = {
      format: "materia.tooling-manifest",
      version: 16,
      phase: 5,
      lengthUnit: materia.tooling.ToolingEnums.ToolingLengthUnit.Millimetres,
      sourceFingerprint: f.sourceFingerprint,
      fabricationFingerprint: f.fingerprint,
      packageFingerprint: "",
      files: records,
      fabrication: ToolingReadiness.evaluate(all),
      validation: "fabrication-validation.json",
      replacementGroups: f.replacementGroups
    };
    manifest.packageFingerprint = fingerprint(manifest);
    sys.io.File.saveContent(dir + "/manifest.json", ToolingCodec.canonical(manifest) + "\n");
  }


  /** Presentation only: collect typed unresolved values and their existing Pending
	 * dependencies. This does not rerun checks or change their status/readiness. */
  public static function missingInputs(
    plan:CastingMoldPlan,
    f:FabricationPlan,
    checks:Array<ToolingValidation>,
    ?process:CastingProcessSpec
  ):Array<MissingToolingInput> {
    var groups:Map<String, MissingToolingInput> = [];
    function add(reason:String, prefixes:Array<String>, artifacts:Array<String>, ?owner:String, ?suffix:String):Void {
      var blocked = checks.filter(check -> check.status == Pending
        && (owner == null || check.affectedIds.indexOf(owner) >= 0) && (suffix == null || StringTools.endsWith(
          check.rule,
          suffix
        )) && prefixes.filter(prefix -> check.rule == prefix || (StringTools.endsWith(
          prefix,
          "."
        ) && StringTools.startsWith(
          check.rule,
          prefix
        ))).length > 0);
      if (blocked.length == 0) return;
      var group = groups.get(reason);
      if (group == null) {
        group = {reason: reason, checks: [], artifacts: []};
        groups.set(reason, group);
      }
      for (check in blocked) if (group.checks.indexOf(check.rule) < 0) group.checks.push(check.rule);
      for (artifact in artifacts) if (group.artifacts.indexOf(artifact) < 0) group.artifacts.push(artifact);
    }
    function number(value:CastingInput, prefixes:Array<String>, artifacts:Array<String>, ?owner:String):Void {
      switch value {
        case Known(_):
        case Unresolved(reason):
          add(reason, prefixes, artifacts, owner);
      }
    }
    function text(value:CastingTextInput, prefixes:Array<String>, artifacts:Array<String>, ?owner:String):Void {
      switch value {
        case Known(_):
        case Unresolved(reason):
          add(reason, prefixes, artifacts, owner);
      }
    }
    var insertFiles = ["insert-manufacturing", "insert-", "fixture-", "datum-", "fabrication-validation.json"];
    var input = plan.document.input;
    for (requirement in input.interfaces) number(
      requirement.finishingProcessErrorMm,
      ["tooling.fixture.budget." + input.id + "." + requirement.id],
      insertFiles,
      input.id
    );
    for (insert in input.inserts) {
      for (requirement in insert.interfaces) number(
        requirement.finishingProcessErrorMm,
        ["tooling.fixture.budget." + insert.id + "." + requirement.id],
        insertFiles,
        insert.id
      );
      switch insert.origin {
        case Fabricated(manufacturing):
          text(manufacturing.processReference, ["fabrication.insert.plan."], insertFiles, insert.id);
          switch manufacturing.blank {
            case Unselected:
              add(
                "Insert blank stock and part-to-stock placement",
                ["fabrication.insert.blank."],
                insertFiles,
                insert.id
              );
            case Stock(_, _, _):
          }
          for (mount in manufacturing.mountingRequirements) {
            var rules = [materia.tooling.casting.InsertManufacturingPlan.mountingRule(insert.id, mount.holeId)];
            text(mount.clearanceReference, rules, insertFiles, insert.id);
            var b = mount.budget;
            for (value in [
              b.clearanceDiameterMm,
              b.placementOffsetMm,
              b.placementAngleRad,
              b.assemblyAdjustmentMm,
              b.cureBowMm,
              b.machiningErrorMm,
              b.pitchAccumulationMm,
              b.requiredMarginMm
            ]) number(
              value,
              rules,
              insertFiles,
              insert.id
            );
          }
        case Catalogue(product):
          if (product.geometryBasis == NominalStudy) add(
            "Verified catalogue insert geometry: " + product.manufacturer + " " + product.reference,
            ["tooling.insert."],
            insertFiles,
            insert.id
          );
      }
      switch insert.anchorage {
        case Bolted(_, anchors):
          for (anchor in anchors) switch anchor.locking {
            case RetainingCompound(reference):
              text(
                reference,
                ["tooling.insert.anchorage." + insert.id + "." + anchor.id + ".locking"],
                [
                  "insert-anchorage",
                  "fabrication-validation.json"
                ],
                insert.id
              );
            case Shoulder:
          }
        case Undeclared, CatalogueRetention:
      }
    }
    var context = input.interfaceContext;
    if (context != null) {
      for (occurrence in context.relationships) {
        var metrology = occurrence.relationship.metrology;
        var rule = ["tooling.relationship.metrology-budget." + haxe.crypto.Sha256.encode(occurrence.id)];
        text(metrology.instrumentReference, rule, ["replication-", "interface-", "fabrication-validation.json"]);
        number(metrology.uncertaintyMm, rule, ["replication-", "interface-", "fabrication-validation.json"]);
      }
    }
    if (f.spec.mixBearingStressMPa == null) add(
      "Cured mix bearing strength",
      ["tooling.insert.anchorage."],
      [
        "insert-anchorage",
        "fabrication-validation.json"
      ],
      null,
      ".capacity.bearing"
    );
    if (f.spec.mixTensileStrengthMPa == null) add(
      "Cured mix tensile strength",
      ["tooling.insert.anchorage."],
      [
        "insert-anchorage",
        "fabrication-validation.json"
      ],
      null,
      ".capacity.cone"
    );
    if (process != null) {
      var compoundRules = ["process.compound."];
      var processFiles = ["pour-", "process-", "castability-", "fabrication-validation.json"];
      text(process.compound.reference, compoundRules, processFiles);
      number(process.compound.maximumMassPerLiftKg, compoundRules, processFiles);
      switch process.compound.thickness {
        case Unresolved(reason):
          add(reason, compoundRules, processFiles);
        case Section(_), Layer(_):
      }
      if (process.mixComposition == null) add(
        "Mix ingredients and weight fractions",
        ["process.pour.mix-quantities"],
        processFiles
      );
      if (process.mixer == null) add(
        "Mixer capacity, mixing/transfer times and working time",
        ["process.pour.batch-schedule"],
        processFiles
      );
      if (process.maximumAirEscapeWidthMm == null) add(
        "Permitted air-escape width",
        ["process.fill.air-escape-width"],
        [
          "fill-",
          "fabrication-validation.json"
        ]
      );
    }
    // Existing physical-evidence checks have named dependencies, not guessed
    // values or reflected nullable field names. Source changes stay separate.
    add(
      "Machine, measured tools, fixtures and work-coordinate qualification",
      [
        "fabrication.cam.",
        "fabrication.fixture-machining."
      ],
      [
        "cam-",
        ".ngc",
        "machining-instructions.md"
      ]
    );
    add(
      "Board/grip, pocket-jig, machining and contact-face sealing trial",
      ["fabrication.former-machining."],
      [
        "former-",
        "machining-instructions.md"
      ]
    );
    add(
      "Compaction method and measured peak pressure",
      ["fabrication.loads.compaction", "anchorage-compaction"],
      [
        "load-",
        "fabrication-validation.json"
      ]
    );
    add(
      "Insert anchorage pull-out/shear coupon",
      ["tooling.insert.anchorage."],
      [
        "insert-anchorage",
        "fabrication-validation.json"
      ],
      null,
      ".capacity"
    );
    add(
      "Closure, brace, restraint and support load-path capacity evidence",
      [
        "fabrication.braces",
        "fabrication.fasteners",
        "fabrication.core-restraint.",
        "fabrication.rigid-restraint.",
        "fabrication.support-loadpath.",
        "closure-loads"
      ],
      [
        "load-",
        "hardware",
        "fabrication-validation.json"
      ]
    );
    add(
      "Resin, EPS, adhesive and release-system compatibility evidence",
      [
        "fabrication.materials.compatibility",
        "materials-process"
      ],
      [
        "material",
        "fabrication-validation.json"
      ]
    );
    add(
      "Insert inspection results and drawing release",
      [
        "fabrication.insert.inspection.",
        "fabrication.insert.final-inspection.",
        "fabrication.insert.drawing."
      ],
      insertFiles
    );
    add(
      "Datum accuracy, fixing, hole protection and post-cure removal evidence",
      [
        "tooling.fixture.attachment.",
        "insert-fixtures"
      ],
      insertFiles
    );
    for (unit in plan.coreUnits) add(
      "EPS breakup and extraction trial",
      ["process.castability.release." + unit.featureId],
      [
        "core-",
        "castability-",
        "fabrication-validation.json"
      ]
    );
    add(
      "Handling equipment, measured mass/CoM and flip procedure",
      ["tooling.handling.flip"],
      [
        "handling",
        "fabrication-validation.json"
      ]
    );
    add(
      "Native reconstruction and sequence review",
      ["fabrication.geometry-and-sequence"],
      ["fabrication-validation.json"]
    );
    var result = [for (group in groups) group];
    for (group in result) {
      group.checks.sort(Reflect.compare);
      group.artifacts.sort(Reflect.compare);
    }
    result.sort((a, b) -> Reflect.compare(a.reason, b.reason));
    return result;
  }

  static function markdown(text:String):String {
    return StringTools.replace(StringTools.replace(StringTools.replace(text, "\r", " "), "\n", " "), "`", "'");
  }

  /** A display selector names the exact shared rule segments. Instance records
	 * stay in the detailed report rather than being repeated in the opening list. */
  static function selectors(ids:Array<String>):Array<String> {
    var families:Map<String, Array<String>> = [];
    for (id in ids) {
      var key = id.split(".").slice(0, 2).join(".");
      var family = families.get(key);
      if (family == null) {
        family = [];
        families.set(key, family);
      }
      family.push(id);
    }
    var result:Array<String> = [];
    for (family in families) {
      family.sort(Reflect.compare);
      if (family.length == 1) {
        result.push("`" + family[0] + "` (1 check)");
        continue;
      }
      var rows = [for (id in family) id.split(".")], first = rows[0], head = 0, tail = 0;
      while (head < first.length
        && rows.filter(row -> row.length <= head || row[head] != first[head]).length == 0) head++;
      while (tail < first.length - head && rows.filter(row -> row.length <= head + tail
        || row[row.length - tail - 1] != first[first.length - tail - 1]).length == 0) tail++;
      var parts = first.slice(0, head);
      parts.push("*");
      if (tail > 0) parts = parts.concat(first.slice(first.length - tail));
      result.push("`" + parts.join(".") + "` (" + family.length + " checks)");
    }
    result.sort(Reflect.compare);
    return result;
  }

  public static function statusText(
    checks:Array<ToolingValidation>,
    missing:Array<MissingToolingInput>,
    files:Array<String>
  ):String {
    var output = new StringBuf(), covered:Map<String, Bool> = [];
    output.add("## Missing inputs and evidence\n\n");
    output
      .add("These inputs block qualification and release of the linked review artifacts. Rule selectors below summarize the instance IDs in [the detailed report](fabrication-validation.json); no instance record or check status is removed.\n\n");
    if (missing.length == 0) output.add("No unresolved inputs block the recorded checks.\n\n");
    for (input in missing) {
      output.add("- **" + markdown(input.reason) + "**\n");
      output.add("  Blocks checks: " + selectors(input.checks).join(", ") + ".\n");
      var artifacts = files.filter(file -> input.artifacts.filter(prefix -> StringTools.startsWith(file,
        prefix) || StringTools.endsWith(
          file,
          prefix
        )).length > 0);
      artifacts.sort(Reflect.compare);
      var links = artifacts.length <= 4 ? [for (file in artifacts) "[" + file + "](" + file + ")"].join(", ")
        : artifacts.length + " files in [the inventory](manifest.json), selected by "
        + [for (prefix in input.artifacts) "`" + prefix + "`"].join(", ");
      output.add("  Blocks artifact release: " + (artifacts.length == 0 ? "see the detailed report" : links) + ".\n\n");
      for (id in input.checks) covered.set(id, true);
    }
    var readiness = ToolingReadiness.evaluate(checks);
    output.add("## Recorded kit status\n\n");
    output.add("Fabrication ready: **" + (readiness.ready ? "yes"
      : "no") + "**. " + checks.filter(row -> row.status == Pass).length + " Pass / " + checks.filter(row -> row.status
      == Pending).length + " Pending / " + checks.filter(row -> row.status
      == Unsupported).length + " Unsupported / " + checks.filter(row -> row.status == Fail).length + " Fail.\n\n");
    for (check in checks) if (check.status
      == Fail) output.add("- **Fail** `" + check.rule + "`: " + markdown(check.message) + "\n");
    output
      .add("\nSource actions and other unresolved checks remain in [the detailed report](fabrication-validation.json). They are not replaced by the input summary.\n\n");
    for (status in [Pending, Unsupported]) {
      var ids = [for (check in checks) if (check.status == status && !covered.exists(check.rule)) check.rule];
      if (ids.length > 0) output.add("- " + status + ": " + selectors(ids).join(", ") + ".\n");
    }
    return output.toString();
  }

  /** Normalize only the generated STEP timestamp, never its geometry. */
  static function reproducibleStep(path:String):Void {
    var s = sys.io.File.getContent(path), prefix = "FILE_NAME('Open CASCADE Shape Model','";
    var start = s.indexOf(prefix);
    if (start < 0) throw "Generated STEP has an unsupported header";
    start += prefix.length;
    var end = s.indexOf("'", start);
    if (end < start) throw "Generated STEP has an invalid header";
    var normalized = s.substr(0, start) + "1970-01-01T00:00:00" + s.substr(end);
    if (normalized != s) sys.io.File.saveContent(path, normalized);
  }

  static function readValidation(raw:Dynamic):Array<ToolingValidation> {
    if (!Std.isOfType(raw, Array)) throw "Invalid tooling validation records";
    var rows:Array<Dynamic> = cast raw, result:Array<ToolingValidation> = [];
    function strings(value:Dynamic):Array<String> {
      if (!Std.isOfType(value, Array)) throw "Invalid tooling validation list";
      var values:Array<Dynamic> = cast value;
      return [for (entry in values) {
        if (!Std.isOfType(entry, String)) throw "Invalid tooling validation text";
        var text:String = cast entry;
        text;
      }
      ];
    }
    for (row in rows) {
      materia.casting.CastingFeatureCodec.fields(
        row,
        [
          "rule",
          "affectedIds",
          "status",
          "severity",
          "message",
          "evidence"
        ]
      );
      var status:materia.tooling.ToolingDiagnostic.ToolingCheckStatus =
        switch materia.casting.CastingRecordCodec.string(
          row,
          "status"
        ) {
        case "pass":
          materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pass;
        case "pending":
          materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Pending;
        case "fail":
          materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Fail;
        case "unsupported":
          materia.tooling.ToolingDiagnostic.ToolingCheckStatus.Unsupported;
        default:
          throw "Invalid tooling validation status";
      };
      var severity:materia.tooling.ToolingDiagnostic.ToolingSeverity = switch materia.casting.CastingRecordCodec.string(
        row,
        "severity"
      ) {
        case "info":
          materia.tooling.ToolingDiagnostic.ToolingSeverity.Info;
        case "warning":
          materia.tooling.ToolingDiagnostic.ToolingSeverity.Warning;
        case "error":
          materia.tooling.ToolingDiagnostic.ToolingSeverity.Error;
        default:
          throw "Invalid tooling validation severity";
      };
      var check:ToolingValidation = {
        rule: materia.casting.CastingRecordCodec.string(row, "rule"),
        affectedIds: strings(Reflect.field(
          row,
          "affectedIds"
        )),
        status: status,
        severity: severity,
        message: materia.casting.CastingRecordCodec.string(
          row,
          "message"
        ),
        evidence: strings(Reflect.field(
          row,
          "evidence"
        ))
      };
      ToolingReadiness.validate(check);
      result.push(check);
    }
    return result;
  }

  public static function verify(dir:String):Void {
    var manifest:Dynamic = haxe.Json.parse(sys.io.File.getContent(dir + "/manifest.json"));
    var keys = [
      "format",
      "version",
      "phase",
      "lengthUnit",
      "sourceFingerprint",
      "fabricationFingerprint",
      "packageFingerprint",
      "files",
      "fabrication",
      "validation",
      "replacementGroups"
    ];
    for (key in Reflect.fields(manifest)) if (keys.indexOf(key) < 0) throw "Unknown tooling manifest field";
    for (key in keys) if (!Reflect.hasField(manifest, key)) throw "Missing tooling manifest field";
    if (manifest.format != "materia.tooling-manifest" || manifest.version != 16) throw "Unsupported tooling manifest";
    if (!Std.isOfType(manifest.files, Array)) throw "Invalid tooling manifest files";
    var files:Array<Dynamic> = cast manifest.files, previous = "";
    for (file in files) {
      var name:String = file.path;
      if (name == null || name.length == 0 || name.indexOf("\\") >= 0 || name == "manifest.json" || Reflect.compare(
        previous,
        name
      ) >= 0) throw "Invalid tooling manifest path/order";
      for (part in name.split("/")) if (part == ""
        || part == "." || part == "..") throw "Invalid tooling manifest path";
      previous = name;
      var path = dir + "/" + name;
      if (!sys.FileSystem.exists(path) || sys.FileSystem.isDirectory(path)) throw "Missing tooling artifact: " + name;
      var bytes = sys.io.File.getBytes(path);
      if (bytes.length != file.bytes || digest(bytes) != file.sha256) throw "Tooling artifact hash differs: " + name;
    }
    if (manifest.validation != "fabrication-validation.json" || files.filter(file -> file.path
      == manifest.validation).length != 1) throw "Missing hashed fabrication validation report";
    readValidation(haxe.Json.parse(sys.io.File.getContent(dir + "/" + manifest.validation)));
    var expected = fingerprint(manifest);
    if (expected != manifest.packageFingerprint) throw "Tooling package fingerprint differs";
  }
}
