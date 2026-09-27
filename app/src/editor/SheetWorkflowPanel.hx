package app.editor;

import haxe.io.Path as SheetPath;
import LayoutAxis;
import LayoutStyle;
import materia.project.MaterialLibrary;
import materia.sheet.SheetCutPlan;
import materia.sheet.SheetInventory;
import materia.sheet.SheetPartRequirement;
import materia.sheet.SheetPlanExporter;
import materia.sheet.SheetPlanValidation;
import materia.sheet.SheetPlanValidator;
import nativekit.ui.properties.PropertyDescriptor;
import nativekit.ui.properties.PropertyDescriptorOptions;
import nativekit.ui.properties.PropertyOption;
import nativekit.ui.properties.PropertyType;
import nativekit.ui.properties.PropertyValue;
import nativekit.ui.widgets.KeyedView;
import nativekit.ui.widgets.controls.Button;
import nativekit.ui.widgets.controls.Select;
import nativekit.ui.widgets.controls.SelectOption;
import nativekit.ui.widgets.layout.Column;
import nativekit.ui.widgets.layout.Row;
import nativekit.ui.widgets.properties.PropertyInspector;
import nativekit.ui.widgets.scroll.ScrollView;
import nativekit.ui.widgets.text.Text;
import sys.FileSystem;
import sys.io.AtomicFile;

/** Stock selection, manual plan editing, validation, exports, and cut confirmation in Materia. */
@:access(app.Main.ReferenceEditorApp) class SheetWorkflowPanel {
  public static function build(app:ReferenceEditorApp):Null < nativekit.ui.core.View > {
    var projectReference = app.session.projectReference;
    if (projectReference == null) return null;
    var path = projectReference + ".sheet.json";
    if (!FileSystem.exists(path)) return null;
    try {
      if (app.sheetInventory == null || app.sheetInventoryPath != path) {
        app.sheetInventory = SheetInventory.open(path);
        app.sheetInventoryPath = path;
        app.sheetPlanSelection = "";
        app.sheetPieceSelection = "";
        app.sheetOperationsExpanded = false;
      }
    } catch (error:Dynamic) {
      return new Column(
        "sheet-workflow-error",
        [
          new KeyedView("heading", new Text("SHEET CUTTING PLAN")),
          new KeyedView(
            "error",
            new Text("Could not open sheet records: " + Std.string(error))
          )
        ]
      );
    }
    var inventory:SheetInventory = cast app.sheetInventory;
    if (inventory.record.plans.length == 0) return new Column(
      "sheet-workflow-empty",
      [
        new KeyedView(
          "heading",
          new Text("SHEET CUTTING PLAN")
        ),
        new KeyedView(
          "empty",
          new Text("This project has no cutting plans.")
        )
      ]
    );
    var selectedPlanId = app.sheetPlanSelection;
    var hasPlan = false;
    for (plan in inventory.record.plans) if (plan.id == selectedPlanId) hasPlan = true;
    if (!hasPlan) selectedPlanId = inventory.record.plans[0].id;
    app.sheetPlanSelection = selectedPlanId;
    var selectedPlan = inventory.plan(selectedPlanId);
    var selectedPieceId = app.sheetPieceSelection;
    var hasPiece = selectedPieceId == "";
    for (piece in inventory.record.stockPieces) if (piece.id == selectedPieceId) hasPiece = true;
    if (!hasPiece) selectedPieceId = "";
    app.sheetPieceSelection = selectedPieceId;
    var selectedPiece:Null<materia.sheet.StockPiece> = selectedPieceId == "" ? null : inventory.piece(selectedPieceId);

    var planningChildren:Array<KeyedView> = [new KeyedView("heading", new Text("Cut planning"))];
    var operationsChildren:Array<KeyedView> = [];
    var planOptions = [for (plan in inventory.record.plans) new SelectOption<String>(plan.id, plan.id, plan.id)];
    planningChildren.push(new KeyedView("plan-select",
      new Select<String>("sheet-plan-select", planOptions, selectedPlanId, function(value) {
      app.sheetPlanSelection = value;
      app.commands.refresh();
    }
    )));
    var pieceOptions:Array<SelectOption < String>> = [];
    var stockSpec = inventory.stockSpec(selectedPlan.stockSpecId);
    pieceOptions.push(new SelectOption<String>("nominal",
      "Nominal stock · " + fmt(stockSpec.width) + " × " + fmt(stockSpec.height) + " " + stockSpec.lengthUnit,
      ""));
    for (piece in inventory.record.stockPieces) pieceOptions.push(new SelectOption<String>(
      piece.id,
      piece.id + " · " + piece.state + " · measured " + fmt(piece.width) + " × " + fmt(piece.height) + " " + piece.lengthUnit,
      piece.id
    ));
    planningChildren.push(new KeyedView("source-select",
      new Select<String>("sheet-piece-select", pieceOptions, selectedPieceId, function(value) {
      app.sheetPieceSelection = value;
      app.commands.refresh();
      }
    )));
    planningChildren.push(new KeyedView("stock-description", new Text(stockDescription(inventory, selectedPlan,
      selectedPieceId == "" ? null : inventory.piece(selectedPieceId)))));
    for (requirementReference in selectedPlan.requirements) {
      var item = inventory.requirement(requirementReference.id);
      planningChildren.push(new KeyedView(
        "finished-bom-" + item.id,
        new Text("Finished-part BOM · " + item.partId + " rev " + item.partRevision + " · quantity " + item.quantity)
      ));
    }
    var canAllocate = selectedPiece != null && (selectedPiece.state == "available" ||
      (selectedPiece.state == "allocated" && selectedPiece.allocationPlanId == selectedPlanId));
    var canRelease = selectedPiece != null && selectedPiece.state == "allocated" &&
      selectedPiece.allocationPlanId == selectedPlanId;
    var canComplete = canRelease;
    operationsChildren.push(new KeyedView("stock-actions", new Row("sheet-stock-actions",
      [new KeyedView("register", button(app, "Register sheet", "sheet-register", function() {
      var nextId = "sheet-" +(inventory.record.stockPieces.length + 1);
      while (containsPiece(inventory, nextId)) nextId += "-new";
      var sheet = inventory.registerSheet(nextId, selectedPlan.stockSpecId);
      app.sheetPieceSelection = sheet.id;
    }
    )), new KeyedView(
      "allocate",
      button(
        app,
        "Allocate",
        "sheet-allocate",
        function() inventory.allocate(
          selectedPieceId,
          selectedPlanId
        ), canAllocate
      )
    ), new KeyedView(
      "release",
      button(
        app,
        "Release",
        "sheet-release",
        function() inventory.cancelAllocation(
          selectedPieceId,
          selectedPlanId
        ), canRelease
      )
    )], ReferenceEditorApp.actionRowStyle())));

    var validation:Null<SheetPlanValidation> = null;
    try validation = inventory.preview(selectedPlanId, selectedPieceId) catch (_:Dynamic) {
    }
    planningChildren.push(new KeyedView(
      "validation",
      new Text(validation == null ? "Plan could not be validated."
      : validation.valid ? "Plan valid · area balance " + fmt(validation.areaBalanceErrorMm2) + " mm²"
      : "Plan invalid · " + validation.messages.join(" · "))
    ));
    if (validation != null) for (region in validation.regions) if (region.retained) planningChildren.push(new KeyedView(
      "remnant-" + region.id,
      new Text('Remnant ${region.id} · ${fmt(region.widthMm)} × ${fmt(region.heightMm)} mm')
    ));
    if (validation != null) planningChildren.push(new KeyedView(
      "layout-preview",
      new Text(layoutPreview(
        selectedPlan,
        stockSpec,
        selectedPieceId == "" ? null : inventory.piece(selectedPieceId)
      ))
    ));
    for (placement in selectedPlan.placements) planningChildren.push(new KeyedView(
      "placement-line-" + placement.id,
      new Text('${placement.id} · (${fmt(placement.x)}, ${fmt(placement.y)}) · ${fmt(placement.width)} × ${fmt(placement.height)} ${selectedPlan.lengthUnit} · ${placement.rotation}°')
    ));

    planningChildren.push(new KeyedView("planning-actions", new Row("sheet-planning-actions",
      [new KeyedView("validate", button(app, "Validate", "sheet-validate", function() {
      var result = inventory.preview(selectedPlanId, selectedPieceId);
      app.log(result.valid ? "Sheet plan is valid · area balance " + fmt(result.areaBalanceErrorMm2) + " mm²"
        : "Sheet plan is invalid · " + result.messages.join(" · "));
    }
    )), new KeyedView(
      "refresh",
      button(
        app,
        "Refresh references",
        "sheet-refresh",
        function() inventory.revalidatePlan(selectedPlanId)
      )
    ), new KeyedView(
      "save",
      button(
        app,
        "Save",
        "sheet-save",
        function() inventory.save()
      )
    ), new KeyedView("export", button(app, "Export CSV + SVG", "sheet-export", function() {
      var current = inventory.preview(selectedPlanId, selectedPieceId);
      if (!current.valid) throw "Fix plan validation messages before exporting";
      var stock = inventory.stockSpec(selectedPlan.stockSpecId);
      var requirements =
        [for (requirementReference in selectedPlan.requirements) inventory.requirement(requirementReference.id)];
      var prefix = SheetPath.join([SheetPath.directory(path), selectedPlan.id]);
      AtomicFile.write(
        prefix + ".csv",
        SheetPlanExporter.csv(
          selectedPlan,
          stock,
          requirements,
          current,
          selectedPieceId == "" ? null : inventory.piece(selectedPieceId)
        )
      );
      AtomicFile.write(
        prefix + ".svg",
        SheetPlanExporter.svg(
          selectedPlan,
          stock,
          requirements,
          current,
          selectedPieceId == "" ? null : inventory.piece(selectedPieceId)
        )
      );
      app.log("Exported " + prefix + ".csv and .svg");
    }
    ))], ReferenceEditorApp.actionRowStyle())));
    operationsChildren.push(new KeyedView("confirm-actions", new Row("sheet-confirm-actions",
      [new KeyedView("confirm", button(app, "Confirm completed cutting", "sheet-confirm", function() {
      var timestamp = Std.string(Date.now().getTime());
      var id = "cut-" + selectedPlanId + "-" + timestamp;
      var execution = inventory.execute(selectedPlanId, selectedPieceId, id, true);
      app.log("Recorded "
        + execution.id + ": " + execution.blanks.length + " blanks, " + execution.remnantPieceIds.length + " remnants");
    }, canComplete)
    ), new KeyedView("revalidate-bom", button(app, "Copy finished BOM to log", "sheet-bom", function() {
      var lines:Array<String> = [];
      for (requirementReference in selectedPlan.requirements) {
        var item = inventory.requirement(requirementReference.id);
        lines.push(item.partId + "," + item.quantity + "," + item.materialId);
      }
      app.log("Finished BOM (part, quantity, material): " + lines.join(" · "));
    }
    ))], ReferenceEditorApp.actionRowStyle())));

    var properties:Array<PropertyDescriptor> = [];
    for (placement in selectedPlan.placements) {
      var placementId = placement.id;
      properties.push(planNumber(
        app,
        inventory,
        selectedPlanId,
        "placement:" + placementId + ":x",
        placementId + " X",
        function(current) return placementValue(
          current,
          placementId,
          "x"
        ),
        function(
          current,
          value
        ) setPlacement(
          current,
          placementId,
          "x",
          value
        ),
        selectedPlan.lengthUnit,
        0,
        1
      ));
      properties.push(planNumber(
        app,
        inventory,
        selectedPlanId,
        "placement:" + placementId + ":y",
        placementId + " Y",
        function(current) return placementValue(
          current,
          placementId,
          "y"
        ),
        function(
          current,
          value
        ) setPlacement(
          current,
          placementId,
          "y",
          value
        ),
        selectedPlan.lengthUnit,
        0,
        1
      ));
      properties.push(planNumber(
        app,
        inventory,
        selectedPlanId,
        "placement:" + placementId + ":width",
        placementId + " blank width",
        function(current) return placementValue(
          current,
          placementId,
          "width"
        ),
        function(
          current,
          value
        ) setPlacement(
          current,
          placementId,
          "width",
          value
        ),
        selectedPlan.lengthUnit,
        0.001,
        1
      ));
      properties.push(planNumber(
        app,
        inventory,
        selectedPlanId,
        "placement:" + placementId + ":height",
        placementId + " blank height",
        function(current) return placementValue(
          current,
          placementId,
          "height"
        ),
        function(
          current,
          value
        ) setPlacement(
          current,
          placementId,
          "height",
          value
        ),
        selectedPlan.lengthUnit,
        0.001,
        1
      ));
      var rotationOptions = new PropertyDescriptorOptions();
      rotationOptions.category = "Part placements";
      rotationOptions.recordHistory = false;
      rotationOptions.options = [new PropertyOption("0", "0°"), new PropertyOption("90", "90°")];
      properties.push(new PropertyDescriptor("sheet:" + selectedPlanId + ":placement:" + placementId + ":rotation",
        placementId + " rotation",
        PropertyType.Enum, function(_) return PropertyValue.Enum(Std.string(placement.rotation)), function(
          _,
          value
        ) switch (value) {
        case PropertyValue.Enum(key):
          var rotation = Std.parseInt(key);
          inventory.editPlan(selectedPlanId, function(plan) {
            for (item in plan.placements) if (item.id == placementId) item.rotation = rotation;
          }
          );
          app.commands.refresh();
        default:
          throw "Rotation requires a supported angle";
      }, rotationOptions));
    }
    for (index in 0...selectedPlan.cuts.length) {
      var cutId = selectedPlan.cuts[index].id, cutIndex = index;
      properties.push(planNumber(
        app,
        inventory,
        selectedPlanId,
        "cut:" + cutId + ":position",
        "Cut " +(index + 1) + " position",
        function(current) return cutPosition(
          current,
          cutId
        ),
        function(
          current,
          value
        ) setCutPosition(
          current,
          cutId,
          value
        ),
        selectedPlan.lengthUnit,
        0.001,
        1
      ));
      planningChildren.push(new KeyedView(
        "cut-order-" + cutId,
        new Row(
          "cut-order-row-" + cutId,
          [
            new KeyedView(
              "cut-id",
              new Text((index + 1) + " · " + cutId)
            ),
            new KeyedView(
              "move-up",
              button(
                app,
                "↑",
                "sheet-cut-up-" + cutId,
                function() moveCut(
                  inventory,
                  selectedPlanId,
                  cutIndex,
                  -1
                )
              )
            ),
            new KeyedView(
              "move-down",
              button(
                app,
                "↓",
                "sheet-cut-down-" + cutId,
                function() moveCut(
                  inventory,
                  selectedPlanId,
                  cutIndex,
                  1
                )
              )
            )
          ],
          ReferenceEditorApp.actionRowStyle()
        )
      ));
    }
    for (requirementReference in selectedPlan.requirements) {
      var requirementId = requirementReference.id;
      var item = inventory.requirement(requirementId);
      planningChildren.push(new KeyedView(
        "requirement-link-" + requirementId,
        new Text(item.partId + " · part revision " + item.partRevision + " · quantity " + item.quantity)
      ));
      properties.push(requirementNumber(
        app,
        inventory,
        requirementId,
        "quantity",
        requirementId + " quantity",
        "",
        1,
        1
      ));
      properties.push(requirementNumber(
        app,
        inventory,
        requirementId,
        "blankWidth",
        requirementId + " blank width",
        item.lengthUnit,
        0.001,
        1
      ));
      properties.push(requirementNumber(
        app,
        inventory,
        requirementId,
        "blankHeight",
        requirementId + " blank height",
        item.lengthUnit,
        0.001,
        1
      ));
      properties.push(requirementNumber(
        app,
        inventory,
        requirementId,
        "thickness",
        requirementId + " thickness",
        item.lengthUnit,
        0.001,
        0.1
      ));
      var materialOptions = new PropertyDescriptorOptions();
      materialOptions.category = "Part requirements";
      materialOptions.recordHistory = false;
      for (material in MaterialLibrary.all()) materialOptions.options.push(new PropertyOption(
        material.id,
        material.name
      ));
      if (inventory.record.customMaterials
        != null) for (material in inventory.record.customMaterials) materialOptions.options.push(new PropertyOption(
          material.id,
          material.name
        ));
      properties.push(new PropertyDescriptor("sheet:" + selectedPlanId + ":requirement:" + requirementId + ":material",
        requirementId + " material", PropertyType.Enum,
        function(_) return PropertyValue.Enum(inventory.requirement(requirementId).materialId), function(
          _,
          value
        ) switch (value) {
        case PropertyValue.Enum(key):
          inventory.editRequirement(requirementId, function(current) current.materialId = key);
        default:
          throw "Material requires a selection";
      }, materialOptions));
    }
    properties.push(planNumber(
      app,
      inventory,
      selectedPlanId,
      "kerf",
      "Saw kerf",
      function(current) return current.kerf,
      function(
        current,
        value
      ) current.kerf = value,
      selectedPlan.lengthUnit,
      0.001,
      0.1
    ));
    properties.push(planNumber(
      app,
      inventory,
      selectedPlanId,
      "edgeMargin",
      "Edge margin",
      function(current) return current.edgeMargin,
      function(
        current,
        value
      ) current.edgeMargin = value,
      selectedPlan.lengthUnit,
      0,
      1
    ));
    planningChildren.push(new KeyedView(
      "numeric-editors",
      new PropertyInspector(
        "sheet-plan-inspector:" + selectedPlanId,
        properties,
        null,
        null,
        null,
        null,
        "Placements, cuts, and part requirements"
      )
    ));
    for (execution in inventory.record.executions) if (execution.planId == selectedPlanId) {
      operationsChildren.push(new KeyedView("execution-" + execution.id,
        new Text("Execution " + execution.id + " · consumed " + execution.consumedPieceId)));
      for (blank in execution.blanks) operationsChildren.push(new KeyedView("blank-" + blank.id,
        new Text("Blank " + blank.partId + " · " + blank.widthMm + " × " + blank.heightMm + " mm · from " + blank.sourcePieceId)));
      for (remnantId in execution.remnantPieceIds) {
        var remnant = inventory.piece(remnantId);
        operationsChildren.push(new KeyedView("output-remnant-" + remnantId,
          new Text("Remnant " + remnant.id + " · " + fmt(remnant.width) + " × " + fmt(remnant.height) + " " + remnant.lengthUnit + " · " + remnant.state)));
      }
    }
    var operationsSection:Array<KeyedView> = [
      new KeyedView("operations-heading", new Text("Operations")),
      new KeyedView("operations-toggle", button(app,
        app.sheetOperationsExpanded ? "Hide operations" : "Show operations",
        "sheet-operations-toggle", function() app.sheetOperationsExpanded = !app.sheetOperationsExpanded))
    ];
    if (app.sheetOperationsExpanded) operationsSection.push(new KeyedView("operations-content",
      new Column("sheet-operations-content", operationsChildren, sectionStyle())));
    var children:Array<KeyedView> = [
      new KeyedView("cut-planning-section", new Column("sheet-cut-planning", planningChildren, sectionStyle())),
      new KeyedView("operations-section", new Column("sheet-operations", operationsSection, sectionStyle()))
    ];
    var contentStyle = sectionStyle();
    var scrollStyle = ReferenceEditorApp.fillStyle();
    scrollStyle.clipHorizontal = true;
    return new ScrollView(
      "sheet-workflow-scroll",
      new Column("sheet-workflow-panel", children, contentStyle),
      scrollStyle
    );
  }

  static function button(app:ReferenceEditorApp,
    label:String, key:String, action:Void -> Void, enabled:Bool = true):Button {
    var result = new Button(label, null, function() {
      try {
        action();
        app.commands.refresh();
      } catch (error:Dynamic) app.log("Sheet workflow: " + Std.string(error));
    }, key);
    result.enabled = enabled;
    return result;
  }

  static function layoutPreview(plan:SheetCutPlan, stock:materia.sheet.SheetStockSpec,
    piece:Null<materia.sheet.StockPiece>):String {
    var scale = SheetPlanValidator.millimetres(1, plan.lengthUnit);
    var stockUnit = piece == null ? stock.lengthUnit : piece.lengthUnit;
    var widthMm = SheetPlanValidator.millimetres(piece == null ? stock.width : piece.width, stockUnit);
    var heightMm = SheetPlanValidator.millimetres(piece == null ? stock.height : piece.height, stockUnit);
    var columns = 32, rows = 12;
    var cells:Array<Array < String>> = [];
    for (_ in 0...rows) {
      var row:Array<String> = [];
      for (_ in 0...columns) row.push(" ");
      cells.push(row);
    }
    var symbols = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
    for (placementIndex in 0...plan.placements.length) {
      var placement = plan.placements[placementIndex];
      var x0 = placement.x * scale, y0 = placement.y * scale;
      var x1 = x0 + placement.width * scale, y1 = y0 + placement.height * scale;
      var symbol = symbols.charAt(placementIndex % symbols.length);
      for (row in 0...rows) for (column in 0...columns) {
        var x =(column + 0.5) * widthMm / columns;
        var y = heightMm -(row + 0.5) * heightMm / rows;
        if (x >= x0 && x < x1 && y >= y0 && y < y1) cells[row][column] = symbol;
      }
    }
    var border = "+";
    for (_ in 0...columns) border += "-";
    border += "+";
    var lines:Array<String> = [
      "LAYOUT PREVIEW · " + (piece == null ? "Nominal stock" : "Physical sheet " + piece.id) +
        " · lower-left origin · " + fmt(widthMm) + " × " + fmt(heightMm) + " mm",
      border
    ];
    for (row in cells) lines.push("|" + row.join("") + "|");
    lines.push(border);
    for (index in 0...plan.placements.length) lines.push(symbols.charAt(index
      % symbols.length) + " = " + plan.placements[index].id);
    lines.push("Dimensions shown in " + plan.lengthUnit + " for placement values; preview grid is proportional.");
    return lines.join("\n");
  }

  static function stockDescription(inventory:SheetInventory, plan:SheetCutPlan,
    piece:Null<materia.sheet.StockPiece>):String {
    var stock = inventory.stockSpec(plan.stockSpecId);
    if (piece == null) return "Input · Nominal stock · " + stock.id + " specification dimensions · " + stock.width
      + " × " + stock.height + " × " + stock.thickness + " " + stock.lengthUnit + " · " + stock.materialId;
    return "Input · Physical sheet " + piece.id + " · measured " + piece.width + " × " + piece.height + " × " + piece.thickness
      + " " + piece.lengthUnit + " · specification " + stock.id + " · " + stock.materialId;
  }

  static function sectionStyle():LayoutStyle {
    var result = ReferenceEditorApp.fillStyle();
    result.height = LayoutAxis.fit();
    result.childGap = 6.0;
    return result;
  }

  static function planNumber(
    app:ReferenceEditorApp,
    inventory:SheetInventory,
    planId:String,
    id:String,
    label:String,
    read:SheetCutPlan -> Float,
    write:(
      SheetCutPlan,
      Float
    ) -> Void,
    unit:String,
    minimum:Float,
    step:Float
  ):PropertyDescriptor {
    var options = new PropertyDescriptorOptions();
    options.category = "Placements and cuts";
    options.recordHistory = false;
    options.minimum = minimum;
    options.maximum = 1000000;
    options.step = step;
    options.unit = unit;
    return new PropertyDescriptor("sheet:" + planId + ":" + id,
      label, PropertyType.Float, function(_) return PropertyValue.Float(read(inventory.plan(planId))), function(
        _,
        value
      ) {
      var number:Float = switch (value) {
        case PropertyValue.Float(next):
          next;
        case PropertyValue.Int(next):
          next;
        default:
          throw label + " requires a number";
      };
      inventory.editPlan(planId, function(plan) write(plan, number));
      app.commands.refresh();
    }, options);
  }

  static function placementValue(plan:SheetCutPlan, id:String, field:String):Float {
    for (item in plan.placements) if (item.id == id) return switch field {
      case "x":
        item.x;
      case "y":
        item.y;
      case "width":
        item.width;
      default:
        item.height;
    };
    throw 'Missing placement "$id"';
  }
  static function setPlacement(plan:SheetCutPlan,
    id:String, field:String, value:Float):Void for (item in plan.placements) if (item.id == id) switch field {
    case "x":
      item.x = value;
    case "y":
      item.y = value;
    case "width":
      item.width = value;
    default:
      item.height = value;
  }
  static function cutPosition(plan:SheetCutPlan, id:String):Float {
    for (item in plan.cuts) if (item.id == id) return item.position;
    throw 'Missing cut "$id"';
  }
  static function setCutPosition(plan:SheetCutPlan,
    id:String, value:Float):Void for (item in plan.cuts) if (item.id == id) item.position = value;

  static function requirementNumber(
    app:ReferenceEditorApp,
    inventory:SheetInventory,
    id:String,
    field:String,
    label:String,
    unit:String,
    minimum:Float,
    step:Float
  ):PropertyDescriptor {
    var options = new PropertyDescriptorOptions();
    options.category = "Part requirements";
    options.recordHistory = false;
    options.minimum = minimum;
    options.maximum = 1000000;
    options.step = step;
    if (unit != "") options.unit = unit;
    return new PropertyDescriptor("sheet:requirement:" + id + ":" + field,
      label, PropertyType.Float, function(_) return PropertyValue.Float(requirementValue(
        inventory.requirement(id),
        field
      )), function(
        _,
        value
      ) {
      var number:Float = switch (value) {
        case PropertyValue.Float(next):
          next;
        case PropertyValue.Int(next):
          next;
        default:
          throw label + " requires a number";
      };
      if (field == "quantity" && number != Math.ffloor(number)) throw "Quantity must be a whole number";
      inventory.editRequirement(id, function(item) setRequirementValue(item, field, number));
      app.commands.refresh();
    }, options);
  }

  static function requirementValue(item:SheetPartRequirement, field:String):Float return switch field {
    case "quantity":
      item.quantity;
    case "blankWidth":
      item.blankWidth;
    case "blankHeight":
      item.blankHeight;
    default:
      item.thickness;
  };
  static function setRequirementValue(item:SheetPartRequirement, field:String, value:Float):Void switch field {
    case "quantity":
      item.quantity = Std.int(value);
    case "blankWidth":
      item.blankWidth = value;
    case "blankHeight":
      item.blankHeight = value;
    default:
      item.thickness = value;
  }

  static function moveCut(inventory:SheetInventory,
    planId:String, index:Int, delta:Int):Void inventory.editPlan(planId, function(plan) {
    var target = index + delta;
    if (target < 0 || target >= plan.cuts.length) return;
    var cut = plan.cuts.splice(index, 1)[0];
    plan.cuts.insert(target, cut);
  }
  );
  static function containsPiece(inventory:SheetInventory, id:String):Bool {
    for (piece in inventory.record.stockPieces) if (piece.id == id) return true;
    return false;
  }
  static function fmt(value:Float):String return Std.string(Math.round(value * 1000) / 1000);
}
