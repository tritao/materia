#!/usr/bin/env python3
"""Draft BOM and dimensioned interface layout for review, not machining release."""
import csv
import hashlib
import json
from pathlib import Path
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Polygon

ROOT = Path(__file__).resolve().parent


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    directory = ROOT / "build/straight"
    current = json.loads((directory / "current.json").read_text())
    layout_path = Path(current["layoutPath"])
    layout = json.loads(layout_path.read_text())
    prepared = json.loads(Path(current["preparationPath"]).read_text())
    audit = json.loads((directory / "exported-parts-audit.json").read_text())
    structure = json.loads((directory / "structure-report.json").read_text())
    if audit["preparationSha256"] != digest(Path(current["preparationPath"])) or structure["layoutSha256"] != digest(layout_path):
        raise ValueError("Prototype pack requires current exported-part and structural reviews")
    config = layout["straightForearm"]["config"]
    cover_config = config["cover"]
    cover_description = f"OD{cover_config['outerDiameterMm']} / main ID80 / lip ID70.4 / L{cover_config['lengthMm']}; shoulder inset {cover_config['shoulderInsetMm']}; edge R{cover_config['outerEdgeFilletMm']}"
    native = prepared["audit"]
    rows = []
    for asset in layout["assets"]:
        if asset["id"].startswith("straight"):
            continue
        occurrences = [o for o in layout["occurrences"] if o["asset"] == asset["id"]]
        family = "eRob110T100 candidate" if asset["id"] == "asset2" else "eRob80T100 candidate" if asset["id"] == "asset15" else asset["file"]
        notes = "Complete purchased-module envelope; ordering code/revision unconfirmed" if "candidate" in family else "Retained supplier-reference geometry; make/buy and material unconfirmed"
        rows.append([asset["id"],family,len(occurrences),"Supplier specification pending",notes,asset["sha256"]])
    rows += [
        ["forearm-tube","70 x 4 mm tube, 206 mm cut length",1,"AL6061-T6 assumption","24 radial M5 clearance holes with spotfaces, two match-reamed dowel holes; see STEP", ""],
        ["forearm-proximal","Proximal mounting adapter",1,"AL6061-T6 assumption","Retained OEM end plus transition and 25 mm spigot; thread/fit review pending", ""],
        ["forearm-distal","Distal mounting adapter",1,"AL6061-T6 assumption","Retained OEM end plus transition and 25 mm spigot; thread/fit review pending", ""],
        ["adapter-screws","M5 x 12 socket head, ISO 4762 style",2*config["fastening"]["screwsPerEnd"],"Class 8.8 assumption","Exact supplier, preload, thread stripping and fatigue not released", ""],
        ["adapter-dowels","Nominal 5 x 10 mm dowel",2,"Steel assumption","One per end; assembly match-reaming/fit specification pending", ""],
        ["service-covers",cover_description,2,"ABS assumption","Rounded edges; retention details pending; slide away to access fasteners", ""]]
    with (directory / "prototype-bom.csv").open("w",newline="") as output:
        writer = csv.writer(output)
        writer.writerow(["item","description","quantity","material_basis","review_notes","supplier_geometry_sha256"])
        writer.writerows(rows)
    fig = plt.figure(figsize=(12,8),facecolor="white")
    ax = fig.add_axes([.055,.40,.89,.48])
    ax.set(xlim=(-12,308),ylim=(-76,76),aspect="equal")
    ax.axis("off")
    orange, steel, cover = "#e6a24c", "#718897", "#445a6a"
    # OEM end boxes are intentionally symbolic; the pinned STEP defines their profile/holes.
    for x in (0,266):
        ax.add_patch(Rectangle((x,-49),30,98,facecolor="#f7ead5",edgecolor=orange,hatch="///",linewidth=1))
        ax.text(x+15,0,"OEM\nKEEP",ha="center",va="center",fontsize=8)
    for sign in (-1,1):
        ax.add_patch(Polygon([(30,sign*39.6034),(45,sign*35),(45,sign*21),(30,sign*31)],facecolor=orange,edgecolor="black",linewidth=.6))
        ax.add_patch(Polygon([(251,sign*35),(266,sign*32.9462),(266,sign*25.5),(251,sign*21)],facecolor=orange,edgecolor="black",linewidth=.6))
        for x in (45,226):
            bottom = 21 if sign>0 else -30.98
            ax.add_patch(Rectangle((x,bottom),25,9.98,facecolor=orange,edgecolor="black",linewidth=.5))
        ax.add_patch(Rectangle((45,31 if sign>0 else -35),206,4,facecolor=steel,edgecolor="black",linewidth=.6))
        radius = cover_config['outerDiameterMm']/2
        for x in (45+cover_config['shoulderInsetMm'],251-cover_config['shoulderInsetMm']-cover_config['lengthMm']):
            ax.add_patch(Rectangle((x,40 if sign>0 else -radius),cover_config['lengthMm'],radius-40,facecolor=cover))
    ax.axhline(0,color="#a0a7ab",linewidth=.5,linestyle="--")
    def dimension(x0,x1,y,label):
        ax.annotate("",xy=(x0,y),xytext=(x1,y),arrowprops={"arrowstyle":"<->","linewidth":.7})
        for x in (x0,x1):ax.plot([x,x],[y-3,y+3],color="black",linewidth=.6)
        ax.text((x0+x1)/2,y+2,label,ha="center",va="bottom",fontsize=9)
    dimension(0,296,65,"296 overall link envelope")
    dimension(45,251,53,"206 tube cut length")
    dimension(0,30,-58,"30 retained")
    dimension(30,45,-69,"15 transition")
    dimension(251,266,-69,"15 transition")
    dimension(266,296,-58,"30 retained")
    ax.text(148,-13,"42 nominal adapter duct",ha="center",fontsize=9)
    fig.suptitle("Straight forearm · draft interface layout",fontsize=19,y=.96)
    fig.text(.5,.905,"Millimetres · positive horizontal direction is supplier local -Y · 425 / 401 mm joint spacing retained",ha="center",fontsize=10)
    table_data = [
        ["Tube", "OD70 / ID62 / L206; spotface depth 0.5; minimum local wall 3.5"],
        ["Spigots", "OD61.96 / bore42 / engagement25; nominal diametral clearance0.04"],
        ["Each adapter", "12 x M5 x 12, two rows at 8 and 18 from shoulder; six holes/row, 30 degree stagger"],
        ["Locating pin", "One nominal D5 x 10 per end, 12.5 from shoulder, 15 degree clocking"],
        ["Service covers", cover_description+"; retention pending"],
        ["Source mounting ends", "First/last 30 unchanged; protected boxes are symbolic. Use supplier/derived STEP for shape and holes."]]
    table_ax = fig.add_axes([.055,.10,.89,.25]);table_ax.axis("off")
    table = table_ax.table(cellText=table_data,colWidths=[.18,.82],loc="center",cellLoc="left")
    table.auto_set_font_size(False);table.set_fontsize(8);table.scale(1,1.7)
    for cell in table.get_celld().values():cell.set_edgecolor("#c4c9ce");cell.set_linewidth(.5)
    fig.text(.5,.035,"NOT A MACHINING RELEASE · fits, preload, thread stripping, local compliance and fatigue remain unqualified",ha="center",fontsize=10,color="#8a3c25")
    for extension in ("svg","pdf"):
        fig.savefig(directory/("interface-review."+extension))
    pack = {"status":"draft_not_ordered_not_manufacturing_release","layoutSha256":digest(layout_path),
            "preparationSha256":digest(Path(current["preparationPath"])),
            "exportedPartAuditSha256":digest(directory/"exported-parts-audit.json"),
            "structureReportSha256":digest(directory/"structure-report.json"),
            "files":["prototype-bom.csv","interface-review.svg","interface-review.pdf"],
            "pending":["Machinist review of retained OEM geometry and workholding","Spigot and match-reamed dowel fits",
                       "Bolt preload and thread stripping","Measured joint slip, hysteresis and tube ovalisation",
                       "Cover retention details","Cable bundle size, bend radius and strain relief",
                       "Full-workspace shoulder drive sizing"],
            "sources":{"tubeFamily":"https://www.hydro.com/globalassets/01-products--services/extruded-profiles/americas/ena-resources/alloy-data-sheets/hydro_2019_data_sheet_6061.pdf",
                       "M5StressArea":"https://www.bossard.com/global-en/-/media/bossard-group/website/documents/technical-resources/en/f-004-en.pdf"}}
    (directory/"prototype-pack.json").write_text(json.dumps(pack,indent=2)+"\n")
    print(directory/"prototype-pack.json")


if __name__ == "__main__":
    main()
