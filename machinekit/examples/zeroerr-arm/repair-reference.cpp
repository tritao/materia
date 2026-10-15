// Local derived CAD only: preserve supplier STEP and reject significant changes.
#include <STEPControl_Reader.hxx>
#include <STEPControl_Writer.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <ShapeFix_Shape.hxx>
#include <TopExp_Explorer.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <gp_Pnt.hxx>
#include <fstream>
#include <iostream>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <utility>

static double volume(const TopoDS_Shape& s) {
  GProp_GProps p; BRepGProp::VolumeProperties(s,p,1e-8); return p.Mass();
}
static void bounds(const TopoDS_Shape& s,double* b) {
  Bnd_Box box; BRepBndLib::AddOptimal(s,box,false,false); box.Get(b[0],b[1],b[2],b[3],b[4],b[5]);
}
int main(int argc,char** argv) {
  try {
    if(argc!=4 && argc!=5) throw std::runtime_error("usage: repair-reference source.STEP output-prefix audit.jsonl [partition110]");
    STEPControl_Reader reader;
    if(reader.ReadFile(argv[1])!=IFSelect_RetDone || !reader.TransferRoots()) throw std::runtime_error("STEP import failed");
    std::ofstream audit(argv[3]); audit.precision(17);
    int i=0;
    for(TopExp_Explorer it(reader.OneShape(),TopAbs_SOLID);it.More();it.Next(),++i) {
      auto source=it.Current();
      bool changed=!BRepCheck_Analyzer(source).IsValid();
      if(!changed && !(argc==5 && i==0)) continue;
      auto repaired=source;
      if(changed) {
        ShapeFix_Shape fix(source); fix.SetPrecision(1e-7); fix.SetMaxTolerance(1e-3); fix.Perform();
        repaired=fix.Shape();
      }
      if(!BRepCheck_Analyzer(repaired).IsValid()) throw std::runtime_error("ShapeFix could not repair supplier solid");
      int count=0; for(TopExp_Explorer s(repaired,TopAbs_SOLID);s.More();s.Next()) ++count;
      if(count!=1) throw std::runtime_error("Repair changed the solid count");
      double a[6],b[6],delta=0; bounds(source,a);bounds(repaired,b);
      for(int k=0;k<6;++k) delta=std::max(delta,std::abs(a[k]-b[k]));
      double before=volume(source),after=volume(repaired);
      double fraction=std::abs(after-before)/std::max(std::abs(before),1.0);
      if(delta>0.02 || fraction>0.001) throw std::runtime_error("Repair exceeds 0.02 mm bounds / 0.1% volume review limits");
      std::string path=std::string(argv[2])+"-solid"+std::to_string(i)+".STEP";
      STEPControl_Writer writer;
      if(writer.Transfer(repaired,STEPControl_AsIs)!=IFSelect_RetDone || writer.Write(path.c_str())!=IFSelect_RetDone)
        throw std::runtime_error("Repaired STEP export failed");
      audit<<"{\"solid\":"<<i<<",\"repaired\":"<<(changed?"true":"false")<<",\"boundsDeltaMm\":"<<delta<<",\"volumeBeforeMm3\":"<<before
        <<",\"volumeAfterMm3\":"<<after<<",\"volumeChangeFraction\":"<<fraction;
      if(argc==5 && i==0) {
        // The source body merges the projecting output spigot into the T case.
        // Preserve all material while separating it at the front casing datum.
        auto below=BRepPrimAPI_MakeBox(gp_Pnt(-200,-200,-200),400,400,200).Shape();
        auto above=BRepPrimAPI_MakeBox(gp_Pnt(-200,-200,0),400,400,400).Shape();
        BRepAlgoAPI_Common cutRotor(repaired,below), cutStator(repaired,above);
        if(!cutRotor.IsDone() || !cutStator.IsDone()) throw std::runtime_error("Output-spigot partition failed");
        auto rotor=cutRotor.Shape(), stator=cutStator.Shape();
        if(!BRepCheck_Analyzer(rotor).IsValid() || !BRepCheck_Analyzer(stator).IsValid())
          throw std::runtime_error("Invalid output-spigot partition");
        double rb[6];bounds(rotor,rb);
        for(int k:{0,1,3,4}) if(std::abs(rb[k])>49.001) throw std::runtime_error("Spigot exceeds drawing diameter 98 mm");
        double error=std::abs(volume(rotor)+volume(stator)-after);
        // Curved STEP faces have integration errors that change when split.
        // Verify the material sets with both Boolean differences instead.
        BRepAlgoAPI_Fuse joined(rotor,stator);
        if(!joined.IsDone()) throw std::runtime_error("Partition reunion failed");
        BRepAlgoAPI_Cut missing(repaired,joined.Shape()), extra(joined.Shape(),repaired);
        if(!joined.IsDone() || !missing.IsDone() || !extra.IsDone()) throw std::runtime_error("Partition set comparison failed");
        for(auto difference:{missing.Shape(),extra.Shape()})
          if(TopExp_Explorer(difference,TopAbs_SOLID).More()) throw std::runtime_error("Partition changes the material set");
        for(auto item:{std::make_pair("-rotor.STEP",rotor),std::make_pair("-stator.STEP",stator)}) {
          STEPControl_Writer out;
          if(out.Transfer(item.second,STEPControl_AsIs)!=IFSelect_RetDone ||
              out.Write((std::string(argv[2])+"-solid0"+item.first).c_str())!=IFSelect_RetDone)
            throw std::runtime_error("Partition STEP export failed");
        }
        audit<<",\"partitionZ\":0,\"partitionDifferenceSolids\":0,\"partitionVolumeIntegrationDifferenceMm3\":"<<error;
      }
      audit<<"}\n";
    }
    return 0;
  } catch(const std::exception& e) { std::cerr<<e.what()<<"\n";return 1; }
}
