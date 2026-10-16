// Prototype straight tube and independent adapters; supplier end material is retained.
#include <STEPControl_Reader.hxx>
#include <STEPControl_Writer.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepTools.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Trsf.hxx>
#include <Standard_Failure.hxx>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

struct Part {std::string id,role; TopoDS_Shape shape;};
static int solids(const TopoDS_Shape& s) {int n=0;for(TopExp_Explorer e(s,TopAbs_SOLID);e.More();e.Next())++n;return n;}
static double volume(const TopoDS_Shape& s) {GProp_GProps p;BRepGProp::VolumeProperties(s,p,1e-8);return p.Mass();}
static void valid(const TopoDS_Shape& s) {
 if(solids(s)!=1||!BRepCheck_Analyzer(s).IsValid()||volume(s)<=0)throw std::runtime_error("Derived part is not one valid positive-volume solid");
}
static TopoDS_Shape cut(const TopoDS_Shape&a,const TopoDS_Shape&b) {
 BRepAlgoAPI_Cut op(a,b);if(!op.IsDone())throw std::runtime_error("Cut failed");return op.Shape();
}
static TopoDS_Shape fuse(const TopoDS_Shape&a,const TopoDS_Shape&b) {
 BRepAlgoAPI_Fuse op(a,b);if(!op.IsDone())throw std::runtime_error("Fuse failed");op.SimplifyResult();return op.Shape();
}
static TopoDS_Shape band(const TopoDS_Shape&s,double low,double high) {
 BRepAlgoAPI_Common op(s,BRepPrimAPI_MakeBox(gp_Pnt(-200,low,-200),400,high-low,400).Shape());
 if(!op.IsDone())throw std::runtime_error("Band failed");return op.Shape();
}
static TopoDS_Shape shift(const TopoDS_Shape&s,double y) {
 gp_Trsf t;t.SetTranslation(gp_Vec(0,y,0));return BRepBuilderAPI_Transform(s,t,true).Shape();
}
static TopoDS_Shape cylinder(double radius,double low,double high) {
 return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0,low,0),gp_Dir(0,1,0)),radius,high-low).Shape();
}
static TopoDS_Shape annulus(double outer,double inner,double low,double high) {
 return cut(cylinder(outer,low,high),cylinder(inner,low-.1,high+.1));
}
static TopoDS_Wire circle(double radius,double y) {
 return BRepBuilderAPI_MakeWire(BRepBuilderAPI_MakeEdge(gp_Circ(gp_Ax2(gp_Pnt(0,y,0),gp_Dir(0,1,0)),radius))).Wire();
}
static TopoDS_Shape loft(const TopoDS_Wire&a,const TopoDS_Wire&b) {
 BRepOffsetAPI_ThruSections op(true,false,1e-7);op.AddWire(a);op.AddWire(b);op.Build();
 if(!op.IsDone())throw std::runtime_error("Adapter loft failed");return op.Shape();
}
struct Section {TopoDS_Wire outer,inner;};
static Section section(const TopoDS_Shape&s,double y) {
 for(TopExp_Explorer e(s,TopAbs_FACE);e.More();e.Next()) {
  auto face=TopoDS::Face(e.Current());BRepAdaptor_Surface surface(face);
  if(surface.GetType()!=GeomAbs_Plane||std::abs(surface.Plane().Location().Y()-y)>1e-5||std::abs(surface.Plane().Axis().Direction().Y())<.99999)continue;
  auto outer=BRepTools::OuterWire(face);TopoDS_Wire inner;int count=0;
  for(TopExp_Explorer w(face,TopAbs_WIRE);w.More();w.Next())if(!w.Current().IsSame(outer)){inner=TopoDS::Wire(w.Current());++count;}
  if(count!=1)throw std::runtime_error("Protected cut section must have one hollow duct");
  return {outer,inner};
 }
 throw std::runtime_error("Missing adapter cut section");
}
static TopoDS_Shape radial(double angle,double y,double start,double depth,double radius) {
 double x=std::cos(angle),z=std::sin(angle);
 return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(x*start,y,z*start),gp_Dir(-x,0,-z)),radius,depth).Shape();
}
static TopoDS_Shape screw(double angle,double y,double seat) {
 double x=std::cos(angle),z=std::sin(angle);
 auto shank=radial(angle,y,seat+.1,12.1,2.5);
 auto head=radial(angle,y,seat+5,5.1,4.25);
 auto body=fuse(shank,head);
 // Four-millimetre hex socket, with a two-millimetre floor.
 BRepBuilderAPI_MakePolygon polygon;
 for(int i=0;i<6;++i) {
  double a=i*std::acos(-1.0)/3,r=4/std::sqrt(3.0),t=r*std::cos(a),v=r*std::sin(a);
  polygon.Add(gp_Pnt(x*(seat+5.1)-z*t,y+v,z*(seat+5.1)+x*t));
 }
 polygon.Close();auto face=BRepBuilderAPI_MakeFace(polygon.Wire()).Face();
 return cut(body,BRepPrimAPI_MakePrism(face,gp_Vec(-x*3.1,0,-z*3.1)).Shape());
}
static void sameMaterial(const TopoDS_Shape&a,const TopoDS_Shape&b) {
 if(solids(cut(a,b))!=0||solids(cut(b,a))!=0)throw std::runtime_error("Supplier end-interface material changed");
}

int main(int argc,char**argv) {
 try {
  if(argc!=16)throw std::runtime_error("usage: straight-link source.STEP directory sourceLength removed protected transition engagement OD wall clearance bore coverOD coverLength coverInset coverFillet");
  std::string directory=argv[2];double length=std::stod(argv[3]),removed=std::stod(argv[4]),keep=std::stod(argv[5]);
  double transition=std::stod(argv[6]),engage=std::stod(argv[7]),radius=std::stod(argv[8])/2,wall=std::stod(argv[9]);
  double clearance=std::stod(argv[10]),bore=std::stod(argv[11])/2,inner=radius-wall,pilot=inner-clearance/2;
  double coverRadius=std::stod(argv[12])/2,coverLength=std::stod(argv[13]),coverInset=std::stod(argv[14]),coverFillet=std::stod(argv[15]);
  if(length<=removed+2*(keep+transition+engage)||keep<25||transition<=0||engage<22||wall<3||
     clearance<=0||clearance>.1||bore<15||pilot-bore<9||radius>35)throw std::runtime_error("Invalid straight-link dimensions or insufficient cover clearance");
  // Keep both lips outside the axial screw-head envelope, 3.75..22.25 mm.
  if(coverRadius<41.5||coverRadius>42||coverInset+1.5>3.5||coverInset<.5||
     coverInset+coverLength-1.5<22.5||coverInset+coverLength>30.5||
     coverFillet<=0||2*coverFillet>coverRadius-40)throw std::runtime_error("Cover wall, rounding or fastener clearance is insufficient");
  STEPControl_Reader reader;if(reader.ReadFile(argv[1])!=IFSelect_RetDone||!reader.TransferRoots())throw std::runtime_error("Source import failed");
  auto source=reader.OneShape();valid(source);
  double near=-keep,originalFar=-length+keep,far=originalFar+removed;
  double nearShoulder=near-transition,farShoulder=far+transition;
  auto proximal=band(source,near,1),originalDistal=band(source,-length-1,originalFar),distal=shift(originalDistal,removed);
  auto a=section(proximal,near),b=section(distal,far);
  auto proximalTransition=cut(loft(a.outer,circle(radius,nearShoulder)),loft(a.inner,circle(bore,nearShoulder)));
  auto distalTransition=cut(loft(b.outer,circle(radius,farShoulder)),loft(b.inner,circle(bore,farShoulder)));
  proximal=fuse(fuse(proximal,proximalTransition),annulus(pilot,bore,nearShoulder-engage,nearShoulder+.1));
  distal=fuse(fuse(distal,distalTransition),annulus(pilot,bore,farShoulder-.1,farShoulder+engage));
  auto tube=annulus(radius,inner,farShoulder,nearShoulder);
  std::vector<Part> parts;
  const double pi=std::acos(-1.0),seat=radius-.5;
  for(int end=0;end<2;++end) {
   double shoulder=end==0?nearShoulder:farShoulder,sign=end==0?-1:1;
   std::string prefix=end==0?"proximal":"distal";auto& adapter=end==0?proximal:distal;
   for(int row=0;row<2;++row)for(int index=0;index<6;++index) {
    double angle=(index*60+row*30)*pi/180,y=shoulder+sign*(8+row*10);
    tube=cut(cut(tube,radial(angle,y,radius+1.5,2,5)),radial(angle,y,radius+1,wall+2,2.75));
    adapter=cut(adapter,radial(angle,y,inner+.1,13.1-wall,2.1));
    parts.push_back({prefix+"-screw"+std::to_string(row*6+index),"screw",screw(angle,y,seat)});
   }
   double angle=15*pi/180,y=shoulder+sign*12.5;
   tube=cut(tube,radial(angle,y,radius+1,wall+2,2.5));
   adapter=cut(adapter,radial(angle,y,inner+.1,13.1-wall,2.5));
   parts.push_back({prefix+"-dowel","pin",radial(angle,y,seat,10,2.5)});
   // Separate sliding service cover: screw heads have 0.5 mm radial clearance.
   double low=end==0?shoulder-coverInset-coverLength:shoulder+coverInset,high=low+coverLength;
   auto cover=annulus(coverRadius,40,low,high);
   cover=fuse(cover,annulus(coverRadius,radius+.2,low,low+1.5));
   cover=fuse(cover,annulus(coverRadius,radius+.2,high-1.5,high));
   BRepFilletAPI_MakeFillet rounded(cover);
   for(TopExp_Explorer edge(cover,TopAbs_EDGE);edge.More();edge.Next()) {
    auto value=TopoDS::Edge(edge.Current());BRepAdaptor_Curve curve(value);
    if(curve.GetType()==GeomAbs_Circle&&std::abs(curve.Circle().Radius()-coverRadius)<1e-6)rounded.Add(coverFillet,value);
   }
   rounded.Build();if(!rounded.IsDone())throw std::runtime_error("Cover edge rounding failed");
   cover=rounded.Shape();
   parts.push_back({prefix+"-cover","cover",cover});
  }
  valid(tube);valid(proximal);valid(distal);
  sameMaterial(band(proximal,near,1),band(source,near,1));
  sameMaterial(band(distal,-length+removed-1,far),shift(originalDistal,removed));
  auto joined=fuse(fuse(proximal,tube),distal);valid(joined);
  // Only face contact is allowed between the three structural pieces.
  double overlap=0;
  for(auto pair:std::vector<std::pair<TopoDS_Shape,TopoDS_Shape>>{{proximal,tube},{distal,tube},{proximal,distal}}) {
   BRepAlgoAPI_Common common(pair.first,pair.second);if(!common.IsDone())throw std::runtime_error("Structural overlap check failed");
   overlap+=std::abs(volume(common.Shape()));
  }
  if(overlap>1e-5)throw std::runtime_error("Structural link pieces overlap");
  parts.insert(parts.begin(),{{"tube","tube",tube},{"proximal","adapter",proximal},{"distal","adapter",distal}});
  std::ofstream audit(directory+"/native-audit.json");audit.precision(17);
  audit<<"{\"method\":\"protected-ends-straight-tube-radial-fasteners-v3\",\"sourceVolumeMm3\":"<<volume(source)
   <<",\"structuralVolumeMm3\":"<<volume(tube)+volume(proximal)+volume(distal)<<",\"nearCutY\":"<<near<<",\"originalFarCutY\":"<<originalFar
   <<",\"farCutY\":"<<far<<",\"nearShoulderY\":"<<nearShoulder<<",\"farShoulderY\":"<<farShoulder
   <<",\"tubeLengthMm\":"<<nearShoulder-farShoulder<<",\"minimumCableBoreMm\":"<<2*bore
   <<",\"tubeOuterDiameterMm\":"<<2*radius<<",\"tubeWallMm\":"<<wall<<",\"spigotDiameterMm\":"<<2*pilot
   <<",\"spigotEngagementMm\":"<<engage<<",\"screwCountPerEnd\":12,\"dowelCountPerEnd\":1,\"coverOuterDiameterMm\":"<<2*coverRadius<<",\"coverInnerDiameterMm\":80,\"coverLengthMm\":"<<coverLength
   <<",\"coverShoulderInsetMm\":"<<coverInset<<",\"coverFilletMm\":"<<coverFillet
   <<",\"protectedEndsMaterialDifferenceSolids\":0,\"joinedStructuralSolidValid\":true,\"structuralOverlapMm3\":"<<overlap<<",\"parts\":[";
  bool first=true;
  for(auto& part:parts) {
   valid(part.shape);std::string file=part.id+".STEP";
   STEPControl_Writer writer;if(writer.Transfer(part.shape,STEPControl_AsIs)!=IFSelect_RetDone||writer.Write((directory+"/"+file).c_str())!=IFSelect_RetDone)throw std::runtime_error("Part export failed");
   if(!first)audit<<",";first=false;
   audit<<"{\"id\":\""<<part.id<<"\",\"role\":\""<<part.role<<"\",\"file\":\""<<file<<"\",\"validConnectedSolid\":true,\"volumeMm3\":"<<volume(part.shape)<<"}";
  }
  audit<<"]}\n";
  return 0;
 }catch(const Standard_Failure&e){std::cerr<<e.what()<<"\n";return 1;}
 catch(const std::exception&e){std::cerr<<e.what()<<"\n";return 1;}
}
