// The vendored coal core builds without Boost or assimp and answers distance queries:
// boxes, a triangle mesh against a sphere, and an overlap with its signed distance.
#include <coal/distance.h>
#include <coal/collision.h>
#include <coal/shape/geometric_shapes.h>
#include <coal/BVH/BVH_model.h>
#include <cassert>
#include <cmath>
#include <cstdio>
int main() {
  using namespace coal;
  // Two unit boxes 3 m apart along x: 2 m of clearance.
  CollisionGeometryPtr_t box(new Box(1.0, 1.0, 1.0));
  Transform3s a = Transform3s::Identity(), b = Transform3s::Identity();
  b.setTranslation(Vec3s(3.0, 0.0, 0.0));
  DistanceRequest request;
  DistanceResult result;
  CollisionObject first(box, a), second(box, b);
  distance(&first, &second, request, result);
  std::printf("box-box distance %.12f, nearest x %.6f and %.6f\n", result.min_distance,
              result.nearest_points[0].x(), result.nearest_points[1].x());
  assert(std::abs(result.min_distance - 2.0) < 1e-9);
  // A triangle mesh (a single quad) against a sphere: 0.5 m above it, radius 0.2.
  auto mesh = std::make_shared<BVHModel<OBBRSS>>();
  std::vector<Vec3s> points = {Vec3s(-1, -1, 0), Vec3s(1, -1, 0), Vec3s(1, 1, 0), Vec3s(-1, 1, 0)};
  std::vector<Triangle> triangles = {Triangle(0, 1, 2), Triangle(0, 2, 3)};
  mesh->beginModel();
  mesh->addSubModel(points, triangles);
  mesh->endModel();
  Transform3s above = Transform3s::Identity();
  above.setTranslation(Vec3s(0.2, 0.1, 0.5));
  CollisionObject plate(mesh, Transform3s::Identity()), ball(CollisionGeometryPtr_t(new Sphere(0.2)), above);
  DistanceResult meshResult;
  distance(&plate, &ball, request, meshResult);
  std::printf("mesh-sphere distance %.12f\n", meshResult.min_distance);
  assert(std::abs(meshResult.min_distance - 0.3) < 1e-6);
  // Overlapping: collision reports contact and signed distance goes negative.
  b.setTranslation(Vec3s(0.8, 0.0, 0.0));
  CollisionObject overlapping(box, b);
  CollisionRequest collisionRequest;
  CollisionResult collisionResult;
  collide(&first, &overlapping, collisionRequest, collisionResult);
  DistanceResult penetration;
  distance(&first, &overlapping, request, penetration);
  std::printf("overlap: collision %d, signed distance %.6f\n", collisionResult.isCollision(), penetration.min_distance);
  assert(collisionResult.isCollision() && penetration.min_distance < -0.19);
  return 0;
}
