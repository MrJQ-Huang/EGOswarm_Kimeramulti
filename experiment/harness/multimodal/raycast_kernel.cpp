// Analytic nearest-surface ray caster. No ROS, Python headers or renderer SDK.
// g++ -shared -O3 -fPIC -std=c++11 raycast_kernel.cpp -o raycast_kernel.so
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

extern "C" void raycast_scene(const float* origin, const float* rays, int ray_count,
                             const float* primitives, int primitive_count,
                             float minimum, float maximum, float* distance, int32_t* identifiers) {
  const double infinity = std::numeric_limits<double>::infinity();
  for (int r = 0; r < ray_count; ++r) {
    const float* d = rays + 3*r;
    double closest = infinity; int identifier = -1;
    const auto accept = [&](double t, int index) {
      // Range clipping happens after occlusion, so a too-near obstacle cannot
      // become transparent and reveal another object's farther surface.
      if (t > 1e-8 && t < closest) { closest = t; identifier = index; }
    };
    for (int k = 0; k < primitive_count; ++k) {
      const float* p = primitives+8*k;
      const int kind = static_cast<int>(p[0]);
      if (kind == 0) {  // bounded floor: z, xlow, xhigh, ylow, yhigh
        if (std::abs(d[2]) < 1e-12) continue;
        const double t = (p[1]-origin[2])/d[2];
        const double x = origin[0]+t*d[0], y = origin[1]+t*d[1];
        if (x >= p[2] && x <= p[3] && y >= p[4] && y <= p[5]) accept(t, k);
      } else if (kind == 1) {  // axis-aligned box: center xyz, half size xyz
        double entry = -infinity, leave = infinity; bool miss = false;
        for (int axis = 0; axis < 3; ++axis) {
          const double low = p[1+axis]-p[4+axis], high = p[1+axis]+p[4+axis];
          if (std::abs(d[axis]) < 1e-12) {
            if (origin[axis] < low || origin[axis] > high) { miss = true; break; }
          } else {
            double a = (low-origin[axis])/d[axis], b = (high-origin[axis])/d[axis];
            if (a > b) std::swap(a, b);
            entry = std::max(entry, a); leave = std::min(leave, b);
            if (entry > leave) { miss = true; break; }
          }
        }
        if (!miss) accept(entry > 1e-8 ? entry : leave, k);
      } else if (kind == 2) {  // cylinder: center xy, bottom z, radius, height
        const double x = origin[0]-p[1], y = origin[1]-p[2];
        const double radius = p[4], bottom = p[3], top = p[3]+p[5];
        const double a = double(d[0])*d[0]+double(d[1])*d[1];
        const double b = 2*(x*d[0]+y*d[1]);
        const double c = x*x+y*y-radius*radius, disc = b*b-4*a*c;
        if (a > 1e-12 && disc >= 0) {
          const double root = std::sqrt(disc);
          for (int sign = -1; sign <= 1; sign += 2) {
            const double t = (-b+sign*root)/(2*a), z = origin[2]+t*d[2];
            if (z >= bottom && z <= top) accept(t, k);
          }
        }
        if (std::abs(d[2]) > 1e-12) {
          for (int cap = 0; cap < 2; ++cap) {
            const double t = ((cap ? top : bottom)-origin[2])/d[2];
            const double px = x+t*d[0], py = y+t*d[1];
            if (px*px+py*py <= radius*radius) accept(t, k);
          }
        }
      }
    }
    const bool valid = closest >= minimum && closest <= maximum && std::isfinite(closest);
    distance[r] = valid ? static_cast<float>(closest) : std::numeric_limits<float>::infinity();
    identifiers[r] = valid ? identifier : -1;
  }
}
