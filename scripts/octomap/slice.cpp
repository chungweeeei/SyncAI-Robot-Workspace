// Read a .bt back and write a top-down PGM: black = occupied voxel in [zlo,zhi], white = free there, grey = unknown.
#include <octomap/octomap.h>
#include <fstream>
#include <vector>
#include <iostream>
int main(int c, char** v) {
  if (c < 5) { std::cerr << "slice <map.bt> <out.pgm> <zlo> <zhi>\n"; return 1; }
  octomap::OcTree t(v[1]); double zlo = atof(v[3]), zhi = atof(v[4]), r = t.getResolution();
  double x0, y0, z0, x1, y1, z1; t.getMetricMin(x0, y0, z0); t.getMetricMax(x1, y1, z1);
  int W = (x1 - x0) / r + 1, H = (y1 - y0) / r + 1; std::vector<unsigned char> img(W * H, 205);
  for (auto it = t.begin_leafs(); it != t.end_leafs(); ++it) {
    double z = it.getZ(), s = it.getSize(); if (z + s / 2 < zlo || z - s / 2 > zhi) continue;
    bool occ = t.isNodeOccupied(*it);
    for (double x = it.getX() - s / 2 + r / 2; x < it.getX() + s / 2; x += r)
      for (double y = it.getY() - s / 2 + r / 2; y < it.getY() + s / 2; y += r) {
        int i = (x - x0) / r, j = H - 1 - int((y - y0) / r); if (i < 0 || j < 0 || i >= W || j >= H) continue;
        auto& p = img[j * W + i]; if (occ) p = 0; else if (p == 205) p = 254;
      }
  }
  std::ofstream f(v[2], std::ios::binary); f << "P5\n" << W << " " << H << "\n255\n"; f.write((char*)img.data(), img.size());
  std::cout << "res " << r << " leafs " << t.getNumLeafNodes() << " img " << W << "x" << H << std::endl;
}
