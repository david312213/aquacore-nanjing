#include <octomap/octomap.h>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

void snapshot(const octomap::OcTree& tree, const std::string& prefix, size_t scans) {
  std::ofstream out(prefix + std::to_string(scans) + ".csv");
  out << "x,y,z,size\n";
  for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it)
    if (tree.isNodeOccupied(*it) && it.getZ() >= -2 && it.getZ() <= 3)
      out << it.getX() << ',' << it.getY() << ',' << it.getZ() << ',' << it.getSize() << '\n';
  if (!out) throw std::runtime_error("snapshot write failed");
}

// Standalone, offline only: no ROS node, master, network, or vehicle interfaces.
// Stream: uint32 point count, float32 origin[3], float32 world points[N][3].
int main(int argc, char **argv) {
  try {
    if (argc != 5 && argc != 7) throw std::runtime_error("usage: builder stream output_prefix resolution max_range [snapshot_prefix every_n_scans]");
    const size_t interval = argc == 7 ? std::stoul(argv[6]) : 0;
    if (argc == 7 && interval == 0) throw std::runtime_error("snapshot interval must be positive");
    const double res = std::stod(argv[3]), range = std::stod(argv[4]);
    if (res < .05 || res > 1. || range <= 0 || range > 50) throw std::runtime_error("invalid limits");
    std::ifstream in(argv[1], std::ios::binary);
    if (!in) throw std::runtime_error("cannot open stream");
    octomap::OcTree tree(res);
    const auto start = std::chrono::steady_clock::now();
    size_t scans = 0, points = 0;
    while (true) {
      uint32_t n;
      in.read(reinterpret_cast<char*>(&n), sizeof(n));
      if (in.eof() && in.gcount() == 0) break;
      if (!in || n > 1000000) throw std::runtime_error("bad count");
      float origin[3];
      in.read(reinterpret_cast<char*>(origin), sizeof(origin));
      octomap::Pointcloud cloud;
      for (uint32_t i = 0; i < n; ++i) {
        float p[3];
        in.read(reinterpret_cast<char*>(p), sizeof(p));
        if (!in || !std::isfinite(p[0]) || !std::isfinite(p[1]) || !std::isfinite(p[2]))
          throw std::runtime_error("bad point");
        cloud.push_back(p[0], p[1], p[2]);
      }
      tree.insertPointCloud(cloud, octomap::point3d(origin[0], origin[1], origin[2]), range, false, false);
      ++scans; points += n;
      if (interval && scans % interval == 0) snapshot(tree, argv[5], scans);
      if (scans % 200 == 0) std::cout << "scans=" << scans << " nodes=" << tree.size() << std::endl;
    }
    if (!scans) throw std::runtime_error("empty input");
    if (interval && scans % interval) snapshot(tree, argv[5], scans);
    tree.updateInnerOccupancy(); tree.prune();
    const std::string prefix(argv[2]);
    if (!tree.write(prefix + ".ot")) throw std::runtime_error("full map write failed");
    std::ofstream cells(prefix + "_occupied.csv");
    cells << "x,y,z,size,probability\n";
    size_t occupied = 0, free = 0;
    for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it) {
      if (tree.isNodeOccupied(*it)) {
        ++occupied;
        cells << it.getX() << ',' << it.getY() << ',' << it.getZ() << ',' << it.getSize() << ',' << it->getOccupancy() << '\n';
      } else ++free;
    }
    cells.close();
    const auto bytes = tree.memoryUsage();
    const auto nodes = tree.size();
    if (!tree.writeBinary(prefix + ".bt")) throw std::runtime_error("binary map write failed");
    octomap::OcTree check(res);
    if (!check.readBinary(prefix + ".bt") || check.size() == 0) throw std::runtime_error("binary reload failed");
    auto *full = octomap::AbstractOcTree::read(prefix + ".ot");
    if (!full || full->size() != nodes) throw std::runtime_error("full map reload failed");
    delete full;
    const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
    std::ofstream report(prefix + "_build.json");
    report << "{\"scans\":" << scans << ",\"points\":" << points
           << ",\"occupied_leaves\":" << occupied << ",\"free_leaves\":" << free
           << ",\"tree_memory_bytes\":" << bytes << ",\"nodes\":" << nodes
           << ",\"seconds\":" << seconds << ",\"reload_ok\":true}";
    std::cout << "DONE scans=" << scans << " occupied=" << occupied << " free=" << free << " seconds=" << seconds << std::endl;
  } catch (const std::exception& e) { std::cerr << e.what() << std::endl; return 1; }
}
