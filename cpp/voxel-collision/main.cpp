#include "collides.h"

#include "CTSparseVoxelObject.h"
#include "CTVoxelObject.h"
#include "CTVoxelOctree.h"
#include "CTVoxelOctreeWrap.h"
#include "DerivedVoxelOctree.h"
#include "OctomapWrap.h"
#include "SparseVoxelObject.h"
#include "VoxelObject.h"
#include "VoxelOctree.h"
#include "VoxelOctreeUnion.h"

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include <unistd.h>

constexpr uint64_t bitmask(uint_fast8_t x, uint_fast8_t y, uint_fast8_t z) {
  return uint64_t(1) << (x*16 + y*4 + z);
}

void memory_usage(double &vm_usage, double &resident_set) {
  vm_usage = 0.0;
  resident_set = 0.0;

  //get info from proc directory
  std::ifstream stat_stream("/proc/self/stat", std::ios_base::in);

  //create some variables to get info
  std::string pid, comm, state, ppid, pgrp, session, tty_nr;
  std::string tpgid, flags, minflt, cminflt, majflt, cmajflt;
  std::string utime, stime, cutime, cstime, priority, nice;
  std::string O, itrealvalue, starttime;
  unsigned long vsize;
  long rss;
  stat_stream
    >> pid >> comm >> state >> ppid >> pgrp >> session >> tty_nr
    >> tpgid >> flags >> minflt >> cminflt >> majflt >> cmajflt
    >> utime >> stime >> cutime >> cstime >> priority >> nice
    >> O >> itrealvalue >> starttime >> vsize >> rss; // don't care about the rest
  stat_stream.close();

  // for x86-64 is configured to use 2MB pages
  long page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024;
  vm_usage = vsize / 1024.0;
  resident_set = rss * page_size_kb;
}

void print_memory_usage(std::ostream &out) {
  double vm, rss;
  memory_usage(vm, rss);
  out << "Virtual Memory:    " << vm << " KiB\n"
      << "Resident Set Size: " << rss << " KiB" << std::endl;
}

// return the total seconds to execute the function N times
template <typename Func>
std::pair<double, double> time_func(int N, Func &&f, int K = 10) {
  std::vector<double> timings;
  double sum_x  = 0.0;
  double sum_x2 = 0.0;
  for (int k = 0; k < K; ++k) {
    auto start = std::chrono::system_clock::now();
    for (int i = 0; i < N; i++) {
      f();
    }
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_secs = end - start;
    sum_x += elapsed_secs.count();
    sum_x2 += elapsed_secs.count() * elapsed_secs.count();
  }
  double mean = sum_x / K;
  double variance = sum_x2 / K - mean * mean;
  double standard_deviation = std::sqrt(variance);
  return {mean, standard_deviation};
}

template <typename Func>
void print_timing(const std::string &name, int N, Func &&f, std::ostream& out = std::cout, int K = 10)
{
  auto [mean, stdev] = time_func(N, f, K);
  out << name << ": run " << N << " times (" << K << " trials): " << mean
        << " sec (+- " << stdev << " sec)\n"
      << "  runs in " << mean / N << " sec (+- " << stdev / N << ")\n"
      << "  runs at " << N / mean << " Hz\n";
}

template <typename VType>
void try_voxel_type(const std::string &name,
                    const std::unique_ptr<VType> &v1,
                    const std::unique_ptr<VType> &v2,
                    int N = 1000,
                    std::ostream &out = std::cout) {
  out << "\n"
      //<< "---------------------------------------------------------\n"
      << "  " << name << "\n"
      //<< "\n"
      //<< "v is size ("
      //<< v1->Nx() << ", " << v1->Ny() << ", " << v1->Nz() << ")" << std::endl
      //<< "sizeof(v): " << sizeof(*v1) << std::endl
      ;

  // copies
  //auto &v1 = v;
  //auto v2 = std::make_unique<VType>(*v);
  //auto v3 = std::make_unique<VType>(*v);
  //auto v4 = std::make_unique<VType>(*v);

  //v1->add_point(0.1, 0.1, 0.1);
  //v1->add_sphere(0.5, 0.5, 0.5, 0.25);
  //v2->add_sphere(0.5, 0.5, 0.1, 0.1);
  //v3->add_sphere(0.7, 0.2, 0.2, 0.55);

  for (size_t bx = 0; bx < v1->Nbx(); ++bx) {
    for (size_t by = 0; by < v1->Nby(); ++by) {
      for (size_t bz = 0; bz < v1->Nbz(); ++bz) {
        // dense around each other, but non-overlapping
        v1->set_block(bx, by, bz, 0x3c'3c'3c'3c'3c'3c'3c'3c);
        v2->set_block(bx, by, bz, 0xc3'c3'c3'c3'c3'c3'c3'c3);
      }
    }
  }

  //out << "v1.nblocks(): " << v1->nblocks() << std::endl
  //    << "v2.nblocks(): " << v2->nblocks() << std::endl
  //    << "v3.nblocks(): " << v3->nblocks() << std::endl
  //    << "v4.nblocks(): " << v4->nblocks() << std::endl
  //    ;
  //print_memory_usage(out);

  //out << "  v1.collides(v1): " << collides(*v1, *v1) << std::endl
  //    << "  v1.collides(v2): " << collides(*v1, *v2) << std::endl
  //    << "  v1.collides(v3): " << collides(*v1, *v3) << std::endl
  //    << "  v1.collides(v4): " << collides(*v1, *v4) << std::endl
  //    << "  v2.collides(v1): " << collides(*v2, *v1) << std::endl
  //    << "  v2.collides(v2): " << collides(*v2, *v2) << std::endl
  //    << "  v2.collides(v3): " << collides(*v2, *v3) << std::endl
  //    << "  v2.collides(v4): " << collides(*v2, *v4) << std::endl
  //    << "  v3.collides(v1): " << collides(*v3, *v1) << std::endl
  //    << "  v3.collides(v2): " << collides(*v3, *v2) << std::endl
  //    << "  v3.collides(v3): " << collides(*v3, *v3) << std::endl
  //    << "  v3.collides(v4): " << collides(*v3, *v4) << std::endl
  //    << "  v4.collides(v1): " << collides(*v4, *v1) << std::endl
  //    << "  v4.collides(v2): " << collides(*v4, *v2) << std::endl
  //    << "  v4.collides(v3): " << collides(*v4, *v3) << std::endl
  //    << "  v4.collides(v4): " << collides(*v4, *v4) << std::endl
  //    << std::endl;

  print_timing("collision checking collides(v1, v2)", N,
               [&v1, &v2]() { collides(*v1, *v2); }, out, 100);

  //v1->remove_interior();
  //v2->remove_interior();
  //v3->remove_interior();
  //v4->remove_interior();
  //remove_interior_slow(*v1);
  //remove_interior_slow(*v2);
  //remove_interior_slow(*v3);
  //remove_interior_slow(*v4);
  //remove_interior_medium(*v1);
  //remove_interior_medium(*v2);
  //remove_interior_medium(*v3);
  //remove_interior_medium(*v4);
  //out << "\n"
  //    << "AFTER REMOVING INTERIOR\n";
  //out << "  v1.nblocks(): " << v1->nblocks() << std::endl
  //    << "  v2.nblocks(): " << v2->nblocks() << std::endl
  //    << "  v3.nblocks(): " << v3->nblocks() << std::endl
  //    << "  v4.nblocks(): " << v4->nblocks() << std::endl
  //    << std::endl;
  //print_timing("  collision checking collides(v1, v2)", N,
  //             [&v1, &v2]() { collides(*v1, *v2); }, out);
}

template <typename VType>
void print_voxel_object(const VType *v) {
  int width = std::to_string(v->Ny).size();
  for (size_t i = 0; i < v->Nx; i++) {
    std::cout << "x = " << i << std::endl;
    std::cout << "  y\\z" << std::string(width, ' ');
    for (size_t j = 0; j < v->Ny; j++) {
      std::cout << (j % 10);
    }
    std::cout << '\n' << std::endl;
    for (size_t j = 0; j < v->Ny; j++) {
      std::cout << "  " << std::setw(width) << j << "   ";
      for (size_t k = 0; k < v->Nz; k++) {
        std::cout << (v->cell(i, j, k) ? '1' : '.');
      }
      std::cout << std::endl;
    }
    std::cout << "\n";
  }
}

template <typename V1Type, typename V2Type>
void print_voxel_object_side_by_side(const V1Type *v1, const V2Type *v2) {
  int width = std::to_string(v1->Ny).size();
  for (size_t i = 0; i < v1->Nx; i++) {
    std::cout << "x = " << i << std::endl;
    std::cout << "  y\\z" << std::string(width, ' ');
    for (size_t j = 0; j < v1->Nz; j++) { std::cout << (j % 10); }
    std::cout << "   ";
    for (size_t j = 0; j < v1->Nz; j++) { std::cout << (j % 10); }
    std::cout << '\n' << std::endl;
    for (size_t j = 0; j < v1->Ny; j++) {
      std::cout << "  " << std::setw(width) << j << "   ";
      for (size_t k = 0; k < v1->Nz; k++) {
        std::cout << (v1->cell(i, j, k) ? '1' : '.');
      }
      std::cout << "   ";
      for (size_t k = 0; k < v2->Nz; k++) {
        std::cout << (v2->cell(i, j, k) ? '1' : '.');
      }
      std::cout << std::endl;
    }
    std::cout << "\n";
  }
}

template <typename VoxelType>
void remove_interior_slow(VoxelType &v) {
  const VoxelType copy(v);
  for (size_t ix = 0; ix < v.Nx; ix++) {
    for (size_t iy = 0; iy < v.Ny; iy++) {
      for (size_t iz = 0; iz < v.Nz; iz++) {
        bool curr   = copy.cell(ix, iy, iz);
        bool left   = (ix == 0)      || copy.cell(ix-1, iy, iz);
        bool right  = (ix == v.Nx-1) || copy.cell(ix+1, iy, iz);
        bool front  = (iy == 0)      || copy.cell(ix, iy-1, iz);
        bool behind = (iy == v.Ny-1) || copy.cell(ix, iy+1, iz);
        bool below  = (iz == 0)      || copy.cell(ix, iy, iz-1);
        bool above  = (iz == v.Nz-1) || copy.cell(ix, iy, iz+1);
        if (curr && left && right && front && behind && below && above) {
          v.set_cell(ix, iy, iz, false);
        }
      }
    }
  }
}

template <typename VoxelType>
void remove_interior_medium(VoxelType &v) {
  const VoxelType copy(v);
  const uint64_t full = ~uint64_t(0);
  for (size_t bx = 0; bx < v.Nbx; bx++) {
    for (size_t by = 0; by < v.Nby; by++) {
      for (size_t bz = 0; bz < v.Nbz; bz++) {
        const uint64_t old_b = copy.block(bx, by, bz);
        if (!old_b) { continue; } // do not try if it's already all zeros.

        const uint64_t left   = (bx <= 0)       ? full : copy.block(bx-1, by, bz);
        const uint64_t right  = (bx >= v.Nbx-1) ? full : copy.block(bx+1, by, bz);
        const uint64_t front  = (by <= 0)       ? full : copy.block(bx, by-1, bz);
        const uint64_t behind = (by >= v.Nby-1) ? full : copy.block(bx, by+1, bz);
        const uint64_t below  = (bz <= 0)       ? full : copy.block(bx, by, bz-1);
        const uint64_t above  = (bz >= v.Nbz-1) ? full : copy.block(bx, by, bz+1);
        uint64_t new_b = old_b;

        auto is_interior =
          [left, right, front, behind, below, above, old_b]
          (unsigned ix, unsigned iy, unsigned iz) {
            bool is_in = true;
            uint64_t mask = bitmask(ix, iy, iz);

            if (ix > 0) { mask |= bitmask(ix-1, iy, iz); }
            else { is_in = (is_in && bool(left & bitmask(3, iy, iz))); }

            if (ix < 3) { mask |= bitmask(ix+1, iy, iz); }
            else { is_in = (is_in && bool(right & bitmask(0, iy, iz))); }

            if (iy > 0) { mask |= bitmask(ix, iy-1, iz); }
            else { is_in = (is_in && bool(front & bitmask(ix, 3, iz))); }

            if (iy < 3) { mask |= bitmask(ix, iy+1, iz); }
            else { is_in = (is_in && bool(behind & bitmask(ix, 0, iz))); }

            if (iz > 0) { mask |= bitmask(ix, iy, iz-1); }
            else { is_in = (is_in && bool(below & bitmask(ix, iy, 3))); }

            if (iz < 3) { mask |= bitmask(ix, iy, iz+1); }
            else { is_in = (is_in && bool(above & bitmask(ix, iy, 0))); }

            is_in = (is_in && (mask == (old_b & mask)));
            return is_in;
          };

        for (unsigned ix = 0; ix < 4; ix++) {
          for (unsigned iy = 0; iy < 4; iy++) {
            for (unsigned iz = 0; iz < 4; iz++) {
              if (is_interior(ix, iy, iz)) {
                new_b &= ~bitmask(ix, iy, iz);
              }
            }
          }
        }

        // store this new block if it has any cells set
        v.set_block(bx, by, bz, new_b);
      }
    }
  }
}

int main() {
  std::cout << "Before\n";
  print_memory_usage(std::cout);

  //try_voxel_type("VoxelObject_128", std::make_unique<VoxelObject>(128, 128, 128), 10000);
  //try_voxel_type("VoxelObject_256", std::make_unique<VoxelObject>(256, 256, 256), 1000);
  //try_voxel_type("VoxelObject_512", std::make_unique<VoxelObject>(512, 512, 512), 500);
  //try_voxel_type("CTVoxelObject_128", std::make_unique<CTVoxelObject<128, 128, 128>>(), 10000);
  //try_voxel_type("CTVoxelObject_256", std::make_unique<CTVoxelObject<256, 256, 256>>(), 1000);
  //try_voxel_type("CTVoxelObject_512", std::make_unique<CTVoxelObject<512, 512, 512>>(), 300);
  //try_voxel_type("SparseVoxelObject_128", std::make_unique<SparseVoxelObject>(128, 128, 128), 3000);
  //try_voxel_type("SparseVoxelObject_256", std::make_unique<SparseVoxelObject>(256, 256, 256), 500);
  //try_voxel_type("SparseVoxelObject_512", std::make_unique<SparseVoxelObject>(512, 512, 512), 80);
  //try_voxel_type("CTSparseVoxelObject_128", std::make_unique<CTSparseVoxelObject<128, 128, 128>>(), 100);
  //try_voxel_type("CTSparseVoxelObject_256", std::make_unique<CTSparseVoxelObject<256, 256, 256>>(), 100);
  //try_voxel_type("CTSparseVoxelObject_512", std::make_unique<CTSparseVoxelObject<512, 512, 512>>(), 100);

#define VOXEL_COMPARE(N, K) \
  { \
    std::ostringstream tmpout; \
    try_voxel_type("CTVoxelOctreeWrap_" #N, \
                   std::make_unique<CTVoxelOctreeWrap>(N), \
                   std::make_unique<CTVoxelOctreeWrap>(N), \
                   K/10, tmpout); \
  } \
  std::cout << "\n-------- size " << N << " ---------------\n"; \
  try_voxel_type("CTVoxelOctreeWrap_" #N, \
                 std::make_unique<CTVoxelOctreeWrap>(N), \
                 std::make_unique<CTVoxelOctreeWrap>(N), \
                 K, std::cout); \
  try_voxel_type("VoxelOctree_" #N, \
                 std::make_unique<VoxelOctree>(N), \
                 std::make_unique<VoxelOctree>(N), \
                 K, std::cout); \
  try_voxel_type("CTVoxelOctree_" #N, \
                 std::make_unique<CTVoxelOctree<N>>(), \
                 std::make_unique<CTVoxelOctree<N>>(), \
                 K, std::cout); \
  try_voxel_type("VoxelOctreeUnion_" #N, \
                 std::make_unique<VoxelOctreeUnion>(N), \
                 std::make_unique<VoxelOctreeUnion>(N), \
                 K, std::cout); \
  try_voxel_type("DerivedVoxelOctree_" #N, \
                 std::unique_ptr<AbstractVoxelOctree>(new DerivedVoxelOctree<N>()), \
                 std::unique_ptr<AbstractVoxelOctree>(new DerivedVoxelOctree<N>()), \
                 K, std::cout); \
  { \
    std::ostringstream tmpout; \
    try_voxel_type("CTVoxelOctreeWrap_" #N, \
                   std::make_unique<CTVoxelOctreeWrap>(N), \
                   std::make_unique<CTVoxelOctreeWrap>(N), \
                   K/10, tmpout); \
  } \

  // I heuristically set K to make runtime be about one second
  //VOXEL_COMPARE(  4, 120000000);
  //VOXEL_COMPARE(  8,  40000000);
  //VOXEL_COMPARE( 16,  10000000);
  //VOXEL_COMPARE( 32,   1000000);
  //VOXEL_COMPARE( 64,     70000);
  //VOXEL_COMPARE(128,      3000);
  VOXEL_COMPARE(256,       100);
  VOXEL_COMPARE(512,        10);

#undef VOXEL_COMPARE

  //try_voxel_type("OctomapWrap", std::make_unique<OctomapWrap>(1.0/4.0), 10);
  //try_voxel_type("OctomapWrap", std::make_unique<OctomapWrap>(1.0/8.0), 10);
  //try_voxel_type("OctomapWrap", std::make_unique<OctomapWrap>(1.0/16.0), 10);
  //try_voxel_type("OctomapWrap", std::make_unique<OctomapWrap>(1.0/32.0), 10);
  //try_voxel_type("OctomapWrap", std::make_unique<OctomapWrap>(1.0/64.0), 10);
  //try_voxel_type("OctomapWrap", std::make_unique<OctomapWrap>(1.0/128.0), 10);
  //try_voxel_type("OctomapWrap", std::make_unique<OctomapWrap>(1.0/256.0), 10);
  //try_voxel_type("OctomapWrap", std::make_unique<OctomapWrap>(1.0/512.0), 10);

  ////SparseVoxelObject v(20, 20, 32);
  //CTVoxelOctree<32> v1;
  //v1.add_sphere(0.7, 0.2, 0.2, 0.55);

  //std::cout << "\n"
  //             "*****************\n"
  //             "\n"
  //             "After removing interior points\n"
  //          << std::endl;
  //SparseVoxelObject v2(32, 32, 32);
  //v2.add_sphere(0.7, 0.2, 0.2, 0.55);
  //v2.add_sphere(0.1, 0.6, 0.6, 0.3);
  ////auto v2 = v1;
  //auto v3 = v1;
  //v3.add_sphere(0.1, 0.6, 0.6, 0.3);
  //remove_interior_medium(v2); //v2.remove_interior_slow_1();
  //v3.remove_interior(); //remove_interior_slow(v3);
  //print_voxel_object_side_by_side(&v2, &v3);

  //std::cout << "\n"
  //             "*****************\n"
  //             "\n"
  //             "Full sphere block count:          " << v1.nblocks() << "\n"
  //             //"Spherical shell block count (v2): " << v2.nblocks() << "\n"
  //             "Spherical shell block count (v3): " << v3.nblocks() << "\n";

  //std::cout << "\n\nAfter\n";
  //print_memory_usage(std::cout);

  //std::cout << std::endl;

  return 0;
}
