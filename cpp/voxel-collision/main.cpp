#include "collides.h"

#include "CTSparseVoxelObject.h"
#include "CTVoxelObject.h"
#include "SparseVoxelObject.h"
#include "VoxelOctree.h"
#include "VoxelObject.h"
#include "OctomapWrap.h"

#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>

#include <unistd.h>

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

void print_memory_usage() {
  double vm, rss;
  memory_usage(vm, rss);
  std::cout << "Virtual Memory:    " << vm << " KiB\n"
            << "Resident Set Size: " << rss << " KiB" << std::endl;
}

template <typename VType>
void try_voxel_type(const std::string &name, const std::unique_ptr<VType> &v, int N = 1000) {
  std::cout << "\n"
            << "---------------------------------------------------------\n"
            << "  " << name << "\n"
            << "\n"
            << "v is size ("
            << v->Nx << ", " << v->Ny << ", " << v->Nz << ")" << std::endl
            << "sizeof(v): " << sizeof(*v) << std::endl;

  // copies
  auto &v1 = v;
  auto v2 = std::make_unique<VType>(*v);
  auto v3 = std::make_unique<VType>(*v);
  auto v4 = std::make_unique<VType>(*v);

  v1->add_point(0.1, 0.1, 0.1);
  v1->add_sphere(0.5, 0.5, 0.5, 0.25);
  v2->add_sphere(0.5, 0.5, 0.1, 0.1);
  v3->add_sphere(0.7, 0.2, 0.2, 0.55);

  std::cout << "v1.nblocks(): " << v1->nblocks() << std::endl
            << "v2.nblocks(): " << v2->nblocks() << std::endl
            << "v3.nblocks(): " << v3->nblocks() << std::endl
            << "v4.nblocks(): " << v4->nblocks() << std::endl;
  print_memory_usage();

  std::cout << "  v1.collides(v1): " << collides(*v1, *v1) << std::endl
            << "  v1.collides(v2): " << collides(*v1, *v2) << std::endl
            << "  v1.collides(v3): " << collides(*v1, *v3) << std::endl
            << "  v1.collides(v4): " << collides(*v1, *v4) << std::endl
            << "  v2.collides(v1): " << collides(*v2, *v1) << std::endl
            << "  v2.collides(v2): " << collides(*v2, *v2) << std::endl
            << "  v2.collides(v3): " << collides(*v2, *v3) << std::endl
            << "  v2.collides(v4): " << collides(*v2, *v4) << std::endl
            << "  v3.collides(v1): " << collides(*v3, *v1) << std::endl
            << "  v3.collides(v2): " << collides(*v3, *v2) << std::endl
            << "  v3.collides(v3): " << collides(*v3, *v3) << std::endl
            << "  v3.collides(v4): " << collides(*v3, *v4) << std::endl
            << "  v4.collides(v1): " << collides(*v4, *v1) << std::endl
            << "  v4.collides(v2): " << collides(*v4, *v2) << std::endl
            << "  v4.collides(v3): " << collides(*v4, *v3) << std::endl
            << "  v4.collides(v4): " << collides(*v4, *v4) << std::endl;

  auto start = std::chrono::system_clock::now();
  for (int i = 0; i < N; i++) {
    collides(*v1, *v2);
  }
  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_secs = end - start;
  std::cout << "collision checking collides(v1, v2) " << N << " times: "
            << elapsed_secs.count() << " sec\n"
            << "  runs in " << elapsed_secs.count() / N << " sec\n"
            << "  runs at " << N / elapsed_secs.count() << " Hz\n";
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

template <typename VType>
void print_voxel_object_side_by_side(const VType *v1, const VType *v2) {
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

int main() {
  std::cout << "Before\n";
  print_memory_usage();

  //try_voxel_type("VoxelObject_512", std::make_unique<VoxelObject>(512, 512, 512), 500);
  //try_voxel_type("VoxelObject_256", std::make_unique<VoxelObject>(256, 256, 256), 1000);
  //try_voxel_type("VoxelObject_128", std::make_unique<VoxelObject>(128, 128, 128), 10000);
  //try_voxel_type("CTVoxelObject_512", std::make_unique<CTVoxelObject<512, 512, 512>>(), 300);
  //try_voxel_type("CTVoxelObject_256", std::make_unique<CTVoxelObject<256, 256, 256>>(), 1000);
  //try_voxel_type("CTVoxelObject_128", std::make_unique<CTVoxelObject<128, 128, 128>>(), 10000);
  //try_voxel_type("SparseVoxelObject_512", std::make_unique<SparseVoxelObject>(512, 512, 512), 100);
  //try_voxel_type("SparseVoxelObject_256", std::make_unique<SparseVoxelObject>(256, 256, 256), 100);
  //try_voxel_type("SparseVoxelObject_128", std::make_unique<SparseVoxelObject>(128, 128, 128), 100);
  //try_voxel_type("CTSparseVoxelObject_512", std::make_unique<CTSparseVoxelObject<512, 512, 512>>(), 100);
  //try_voxel_type("CTSparseVoxelObject_256", std::make_unique<CTSparseVoxelObject<256, 256, 256>>(), 100);
  //try_voxel_type("CTSparseVoxelObject_128", std::make_unique<CTSparseVoxelObject<128, 128, 128>>(), 100);
  //try_voxel_type("VoxelOctree_512", std::make_unique<VoxelOctree<512>>(), 500);
  //try_voxel_type("VoxelOctree_256", std::make_unique<VoxelOctree<256>>(), 500);
  //try_voxel_type("VoxelOctree_128", std::make_unique<VoxelOctree<128>>(), 500);
  //try_voxel_type("OctomapWrap", std::make_unique<OctomapWrap>(1.0/4.0), 10);
  //try_voxel_type("OctomapWrap", std::make_unique<OctomapWrap>(1.0/8.0), 10);
  //try_voxel_type("OctomapWrap", std::make_unique<OctomapWrap>(1.0/16.0), 10);
  //try_voxel_type("OctomapWrap", std::make_unique<OctomapWrap>(1.0/32.0), 10);
  //try_voxel_type("OctomapWrap", std::make_unique<OctomapWrap>(1.0/64.0), 10);
  //try_voxel_type("OctomapWrap", std::make_unique<OctomapWrap>(1.0/128.0), 10);
  //try_voxel_type("OctomapWrap", std::make_unique<OctomapWrap>(1.0/256.0), 10);
  //try_voxel_type("OctomapWrap", std::make_unique<OctomapWrap>(1.0/512.0), 10);

  SparseVoxelObject v(20, 20, 32);
  v.add_sphere(0.7, 0.2, 0.2, 0.55);
  //print_voxel_object(&v);

  std::cout << "\n"
               "*****************\n"
               "\n"
               "After removing interior points\n"
            << std::endl;
  auto v2 = v;
  auto v3 = v;
  v2.remove_interior_slow_1();
  v3.remove_interior();
  print_voxel_object_side_by_side(&v2, &v3);

  std::cout << "\n"
               "*****************\n"
               "\n"
               "Full sphere block count:          " << v.nblocks() << "\n"
               "Spherical shell block count (v2): " << v2.nblocks() << "\n"
               "Spherical shell block count (v3): " << v3.nblocks() << "\n";

  std::cout << "\n\nAfter\n";
  print_memory_usage();

  std::cout << std::endl;

  return 0;
}
