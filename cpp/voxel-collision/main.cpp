#include "CTVoxelObject.h"
#include "VoxelObject.h"
#include "SparseVoxelObject.h"

#include <chrono>
#include <fstream>
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
void try_voxel_type(const std::unique_ptr<VType> &v) {
  std::cout << "\n"
            << "---------------------------------------------------------\n"
            << "\n"
            << "v is size ("
            << v->Nx << ", " << v->Ny << ", " << v->Nz << ")" << std::endl
            << "sizeof(v): " << sizeof(*v) << std::endl;

  // copies
  auto v2 = std::make_unique<VType>(*v);
  auto v3 = std::make_unique<VType>(*v);
  auto v4 = std::make_unique<VType>(*v);

  v->add_point(0.1, 0.1, 0.1);
  v->add_sphere(0.5, 0.5, 0.5, 0.25);
  v2->add_sphere(0.5, 0.5, 0.1, 0.1);
  v3->add_sphere(0.7, 0.2, 0.2, 0.55);

  print_memory_usage();

  std::cout << "v.collides(v):   " << v->collides(*v)   << "\n"
               "v.collides(v2):  " << v->collides(*v2)  << "\n"
               "v.collides(v3):  " << v->collides(*v3)  << "\n"
               "v.collides(v4):  " << v->collides(*v4)  << "\n"
               "v2.collides(v):  " << v2->collides(*v)  << "\n"
               "v2.collides(v2): " << v2->collides(*v2) << "\n"
               "v2.collides(v3): " << v2->collides(*v3) << "\n"
               "v2.collides(v4): " << v2->collides(*v4) << "\n"
               "v3.collides(v):  " << v3->collides(*v)  << "\n"
               "v3.collides(v2): " << v3->collides(*v2) << "\n"
               "v3.collides(v3): " << v3->collides(*v3) << "\n"
               "v3.collides(v4): " << v3->collides(*v4) << "\n"
               "v4.collides(v):  " << v4->collides(*v)  << "\n"
               "v4.collides(v2): " << v4->collides(*v2) << "\n"
               "v4.collides(v3): " << v4->collides(*v3) << "\n"
               "v4.collides(v4): " << v4->collides(*v4) << "\n";

  auto start = std::chrono::system_clock::now();
  int N = 80000000;
  for (int i = 0; i < N; i++) {
    v4->collides(*v3);
  }
  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed_secs = end - start;
  std::cout << "collision checking v4->collides(v3) " << N << " times: "
            << elapsed_secs.count() << " sec\n"
            << "  runs in " << elapsed_secs.count() / N << " sec\n"
            << "  runs at " << N / elapsed_secs.count() << " Hz\n";
}

template <typename VType>
void print_voxel_object(const VType *v) {
  for (size_t i = 0; i < v->Nx; i++) {
    std::cout << "i = " << i << std::endl;
    for (size_t j = 0; j < v->Ny; j++) {
      std::cout << "  ";
      for (size_t k = 0; k < v->Nz; k++) {
        std::cout << (v->cell(i, j, k) ? '1' : '.');
      }
      std::cout << std::endl;
    }
  }
}

int main() {
  std::cout << "Before\n";
  print_memory_usage();

  try_voxel_type(std::make_unique<VoxelObject>(512, 512, 512));
  try_voxel_type(std::make_unique<VoxelObject>(256, 256, 256));
  try_voxel_type(std::make_unique<VoxelObject>(128, 128, 128));
  try_voxel_type(std::make_unique<CTVoxelObject<512, 512, 512>>());
  try_voxel_type(std::make_unique<CTVoxelObject<256, 256, 256>>());
  try_voxel_type(std::make_unique<CTVoxelObject<128, 128, 128>>());
  try_voxel_type(std::make_unique<SparseVoxelObject<512, 512, 512>>());
  try_voxel_type(std::make_unique<SparseVoxelObject<256, 256, 256>>());
  try_voxel_type(std::make_unique<SparseVoxelObject<128, 128, 128>>());

  std::cout << "\n\nAfter\n";
  print_memory_usage();
  //auto v = std::make_shared<VoxelObject<32, 32, 64>>();

  //v->add_sphere(0.5, 0.5, 0.5, 0.25);
  //v->add_sphere(0.5, 0.5, 0.1, 0.1);

  //std::cout << "\n\n-----------------------------------------\n\n";
  //print_voxel_object(v.get());

  return 0;
}
