#include <cmath>
#include <cstdio>
#include <vector>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <map>
using namespace std;


int main() {
    std::map<int, int> counts;
    int n;
    std::cin >> n;
    for (int i = 0; i < n; i++) {
        int x;
        std::cin >> x;
        counts[x] += 1;
    }

    int idx_low = 0;
    int idx_high = 0;
    double mean = 0.0;
    int mode = 0;
    int mode_count = 0;
    double median = 0.0;
    int mid_idx1 = (n-1) / 2;
    int mid_idx2 = n / 2;
    for (auto kv : counts) {
        int k = kv.first;
        int v = kv.second;
        idx_high = idx_low + v;
        mean += k*v;
        if (v > mode_count) {
            mode = k;
            mode_count = v;
        }
        if (idx_low <= mid_idx1 && mid_idx1 < idx_high) {
            median += k;
        }
        if (idx_low <= mid_idx2 && mid_idx2 < idx_high) {
            median += k;
        }
        idx_low = idx_high;
    }
    median /= 2;
    mean /= n;
    std::cout << std::setprecision(1) << std::fixed
              << mean << std::endl
              << median << std::endl
              << mode << std::endl;
    return 0;
}

