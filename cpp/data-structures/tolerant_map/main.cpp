    #include <iostream>
    #include <string>
    #include <map>
    #include <set>
    #include <cmath>  
    #include <vector>

    struct ItemKey
    {
        double m_t;
        double m_f;
        static constexpr double t_eps = 3;
        static constexpr double f_eps = 0.1;

        ItemKey(double t, double f) : m_t(t), m_f(f) {}

        bool operator<(const ItemKey& other) const
        {
            // Here it is assumed that f_eps and t_eps are positive
            // We also ignore overflow, underflow, and NaN
            // This is written for readability, and assumed the compiler will be
            // able to optimize it.
            auto fuzzy_less_than = [] (double a, double b, double eps) {
              return a < b - eps;
            };
            bool f_is_less_than    = fuzzy_less_than(this->m_f, other.m_f, f_eps);
            bool f_is_greater_than = fuzzy_less_than(other.m_f, this->m_f, f_eps);
            bool f_is_equal        = !f_is_less_than && !f_is_greater_than;
            bool t_is_less_than    = fuzzy_less_than(this->m_t, other.m_t, t_eps);

            return f_is_less_than || (f_is_equal && t_is_less_than);
        }
    };

    void simple_test_map() {
        std::map<ItemKey, size_t> counter1;
        counter1[{3.0, 10.0}] += 1;
        counter1[{5.0, 10.0}] += 1;
        counter1[{7.0, 10.0}] += 1;
        for (auto &itempair : counter1) {
           std::cout << "simple_test_map()::counter1: ("
                     << itempair.first.m_t << ", "
                     << itempair.first.m_f << ") - "
                     << itempair.second << "\n";
        }
        std::cout << std::endl;

        std::map<ItemKey, size_t> counter2;
        counter2[{5.0, 10.0}] += 1;
        counter2[{3.0, 10.0}] += 1;
        counter2[{7.0, 10.0}] += 1;
        for (auto &itempair : counter2) {
           std::cout << "simple_test_map()::counter2: ("
                     << itempair.first.m_t << ", "
                     << itempair.first.m_f << ") - "
                     << itempair.second << "\n";
        }
        std::cout << std::endl;
    }

    void simple_test_multiset() {
        std::multiset<ItemKey> counter1 {{3.0, 10.0}, {5.0, 10.0}, {7.0, 10.0}};
        for (auto &item : counter1) {
           std::cout << "simple_test_multiset()::counter1: ("
                     << item.m_t << ", "
                     << item.m_f << ")\n";
        }
        std::cout << std::endl;
        std::multiset<ItemKey> counter2 {{5.0, 10.0}, {3.0, 10.0}, {7.0, 10.0}};
        for (auto &item : counter2) {
           std::cout << "simple_test_multiset()::counter2: ("
                     << item.m_t << ", "
                     << item.m_f << ")\n";
        }
        std::cout << std::endl;
        std::cout << "simple_test_multiset()::counter2.size() = "
                  << counter2.size() << std::endl;
        for (auto &item : counter1) {
           std::cout << "simple_test_multiset()::counter2.count("
                     << item.m_t << ", "
                     << item.m_f << ") = "
                     << counter1.count(item) << std::endl;
        }
        std::cout << std::endl;
    }

    int main()
    {
        using namespace std;

        simple_test_map();
        simple_test_multiset();

        // The pairs are the respective values of m_t and m_f.
        vector<pair<double, double>> pairs;

        // These two should belong in one bucket
        // -> (109.9, 9.0), because m_f differs by 0.09 and m_t differs by just 1
        pairs.emplace_back(109.9, 9.0);
        pairs.emplace_back(110.9, 9.09);

        // This one is separate from above two beause even though m_t is in range,
        // m_f is beyong tolerance level
        pairs.emplace_back(109.5, 10.0);

        // Same for this as well, here both m_t and m_f are beyong tolerance of any
        // of the two categories found above
        pairs.emplace_back(119.9, 19.0);

        // This one matches the second bucket - (109.5, 10.0)
        pairs.emplace_back(109.9, 10.05);

        // And this one too.
        pairs.emplace_back(111.9, 9.87);

        map<ItemKey, size_t> itemMap;

        for (const auto& item: pairs)
        {
            ItemKey key(item.first, item.second);
            auto iter = itemMap.find(key);
            if (iter  == itemMap.end())
            {
                itemMap[key] = 1;
            }
            else
            {
                itemMap[iter->first] = itemMap[iter->first] + 1;
            }
        }

        // The map should have three keys
        // - (109.9, 9.0) -> count 2
        // - (109.5, 10.0) -> count 3
        // - (119.9, 19.0) -> count 1
        cout << itemMap.size() << endl;

        cout << "itemMap contents:" << endl;
        for (auto& item : itemMap) {
            cout << "  ("
                 << item.first.m_t << ", "
                 << item.first.m_f << ") - "
                 << item.second << endl;
        }

        return 0;
    }

