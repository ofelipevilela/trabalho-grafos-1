#ifndef CONJUNTO_SET_HPP
#define CONJUNTO_SET_HPP

#include <vector>

class conjunto {
public:
    conjunto(size_t n) : parent(n), rank(n, 0) {
        for (size_t i = 0; i < n; ++i) {
            parent[i] = i;
        }
    }

    size_t find(size_t x) {
        if (parent[x] != x) {
            parent[x] = find(parent[x]);  // Path compression
        }
        return parent[x];
    }

    void union_sets(size_t x, size_t y) {
        size_t root_x = find(x);
        size_t root_y = find(y);

        if (root_x != root_y) {
            if (rank[root_x] < rank[root_y]) {
                parent[root_x] = root_y;
            } else if (rank[root_x] > rank[root_y]) {
                parent[root_y] = root_x;
            } else {
                parent[root_y] = root_x;
                rank[root_x]++;
            }
        }
    }

private:
    std::vector<size_t> parent;
    std::vector<size_t> rank;
};

#endif // CONJUNTO_SET_HPP