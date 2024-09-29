#ifndef UTIL_HPP
#define UTIL_HPP

#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <set>
#include <unordered_set>
#include <map>
#include <unordered_map>
#include <queue>
#include <stack>
#include <chrono>
#include <random>
#include <bitset>
#include <deque>
#include <tuple>
#include <numeric>
#include <climits>
#include <cassert>
#include <iterator>
#include <fstream>
#include <Eigen/Dense>

#define ll long long
#define ull unsigned long long
#define pb push_back
#define all(x) (x).begin(), (x).end()
#define rall(v) v.rbegin(), v.rend()

const int INF = 1e9;
const ll LINF = 1e18;
const int NINF = -1e9;
const ll LNINF = -1e18;
const int MOD = 1e9 + 7;

template <typename T>
std::vector<T> vector_in(int n) {
    std::vector<T> v(n);
    for (int i = 0; i < n; i++) {
        std::cin >> v[i];
    }
    return v;
}

template <typename T>
void vector_out(const std::vector<T> &v) {
    std::copy(v.begin(), v.end(), std::ostream_iterator<T>(std::cout, " "));
    std::cout << '\n';
}

inline std::mt19937 rng(chrono::steady_clock::now().time_since_epoch().count());

inline int rand(int i, int j) {
    return std::uniform_int_distribution<int>(i, j)(rng);
}

#endif  // UTIL_HPP