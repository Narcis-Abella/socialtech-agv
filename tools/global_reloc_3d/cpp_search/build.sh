#!/bin/sh
# usage: build.sh [OUT.so]  — libcorr_search.so next to this script by default. -O3 without -march: the same binary for every Orin.
set -e
cd "$(dirname "$0")"
${CXX:-g++} -std=c++17 -O3 -shared -fPIC -pthread corr_search.cpp -o "${1:-libcorr_search.so}"
