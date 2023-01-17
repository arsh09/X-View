#include "test_argsort.h"

#include <glog/logging.h>

#include <algorithm>
#include <vector>

namespace x_view_test {

void testArgsortColums() {

  x_view::MatrixXr matrix(10, 20);
  for(int j = 0; j < matrix.cols(); ++j) {
    for(int i = 0; i < matrix.rows(); ++i) {
      matrix(i, j) = -i;
    }
  }

  for(int j = 0; j < matrix.cols(); ++j) {
    const Eigen::VectorXi order = x_view::argsort(matrix.col(j));
    for(int i = 0; i < order.rows(); ++i) {
      CHECK_EQ(order[i], order.rows() - i - 1);
    }
  }

}

void testArgsortRows() {
  x_view::MatrixXr matrix(20, 10);
  for(int j = 0; j < matrix.cols(); ++j) {
    for(int i = 0; i < matrix.rows(); ++i) {
      matrix(i, j) = -j;
    }
  }

  for(int i = 0; i < matrix.rows(); ++i) {
    const Eigen::VectorXi order = x_view::argsort(matrix.row(i));
    for(int j = 0; j < order.rows(); ++j) {
      CHECK_EQ(order[j], order.rows() - j - 1);
    }
  }
}

void testRandom() {
  for(int size = 5; size < 100; size += 5) {
    Eigen::VectorXd r = Eigen::VectorXd::Random(size);
    Eigen::VectorXd sorted = r;
    std::sort(sorted.data(), sorted.data() + sorted.size());

    Eigen::VectorXi order = x_view::argsort(r);
    Eigen::VectorXd ordered(size);
    for(int i = 0; i < order.size(); ++i) {
      ordered(i) = r(order(i));
    }

    for (int i = 0; i < size; ++i) {
      CHECK_DOUBLE_EQ(ordered[i], sorted[i]);
    }
  }
}

void testRepeating() {
  const int size = 5;
  Eigen::VectorXd r;
  r.resize(size);
  r << 1, 2, 0, 0, 1;

  const Eigen::VectorXi order = x_view::argsort(r);

  for(int i = 0; i < size - 1; ++i) {
    CHECK_LE(r(order(i)), r(order(i+1)));
  }
}
}
