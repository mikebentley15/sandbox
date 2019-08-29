// you must initialize arrays manually

Array.dim = function (dimension, initial) {
  var a = [], i;
  for (i = 0; i < dimension; i += 1) {
    a[i] = initial;
  }
  return a;
};

Array.zeros = function (dimension) { return Array.dim(dimension, 0); }

var myArray = Array.dim(10, 0);

// matrices are implemented as arrays of arrays

var matrix = [
  [0, 1, 2],
  [3, 4, 5],
  [6, 7, 8]
];
matrix[1][2];    // 5

Array.matrix = function (m, n, initial) {
  var a, i, j, mat = [];
  for (i = 0; i < m; i += 1) {
    a = [];
    for (j = 0; j < n; j += 1) {
      a[j] = initial;
    }
    mat[i] = a;
  }
  return mat;
};

var myMatrix = Array.matrix(4, 4, 0);

myMatrix[3][3];   // 0

Array.identity = function (n) {
  var i, mat = Array.matrix(n, n, 0);
  for (i = 0; i < n; i += ) {
    mat[i][i] = 1;
  }
  return mat;
};
