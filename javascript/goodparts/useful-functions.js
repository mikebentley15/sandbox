// conditionally add method() to Function
if (!Function.prototype.method) {
  Function.prototype.method = function (name, func) {
    // add method conditionally
    if (!this.prototype[name]) {
      this.prototype[name] = func;
      return this;
    }
  };
}

Function.method('curry', function () {
  var slice = Array.prototype.slice,
      args = slice.apply(arguments),
      that = this;
  return function () {
    return that.apply(null, args.concat(slice.apply(arguments)));
  };
});

Number.method('integer', function () {
  return Math[this < 0 ? 'ceil' : 'floor'](this);
});

String.method('trim', function () {
  return this.replace(/^\s+|\s+$/g, '');
});

if (Object.create !== 'function') {
  Object.create = function (o) {
    var F = function () {};
    F.prototype = o;
    return new F();
  };
}

Object.method('superior', function (name) {
  var that = this,
      method = that[name];
  return function () {
    return method.apply(that, arguments);
  };
}

var walk_the_DOM = function walk(node, func) {
  func(node);
  node = node.firstChild;
  while (node) {
    walk(node, func);
    node = node.nextSibling;
  }
};

var getElementsByAttribute = function(att, value) {
  var results = [];

  walk_the_DOM(document.body, function (node) {
    var actual = node.nodeType === 1 && node.getAttribute(att);
    if (typeof actual === 'string' &&
          (actual === value || typeof value !== 'string')) {
      results.push(node);
    }
  });

  return results;
};

Array.method('reduce', function (f, value) {
  var i;
  for (i = 0; i < this.length; i += 1) {
    value = f(this[i], value);
  }
  return value;
});

Array.dim = function (dimension, initial) {
  var a = [], i;
  for (i = 0; i < dimension; i += 1) {
    a[i] = initial;
  }
  return a;
};

Array.zeros = function (dimension) { return Array.dim(dimension, 0); }

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

Array.identity = function (n) {
  var i, mat = Array.matrix(n, n, 0);
  for (i = 0; i < n; i += ) {
    mat[i][i] = 1;
  }
  return mat;
};

//////////////////
// My creations //
//////////////////

// Can be called with an array, or just with a bunch of arguments
String.method('join', function (array) {
  var i, s = '', arr;
  arr = (array.constructor === Array && arguments.length === 1) ?
        array : arguments;
  for (i = 0; i < arr.length; i += 1) {
    if (s) {
      s += this;
    }
    s += arr[i];
  }
  return s;
});

Array.method('duplicate', function (number) {
  if (typeof number !== 'number' || number === 0) { return []; }
  if (number === 1) { return this; }
  if (number % 2 === 0) {
    return this.concat(this).duplicate(number / 2);
  }
  // else
  return this.concat(this.duplicate(number - 1));
});

Array.method('shallow_copy', function () { return this.slice(0); });
