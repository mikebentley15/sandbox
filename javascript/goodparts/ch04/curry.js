// Functions are values.  We can manipulate function values in interesting
// ways.  Currying allows us to produce a new function by combining a function
// and an argument.

// Javascript does not have a 'curry' method, but we can fix that
// Note: the arguments variable is not quite an array, so we have to do this
// weird thing with the slice function.

Function.method('curry', function () {
  var slice = Array.prototype.slice,
      args = slice.apply(arguments),
      that = this;
  return function () {
    return that.apply(null, args.concat(slice.apply(arguments)));
  };
});

var add = function (a, b) { return a + b; }
var add1 = add.curry(1);
document.writeln(add1(6));  // 7
