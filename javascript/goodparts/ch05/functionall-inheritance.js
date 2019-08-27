// Here are four steps to making an inheritable object with protection
//   1. Make a constructor function that returns an object (without new)
//   2. Optionally define private instance variables and methods that can be
//      captured.
//   3. Augments the new object with public methods that have privileged access
//      to the private parameters and functions.
//   4. Return the new object

// here is the pseudocode

// spec: an object containing all parameters for the new object
// my: a container of secrets that are shared by the constructors in the
//     inheritance chain.
var constructor = function (spec, my) {
  var that, other_private_instance_variables;
  my = my || {};

  // add shared variables and functions to my

  that = {/* a new object here */};

  // add privileged methods to that

  return that;
};

// now the mammal example with this new pattern

var mammal = function (spec) {
  var that = {};

  that.get_name = function () {
    return spec.name;
  };

  that.says = function () {
    return spec.saying || '';
  };

  return that;
};

var myMammal = mammal({name: 'Herb'});

var cat = function (spec) {
  spec.saying = spec.saying || 'meow';
  var that = mammal(spec);
  that.purr = function (n) {
    return '-'.join(['r'].duplicate(n));
  };
  that.get_name = function () {
    return ' '.join(that.says(), spec.name, that.says());
  };
  return that;
};

var myCat = cat({name: 'Henrietta'});

// This functional pattern gives us a way to access super methods.  Let us make
// a superior() method that takes a method name and returns a function that
// invokes that method.

Object.method('superior', function (name) {
  var that = this,
      method = that[name];
  return function () {
    return method.apply(that, arguments);
  };
}

var coolcat = function (spec) {
  var that = cat(spec),
      super_get_name = that.superior('get_name');
  that.get_name = function (n) {
    return ' '.join('like', super_get_name(), 'baby');
  };
  return that;
};

var myCoolCat = coolcat({name: 'Bix'});
var name = myCoolCat.get_name();  // 'like meow Bix meow baby'

// If all of the state of an object is private, then the object is
// _tamper-proof_.  Properties of the object can be replaced or deleted, but
// the integrity of the object is not compromised.
//
// If we create an object in the functional style, and if all of the methods of
// the object make no use of this or that, then the object is _durable_.  A
// durable object is simply a collection of functions that act as
// _capabilities_.
