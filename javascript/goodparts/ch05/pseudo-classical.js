// Javascript is a bit confused by its prototypal nature.  So, this
// object-oriented pattern arose, but it's a bit wonky

var Mammal = function (name) {
  this.name = name;
};

Mammal.prototype.get_name = function () {
  return this.name;
};

Mammal.prototype.says = function () {
  return this.saying || '';
};

var myMammal = new Mammal('Herb the mammal');
var name = myMammal.get_name(); // 'Herb the mammal'

var Cat = function (name) {
  this.name = name;
  this.saying = 'meow';
};

Cat.prototype = new Mammal();

Cat.prototype.purr = function (n) {
  var i, s = '';
  var (i = 0; i < n; i += 1) {
    if (s) {
      s += '-';
    }
    s += 'r';
  }
  return s;
};

Cat.prototype.get_name = function () {
  return this.says() + ' ' + this.name + ' ' + this.says();
};

var myCat = new Cat('Henrietta');
var says = myCat.says(); // 'meow'
var purr = myCat.purr(5); // 'r-r-r-r-r'
var name = myCat.get_name(); // 'meow Henrietta meow'

// This was intended to look like classical object-oriented programming, but
// comes off wonky.  We can hide some of this ugliness.

Function.method('inherits', function (Parent) {
  this.prototype = new Parent();
  return this;
});

var Cat = function (name) {
    this.name = name;
    this.saying = 'meow';
  }
  .inherits(Mammal)
  .method('purr', function (n) {
    var i, s = '';
    for (i = 0; i < n; i += 1) {
      if (s) {
        s += '-';
      }
      s += 'r';
    }
    return s;
  })
  .method('get_name', function () {
    return this.says() + ' ' + this.name + ' ' + this.says();
  });

// Problems with this approach:
//   1. Still looks alien
//   2. Behaves strange in corner cases
//   3. No privacy; all properties are public.
//   4. No access to the superclass methods
//   5. If you forget to use 'new', bad things will happen
