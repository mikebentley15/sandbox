// Javascript allows basic types to be augmented

// a method to easily add new methods to any basic type

Function.prototype.method = function(name, func) {
  // add method conditionally
  if (!this.prototype[name]) {
    this.prototype[name] = func;
    return this;
  }
};

Number.method('integer', function() {
  return Math[this < 0 ? 'ceil' : 'floor'](this);
});

document.writeln((-10 / 3).integer());  // -3

// A method to trim from a string

String.method('trim', function() {
  return this.replace(/^\s+|\s+$/g, '');
});

document.writeln('"' + '  neat   '.trim() + '"');
