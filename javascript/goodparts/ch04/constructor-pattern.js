// This is not a recommended way to create or define objects.  Sure, it may
// appear more similar to what you would expect from other languages, but it
// was actually a dangerous addition to the language.

// By convention, types that are constructor functions must be called with the
// "new" operator.  If you don't, then weird things happen.  For this reason,
// constructor-based objects start with a capitol letter.

// Creates a constructor function called Quo.
// It makes an object with a status property

var Quo = function (string) {
  this.status = string;
};

// Give all instances of Quo a public method called get_status.

Quo.prototype.get_status = function() {
  return this.status;
};

// Make an instance of Quo.

var myQuo = new Quo('confused');

document.writeln(myQuo.get_status());  // confused
