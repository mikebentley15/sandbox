// A module is a function or object that presents an interface but that hides
// its state and implementation.  This helps us eliminate the use of global
// variables.

// augment String with deentityify() that converts html codes with the actual
// character it represents.

String.method('deentityify', function() {

  // The entity table.  It maps entity names to characters.
  var entity = {
    quot: '"',
    lt:   '<',
    gt:   '>'
  };

  // Return the deentityify method.

  return function () {

    // This is the deentityify method.  It calls the string replace method,
    // looking for substrings that start with '&' and end with ';'.  If the
    // characters in between are in the entity table, then replace the entity
    // with the character from the table.  It uses a regular expression.

    return this.replace(/&([^&;]+);/g,
      function (a, b) {
        var r = entity[b];
        return typeof r === 'string' ? r : a;
      }
    );
  };

}());

document.writeln('&lt;&quot;&gt;'.deentityify());  // <">

// Here is a secure generator for serial numbers
//
// It does not use this or that, and as a result, it cannot be hijacked.  The
// methods could be replaced, but that does not give access to the captured
// secret internal variables or internal implementation.

var serial_maker = function () {

  // Produce an object that produces unique strings.  A unique string is made
  // up of two parts: a prefix and a sequence number.  The object comes with
  // methods for setting the prefix and sequence number, and a gensym() method
  // that produces unique strings.

  var prefix = '';
  var seq = 0;
  return {
    set_prefix: function (p) {
      prefix = String(p);
    },
    set_seq: function (s) {
      seq = s;
    },
    gensym: function () {
      var result = prefix + seq;
      seq += 1;
      return result;
    }
  };
};
var seqer = serial_maker();
seqer.set_prefix('Q');
seqer.set_seq(1000);
var unique = seqer.gensym();  // unique is "Q1000"
