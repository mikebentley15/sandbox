// This type of closure provides an object with two methods: increment(inc=1)
// and getValue().  The value attribute is private since it is used by capture.
// Even added functions to the object cannot access the value attribute since
// it cannot create a closure on it.

var myObject = (function() {
  var value = 0;

  return {
    increment: function(inc) {
      value += typeof inc === 'number' ? inc : 1;
    },
    getValue: function() {
      return value;
    }
  };
}());

// A more useful example

// Define a function that sets a DOM node's color to yellow and then fades it
// to white.
//
// The internal step() function inside of fade has captured the level variable.
// The setTimeout() function registers a callback with the Javascript event
// loop to be called in 100 milliseconds.  This approach leaves the browser
// responsive while scheduling actions to be done later.

var fade = function (node) {
  var level = 1;
  var step = function() {
    var hex = level.toString(16);
    node.style.backgroundColor = '#FFFF' + hex + hex;
    if (level < 15) {
      level += 1;
      setTimeout(step, 100);
    }
  };
  setTimeout(step, 100);
};

fade(document.body);

// BEGIN BAD EXAMPLE

// Make a function that assigns event handler functions to an array of nodes
// the wrong way.  When you click on a node, an alert box is supposed to
// display the ordinal of the node.  But it always displays the number of nodes
// instead.

var add_the_handlers = function (nodes) {
  var i;
  for (i = 0; i < nodes.length; i += 1) {
    nodes[i].onclick = function (e) {
      alert(i);
    };
  }
};

// END BAD EXAMPLE

// BEGIN BETTER EXAMPLE

// Make a function that assigns event handler functions to an array of nodes.
// When you click on a node, an alart box will display the ordinal of the node.

var add_the_handlers = function (nodes) {
  var create_alert_function = function (i) {
    return function (e) {
      alert(i);
    };
  };
  var i;
  for (i = 0; i < nodes.length; i += 1) {
    nodes[i].onclick = create_alert_function(i);
  }
};

// END BETTER EXAMPLE

