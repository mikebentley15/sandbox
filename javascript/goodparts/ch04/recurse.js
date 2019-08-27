// Define a walk_the_DOM function that visits every node of the tree in HTML
// source order, starting from some given node.  It invokes a function, passing
// it each node in turn.  walk_the_DOM calls itself to process each of the
// child nodes.

var walk_the_DOM = function walk(node, func) {
  func(node);
  node = node.firstChild;
  while (node) {
    walk(node, func);
    node = node.nextSibling;
  }
};

// Define a getElementsByAttribute function.  It takes an attribute name string
// and an optional matching value.  It calls walk the DOM, passing it a
// function that looks for an attribute name in the mode.  The matching nodes
// are accumulated in a results array.

var getElementsByAttribute = function(att, value) {
  var reults = [];

  walk_the_DOM(document.body, function (node) {
    var actual = node.nodeType === 1 && node.getAttribute(att);
    if (typeof actual === 'string' &&
          (actual === value || typeof value !== 'string')){
      results.push(node);
    }
  });

  return results;
};

// Make a factorial function with tail recursion.  It is tail recursive because
// it returns the result of calling itself.
//
// Javascript does not currently optimize this form.

var factorial = function factorial(i, a) {
  a = a || 1;
  if (i < 2) {
    return a;
  }
  return factorial (i - 1, a * i);
};

document.writeln(factorial(4));   // 24
