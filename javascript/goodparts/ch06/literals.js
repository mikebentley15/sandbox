var empty = [];
var numbers = [
  'zero', 'one', 'two', 'three', 'four',
  'five', 'six', 'seven', 'eight', 'nine'
];

empty[1];       // undefined
numbers[1];     // 'one'

empty.length    // 0
numbers.length  // 10

// similar results can be obtained using objects

var numbers_object = {
  '0': 'zero',
  '1': 'one',
  '2': 'two',
  '3': 'three',
  '4': 'four',
  '5': 'five',
  '6': 'six',
  '7': 'seven',
  '8': 'eight',
  '9': 'nine',
};

// but then you don't get the nice functions available through Array

numbers_object.length  // undefined

// you can store all sorts of types within an Array

var misc = [
  'string', 98.6, true, false, null, undefined,
  ['nested', 'array'], {object: true}, NaN, Infinity
];
misc.length    // 10

