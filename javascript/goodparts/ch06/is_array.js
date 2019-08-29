// The typeof operator is not too helpful here, it just returns 'object

L = [1,2,3,4];
typeof L;   // 'object'

// this version of is_array() will work in many situations

var is_array = function (value) {
  return value && typeof value === 'object' && value.constructor === Array;
};

// if the array belongs to a different window for frame, you need to do
// something different

var is_array = function (value) {
  return Object.prototype.toString.apply(value) === '[object Array]';
};
