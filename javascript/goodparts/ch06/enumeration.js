// You can enumerate through a list using for..in
// But, it is not guaranteed to iterate in order - useless

var L = [1,2,3,4];
for (var val in L) {
  console.log(val);
}

// To ensure they are iterated in order, go through the indices manually
// But, this does not handle holes very well, so don't create holes...

var i;
for (i = 0; i < L.length; i += 1) {
  console.log(L[i]);
}
