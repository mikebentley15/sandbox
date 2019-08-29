// since javascript arrays are really objects, the delete operator can be used
// to remove elements from an array

var L = [1, 2, 3, 4];
delete L[2]; // [1, 2, undefined, 4]

// this leaves a hole in the array.

// instead use the splice() method.  It can do surgery on an array
//   splice(index, delete_count)
// this can be an expensive operation for long arrays

L.splice(2, 1);  // [1, 2, 4]
