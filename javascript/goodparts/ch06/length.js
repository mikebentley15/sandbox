// every array has a length property
// but, it is not the number of elements within the array.
// rather, it is the largest contained index plus one

var myArray = [];
myArray.length         // 0

myArray[1000000] = true;
myArray.length         // 1000001
// myArray only contains one property

// the length field can be modified

// making it larger doesn't do much
// making it smaller deletes elements off of the end

L = [1, 2, 3, 4, 5, 6, 7, 8, 9];
L.length;       // 9
L;              // [1, 2, 3, 4, 5, 6, 7, 8, 9]

L.length = 15;
L.length;       // 15
L;              // [1, 2, 3, 4, 5, 6, 7, 8, 9, <6 empty items>]

L.length = 5;
L.length;       // 5
L;              // [1, 2, 3, 4, 5]

L.length = 9;
L.length;       // 9
L;              // [1, 2, 3, 4, 5, <4 empty items>]

L.length = 3;
L;              // [1, 2, 3]

// new elements can be added using the length

L[L.length] = 'N';
L;              // [1, 2, 3, 'N']

// but it is more conventient to use push()

L.push('T');
L;              // [1, 2, 3, 'N', 'T']
