// It stinks that javascript depends so heavily on global variables.  What can
// we do about it?
//
// Here is one approach to global abatement: have one global object storing all
// your stuff.

var MYAPP = {};

MYAPP.stooge = {
  'first-name': 'Joe',
  'last-name': 'Howard'
};

MYAPP.flight = {
  airline: 'Oceanic',
  number: 815,
  departure: {
    IATA: 'SYD',
    time: '2004-09-22 14:55',
    city: 'Sydney'
  },
  arrival: {
    IATA: 'LAX',
    time: '2004-09-23 10:42',
    city: 'Los Angeles'
  }
};
