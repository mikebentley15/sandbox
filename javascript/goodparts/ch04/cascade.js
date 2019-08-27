// If we have functions that mutate the object to return this instead of
// undefined, then we can implement cascading.  This is calling many methods on
// the same object in sequence in a single statement.

// This type of interface is very expressive and helps control the tendency to
// make interfaces that try to do too much at once.

getElement('myBoxDiv')
  .mode(350, 150)
  .width(100)
  .height(100)
  .color('red')
  .border('10px outset')
  .padding('4px')
  .appendText('Please stand by')
  .on('mousedown', function (m) {
    this.startDrag(m, this.getNinth(m));
  })
  .on('mousemove', 'drag')
  .on('mouseup', 'stopDrag')
  .later(2000, function () {
    this
      .color('yellow')
      .setHTML('What hath God wraught')
      .slide(400, 40, 200, 200);
  })
  .tip('This box is resizeable');
