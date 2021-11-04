#include <stdio.h>
#include <stdlib.h>
#include <linux/input.h>

// my keys have the following event codes:
//
// - pageup:   104
// - pagedown: 109
// - home:     102
// - end:      107
//
// this was captured by doing
//
//   sudo cat /dev/input/by-path/*-kbd | ./printevent
//
// looking in /usr/lib/linux/input-event-codes.h, these numbers match to
// 
// - pageup:   104  KEY_PAGEUP
// - pagedown: 109  KEY_PAGEDOWN
// - home:     102  KEY_HOME
// - end:      107  KEY_END
//
// this strategy could be used to make all sorts of mappings easily


int main(void) {
  // turn off buffering of standard in and out
  setbuf(stdin, NULL);
  setbuf(stdout, NULL);

  // read from stdin, modify, write to stdout
  struct input_event event;
  while (fread(&event, sizeof(event), 1, stdin) == 1) {
    if (event.type == EV_KEY) {
      switch(event.code) {
        // swap pageup <-> home
        case KEY_PAGEUP:   event.code = KEY_HOME    ; break;
        case KEY_HOME:     event.code = KEY_PAGEUP  ; break;
        // swap pagedown <-> end
        case KEY_PAGEDOWN: event.code = KEY_END     ; break;
        case KEY_END:      event.code = KEY_PAGEDOWN; break;
      }
    }
    fwrite(&event, sizeof(event), 1, stdout);
  }

  return 0;
}
