misty.Debug("The hello_world skill is starting...");

function getRandomInt(min, max) {
  return Math.floor(Math.random() * (max - min + 1)) + min;
}

misty.RegisterTimerEvent("look_around", 100, false);

function _look_around(repeat = true) {
  misty.MoveHeadDegrees(
    getRandomInt(-40, 20),
    getRandomInt(-30, 30),
    getRandomInt(-40, 40),
    30
  );

  if (repeat) {
    misty.RegisterTimerEvent("look_around", getRandomInt(5, 10) * 100, false);
  }
}

