misty.StartFaceDetection();

var LED = {
  purple: function() {
    misty.Debug("Setting LED to purple");
    misty.ChangeLED(148, 0, 211);
  },
  green: function() {
    misty.Debug("Setting LED to green");
    misty.ChangeLED(0, 255, 0);
  },
  red: function() {
    misty.Debug("Setting LED to red");
    misty.ChangeLED(255, 0, 0);
  },
  blue: function() {
    misty.Debug("Setting LED to blue");
    misty.ChangeLED(0, 0, 255);
  }
}

misty.RegisterTimerEventLED.green();

var registerFaceDetection = function() {
  misty.AddPropertyTest("FaceDetect", "PersonName", "exists", "", "string");
  misty.RegisterEvent("FaceDetect", "FaceRecognition", 1000, true);
}

var _FaceDetect = function(data) {
  misty.Debug(JSON.stringify(data));
  LED.purple();
  misty.DisplayImage("e_Joy.jpg");
  misty.MoveArmDegrees("both", -80, 10);
  misty.RegisterTimerEvent("timeoutToNormal", 5000, false);
}

registerFaceDetection();

var _timeoutToNormal = function() {
  misty.Pause(100);
  misty.MoveHeadPosition(0.1, 0.1, 0.1, 40);
  misty.MoveArmDegrees("both", 70, 10);
  LED.green();
  misty.DisplayImage("e_DefaultContent.jpg");
}

