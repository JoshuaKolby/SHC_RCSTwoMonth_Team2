int solenoidPin = 3;
int timesOpened;
bool isOn;

void setup() {
  pinMode(solenoidPin, OUTPUT);
  timesOpened = 0;
  isOn = false;
}

void loop() {
  if(timesOpened < 5) {
    if(isOn) {
      delay(1000);
      timesOpened += 1;
      isOn = false;
    } else {
      delay(4000);
      isOn = true;
    }
  }
}
