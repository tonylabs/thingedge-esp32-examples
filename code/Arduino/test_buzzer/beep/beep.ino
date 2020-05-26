#define BUZZER_PIN 14

int freq = 10; //@10, 100, 1000, 4500
int channel = 0;
int resolution = 8;


void setup()
{
  Serial.begin(115200);
  ledcSetup(channel, freq, resolution);
  ledcAttachPin(BUZZER_PIN, channel);
}

void loop()
{
  ledcWriteTone(channel, freq);
  delay(500);
  ledcWriteTone(channel, 0);
  delay(500);
}
