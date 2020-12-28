#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <Adafruit_MLX90614.h>

#if defined(ARDUINO) && ARDUINO >= 100
#define printByte(args)  write(args);
#else
#define printByte(args)  print(args,BYTE);
#endif

LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display
MAX30105 particleSensor;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

const int buzzer =  10;
uint8_t heart[8] = {0x0, 0xa, 0x1f, 0x1f, 0xe, 0x4, 0x0};
int oldHeart = 0;
float oldTemp = 0;
bool rwMood = false;
int rwMoodCount = 0;

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;

int countAvg = 0;
int oldavg = 0;
int countDone = 0;

int sampleAvgComp = 5;
float maxTempWorng = 37;
int rTime = 10000;
int wrongTime = 10000;

void setup() {
  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, LOW);
  Serial.begin(115200);
  Serial.println("Initializing...");
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  // Serial.println("Place your index finger on the sensor with steady pressure.");
  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED

  mlx.begin();

  lcd.init();
  lcd.backlight();
  lcd.createChar(0, heart);
  lcd.setCursor(0, 0);
  lcd.print("This Project by");
  lcd.setCursor(0, 1);
  lcd.print("AE Team");
  delay(10000);
  lcd.clear();
}

void loop() {
  countDone = 0;
  long irValue = particleSensor.getIR();

  if (checkForBeat(irValue) == true)
  {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();
    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 35)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable
      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

  Serial.println("----------------------------");
  Serial.print("IR=");
  Serial.print(irValue);
  Serial.print(", BPM=");
  Serial.print(beatsPerMinute);
  // Serial.print("\t");
  Serial.print(", Avg BPM=");
  Serial.println(beatAvg);
  //
  Serial.print("oldavg=");
  Serial.print(oldavg);
  Serial.print(", countAvg=");
  Serial.println(countAvg);

  if (irValue < 50000) {
    Serial.print("No finger?");
    if ((oldHeart || oldTemp != 0) && countDone == 0 ) {
      lastResalt(oldHeart, oldTemp);
    }
    noFing();
    rwMood = false;
    rwMoodCount = 0;
  } else {
    rwMood = true;
    rwMoodCount = 1;
    lcdWrite(beatAvg, mlx.readObjectTempC());
  }

  if (countAvg <= sampleAvgComp) {
    if ((oldHeart || oldTemp != 0) && countAvg == sampleAvgComp) {
      lastResalt(oldHeart, oldTemp);
      countDone = 1;
    }
    if (oldavg != beatAvg) {
      oldavg = beatAvg;
      countAvg++;
    }
  }
  Serial.print(", countDone=");
  Serial.print(countDone);
  Serial.println();
}

void lcdWrite(int heart, float temp) {
  if (rwMoodCount == 1 ) {

    lcdScreenInit();
    rwMoodCount = 0;
  }

  if (oldHeart != heart) {
    lcd.setCursor(5, 0);
    lcd.print("   ");
    lcd.setCursor(5, 0);
    lcd.print(heart);
    oldHeart = heart;
  }

  if (oldTemp != temp) {
    lcd.setCursor(5, 1);
    lcd.print("   ");
    lcd.setCursor(5, 1);
    lcd.print(temp);
    oldTemp = temp;
  }
}

void noFing() {
  lcd.setCursor(0, 0);
  lcd.print("Please Put Your");
  lcd.setCursor(0, 1);
  lcd.print("Finger");
  delay(1000);
  lcd.noDisplay();
  delay(500);
  lcd.display();
  lcd.clear();
}

void lcdScreenInit() {
  lcd.setCursor(0, 0);
  lcd.print("Avg");
  lcd.printByte(0);
  lcd.setCursor(4, 0);
  lcd.print("=");
  lcd.setCursor(0, 1);
  lcd.print("Temp=");
}

void lastResalt(int heart, float temp) {
  bebbeb();
  lcd.setCursor(0, 0);
  lcd.print("Avg");
  lcd.printByte(0);
  lcd.setCursor(4, 0);
  lcd.print("=");
  lcd.setCursor(0, 1);
  lcd.print("Temp=");
  lcd.setCursor(15, 0);
  lcd.print("R");
  lcdWrite(heart, temp);

  if (temp >= maxTempWorng) {
    digitalWrite(buzzer, HIGH);
    delay(wrongTime);
  }

  delay(rTime);
  lcd.clear();
  oldTemp = 0;
  oldHeart = 0;
  oldavg = 0;
  beatAvg = 0;
  beatsPerMinute = 0;
  countAvg = 0;
  digitalWrite(buzzer, LOW);
}

void bebbeb() {
  digitalWrite(buzzer, HIGH);
  delay(300);
  digitalWrite(buzzer, LOW);
  delay(150);
  digitalWrite(buzzer, HIGH);
  delay(300);
  digitalWrite(buzzer, LOW);
}
