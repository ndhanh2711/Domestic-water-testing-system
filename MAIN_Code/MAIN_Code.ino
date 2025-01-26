#include <Arduino.h>

//---------------Định nghĩa và đặt tên cho các chân cảm biến--------------
// Cảm biến lưu lượng nước
#define LED_BUILTIN 2
#define SENSOR 39
  long currentMillis = 0;
  long previousMillis = 0;
  int interval = 1000;
  boolean ledState = LOW;
  float calibrationFactor = 4.5;
  volatile byte pulseCount;
  byte pulseSec = 0;
  float flowRate;
  unsigned int flowMilliLitres;
  unsigned long totalMilliLitres;
// Cảm biến nồng độ chất tan
#define TdsSensorPin 34
#define VREF 3.3              // analog reference voltage(Volt) of the ADC
#define SCOUNT  30            // sum of sample point
  int analogBuffer[SCOUNT];     // store the analog value in the array, read from ADC
  int analogBufferTemp[SCOUNT];
  int analogBufferIndex = 0;
  int copyIndex = 0;

  float averageVoltage = 0;
  float tdsValue = 0;
  float temperature = 25;       // current temperature for compensation

// Cảm biến độ PH

//-----------------Các hàm khởi tạo và tính toán cảm biến----------------
// Cảm biến lưu lượng nước
void IRAM_ATTR pulseCounter() {
  pulseCount++;
}
// Cảm biến nồng độ chất tan trong dung dịch
// median filtering algorithm
int getMedianNum(int bArray[], int iFilterLen){
  int bTab[iFilterLen];
  for (byte i = 0; i<iFilterLen; i++)
  bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0){
    bTemp = bTab[(iFilterLen - 1) / 2];
  }
  else {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}
//----Hàm SETUP----
void setup() {
  Serial.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SENSOR, INPUT_PULLUP);

  pulseCount = 0;
  flowRate = 0.0;
  flowMilliLitres = 0;
  totalMilliLitres = 0;
  previousMillis = 0;

  attachInterrupt(digitalPinToInterrupt(SENSOR), pulseCounter, FALLING);
  pinMode(TdsSensorPin,INPUT);
}

void loop() {
  currentMillis = millis();
  if (currentMillis - previousMillis > interval) {
    pulseSec = pulseCount;
    pulseCount = 0;

    // Because this loop may not complete in exactly 1 second intervals, we calculate
    // the number of milliseconds that have passed since the last execution and use
    // that to scale the output. We also apply the calibrationFactor to scale the output
    // based on the number of pulses per second per units of measure
    // (litres/minute in this case) coming from the sensor.
    flowRate = (1000.0 / (millis() - previousMillis)) * pulseSec;
    previousMillis = millis();
  }

  // Divide the flow rate in litres/minute by 60 to determine how many millilitres
  // passed through the sensor in this 1 second interval, then multiply by 1000 to
  // convert to millilitres.
  flowMilliLitres = (flowRate / 60) * 1000;

  // Add the millilitres passed in this second to the cumulative total
  totalMilliLitres += flowMilliLitres;

  // Print the flow rate for this second in litres / minute
  Serial.print("Flow rate: "); 
  Serial.print(int(flowRate));  // Print the integer part of the variable
  Serial.print("L/min");
  Serial.print("\t");           // Print tab space

  // Print the cumulative total of litres flowed since starting
  Serial.print("Output Liquid Quantity: "); 
  Serial.print(totalMilliLitres);
  Serial.print("mL / "); 
  Serial.print(totalMilliLitres / 1000);
  Serial.println("L");
}