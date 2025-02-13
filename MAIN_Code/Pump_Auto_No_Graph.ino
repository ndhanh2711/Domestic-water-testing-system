//Khai báo các thư viện, chân kết nối với ngoại vi
#include <Arduino.h>
#include <LiquidCrystal.h>
#include <Preferences.h>  // Thay thế NVSFlash.h bằng Preferences
#include <ctime>  // Để dùng hàm mktime()
#define BLYNK_PRINT Serial
//Định nghĩa địa chỉ, tên, mã kết nối với app Blynk IoT
#define BLYNK_TEMPLATE_ID "TMPL6TGETSmgF"
#define BLYNK_TEMPLATE_NAME "watering quality testing"
#define BLYNK_AUTH_TOKEN "aAL7Ba4mxXeoyjMifkDAJSRqA8asqKRl"

// Khai báo thư viện wifi và blynk 
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

// Thông tin WiFi
char ssid[] = "30 PDG";
char pass[] = "bktech1017";

// char ssid[] = "Quang Hai T3";
// char pass[] = "19741975";
// Chân GPIO32
#define GPIO32_PIN 32 // Chân điều khiển máy bơm

// Khai báo chân kết nối LCD với ESP32
const int RS = 33, EN = 25, D4 = 26, D5 = 27, D6 = 14, D7 = 13;

// Khởi tạo đối tượng LCD
LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);

//Dùng biến toàn cục cho cảm biến
int chattan_value;
float ph_value;

//++++++++++
// Khai báo đối tượng Preferences
Preferences preferences;
float totalMilliLitres30Days = 0;   // Tổng lượng nước trong 30 ngày
int dayCounter = 0;                 // Đếm số ngày
//✅_______________________________________________________________CAM BIEN LUU LUONG__________________________________________________________________
#define LED_BUILTIN 2
#define SENSOR 39

    long currentMillis = 0;
    long previousMillis = 0;
    int interval = 1000;
    boolean ledState = LOW;

    //Hệ số hiệu chuẩn cảm 
    float calibrationFactor = 4.5;
    //Biến kiểm soát lưu  
    volatile byte pulseCount;
    byte pulseSec = 0;
    float flowRate;
    float flowMilliLitres;
    float totalMilliLitres;

    //Biến trạng thái nước chảy
    bool waterFlowing = false;

void IRAM_ATTR pulseCounter() {
  pulseCount++;
  waterFlowing = true;  // Có xung nghĩa là nước đang chảy
}

// Hàm tính toán lưu lượng nước
float calculateFlowRate() {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis > interval) {
        int pulseSec = pulseCount;  // Lấy số xung trong khoảng thời gian
        pulseCount = 0;  // Reset bộ đếm xung

        if (pulseSec == 0) {
            waterFlowing = false;
            return 0.0;
        }

        // Tính toán lưu lượng (L/min)
        flowRate = (1000.0 / (currentMillis - previousMillis)) * pulseSec;
        previousMillis = currentMillis;

        // Tính tổng lượng nước đã chảy (mL)
        float flowMilliLitres = (flowRate / 60) * 1000;
        totalMilliLitres += flowMilliLitres/5;
        totalMilliLitres30Days += flowMilliLitres/5;
        return flowRate;
    }
    return flowRate;
}
// 📌 Hàm lấy ngày hiện tại
String getCurrentDate() {
  time_t now = time(nullptr);
  struct tm* timeinfo = localtime(&now);
  char currentDate[20];
  strftime(currentDate, sizeof(currentDate), "%Y-%m-%d", timeinfo);
  return String(currentDate);
}

void readFromNVS() {
  preferences.begin("storage", false);  

  // Đọc dữ liệu từ NVS
  String storedDate = preferences.getString("date", "");  
  totalMilliLitres = preferences.getFloat("value", 0);
  totalMilliLitres30Days = preferences.getFloat("value30", 0);
  dayCounter = preferences.getInt("dayCounter", 0);

  String currentDate = getCurrentDate();

  // 📌 Chuyển ngày thành timestamp để tính số ngày trôi qua
  struct tm timeStored = {0}, timeNow = {0};

  sscanf(storedDate.c_str(), "%d-%d-%d", &timeStored.tm_year, &timeStored.tm_mon, &timeStored.tm_mday);
  sscanf(currentDate.c_str(), "%d-%d-%d", &timeNow.tm_year, &timeNow.tm_mon, &timeNow.tm_mday);
  
  timeStored.tm_year -= 1900;  // struct tm cần tính từ năm 1900
  timeStored.tm_mon -= 1;      // struct tm tính tháng từ 0-11
  timeNow.tm_year -= 1900;
  timeNow.tm_mon -= 1;

  time_t t1 = mktime(&timeStored);
  time_t t2 = mktime(&timeNow);

  if (t1 != -1 && t2 != -1) {
    int daysPassed = (t2 - t1) / 86400;  // 86400s = 1 ngày
    if (daysPassed > 0) {
      totalMilliLitres = 0;

      dayCounter += daysPassed;  // 📌 Cộng dồn đúng số ngày

      if (dayCounter >= 30) {
        totalMilliLitres30Days = 0;
        dayCounter = 0;
      }

      saveToNVS();
    }
  }

  preferences.end();  
}

// 📌 Lưu dữ liệu vào NVS mỗi khi cập nhật nước trong ngày
void saveToNVS() {
  preferences.begin("storage", false);
  
  String currentDate = getCurrentDate();
  preferences.putString("date", currentDate);
  preferences.putFloat("value", totalMilliLitres);    
  preferences.putFloat("value30", totalMilliLitres30Days);   
  preferences.putInt("dayCounter", dayCounter);

  preferences.end();  
}


// 📌 Gửi dữ liệu lên Blynk
void sendData() {
  Blynk.virtualWrite(V3, totalMilliLitres / 1000);  
  Blynk.virtualWrite(V4, totalMilliLitres30Days / 1000);

  Serial.print("Today: ");
  Serial.print(totalMilliLitres / 1000);
  Serial.print(" L | Total Days: ");
  Serial.print(dayCounter);
  Serial.print(" | Month Total: ");
  Serial.println(totalMilliLitres30Days / 1000);

  saveToNVS();  
}
//✅____________________________________________________________CAM BIEN NONG DO CHAT TAN______________________________________________________________
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
int chattan_calculation(){
  static unsigned long analogSampleTimepoint = millis();
  if(millis()-analogSampleTimepoint > 40U){     //every 40 milliseconds,read the analog value from the ADC
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);    //read the analog value and store into the buffer
    analogBufferIndex++;
    if(analogBufferIndex == SCOUNT){ 
      analogBufferIndex = 0;
    }
  }   
  
    for(copyIndex=0; copyIndex<SCOUNT; copyIndex++){
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
      
      // read the analog value more stable by the median filtering algorithm, and convert to voltage value
      averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 4096.0;
      
      //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0)); 
      float compensationCoefficient = 1.0+0.02*(temperature-25.0);
      //temperature compensation
      float compensationVoltage=averageVoltage/compensationCoefficient;
      
      //convert voltage value to tds value
      tdsValue=(133.42*compensationVoltage*compensationVoltage*compensationVoltage - 255.86*compensationVoltage*compensationVoltage + 857.39*compensationVoltage)*0.5;
      
      // Serial.print("voltage:");
      // Serial.print(averageVoltage,2);
      // Serial.print("V   ");
      // Serial.print("TDS Value:");
      // Serial.print(tdsValue,0);
      // Serial.println("ppm");
    }

    return tdsValue;
}
int chattan_getValue(){
  int chattan = chattan_calculation();
  if(chattan >= 20 && chattan <= 2419){
  return chattan;
  }
  else{
    return 0;
  }
}
//✅____________________________________________________________________CAM BIEN DO PH_________________________________________________________________
#define SensorPin 35        // Chân Analog trên ESP32
#define Offset 3.0          // Hiệu chỉnh giá trị pH
#define SamplingInterval 20  // Thời gian lấy mẫu (ms)
#define ArrayLength 40       // Số lần lấy mẫu trung bình

int pHArray[ArrayLength]; 
int pHArrayIndex = 0;

float ph_calculation() {
    static unsigned long samplingTime = millis();
    static float pHValue, voltage;

    if (millis() - samplingTime > SamplingInterval) {
        pHArray[pHArrayIndex++] = analogRead(SensorPin);
        if (pHArrayIndex == ArrayLength) pHArrayIndex = 0;

        voltage = averageArray(pHArray, ArrayLength) * (3.3 / 4095.0);
        pHValue = 7.0 + ((2.5 - voltage) * Offset);  // Hiệu chỉnh theo khoảng 2.5V

        samplingTime = millis();
    }

    // Serial.print("Voltage: ");
    // Serial.print(voltage, 3);
    // Serial.print(" V | pH value: ");
    // Serial.println(pHValue, 2);

    return pHValue;
}

float ph_getValue() {
    float ph = ph_calculation();
    if (ph >= 0 && ph <= 14) {  // Điều kiện hợp lệ của pH
        return ph;
    }
    return 0;  // Giá trị không hợp lệ
}

double averageArray(int* arr, int number) {
    if (number <= 0) return 0;

    int min = arr[0], max = arr[0];
    long sum = 0;

    for (int i = 0; i < number; i++) {
        if (arr[i] < min) min = arr[i];
        if (arr[i] > max) max = arr[i];
        sum += arr[i];
    }

    sum -= (min + max);  // Loại bỏ giá trị lớn nhất & nhỏ nhất
    return (double)sum / (number - 2);
}

//✅___________________________________________________________________SET---UP_______________________________________________________________________
//Sử dụng freeRTOS để chạy wifi và xử lí các ngoại vi khác một cách song song
void setup() {
  //Khởi tạo baud rate
  Serial.begin(115200);

  //DEBUG
  esp_reset_reason_t reason = esp_reset_reason();
    Serial.print("ESP Reset Reason: ");
    Serial.println(reason);  // In ra lý do reset


  //Các khai báo cho các ngoại vi khác______________________
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SENSOR, INPUT_PULLUP);

  pulseCount = 0;
  flowRate = 0.0;
  flowMilliLitres = 0;
  totalMilliLitres = 0;
  previousMillis = 0;
  //Thiết lập ngắt ngoài trên chân cảm biến
  attachInterrupt(digitalPinToInterrupt(SENSOR), pulseCounter, FALLING);
  pinMode(TdsSensorPin,INPUT);
  //KHỞI TẠO LCD 16x2
    lcd.begin(16, 2);
  // KẾT NỐI VỚI WIFI VÀ BLYNK APP
    WiFi.begin(ssid, pass);

    Serial.print("Đang kết nối WiFi...");

    //LCD hiển thị trạng thái kết nối với 
    lcd.setCursor(0,0);
    lcd.print("Connecting......");

    int retry = 0;
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);

        Serial.print(".");
        retry++;
       if (retry > 20) {  

        Serial.println("\nKhông thể kết nối WiFi! Resetting...");
        ESP.restart();  // Reset lại ESP32
      }
    }
    //LCD 
    Serial.println("\nWiFi đã kết nối!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());

    lcd.setCursor(0,0);
    lcd.print("Wifi connected");
    lcd.setCursor(0,1);
    lcd.print("Initializing....");
    // Kết nối Blynk
    Blynk.config(BLYNK_AUTH_TOKEN);
    Blynk.connect();

    //BƠM NƯỚC...........
    // Thiết lập chân GPIO32 làm đầu ra
    pinMode(GPIO32_PIN, OUTPUT);
    digitalWrite(GPIO32_PIN, LOW);  // Đảm bảo GPIO32 tắt khi khởi động


    configTime(7 * 3600, 0, "pool.ntp.org");  // Đồng bộ thời gian
    delay(2000);  // Đợi thời gian cập nhật

    readFromNVS();  // Đọc dữ liệu đã lưu trong NVS
    // Gửi dữ liệu ban đầu
    sendData();
  //Tạo các task_________________________________
   // Tạo Task
    //Task thu dữ liệu cảm biến
    xTaskCreatePinnedToCore(CollectData, "CollectData",       4096, NULL, 5, NULL, 1);
    //Task hiển thị LCD
    xTaskCreatePinnedToCore(Task1, "Task1",                   4096, NULL, 2, NULL, 0);
    //Task nhận biết cảm biến lưu lượng
    xTaskCreatePinnedToCore(FlowSensorTask, "FlowSensorTask", 4096, NULL, 1, NULL, 1);
    //Task gửi dữ liệu lên BLYNK
    xTaskCreatePinnedToCore(SensorTask, "SensorTask",         4096, NULL, 3, NULL, 0);
    //Task chạy các lệnh của BLYNK
    xTaskCreatePinnedToCore(BlynkTask,    "BlynkTask",        4096, NULL, 4, NULL, 0);
}

//_________________________________________HÀM CHẠY CÁC TASK_________________________________
int isDirty = 0;
//✅________________________________________________________________________________________________________________________COLLECT_Data_Task
void CollectData(void *pvParameters){
  while(1){
    // Đọc dữ liệu cảm biến
        chattan_value = chattan_getValue();

        ph_value = ph_getValue();
        
        // Kiểm tra chất lượng nước
        isDirty = (chattan_value > 1000 || ph_value < 6 || ph_value > 8.5);  // Kiểm tra chất lượng nước
        digitalWrite(GPIO32_PIN, isDirty ? HIGH : LOW); 
       // Điều khiển bơm thực tế
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
//✅________________________________________________________________________________________________________________________LCD_DISPLAY

void Task1(void *pvParameters) {
    while (1) {
        // In giá trị cảm biến ra Serial Monitor để debug
        // Serial.print("[DEBUG] Chất tan: ");
        // Serial.print(chattan);
        // Serial.print(" ppm  |  pH: ");
        // Serial.println(ph);

        // Kiểm tra nếu cảm biến trả về -1 (lỗi)
        // if (chattan == -1 || ph == -1) {
        //     Serial.println("[ERROR] Cảm biến lỗi, kiểm tra lại kết nối!");
        // }
        // Hiển thị dữ liệu lên LCD
        lcd.clear();
        if (isDirty) {
            lcd.setCursor(0, 0);
            lcd.print("Water Dirty");
            lcd.setCursor(0, 1);
            lcd.print("Draining Water");
        } else {
            lcd.setCursor(0, 0);
            lcd.print("Water is Stable");
            lcd.setCursor(0, 1);
            lcd.print("Day:");
            lcd.print(dayCounter);
            lcd.print("/30");
        }
        vTaskDelay(pdMS_TO_TICKS(3000));

        // Hiển thị thông số cảm biến
        lcd.clear();
        if (chattan_value == -1) {
            lcd.setCursor(0, 0);
            lcd.print("Sensor Error...");
        } else {
            lcd.setCursor(0, 0);
            lcd.print("Solute: ");
            lcd.print(chattan_value);
            lcd.print("ppm");
        }

        if (ph_value == -1) {
            lcd.setCursor(0, 1);
            lcd.print("Sensor Error...");
        } else {
            lcd.setCursor(0, 1);
            lcd.print("pH: ");
            lcd.print(ph_value);
        }
        vTaskDelay(pdMS_TO_TICKS(3000));

        // Thêm delay để tránh vòng lặp quá nhanh
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

//________________________________________________________________________________________________________CẢM BIẾN LƯU LƯỢNG
// ✅Task xử lý lưu lượng nước
// Task đọc cảm biến lưu lượng
void FlowSensorTask(void *pvParameters) {
    while (1) {
        if (!waterFlowing) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        // Gọi hàm tính toán lưu lượng
        float currentFlowRate = calculateFlowRate();

        // Serial.print("Lưu lượng: ");
        // Serial.print(currentFlowRate);
        // Serial.println(" L/min");

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

//_______________________________________________________________________________________________________________________BLYNK
  //✅ Task gửi dữ liệu lên Blynk
void SensorTask(void *pvParameters) {
    while (1) {

        Blynk.virtualWrite(V1, chattan_value); // Chất tan ppm
        Blynk.virtualWrite(V2, ph_value);      // Độ pH
        Blynk.virtualWrite(V3, totalMilliLitres/1000);
        Blynk.virtualWrite(V4, totalMilliLitres30Days / 1000);
        sendData();
        float flow = calculateFlowRate(); // Lấy giá trị từ hàm tính toán
        Blynk.virtualWrite(V0, flow);

        // Serial.print("BLYNK     Chattan: ");
        // Serial.print(chattan);
        // Serial.print("ppm     pH: ");
        // Serial.println(ph);

        //Serial.println("Đã gửi dữ liệu lên Blynk...");
        
        // Chờ 5 giây trước khi gửi tiếp
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
TaskHandle_t BlynkTaskHandle = NULL;

//✅ Hàm task Blynk
void BlynkTask(void *pvParameters) {
  while (true) {
    Blynk.run();  // Gọi Blynk.run() liên tục
    vTaskDelay(10 / portTICK_PERIOD_MS);  // Delay nhỏ để tránh chiếm quá nhiều CPU
  }
}

//__________________________________________________________________L--O--O--P_______________________________________________________________________
// ✅ Loop không làm gì cả
void loop() {
    // Không cần làm gì vì mọi thứ đã chạy trong task
}

