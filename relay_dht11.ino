
#include <DHT.h>

const int relayPin1 = 4; // 릴레이 모듈에 연결된 아두이노 핀 번호
const int relayPin2 = 5;

#define DHTPIN 3     // DHT11 센서가 연결된 핀 
#define DHTTYPE DHT11 // 사용하는 DHT 센서 유형

DHT dht(DHTPIN, DHTTYPE);

const int led_pin = 10;
const int cds_pin = 12;
const int sig_pin = 9;

void setup() {
  Serial.begin(9600);
  dht.begin();
  pinMode(relayPin1, OUTPUT); // 8번 핀을 출력 모드로 설정
  pinMode(relayPin2, OUTPUT);

  digitalWrite(relayPin1, HIGH);
  digitalWrite(relayPin2, HIGH); // 릴레이 모듈을 끔

      
  pinMode(led_pin, OUTPUT);
  pinMode(cds_pin, INPUT);
  pinMode(sig_pin, INPUT);

  digitalWrite(led_pin, LOW);
  delay(2000);
}

unsigned long timer_2000 = 0;
unsigned long timer_1000 = 0;

unsigned long rasp_timer = 0;

void loop() {
  if (rasp_timer > 0)
  {
    if (millis() > rasp_timer + 10000)
    {
      rasp_timer = 0;
      digitalWrite(relayPin1, HIGH);
      digitalWrite(relayPin2, HIGH); // 릴레이 모듈을 켬
      Serial.println("user order >> Relay turned OFF"); // 상태를 시리얼 모니터에 출력  
    }
    else
    {
      //라즈베리파이한테 신호 받으면 FAN 10초간 작동
      digitalWrite(relayPin1, LOW);
      digitalWrite(relayPin2, LOW); // 릴레이 모듈을 끔
      
      
    }
  }

  if (digitalRead(sig_pin)== HIGH)
  {
    rasp_timer = millis();
    Serial.println("user order >> Relay turned ON"); // 상태를 시리얼 모니터에 출력

  }




  if (millis() > timer_1000 + 1000)
  { //1초마다 실행
    if (digitalRead(cds_pin) == HIGH) //어두워 졌을때
    {
      digitalWrite(led_pin, HIGH);
      Serial.println("어두움, LED 자동 켜짐");
    }
    else
    {
      digitalWrite(led_pin, LOW);
      Serial.println("밝음, LED 자동 꺼짐");
    }
    timer_1000 = millis();
  }

  if (millis() > timer_2000 + 2000)
  { // 2초마다 측정
    float humidity = dht.readHumidity(); // 습도 값을 읽음
    float temperature = dht.readTemperature(); // 온도 값을 읽음 (섭씨)

    // 센서에서 온도 및 습도를 읽지 못한 경우 NaN 값을 반환하므로 확인
    if (isnan(humidity) || isnan(temperature)) {
      Serial.println("Failed to read from DHT sensor!");
      return;
    }

    // 온도 및 습도를 시리얼 모니터에 출력
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.print(" %\t");
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" °C");

    // 습도가 60 이상인 경우 릴레이 모듈을 켬
    if (humidity >= 60) {
      digitalWrite(relayPin1, LOW);
      digitalWrite(relayPin2, LOW); // 릴레이 모듈을 끔
      Serial.println("Relay turned ON"); // 상태를 시리얼 모니터에 출력
    }
    //온도 40도 이상 릴레이 모듈을 켬
    else if (temperature >= 40) {
      digitalWrite(relayPin1, LOW);
      digitalWrite(relayPin2, LOW); // 릴레이 모듈을 끔
      Serial.println("Relay turned ON");
    }
    else {
      digitalWrite(relayPin1, HIGH);
      digitalWrite(relayPin2, HIGH); // 릴레이 모듈을 켬
      Serial.println("Relay turned OFF"); // 상태를 시리얼 모니터에 출력
    }
    timer_2000 = millis();
  }


}
