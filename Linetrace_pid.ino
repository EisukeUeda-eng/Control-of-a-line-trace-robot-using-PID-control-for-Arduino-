#include <DRV8835MotorShield.h>


DRV8835MotorShield motors;

#define Kp 0.4
#define Ki 0.004
#define Kd 0.4

void setup() {
  Serial.begin(9600);
}

// sensorLeftValue, sensorCenterValue, sensorRightValue はそれぞれ左, 中央, 右のセンサが読み取った値を格納する変数

int sensorLeftValue;
int sensorCenterValue;
int sensorRightValue;

static int Integral = 0; //Integral は I 制御における積分値を表す変数
     
static int diff[2] = {0}; //diff[2]は現在の偏差と過去の偏差を記録しておく配列

void loop() {
  
  
  int targetCenterValue = 570;

  // Rightspeed と Leftspeed はそれぞれ左右のモータの操作量

  static float Rightspeed;
  static float Leftspeed;

  //Speedp, Speedi, Speedd は PID 制御による各制御の操作量

  float Speedp;
  float Speedi;
  float Speedd;

  //各センサからの値の読み取り
  sensorLeftValue = analogRead(A2);
  sensorCenterValue = analogRead(A1);
  sensorRightValue = analogRead(A0);



  //交差点を走行するアルゴリズム
  //左, 中央, 右の センサが全て黑色を読み込んだ時に直進する

  if(sensorCenterValue > 800 && sensorLeftValue > 800 && sensorRightValue > 800) {
    motors.setM1Speed(200);
    motors.setM2Speed(200);
    delay(10);
  }

  diff[0] = diff[1];                   //D制御における微分の表現
  diff[1] = targetCenterValue - (sensorCenterValue-50);  //左右のセンサが共に黑色を読み込んだ時の値の差, および共に白色を読み込んだ時の差を補正するため

  Integral += diff[1];               　 //I制御における積分の表現

  Speedp = Kp * diff[1];             　 //P制御
  Speedi = Ki * Integral;　　　　　　　　 //I制御
  Speedd = Kd * (diff[1] - diff[0]);　　//D制御

  //左右のモータそれぞれへのPID制御

  Rightspeed = 250 - (Speedp + Speedi + Speedd);
  Leftspeed = 250 + (Speedp + Speedi + Speedd);

  motors.setM1Speed(Leftspeed);
  motors.setM2Speed(Rightspeed);


  //直角を走行するアルゴリズム
  //右回りの直角はPID制御で走行することができたので, 左回りの直角を走行する際のプログラムのみをコーディングした
  //左回りの直角を走行するアルゴリズムは左, 中央のセンサが黑色, 右のセンサが白色を読み込んだ時に少し前進してから, 右側のモータのみを動作させて直角を曲がる

  if(sensorCenterValue > 1000 && sensorLeftValue > 1000 && sensorRightValue < 750) {
    motors.setM1Speed(200);
    motors.setM2Speed(200);
    delay(10);

    motors.setM1Speed(0);
    motors.setM2Speed(200);
    delay(2250);
  }

  //各センサが読み込んだ値の表示

  Serial.print(sensorLeftValue);
  Serial.print(",");
  Serial.print(sensorCenterValue);
  Serial.print(",");
  Serial.print(sensorRightValue);
  Serial.println();

}
