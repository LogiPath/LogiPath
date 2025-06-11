#include <Arduino.h>

#include <SPI.h>

#include <deprecated.h>
#include <MFRC522.h>
#include <MFRC522Extended.h>
#include <require_cpp11.h>

#include <Servo.h> 
#include <EEPROM.h>

#define START_ADDRESS 240

#define SERVO_DOWN  90-50 // Down 위치, 각 로봇에 맞도록 [-50 .. -10] 범위에서 조정하세요.
#define SERVO_UP    180-60  // Up 위치 (떨림 방지)
#define SERVO_DEF   SERVO_DOWN 


#define LINEDETECT_THRESHOLD_MIN 730  //black
#define BLANKDETECT_THERSHOLD_MAX 500


#define FORWARD   0 // 전진 방향
#define BACKWARD  1 // 후진 방향



struct pair {
    int first;
    int second;
};

struct pPair {
    double first;
    pair second;
};


enum {
        eInitialPosition = 0, 
        eWareHousePosition, 
        eTargetPosition
    } currentPosition;


static const pair START_POINT1 = {3, 0};
static const pair START_POINT2 = {5, 0};
static const pair WARE_HOUSE_POINT1 = {0, 3};
static const pair WARE_HOUSE_POINT2 = {0, 4};

static const pair SEOUL = {0, 7};
static const pair INCHEON= { 1, 7 };
static const pair SEJONG = { 2, 7 };
static const pair DAEJEON = {3, 7 };

static const pair DAEGU= { 4, 7 };
static const pair GWANGJU = { 5, 7 };
static const pair CHUNCHEON = {6, 7 };
static const pair JEJU = { 7, 7 };

  enum APP_STATE{
      STATE_NONE=0,
      STATE_RFIDREAD,
      STATE_CMDLIST,
      STATE_LINECOUNTER,
      STATE_LINETRACER,
      STATE_TURNRIGHT,
      STATE_TURNLEFT,
      STATE_TURNHALF 
  } state;


  class Controller{
private:
  uint8_t RFIDReaderSlaveSelect = 2;
  uint8_t pinBuzzer = 3;
  uint8_t RFIDReaderReset = 4;
  uint8_t RightWheelPWM = 6;
  uint8_t LeftWheelPWM = 5;
  uint8_t RightWheelDir = 8;
  uint8_t LeftWheelDir = 7;
  uint8_t LiftServo = 9;
  uint8_t UserButton = A3;
  uint8_t SensorFrontRight = A2;
  uint8_t SensorFrontLeft = A1;
  uint8_t SensorFrontCenter = A0;
  uint8_t SensorBottomRight = A7;
  uint8_t SensorBottomLeft = A6;

  uint16_t nLineCounter = 0;
  uint16_t targetLineCount = 0;

  int16_t _rightWhite;
  int16_t _leftWhite;
  int16_t _rightBlack;
  int16_t _leftBlack;
  float _motorCalibR;
  float _motorCalibL;


  int Power = 105;
  //int Power_p = 100;
  unsigned long lastRFIDTime = 0;  // 마지막 RFID 읽은 시간

  const String s_strRFIDUidForStart = String("24660774"); // RFID value
   
  
  MFRC522 mfrc522;
  Servo servo;  


  String  strRFID;


  bool isBusy = false;

  
  int MAX = 64;

  double INF = 1e9 + 7;

const int dx1[4] = { 0, 0, 1, -1 };
const int dy1[4] = { -1, 1, 0, 0 };




	int ROW, COL;

	int graph[8][8]={
    {0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0},
	};

	pair ddir = { 0,-1 };



	pair dir[4] = {
		{0, - 1}, // right
		{-1, 0}, // down
		{0, 1}, // left
		{1, 0}, // up
	}; 

public: 
  Controller():mfrc522(RFIDReaderSlaveSelect, RFIDReaderReset)
  {}

  void init();


  void RunOnce() ;

  void ProcessRFIDRead();



  //Lift Controller
  void  LifterUp();


  void  LifterDown();

  // RFID Read
  bool RFIDRead();

  // Functions for Drive

  bool LineTracer(uint16_t nTargetLineCounter);


  void DoLineTrace(uint16_t targetCount) ;


  void LineTrace();

  void ResetLineCounter();

  void Move();

  void  drive(int dir1, int power1, int dir2, int power2);

  void  Forward( int power )  ;

  void  Backward( int power)  ;

  void  TurnLeft( int power ) ;

  void  TurnRight( int power ) ;

  void Stop() ;

  void TurnHalf();



  void PivotTurnLeft() ;

  // Half Pivot Turn for JellibiAGV
  void PivotTurnRight();


  // IR Sensor Value
  int16_t GetLeft() ;

  int16_t GetRight() ;

  void readData();

  bool isThere(String sRegion, int regionNum);





  /////////////////////
  bool visted[8][8];
  pair queue[64];
  int front = 0, rear = 0;

  pair path[64];
  int pIdx = 0;
  int pLen = 0;

  pair cellData[8][8];

  bool flag = false;


	void tracePath(pair cell[8][8], pair dst);
	void BFS(pair src, pair dst);


};

  Controller ctrlr;

void setup() {
    Serial.begin(9600);
    
  ctrlr.init();
  state = STATE_RFIDREAD;

  
  if(Serial){
    Serial.println("init");
  }


}

void loop() {
  // put your main code here, to run repeatedly:
  ctrlr.RunOnce();
}



void Controller::init(){
  pinMode(RightWheelDir, OUTPUT);
  pinMode(LeftWheelDir, OUTPUT);
  pinMode(RightWheelPWM, OUTPUT);
  pinMode(LeftWheelPWM, OUTPUT);
  pinMode(SensorBottomRight, INPUT);
  pinMode(SensorBottomLeft, INPUT);
  
  nLineCounter = 0;
  targetLineCount = 0;
  
   readData();
  Serial.println(_motorCalibR);
  


  SPI.begin();
  mfrc522.PCD_Init();


  LifterUp(); 

  delay(300);

  LifterDown();

  state = STATE_NONE;
  
  //PlayMelody();


}


void Controller::RunOnce() 
{
    //Sensing();
    if (state == STATE_NONE) {
    } else if (state == STATE_RFIDREAD) {
      
        ProcessRFIDRead();
    }
}

//case2에서는 라인을 좀 돌아서 오게, case1은 무조건 빠르게 지나가도록
void Controller::ProcessRFIDRead()
{ 
    if (RFIDRead()) {
      
      isBusy = true;
      mfrc522.PCD_AntennaOff(); 
      
      switch(currentPosition){
      case 0:
        if (strRFID.compareTo("24660774") == 0) { // D->4
          DoLineTrace(1);
          BFS(START_POINT1, WARE_HOUSE_POINT1);
        } else if (strRFID.compareTo("E45B2674") == 0) { // F->5
          DoLineTrace(1);
          BFS(START_POINT2, WARE_HOUSE_POINT2);
        }
        Stop();
        TurnHalf();
        LifterUp();
        currentPosition = eWareHousePosition;
        mfrc522.PCD_AntennaOn();
        break;

      case 1:
        if (strRFID.compareTo("84CA4874") == 0) { // 4->서울
          BFS(WARE_HOUSE_POINT1, SEOUL);
        } else if (strRFID.compareTo("A4263674") == 0) { // 4->인천
          BFS(WARE_HOUSE_POINT1, INCHEON);
        } else if (strRFID.compareTo("446CF4BB") == 0 || strRFID.compareTo("5468E6BB") == 0) { // 4->세종
          BFS(WARE_HOUSE_POINT1, SEJONG);
        } else if (strRFID.compareTo("CED01AAC") == 0) { // 4->대전
          BFS(WARE_HOUSE_POINT1, DAEJEON);
        } else if (strRFID.compareTo("64364D74") == 0 || strRFID.compareTo("74200C74") == 0) { // 5->대구
          BFS(WARE_HOUSE_POINT2, DAEGU);
        } else if (strRFID.compareTo("44583C74") == 0) { // 5->광주
          BFS(WARE_HOUSE_POINT2, GWANGJU);
        } else if (strRFID.compareTo("646AF773") == 0) { // 5->부산
          BFS(WARE_HOUSE_POINT2, JEJU);
        } else if (strRFID.compareTo("F4D30474") == 0) { // 5->제주
          BFS(WARE_HOUSE_POINT2, JEJU);
        } else {
          Serial.println("매칭되는 RFID가 없습니다.");
        }
        mfrc522.PCD_AntennaOn();
        state = STATE_RFIDREAD;
        LifterDown();
        Stop();
        delay(700);
        TurnHalf();
        currentPosition = eTargetPosition;
        break;

      case 2:
        if (strRFID.compareTo("84CA4874") == 0) { // 서울->4
          BFS(SEOUL, WARE_HOUSE_POINT1);
        } else if (strRFID.compareTo("A4263674") == 0) { // 인천->4
          BFS(INCHEON, WARE_HOUSE_POINT1);
        } else if (strRFID.compareTo("446CF4BB") == 0 || strRFID.compareTo("5468E6BB") == 0) { // 세종->4
          BFS(SEJONG, WARE_HOUSE_POINT1);
        } else if (strRFID.compareTo("CED01AAC") == 0) { // 대전->4
          BFS(DAEJEON, WARE_HOUSE_POINT1);
        } else if (strRFID.compareTo("64364D74") == 0 || strRFID.compareTo("74200C74") == 0) { // 대구->5
          BFS(DAEGU, WARE_HOUSE_POINT2);
        } else if (strRFID.compareTo("44583C74") == 0) { // 광주->5
          BFS(GWANGJU, WARE_HOUSE_POINT2);
        } else if (strRFID.compareTo("646AF773") == 0) { // 부산->5
          BFS(JEJU, WARE_HOUSE_POINT2);
        } else if (strRFID.compareTo("F4D30474") == 0) { // 제주->5
          BFS(JEJU, WARE_HOUSE_POINT2);
        } else {
          Serial.println("매칭되는 RFID가 없습니다.");
        }
        Stop();
        TurnHalf();
        LifterUp();
        currentPosition = eWareHousePosition;
        mfrc522.PCD_AntennaOn();
        break;
      }

      isBusy = false;
    }
}




//Lift Controller
void  Controller::LifterUp()
{
  servo.attach( LiftServo ); // 서보 연결
  delay( 10 );

  servo.write( SERVO_UP );  // 리프터 위로 올림
  delay( 300 );

  servo.detach(); // 서보 연결 해제
  delay( 10 );  
}


void  Controller::LifterDown()
{
  servo.attach( LiftServo ); // 서보 연결
  delay( 10 );
  
  servo.write( SERVO_DOWN ); // 리프터 아래로 내림
  delay( 300 );

  servo.detach(); // 서보 연결 해제
  delay( 10 );  
}

// RFID Read
bool Controller::RFIDRead()
{
  if (isBusy) return false;

  // ▶ RFID 중복 인식 방지용: 최근 2초 이내에 인식된 태그는 무시
  if (millis() - lastRFIDTime < 5000) return false;

  if (mfrc522.PICC_IsNewCardPresent()) {
    if (mfrc522.PICC_ReadCardSerial()) {
      strRFID = "";
      for (byte i = 0; i < mfrc522.uid.size; i++) {
        strRFID.concat(String(mfrc522.uid.uidByte[i] < 0x10 ? "0" : ""));
        strRFID.concat(String(mfrc522.uid.uidByte[i], HEX));
      }

      strRFID.toUpperCase();
      lastRFIDTime = millis(); // ▶ 정상 인식되었으니 시간 기록

      Serial.print("\nRFID 인식됨: [");
      Serial.print(strRFID);
      Serial.println("]");

      return true;
    }
  }
  return false;
}


// Functions for Drive

bool Controller::LineTracer(uint16_t nTargetLineCounter)
{   
    LineTrace();
    if (nTargetLineCounter == nLineCounter) {
        if (Serial) {
            Serial.println("LineCount Finished");
        }

        Stop();
        ResetLineCounter();
        
        return true;
    }
    return false;
}


void Controller::DoLineTrace(uint16_t targetCount) 
{ 
    while(!LineTracer(targetCount)){}
}


void Controller::LineTrace(){
  static uint8_t bSignalHigh = 0;

  int _rightValue = GetRight();
  int _leftValue = GetLeft();


  if(_rightValue > LINEDETECT_THRESHOLD_MIN && _leftValue > LINEDETECT_THRESHOLD_MIN){
    
      if(bSignalHigh == 0){
        nLineCounter++;
        if (Serial) {
              Serial.println(String("LINE!!! :")+ String(nLineCounter));
        }
        bSignalHigh = 1;
      }
      Forward(Power);
      delay(50); //초기값 50
  }else if (_leftValue > LINEDETECT_THRESHOLD_MIN)      // 왼쪽(IR1)만 검정
  {
    TurnLeft(currentPosition == eWareHousePosition ? Power-20  : Power); // 좌회전
  }
  else if (_rightValue > LINEDETECT_THRESHOLD_MIN) // 오른쪽(IR2)만 검정
  {
    TurnRight(currentPosition == eWareHousePosition ? Power-20 : Power); // 우회전
  }
  else                        // 양쪽 모두 흰색
  {
    if(bSignalHigh){
      bSignalHigh = 0;
    }
    Forward(Power); // 전진
    
  }
}

void Controller::ResetLineCounter()
{
    nLineCounter = 0;
}

void Controller::Move() // 모터 속도를 120 위로 증가 시키면 정지상태에서는 모터 동작 X, 따라서 더 낮은 속도로 바퀴를 움직여 210의 속도로 동작하게 함.
{
    Stop();
    delay(10);
    analogWrite( LeftWheelPWM, 105);
    analogWrite( RightWheelPWM, 105);//초기값 100
    delay(100);

}

void  Controller::drive(int dir1, int power1, int dir2, int power2)
{
  boolean dirHighLow1, dirHighLow2;

  if(dir1 == FORWARD)  // 1번(왼쪽)모터 방향
    dirHighLow1 = HIGH;
  else // BACKWARD
    dirHighLow1 = LOW;
  
  if(dir2 == FORWARD)  // 2번(오른쪽)모터
    dirHighLow2 = LOW;
  else // BACKWARD
    dirHighLow2 = HIGH;

  digitalWrite(LeftWheelDir, dirHighLow1);
  analogWrite(LeftWheelPWM, power1 * _motorCalibL);

  digitalWrite(RightWheelDir, dirHighLow2);
  analogWrite(RightWheelPWM, power2 * _motorCalibR);
}

void  Controller::Forward( int power )  // 전진
{
  //if(팔레트 인식)
  drive(FORWARD, power, FORWARD, power);
}

void  Controller::Backward( int power )  // 후진
{
  drive(BACKWARD, power, BACKWARD, power);
}

void  Controller::TurnLeft( int power )  // 좌회전
{
  drive(BACKWARD, power, FORWARD, power);
}

void  Controller::TurnRight( int power )  // 우회전
{
  drive(FORWARD, power, BACKWARD, power);
}

void Controller::Stop()  // 정지
{
analogWrite(LeftWheelPWM,  0);
analogWrite(RightWheelPWM, 0);
}

void Controller::TurnHalf(){
  drive(BACKWARD, 80, FORWARD, 80);
  delay(50);
  drive(BACKWARD, 170, FORWARD, 170); // 초기값 210
  delay(450);
  // delay(currentPosition == eWareHousePosition ? 355 : 360);
  Stop();
}



void Controller::PivotTurnLeft() 
{
  if (Serial) Serial.println("Enter Pivot turn Left");
  analogWrite( LeftWheelPWM, 0);
  analogWrite( RightWheelPWM, 0);
  delay(10);
  digitalWrite(LeftWheelDir, 0);
  digitalWrite(RightWheelDir, 0);

  
  Move();
    delay(10);
  analogWrite( LeftWheelPWM, 170);
  analogWrite( RightWheelPWM, 170);
  delay(50); 
  analogWrite( LeftWheelPWM, 90 * _motorCalibL);
  analogWrite( RightWheelPWM, 170 * _motorCalibR);
  delay(currentPosition == eWareHousePosition ? 180 : 150);

  analogWrite( LeftWheelPWM, 0);
  analogWrite( RightWheelPWM, 0);

  if (Serial) Serial.println("Leave Pivot turn Left");
}

// Half Pivot Turn for JellibiAGV
void Controller::PivotTurnRight()
{
  if (Serial) Serial.println("Enter Pivot turn Right");
  analogWrite( LeftWheelPWM, 0);
  analogWrite( RightWheelPWM, 0);
  delay(10);
  digitalWrite(LeftWheelDir, 1);
  digitalWrite(RightWheelDir, 1);

  
  Move();
  delay(10);
  analogWrite( LeftWheelPWM, 170 * _motorCalibL);
  analogWrite( RightWheelPWM, 170 * _motorCalibR);
  delay(50); 
  analogWrite( LeftWheelPWM, 170 * _motorCalibL);
  analogWrite( RightWheelPWM, 90 * _motorCalibR);//초기값 95
  delay(currentPosition == eWareHousePosition ? 180 : 150);

  analogWrite( LeftWheelPWM, 0);
  analogWrite( RightWheelPWM, 0);
  if (Serial) Serial.println("Leave Pivot turn Right");
}


// IR Sensor Value 
int16_t Controller::GetLeft() 
{   
    return analogRead(SensorBottomLeft); //EEPROM.get(240, _leftValue);
}

int16_t Controller::GetRight() 
{
    return analogRead(SensorBottomRight);//EEPROM.get(242, _rightValue);
}

void Controller::readData(){
  int address = START_ADDRESS;

  EEPROM.get( address, _rightWhite );
  address += 2;

  EEPROM.get( address, _leftWhite );
  address += 2;


  EEPROM.get( address, _rightBlack );
  address += 2;

  EEPROM.get( address, _leftBlack );
  address += 2;


  EEPROM.get( address, _motorCalibR );
  address += 4;

  EEPROM.get( address, _motorCalibL );
  address += 4; 
}




//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



void Controller::tracePath(pair cell[8][8], pair dst) {
  pair s[64];
  int sIdx = 0;
  int x = dst.first;
  int y = dst.second;

  s[sIdx] = { x, y };
  sIdx++;
  while (!(cell[x][y].first == x && cell[x][y].second == y)) {
    int tempx = cell[x][y].first;
    int tempy = cell[x][y].second;
    x = tempx;
    y = tempy;
    s[sIdx] = { x, y };
    sIdx++;
  }

  while (sIdx != 0) {
    pair temp = s[sIdx - 1];
    sIdx--;
    Serial.print(temp.first);
    Serial.print(", ");
    Serial.println(temp.second);
    Serial.print(s[sIdx - 1].first);
    Serial.print(", ");
    Serial.println(s[sIdx - 1].second);
    if (sIdx != 0) {
      pair dDir = { temp.first - s[sIdx - 1].first , temp.second - s[sIdx - 1].second };
      int d, dD;
      for (int i = 0; i < 4; i++) {
        if (ddir.first == dir[i].first && ddir.second == dir[i].second) d = i;
        if (dDir.first == dir[i].first && dDir.second == dir[i].second) dD = i;
      }
      int k = (d - dD + 4) % 4;
      if (k == 0) {
        DoLineTrace(1);
        Serial.print("앞");
      } else if (k == 1) {
        PivotTurnLeft();
        Serial.print("왼");
        DoLineTrace(1);
      } else if (k == 2) {
        TurnHalf();
        Serial.print("반");
        DoLineTrace(1);
      } else if (k == 3) {
        PivotTurnRight();
        Serial.print("우");
        DoLineTrace(1);
      }
      ddir = dDir;
    }
  }
}

void Controller::BFS(pair src, pair dst) {
  int x = src.first, y = src.second;
  visted[x][y] = true;
  queue[rear++] = { x, y };
  cellData[x][y] = { x, y };
  while (!flag) {
    x = queue[front % 64].first;
    y = queue[front % 64].second;
    front++;
    for (int i = 0; i < 4; i++) {
      int dx = x + dir[i].first;
      int dy = y + dir[i].second;
      if ((dx >= 0 && dx < 8) && (dy >= 0 && dy < 8)) {
        if (dx == dst.first && dy == dst.second) {
          cellData[dx][dy] = { x, y };
          tracePath(cellData, dst);
          flag = true;
        } else if (!visted[dx][dy] && graph[x][y] == 0) {
          visted[dx][dy] = true;
          queue[rear++ % 64] = { dx, dy };
          cellData[dx][dy] = { x, y };
        }
      }
    }
  }
}