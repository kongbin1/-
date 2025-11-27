

// 1. 값 정의
#include "Debug.h"
#include "Pixy.h"
#include "Manipulator.h"
#include "Mobilebase.h"
#include "PSD.h"
#include "Gripper.h"

#include <Adafruit_NeoPixel.h>

#define RED_LED_PIN 4
#define RGB_LED_PIN 3
#define NUM_PIXELS 1

Adafruit_NeoPixel pixels(NUM_PIXELS, RGB_LED_PIN, NEO_GRBW + NEO_KHZ800);

// 픽시(Pixy) 카메라가 블록을 인식하고 정렬하는 데 사용하는 핵심 목표 좌표

// 픽시 카메라의 x축 중앙 정렬 목표값.
#define PIXY2_X_SETPOINT 169

// 픽시 카메라의 y출 기준선. 위층, 아래층 블럭을 집기 전 이것으로 구분함.
#define PIXY2_Y_SETPOINT 100

// y값 오차 허용 범위.
#define PIXY2_Y_TOLERANCE 1

// 픽시에서 x축이 정렬되었다고 판단하는 오차 허용 범위
// 172 +- 4 => 168~176
#define PIXY_TOLERANCE 2 // 2에서 수정

// PSD센서 정렬이 완료되었다고 판단하는 오차 허용 범위(y축, 회전)
// ✨ [수정 1] Dead Zone 탈출을 위해 허용 오차를 약간 넓힘 (15 -> 20)
#define PSD_TOLERANCE 20

// 픽시 x오차에 반응하는 민감도 혹은 속도 증폭률
#define PIXY_CONTROL_RATIO 0.15 // 0.1에서 수정


// PSD 센서(y축, 회전) 오차에 반응하는 민감도 혹은 속도 증폭률
#define PSD_CONTROL_RATIO 0.1

// 기본 주행 속도
const int32_t ALIGNMENT_DRIVING_SPEED = 250;

// 미션구간 속도
const int32_t MISSION_DRIVE_SPEED = 160;

// 로봇의 틀어짐을 보정할 때 사용할 회전 속도
const int32_t ALIGNMENT_ROTATING_SPEED = 100;

// front_left PSD 센서의 보정값
#define PSD_FL_CORRECTION -5

// 값을 올리면 더 가깝게, 값을 내리면 더 멀리 작동
#define OBSTACLE_FRONT_PSD_SET_POINT 500
#define OBSTACLE_LEFT_PSD_SET_POINT 450

#define BUTTON_PIN 40

// 미션 때의 PSD
// ✨ [수정 2] 물리적 충돌 및 Dead Zone 회피를 위해 SET_POINT를 멀리(255 -> 260) 설정
#define MISSION_FRONT_PSD_SET_POINT 260

#define MISSION_LEFT_PSD_SET_POINT 608
// 픽시
Pixy2SPI_SS pixy;

/*다이나믹셀 모터를 제어하기 위한 객체 : 이름은 dxl*/
Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// 별명으로 자세를 불러오기 위해 eum을 씀
enum ManipulatorPoseID {
  INITIAL_AND_MISSION_INSTRUCTION = 1, // 1번자세
  STORAGE, // 팔을 접은 기본 자세
  PRE_GRIP_UPPER_BLOCK, // 위층 블록을 잡기 직전 자세
  GRIP_UPPER_BLOCK, // 위층 블록을 잡는 자세
  PRE_GRIP_LOWER_BLOCK, // 아래층 블록을 집기 직전 자세
  GRIP_LOWER_BLOCK // 아래층 블록을 잡는 자세
};

// 로봇 팔이 블록을 놓는 자세가 EEPROM에 6번부터 저장되어 있다 라는 시작번호...
#define MANIPULATOR_MISSION_FULFILLMENT_POSE_START_ID 6

// 블록을 찾거나 못찾거나 했을 때의 변수
bool haveFoundBlock = false;

// 미션 수행 총 변수
uint8_t MISSION_BLOCK_CNT = 0; // 실제 미션 칸 수 -> 일단 0으로 처리함

// 블럭 시그니처 맵을 0으로 초기화하는 상수?
const uint8_t BLOCK_SIG_MAP = 0b00000000;

// 찾을 블럭의 시처니처 맵 변수
uint8_t targetBlockSigmap = 0x01;

const uint8_t MAX_BLOCK_CNT = 6; // MAX값을 6칸으로 정의

// 픽시에게 '몇 번째 미션에서 어떤 색을 찾아야 할지' 슌소대로 알려주는 배열
uint8_t targetBlockSigmaps[MAX_BLOCK_CNT] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20}; // 빨~보까지 순서임

// 찾을 블럭을 어느 칸에 놓을지 정하는 자세
// uint8_t goalPositions[MAX_BLOCK_CNT] = {3, 4, 1, 7, 2, 8}; // 미리 입력할 것
// 빨=3 주=4 노=1 초=7 


void setup(){
  // 이동변수
  const float ADJUST_DISTANCE_MM = 20.0;

  // 목표 칸 번호를 저장할 변수
  // uint8_t targetSlot;

  // 이동변수 실행여부 판단 변수
  bool didMove = false;

  // 시간 변수
  unsigned long missionStartTime;

  pinMode(RED_LED_PIN, OUTPUT);
  digitalWrite(RED_LED_PIN, LOW);
  pixels.begin();
  pixels.setBrightness(100);
  pixels.show();

  // Debug.cpp에 있는 함수를 호출하여 PC와 115200속도로 시리얼 모니터 통신 시작
  InitDebug();

  // Motor.cpp에 있는 함수를 호출하여 모터와 1,000,000속도로 통신을 시작
  InitMotorCommunication(dxl);

  // Manipulator.cpp에 있는 함수를 호출하여 팔 모터 5~8번을 초기화
  // 만약 실패하면? 프로그램 멈춤
  while(!InitManipulator(dxl)) {}

  // Mobilebase.cpp에 있는 함수를 호출하여 바퀴 모터 1~4번을 초기화
  // 초기화 실패하면 프로그램 멈춤
  while(!InitMobilebase(dxl)) {}

  //PSD.cpp에 있는 함수를 호출하여, 'PSD 거리 센서'가 연결된 아날로그 핀(A0~A3)을 준비
  InitPSD();

  // Pixy.cpp에 있는 함수를 호출하여, '픽시 카메라'와 SPI 통신을 시작하고 초기화합니다
  InitPixy(pixy);

  // 조명 끔, 그리퍼 엶
  pixy.setLamp(0, 0);
  OpenGripper(pixy);

  // 위 초기화들이 모두 끝난 디버깅프린트
  DEBUG_SERIAL.println("초기화 모두 완료. LED 작동 시작");

  // LED 깜빡이기. red 3번, green 1번
  for (int i = 0; i < 3; i++) {
    pixels.setPixelColor(0, pixels.Color(255, 0, 0));
    pixels.show(); 
    delay(500);
    pixels.setPixelColor(0, pixels.Color(0, 0, 0));
    pixels.show();
    delay(500);

  }
  pixels.setPixelColor(0, pixels.Color(0, 255, 0));
  pixels.show();
  delay(500);
  pixels.setPixelColor(0, pixels.Color(0, 0, 0));
  pixels.show();
  delay(500);

  // 근데 생각해보니 얘는 4가 들어가면 안되는 것 아님? 오류아닌가?
  // targetSlot = goalPositions[MISSION_BLOCK_CNT]; // 넣기

  DEBUG_SERIAL.println("LED작동 완료. 주행 시작");

  // 시작 측정 시작
  missionStartTime = millis();

  // 1번 자세로, 1.4초의 시간을 들여서 1번 모터를 -90.0도 회전시켜서
  RunManipulatorPoseWithPoseDataInEEPROM(dxl, INITIAL_AND_MISSION_INSTRUCTION, 1400, -90.0);

  // 바퀴 4개의 속도를 모두 0으로 지정.
  SetMobileGoalVelocityForSyncWrite( dxl, 0, 0, 0, 0 );
  delay(500);

  // 전방_좌측, 전방_우측 PSD 값을 받아오기 위한 변수를 선언
  int16_t flPSDValue1, frPSDValue1;

  // 목표 지점에 도착했다는 신호가 올 때까지 이 안의 동작을 무한 반복
  while(1) {
    // 1. 실제로 앞 PSD 센서 2개의 값을 읽어옴.
    GetValueFromFrontPSDSensors(&flPSDValue1, &frPSDValue1);

    // 2. 로봇이 움직이는 함수를 호출
    if (!DriveForwardUntilDistanceWithTwoSensors(
          //
          dxl,

          // 현재 전방_왼쪽 값 - 목표값(525) = 왼쪽 오차
          flPSDValue1+PSD_FL_CORRECTION - OBSTACLE_FRONT_PSD_SET_POINT,

          // 현재 전방_오른쪽 값 - 목표값(525) = 오른쪽 오차
          frPSDValue1 - OBSTACLE_FRONT_PSD_SET_POINT,

          // 이 오차 2개가 모두 설정된 PSD_TOLERANCE안으로 들어오면 성공함
          PSD_TOLERANCE,

          // 움직이는 속도
          ALIGNMENT_DRIVING_SPEED)
      ) break;
  }
  // 4개 바퀴 모터 완전 정지
  SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);
  // 램프 킴
  pixy.setLamp(1, 1);

  // 좌측 PSD 값을 저장할 변수
  int16_t slPSDValue2;

  // 목표 지점에 도착완료 신호가 올 때까지 동작
  while(1) {
    // PSD값 가져옴
    GetValueFromSideLeftPSDSensor(&slPSDValue2);
    if (!DriveUntilDistanceWithOneSensor(
      dxl,

      // (현재 왼쪽 값) - (목표값 470) = 오차
      slPSDValue2 - OBSTACLE_LEFT_PSD_SET_POINT,
      // 이 오차가  설정된 값이 되면 성공
      PSD_TOLERANCE,
      // 왼쪽으로 이동
      DRIVE_DIRECTION_LEFT,
      // 모터 속도
      ALIGNMENT_DRIVING_SPEED))
    break;
  }

  // 뒤로 이동시킬 거리 변수
  const float DISTANCE_MM = 100.0;

  // 바퀴모터 4개를  정해진 거리만큼 이동하는 '위치 제어 모드'로 변경
  ChangeMobilebaseMode2ExtendedPositionControlWithTimeBasedProfileMode(dxl);

  // 10cm만큼 뒤로 이동시킴
  DriveDistanceAndMmPerSecAndDirection(dxl, DISTANCE_MM, DRIVE_DIRECTION_BACKWARD);
  // 완료되면 끝
  while(!CheckIfMobilebaseIsInPosition(dxl)) {}

  delay(500);
  // 픽시에게 눈에 모이는 색깔을 찾아 리스트에 저장시킴
  pixy.ccc.getBlocks(true, CCC_SIG_ALL);
  delay(50);
  for(int i = 0; i < pixy.ccc.numBlocks; i++){
    MISSION_BLOCK_CNT++;
    pixy.ccc.blocks[i].print();
  }

  Serial.print("실제 미션 개수 : ");
  Serial.println(MISSION_BLOCK_CNT);

  // 블럭이 1개 이상이라면 아래 실행
  if (pixy.ccc.numBlocks) {
    // ================ [Insertion Sort Logic Start] ================
    // x값이 작은 순서대로 정렬시킴
    for (int i = 1; i < pixy.ccc.numBlocks; i++) {
      Block keyBlock = pixy.ccc.blocks[i];
      int j = i - 1;
      while (j >= 0 && pixy.ccc.blocks[j].m_x > keyBlock.m_x) {
        pixy.ccc.blocks[j + 1] = pixy.ccc.blocks[j];
        j = j - 1;
      }
      pixy.ccc.blocks[j + 1] = keyBlock;
    } // 정렬 끝

    // 인식된 블럭 개수 디버그
    DEBUG_SERIAL.print("인식된 블럭 개수 : ");
    DEBUG_SERIAL.println(pixy.ccc.numBlocks);

    int blocksToSave = min(pixy.ccc.numBlocks, MISSION_BLOCK_CNT);
    // DEBUG_SERIAL.println("Blocks sorted by X-coordinate (left-to-right):");

    for (int i = 0; i < blocksToSave; i++) {
      targetBlockSigmaps[i] = (1 << (pixy.ccc.blocks[i].m_signature - 1));
    }

    // 카메라가 5개를 봤어도 최대 미션은 4개이기 때문에 4개로 저장시켜버림

    // 아래에 추가적으로 디버그 더 넣어도 됨
  }

  // 팔 자세를 1번 자세로(앞 보기임)
  RunManipulatorPoseWithPoseDataInEEPROM(dxl, INITIAL_AND_MISSION_INSTRUCTION, 600, 0.0);

  // 픽시 LED 조명 끄기
  pixy.setLamp(0, 0);

  // 바퀴모터를 정해진 거리만큼 이동하는 '위치 제어 모드'로 변경
  ChangeMobilebaseMode2ExtendedPositionControlWithTimeBasedProfileMode(dxl);

  // 80cm만큼 전방으로 이동하도록 움직임
  DriveDistanceAndMmPerSecAndDirection(dxl, 900.0);
  // 완료될 때 까지 기다리기
  while(!CheckIfMobilebaseIsInPosition(dxl)) {}

  // 3cm만큼 오른쪽으로 이동하도록 움직임
  DriveDistanceAndMmPerSecAndDirection(dxl, 100.0, DRIVE_DIRECTION_RIGHT);
  // 완료될 때 까지 기다리기
  while(!CheckIfMobilebaseIsInPosition(dxl)) {}

  // 바퀴모터를 '속도 제어 모드'로 다시 변경
  ChangeMobilebaseMode2VelocityControlMode(dxl);

  pixy.setLamp(1, 1);
  delay(500);

  int16_t flPSDValue, frPSDValue, slPSDValue;

  // 2. 센서 값 읽어오기
  GetValueFromFrontPSDSensors(&flPSDValue, &frPSDValue);
  GetValueFromSideLeftPSDSensor(&slPSDValue);

  // 3. 시리얼 모니터에 출력 (보기 좋게 정렬)

  // 블럭 집기, 놓기 미션 시작
  for(int i = 0; i < MISSION_BLOCK_CNT; i++){
    // 목표에서 멀리 떨어져 있을 때 사용할 PSD 센서의 민감도
    #define AGGRESSIVE_PSD_RATIO 0.04 // 0.06 

    // 목표에 가까워 졌을 때 사용할 PSD 센서의 민감도
    #define DEFAULT_PSD_RATIO 0.025 // 0.04

    // 현재 민감도를 저장할 변수를 만들고, 일단 기본 값으로 설정(0.04)
    float currentPSDRatio = DEFAULT_PSD_RATIO;
    // 램프 킴

    // 휴식시간주기
    SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);
    delay(200);

    // 블록 탐색 루프에서 사용할 PSD 센서 값 저장용 변수 3개
    int16_t slPSDValue4, flPSDValue4, frPSDValue4;

    // 블럭을 찾을 때까지 반복 (오른쪽으로 이동하는 파트)
    while(!haveFoundBlock) {

      // 1. 좌측 PSD 센서 값을 읽어옴
      GetValueFromSideLeftPSDSensor(&slPSDValue4);

      // 2. 로봇이 너무 오른쪽으로 가서 PSD값이 160미만이 되면
      // 블럭 찾기 포기 후 멈춤
      // 이 안쪽으로 빠지는 이유가 위에서 120이라는 큰 값을 우측으로 이동시켜서?
      if (slPSDValue4 < 100) {    
        // 
        DEBUG_SERIAL.println("좌측 PSD값이 80 작아용");
        SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);
        break;
      }

      // 3. 오차 계산 시작
      // 전방 2개의 PSD 센서 값을 읽어옴
      GetValueFromFrontPSDSensors(&flPSDValue4, &frPSDValue4);

      // 픽시로 현재 미션 블럭을 찾음
      pixy.ccc.getBlocks(true, targetBlockSigmaps[i]); // true인데 false로 바꾸면 ... ?

      // x축(좌우) 오차 계산 :
      int16_t blockXError = (
        // 만약 블럭이 보이면 (numBlocks > 0)
        pixy.ccc.numBlocks
        // 블럭 x좌표를 사용  : 안보이면 오른쪽 최대값을 사용(오른쪽으로 가게 유도)
        ? pixy.ccc.blocks[0].m_x : PIXY_CCC_X_MAX)
        // 목표 x좌표(172)와의 차이를 계산 - define된 x값
        - PIXY2_X_SETPOINT;
      // y축(앞뒤), 회전(틀어짐) 오차를 저장할 변수 준비
      int16_t yPosError, rotationError;

      // 정렬이 완료됬는지 확인하는 깃발
      bool inControl = true;

      // y축 및 회전 오차를 3zone으로 나눠서 계산 :
      // DEBUG_SERIAL.print("전방좌측 : ");
      // Serial.println(flPSDValue4);
      // DEBUG_SERIAL.print("전방우측 : ");
      // Serial.println(frPSDValue4);

      if (slPSDValue4 < 190) {
        // zone1 - 오른쪽구간 : 회전 보정 off, y축은 왼쪽 센서만 사용
        yPosError = flPSDValue4+PSD_FL_CORRECTION - MISSION_FRONT_PSD_SET_POINT;
        rotationError = 0;
      } else if (slPSDValue4 < 230) {
        // zone2 - 중앙구간 : 회전 보정 on, y축은 두 센서 평균 사용(안정적)
        yPosError = (flPSDValue4+PSD_FL_CORRECTION+frPSDValue4)/2 - MISSION_FRONT_PSD_SET_POINT;
        rotationError = frPSDValue4 - (flPSDValue4+PSD_FL_CORRECTION);
      } else {
        // zone3 - 왼쪽구간 : 회전 보정 off, y축은 오른쪽 센서만 사용
        yPosError = frPSDValue4 - MISSION_FRONT_PSD_SET_POINT;
        rotationError = 0;
      }

      // 4. 동적 게인(민감도) 계산
      // 오차가 크면 강하게, 오차가 작으면 약하게 반응하도록 조절
      float dynamicPsdRatio = (abs(yPosError) > PSD_TOLERANCE * 3) ? AGGRESSIVE_PSD_RATIO : DEFAULT_PSD_RATIO;
      float dynamicPixyRatio = (abs(blockXError) > PIXY_TOLERANCE * 3) ? PIXY_CONTROL_RATIO : (PIXY_CONTROL_RATIO / 2.0f);
      // 수정 3 - 회전 오차 민감도를 높게 유지하여 Dead Zone과 회전 오차에 적극 대응
      float dynamicRotationRatio = (abs(rotationError) > PSD_TOLERANCE * 2) ? 0.1f : DEFAULT_PSD_RATIO;

      // 5. 3zone에 맞춰 모터 실행 부분
      if (slPSDValue4 < 190) {
        // Zone 1 (오른쪽): 회전 보정 없는 'LocateWithTwoSensors' 호출 (X, Y축만 보정)
        inControl = LocateWithTwoSensors(dxl, blockXError, yPosError,
                             PIXY_TOLERANCE, PSD_TOLERANCE,
                             dynamicPixyRatio, dynamicPsdRatio,
                             DRIVE_DIRECTION_LEFT,
                             MISSION_DRIVE_SPEED);
      } else if (slPSDValue4 < 230) {
        // Zone 2 (중앙): ★회전 보정 있는★ 'DriveWithPositionAndRotationErrors' 호출
        // (X, Y, 회전 3축 모두 보정)
        inControl = DriveWithPositionAndRotationErrors(dxl, blockXError, yPosError, rotationError,
                               PIXY_TOLERANCE, PSD_TOLERANCE, PSD_TOLERANCE,
                               dynamicPixyRatio, dynamicPsdRatio, dynamicRotationRatio,
                               DRIVE_DIRECTION_LEFT, DRIVE_DIRECTION_FORWARD, ROTATE_CCW,
                               MISSION_DRIVE_SPEED, ALIGNMENT_ROTATING_SPEED);
      } else {
        // // Zone 3 (왼쪽): 회전 보정 없는 'LocateWithTwoSensors' 호출 (X, Y축만 보정)
        inControl = LocateWithTwoSensors(dxl, blockXError, yPosError,
                             PIXY_TOLERANCE, PSD_TOLERANCE,
                             dynamicPixyRatio, dynamicPsdRatio,
                             DRIVE_DIRECTION_LEFT,
                             MISSION_DRIVE_SPEED);
      }

      // 6. 도착 확인
      if (!inControl) {
        // 'Locate...' 또는 'DriveWithPosition...' 함수가 "도착했어(false)"라고 응답하면
        // 다음 반복에서 while 탈출
        // SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);
        haveFoundBlock = true;
      }
    }

    // 블럭 찾음 신호가 올 때까지 이 루프 반복
    // (왼쪽으로 이동하는 파트)
    while(!haveFoundBlock) {
      // 1. 왼쪽 PSD 센서 값 읽기
      GetValueFromSideLeftPSDSensor(&slPSDValue4);
      // 2. 로봇이 왼쪽으로 너무 많이 가서 벽에 충돌할 것 같으면
      // 블럭 찾기 멈춤. 루프 탈출
      if (slPSDValue4 > 650) {
        DEBUG_SERIAL.print("좌측PSD값 : ");
        DEBUG_SERIAL.println(slPSDValue4);
        SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);
        break;
      }

      // 3. 전방 두 개의 PSD 센서 값을 읽어옴
      GetValueFromFrontPSDSensors(&flPSDValue4, &frPSDValue4);

      // 4. 픽시로 현재 미션 블록을 찾음
      pixy.ccc.getBlocks(true, targetBlockSigmaps[i]);
      // 5. x축(좌우) 오차 계산 :
      int16_t blockXError = (
        // 블럭이 보이면
        pixy.ccc.numBlocks
        // 블럭의 x좌표 사용 : 안 보이면 왼쪽 최대값을 사용(왼쪽으로 가도록 유도)
        ? pixy.ccc.blocks[0].m_x : PIXY_CCC_X_MIN)
        // 목표 x좌표 (설정된 값 : 172(현재))와의 차이를 계산
        - PIXY2_X_SETPOINT;

      // y축(앞뒤), 회전(틀어짐) 오차를 저장할 변수 준비
      int16_t yPosError, rotationError;
      // 정렬이 완료되었는지 확인하는 깃발
      bool inControl = true;

      // y축 및 회전 오차 3 zone 계산: slPSDValue4 값이 커지므로 큰 값부터 검사
        if (slPSDValue4 > 225) { // Zone 3 - 왼쪽 구간
        // 오른쪽 센서만 사용, 회전 보정 off
          yPosError = frPSDValue4 - MISSION_FRONT_PSD_SET_POINT;
          rotationError = 0;
      } else if (slPSDValue4 > 170) { // Zone 2 - 중앙구간
        // 두 센서 평균 사용, 회전 보정 on
          yPosError = (flPSDValue4+PSD_FL_CORRECTION+frPSDValue4)/2 - MISSION_FRONT_PSD_SET_POINT;
          rotationError = frPSDValue4 - (flPSDValue4+PSD_FL_CORRECTION);
      } else { // Zone 1 - 오른쪽 구간
        // 왼쪽 센서만 사용, 회전 보정 off
          yPosError = flPSDValue4+PSD_FL_CORRECTION - MISSION_FRONT_PSD_SET_POINT;
          rotationError = 0;
      }

      // 7. 동적 게인(민감도) 계산 (동일)
      float dynamicPsdRatio = (abs(yPosError) > PSD_TOLERANCE * 3) ? AGGRESSIVE_PSD_RATIO : DEFAULT_PSD_RATIO;
      float dynamicPixyRatio = (abs(blockXError) > PIXY_TOLERANCE * 3) ? PIXY_CONTROL_RATIO : (PIXY_CONTROL_RATIO / 2.0f);
      // ✨ [수정 3] 회전 오차 민감도를 높게 유지하여 Dead Zone과 회전 오차에 적극 대응
      float dynamicRotationRatio = (abs(rotationError) > PSD_TOLERANCE * 2) ? 0.1f : DEFAULT_PSD_RATIO;

      // 수정 코드
      // 8. 3zone에 맞춰 모터 실행부분
      if (slPSDValue4 > 225) {
        // 왼쪽 구간
        // 회전 보정 없는 LocateWithTwoSensors 호출(x, y축만 보정)
        inControl = LocateWithTwoSensors(dxl, blockXError, yPosError,
                             PIXY_TOLERANCE, PSD_TOLERANCE,
                             dynamicPixyRatio, dynamicPsdRatio,
                             DRIVE_DIRECTION_LEFT,
                             MISSION_DRIVE_SPEED);
      } else if (slPSDValue4 > 170) {
        // 회전 보정 있는 DriveWithPositionAndRotationErrors 호출(x, y, 회전 3축 모두 보정)
        inControl = DriveWithPositionAndRotationErrors(dxl, blockXError, yPosError, rotationError,
                               PIXY_TOLERANCE, PSD_TOLERANCE, PSD_TOLERANCE,
                               dynamicPixyRatio, dynamicPsdRatio, dynamicRotationRatio,
                               DRIVE_DIRECTION_LEFT, DRIVE_DIRECTION_FORWARD, ROTATE_CCW,
                               MISSION_DRIVE_SPEED, ALIGNMENT_ROTATING_SPEED);
      } else {
        // 오른쪽 구간
        // 회전 보정 없는 Locate ... 호출 (x, y 축만 보정)
        inControl = LocateWithTwoSensors(dxl, blockXError, yPosError,
                             PIXY_TOLERANCE, PSD_TOLERANCE,
                             dynamicPixyRatio, dynamicPsdRatio,
                             DRIVE_DIRECTION_LEFT,
                             MISSION_DRIVE_SPEED);
      }

      if (!inControl) {
        haveFoundBlock = true;
      }
      
    }

    // 블럭을 찾으면
    if (haveFoundBlock) {
      delay(200);
      // ✨ [수정 4-1] 루프 탈출 직후 모터 완전 정지
      // 층수 판단 변수
      const int UPPER_THRESHOLD = PIXY2_Y_SETPOINT - PIXY2_Y_TOLERANCE;
      // 찾은 블록의 y좌표를 가져옴2
      // int blockY = pixy.ccc.blocks[0].m_y;
      // SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);
       // ✨ [수정 4-2] 안정화 시간 확보 (물리적 압력 해소 유도)
      // 거리이동을 하기 위한 위치 제어 모드로 변경
      int blockY = 0;
      if (pixy.ccc.numBlocks > 0) {
        blockY = pixy.ccc.blocks[0].m_y;
      }

      // const int UPPER_THRESHOLD = PIXY2_Y_SETPOINT - PIXY2_Y_TOLERANCE;

      // (2) 거리 변수 선언
      float approachDistance = 0.0;
      float retreatDistance = 0.0;

      // // (3) 층수별 거리 설정 (수정됨)
      if (blockY < UPPER_THRESHOLD) { 
        // ★ 위층(Upper): 55mm
        DEBUG_SERIAL.println(">> 위층 감지: 55mm 전진");
        approachDistance = 55.0; 
        retreatDistance = 45.0; // (55 - 10)
      } else {
        // ★ 아래층(Lower): 40mm
        DEBUG_SERIAL.println(">> 아래층 감지: 40mm 전진");
        approachDistance = 40.0; 
        retreatDistance = 30.0; // (55 - 10)
      }

      // (4) 설정된 거리만큼 전진 실행
      ChangeMobilebaseMode2ExtendedPositionControlWithTimeBasedProfileMode(dxl);

      // ✨ [수정 5] SET_POINT를 260으로 늘렸으므로, 최종 전진 거리를 늘릴 수 있음 (40mm 정도로 조정)
     // 잡기 전 이동 거리 (60.0 -> 40.0)
      DriveDistanceAndMmPerSecAndDirection(dxl, approachDistance); 
      // while(!CheckIfMobilebaseIsInPosition(dxl)) {}
      while(!CheckIfMobilebaseIsInPosition(dxl)) {}
      // 다음 동작을 위해 속도 제어 모드로 변경
      // delay(100);
      ChangeMobilebaseMode2VelocityControlMode(dxl);

      DEBUG_SERIAL.print("현재 Y좌표: ");
      DEBUG_SERIAL.print(pixy.ccc.blocks[0].m_y);

      // 포착된 블럭위치로 팔 뻗기 + 집기
      if (blockY < UPPER_THRESHOLD) {
        DEBUG_SERIAL.println("위쪽 블럭 집기");
        RunManipulatorPoseWithPoseDataInEEPROM(dxl, PRE_GRIP_UPPER_BLOCK, 700, 0.0);
        delay(700);
        RunManipulatorPoseWithPoseDataInEEPROM(dxl, GRIP_UPPER_BLOCK, 700, 0.0);
        // delay(1000);
      } else {
        DEBUG_SERIAL.println("아래쪽 블럭 집기");
        RunManipulatorPoseWithPoseDataInEEPROM(dxl, PRE_GRIP_LOWER_BLOCK, 700, 0.0);
        delay(700);
        RunManipulatorPoseWithPoseDataInEEPROM(dxl, GRIP_LOWER_BLOCK, 700, 0.0);
        // delay(1000);
      }
      
      delay(800);
      // 그리퍼 닫음
      CloseGripper(pixy);
      delay(500);
      // 팔 포즈를 다시 몸쪽으로 당겨옴
      RunManipulatorPoseWithPoseDataInEEPROM(dxl, STORAGE, 700, 0.0);
      // delay(800);
      delay(200);
      ChangeMobilebaseMode2ExtendedPositionControlWithTimeBasedProfileMode(dxl);
      
      DriveDistanceAndMmPerSecAndDirection(dxl, retreatDistance, DRIVE_DIRECTION_BACKWARD);

      // ✨ [수정 5] SET_POINT를 260으로 늘렸으므로, 최종 전진 거리를 늘릴 수 있음 (40mm 정도로 조정)
      // DriveDistanceAndMmPerSecAndDirection(dxl, 30.0, DRIVE_DIRECTION_BACKWARD); // 잡기 전 이동 거리 (60.0 -> 40.0)

      while(!CheckIfMobilebaseIsInPosition(dxl)) {}
      // 다음 동작을 위해 속도 제어 모드로 변경
      // delay(100);
      ChangeMobilebaseMode2VelocityControlMode(dxl);

      // 아래는 블럭 놓기 정렬 루프임(왼쪽으로 이동하는 과정)
      // 빠른 정렬을 위해 민감도를 재설정
      currentPSDRatio = AGGRESSIVE_PSD_RATIO;
      while(1) {
        // 왼쪽 PSD 센서 값을 읽어옴
        GetValueFromSideLeftPSDSensor(&slPSDValue4);
        // x축(좌우) 오차 계산
        // (현재 왼쪽 값) - (목표값 625) = 왼쪽/오른쪽 오차
        int16_t xPosError = slPSDValue4 - MISSION_LEFT_PSD_SET_POINT;

        // 의도적 비활성화 (보정안함)
        int16_t yPosError = 0;
        int16_t rotationError = 0;
        bool inControl = true;


      // 🚀 [수정] 3 zone 모터 실행: slPSDValue4 값이 커지므로 큰 값부터 검사
        if (slPSDValue4 > 225) { // Zone 3 - 왼쪽 구간
        inControl = LocateWithTwoSensors(dxl, xPosError, yPosError,
                             PSD_TOLERANCE, PSD_TOLERANCE, currentPSDRatio, currentPSDRatio,
                             DRIVE_DIRECTION_LEFT,
                             ALIGNMENT_DRIVING_SPEED); // 속도 수정
        } else if (slPSDValue4 > 170) { // Zone 2 - 중앙구간
        inControl = DriveWithPositionAndRotationErrors(dxl, xPosError, yPosError, rotationError,
                               PSD_TOLERANCE, PSD_TOLERANCE, PSD_TOLERANCE,
                               currentPSDRatio, currentPSDRatio, currentPSDRatio,
                               DRIVE_DIRECTION_LEFT, DRIVE_DIRECTION_FORWARD, ROTATE_CCW,
                               ALIGNMENT_DRIVING_SPEED, ALIGNMENT_ROTATING_SPEED); // 속도 수정
        } else { // Zone 1 - 오른쪽 구간
          inControl = LocateWithTwoSensors(dxl, xPosError, yPosError,
                                 PSD_TOLERANCE, PSD_TOLERANCE, currentPSDRatio, currentPSDRatio,
                                 DRIVE_DIRECTION_LEFT,
                                 ALIGNMENT_DRIVING_SPEED); // 속도 수정
        }

        if (!inControl) {
          break;
        }
      }

      // 위치 제어 모드로 변경
      ChangeMobilebaseMode2ExtendedPositionControlWithTimeBasedProfileMode(dxl);
      // 2번 자세로 동작
      RunManipulatorPoseWithPoseDataInEEPROM(dxl, STORAGE, 800, -90.0);
      // delay(1000);
      // 시작ID + 목표 칸 = 목표 자세
/*
      // 기존 넣는 코드
      RunManipulatorPoseWithPoseDataInEEPROM(dxl, MANIPULATOR_MISSION_FULFILLMENT_POSE_START_ID + goalPositions[i], 1000);
      delay(1200);
*/
      // 포즈를 알맞게 바꿔야 함
      uint8_t targetSlot = 0; // 목표 칸 번호 저장 변수

      // 1. 현재 잡은 블록의 색상(targetBlockSigmaps[i]) 확인
      //    (주의: 아래 칸 번호(1, 2, 7, 8...)는 대회 규칙에 맞게 수정 필수!)
      // 초2 보3 빨5 파7
      //
      
      if (targetBlockSigmaps[i] == 0x01) {       // 빨강 (Sig 1)
          targetSlot = 5; 
      }
      // else if (targetBlockSigmaps[i] == 0x02) {  // 주 (Sig 2)
      //     targetSlot = 8; 
      // }
      // else if (targetBlockSigmaps[i] == 0x04) {  // 노 (Sig 3)
      //     targetSlot = 1; 
      // }
      else if (targetBlockSigmaps[i] == 0x08) {  // 초 (Sig 4)
          targetSlot = 2; 
      }
      else if (targetBlockSigmaps[i] == 0x10) {  // 파 (Sig 5)
          targetSlot = 7; 
      }
      else if (targetBlockSigmaps[i] == 0x20) {  // 보 (Sig 6)
          targetSlot = 3; 
      }

      DEBUG_SERIAL.print("결정된 목표 칸: ");
      DEBUG_SERIAL.println(targetSlot);

      RunManipulatorPoseWithPoseDataInEEPROM(dxl, MANIPULATOR_MISSION_FULFILLMENT_POSE_START_ID + targetSlot, 1000);
      delay(1100);
      // 그리퍼 열기
      OpenGripper(pixy);
      delay(600); // 

      // 팔 접기
      RunManipulatorPoseWithPoseDataInEEPROM(dxl, STORAGE, 1000, 0.0);
      delay(500);

      // 속도 제어 모드로 복귀
      ChangeMobilebaseMode2VelocityControlMode(dxl);

      // 리셋
      haveFoundBlock = false;
      // DEBUG_SERIAL.println("끝");
      int16_t slPSDValue_Check;
      GetValueFromSideLeftPSDSensor(&slPSDValue_Check);

      // 현재 위치가 왼쪽 벽(기준값 625)보다 큰지 확인
      if (slPSDValue_Check > 500 && i != (MISSION_BLOCK_CNT-1)) {
        // 중앙(예: 450)으로 돌아올 때까지 오른쪽으로 이동
        while(slPSDValue_Check > 300) {
          // 오른쪽으로 이동 (DRIVE_DIRECTION_RIGHT)
          SetMobileGoalVelocityForSyncWrite(dxl,
            round(MISSION_DRIVE_SPEED),   // FL
            round(-MISSION_DRIVE_SPEED),  // FR
            round(-MISSION_DRIVE_SPEED),  // BL
            round(MISSION_DRIVE_SPEED));  // BR

          GetValueFromSideLeftPSDSensor(&slPSDValue_Check);
        }
        // 중앙 도착. 정지
          SetMobileGoalVelocityForSyncWrite(dxl, 0, 0, 0, 0);
          DEBUG_SERIAL.println("Return complete.");
        delay(500); // 정지 시간 확보
      }

      
    }


  } // 미션 수행 for문 닫기 괄호

  // 미션이 모두 끝났으니
  // 위치 제어 모드로 변경
  ChangeMobilebaseMode2ExtendedPositionControlWithTimeBasedProfileMode(dxl);
  // 후진 - Mobilebase에 저장된 속도를 사용
  DriveDistanceAndMmPerSecAndDirection(dxl, 1280.0, DRIVE_DIRECTION_BACKWARD);
  while(!CheckIfMobilebaseIsInPosition(dxl)) {}

  // 🟢 [추가] 총 걸린 시간 계산 및 출력
  unsigned long missionElapsedTime = millis() - missionStartTime;
  DEBUG_SERIAL.print(missionElapsedTime / 1000.0); // 밀리초(ms)를 초(s) 단위로 변환
  DEBUG_SERIAL.println(" 초.");

  while(1) { delay(1000); }

} // setup 닫기 괄호

void loop() {
  // 40번 버튼 누르면 실행
  // if (digitalRead(BUTTON_PIN) == LOW) {
    
  //   DEBUG_SERIAL.println("\n[버튼 입력] 요청하신 초기 자세(1번, -90.0) 실행");

  //   // 요청하신 코드 그대로 실행
  //   RunManipulatorPoseWithPoseDataInEEPROM(dxl, INITIAL_AND_MISSION_INSTRUCTION, 1400, -90.0);

  //   // 버튼 꾹 누름 방지 (1초 대기)
  //   delay(1000);
  // }

  delay(50);
}
