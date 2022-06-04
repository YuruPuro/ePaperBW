#include <SPI.h>
#include "bitmapdata.h"
#include "Image1.h"
#include "Image2.h"

//IO settings
int BUSY_Pin = 6; 
int RES_Pin = 7; 
int DC_Pin = 9; 
int CS_Pin = 10; 

// ▲▼▲▼▲▼▲▼▲▼▲▼▲▼▲▼▲▼▲▼▲▼
const uint8_t INIT_COMMANDS[] PROGMEM={
  0x74 , 1 , 0x54 ,               // set analog block control
  0x7E , 1 , 0x3B ,               // set digital block control
  0x01 , 3 , 0xF9 , 0x00 , 0x00 , // Driver output control

                                  // アドレスのインクリメント方向は▼で変わる
  0x11 , 1 , 0x05 ,               // [1:0] data entry mode  : Y decrement, X increment
                                  // [2:2] the address counter is updated in the Y direction.
                                  
  0x3C , 1 , 0x03 ,               // BorderWavefrom
  0x2C , 1 , 0x55 ,               // VCOM Voltage
  0x03 , 1 , 0x15 ,               // LUT_DATA[70]
  0x04 , 3 , 0x41 , 0xA8 , 0x32 , // LUT_DATA[70-73]
  0x3A , 1 , 0x30 ,               // Dummy Line LUT_DATA[74]
  0x3B , 1 , 0x0A ,               // Gate time LUT_DATA[75]

  // -----
  0x32 ,70 , 0x80,0x60,0x40,0x00,0x00,0x00,0x00,             //LUT0: BB:     VS 0 ~7
            0x10,0x60,0x20,0x00,0x00,0x00,0x00,             //LUT1: BW:     VS 0 ~7
            0x80,0x60,0x40,0x00,0x00,0x00,0x00,             //LUT2: WB:     VS 0 ~7
            0x10,0x60,0x20,0x00,0x00,0x00,0x00,             //LUT3: WW:     VS 0 ~7
            0x00,0x00,0x00,0x00,0x00,0x00,0x00,             //LUT4: VCOM:   VS 0 ~7
            0x03,0x03,0x00,0x00,0x02,                       // TP0 A~D RP0
            0x09,0x09,0x00,0x00,0x02,                       // TP1 A~D RP1
            0x03,0x03,0x00,0x00,0x02,                       // TP2 A~D RP2
            0x00,0x00,0x00,0x00,0x00,                       // TP3 A~D RP3
            0x00,0x00,0x00,0x00,0x00,                       // TP4 A~D RP4
            0x00,0x00,0x00,0x00,0x00,                       // TP5 A~D RP5
            0x00,0x00,0x00,0x00,0x00,                       // TP6 A~D RP6
  // -----

  0x22 , 1 , 0xB1 ,               // Load TS and then Load LUT from OTP
  0xFF , 0 ,                      // WaitBusy
  0x00 , 0 ,
};

/** 
 *  Tips
 *  電子ペーパーを全画面でリフレッシュすると、画像のちらつきは通常の現象であり、主な機能は前の画像の表示残像をクリアすることです。
 *  ドライバを移植する必要がある場合は、対応するIOを変更するだけで済みます。
 *  BUSYピンは入力モード、その他は出力モードです。
**/

// ▲▼▲▼▲▼▲▼▲▼▲▼▲▼▲▼▲▼
void EPaperInit(void) {
  digitalWrite(RES_Pin,LOW);  // Module reset
  delay(10); //At least 10ms delay
  digitalWrite(RES_Pin,HIGH);
  delay(10); //At least 10ms delay

  // -- Software Reset --
  EPaperBusyWait();
  digitalWrite(CS_Pin, LOW);
  digitalWrite(DC_Pin, HIGH);
  SPI.transfer(0x12); // soft reset
  EPaperBusyWait();

  // -- ePaper Init --
  int p = 0 ;
  while(1) {
    uint8_t cmd = pgm_read_byte(&INIT_COMMANDS[p]) ; p++ ;
    uint8_t cnum= pgm_read_byte(&INIT_COMMANDS[p]) ; p++ ;
    if (cmd == 0) {
      break ;
    } else
    if (cmd == 0xFF) {
      EPaperBusyWait();
    } else {
      digitalWrite(DC_Pin, LOW);
      SPI.transfer(cmd); // Send Comand
      if ( cnum > 0) {
        digitalWrite(DC_Pin, HIGH);
        for (int i=0;i<cnum;i++) {
          uint8_t data = pgm_read_byte(&INIT_COMMANDS[p]) ; p++ ;
          SPI.transfer(data); // Send Data
        }
      }
    }
  }
  digitalWrite(CS_Pin, HIGH);
}

void EPaperUpdate( ) {
    digitalWrite(DC_Pin, LOW);
    SPI.transfer(0x22);
    digitalWrite(DC_Pin, HIGH);
    SPI.transfer(0xC7);
    digitalWrite(DC_Pin, LOW);
    SPI.transfer(0x20);
    EPaperBusyWait();  
}

void EPaperBusyWait(void) { 
  while(digitalRead(BUSY_Pin)==HIGH);   //=HIGH BUSY
}

void EPaperDeepSleep(void) {
  digitalWrite(CS_Pin, LOW);
  digitalWrite(DC_Pin, LOW);
  SPI.transfer(0x10);
  digitalWrite(DC_Pin, HIGH);
  SPI.transfer(0x01);
  digitalWrite(CS_Pin, HIGH);
  delay(100);
}

// ▲▼▲▼▲▼▲▼▲▼▲▼▲▼▲▼▲▼
void DispImage(uint8_t x , uint16_t y , uint8_t w , uint8_t h , uint8_t *ImageAddr,bool flagUpdate) {
  uint8_t wData , data ;
  digitalWrite(CS_Pin, LOW);

  // --- 描画範囲設定
  int dataSize = SetImageSize(x,y,w,h) ;

  digitalWrite(DC_Pin, LOW);
  SPI.transfer(0x24);
  digitalWrite(DC_Pin, HIGH);
  for(int i=0;i<dataSize;i++){
    wData = pgm_read_byte(&ImageAddr[i]) ;
    data = 0 ;
    for ( int i = 0 ; i < 8 ; i++) {
      data = (data << 1) | (wData & 0x01) ;
      wData >>= 1 ;
    }
    SPI.transfer(data);
  }

  if (flagUpdate) {
    EPaperUpdate( );
  }
  digitalWrite(CS_Pin, HIGH);
}

void EPaperCLS(bool flagUpdate){
  digitalWrite(CS_Pin, LOW);
  
  // --- 描画範囲設定
  SetImageSize(0,249,16,250) ;

  digitalWrite(DC_Pin, LOW);
  SPI.transfer(0x24);
  digitalWrite(DC_Pin, HIGH);
  for(int i=0;i<4000;i++){
    SPI.transfer(0xFF);
  }

  if (flagUpdate) {
    EPaperUpdate( );
  }
  digitalWrite(CS_Pin, HIGH);
  EPaperBusyWait();  
}

int SetImageSize(uint8_t x , uint16_t y , uint8_t w , uint8_t h) {
  uint8_t wData , data ;
  uint8_t xe = x + (w - 1) ;
  uint16_t ye = y - (h - 1) ;
  uint8_t ysH = (y >> 8) & 0xFF ;
  uint8_t ysL = y & 0xFF ;
  uint8_t yeH = (ye >> 8) & 0xFF ;
  uint8_t yeL = ye & 0xFF ;
  int dataSize = w*h ;

  // --- 描画範囲設定
  digitalWrite(DC_Pin, LOW);
  SPI.transfer(0x44);
  digitalWrite(DC_Pin, HIGH);
  data = x ; SPI.transfer(data);
  data = xe ; SPI.transfer(data);

  digitalWrite(DC_Pin, LOW);
  SPI.transfer(0x45);
  digitalWrite(DC_Pin, HIGH);
  data = ysL ; SPI.transfer(data);
  data = ysH ; SPI.transfer(data);
  data = yeL ; SPI.transfer(data);
  data = yeH ; SPI.transfer(data);
  EPaperBusyWait();

  digitalWrite(DC_Pin, LOW);
  SPI.transfer(0x4E);
  digitalWrite(DC_Pin, HIGH);
  data = x ; SPI.transfer(data);

  digitalWrite(DC_Pin, LOW);
  SPI.transfer(0x4F);
  digitalWrite(DC_Pin, HIGH);
  data = ysL ; SPI.transfer(data);
  data = ysH ; SPI.transfer(data);
  EPaperBusyWait();

  return dataSize ;
}

// ▲▼▲▼▲▼▲▼▲▼▲▼▲▼▲▼▲▼▲▼▲▼
void setup() {
  SPI.begin();  //SPIを初期化、SCK、MOSI、SSの各ピンの動作は出力、SCK、MOSIはLOW、SSはHIGH
  SPI.setClockDivider(SPI_CLOCK_DIV2);  // 8MHz
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);

  pinMode(BUSY_Pin, INPUT);
  pinMode(RES_Pin, OUTPUT);
  pinMode(DC_Pin, OUTPUT);
  pinMode(CS_Pin, OUTPUT);

  EPaperInit( );
}

void loop() {
  //Clean
  EPaperCLS(false);
  DispImage(2,220,8,88,bitmapData,false) ;
  DispImage(6,100,8,88,bitmapData,true) ;
  delay(5000);

  DispImage(0,249,16,250,image1,true) ;
  delay(5000);

  DispImage(0,249,16,250,image2,true) ;
  delay(5000);
}
