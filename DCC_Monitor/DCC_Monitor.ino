//--------------------------------------------------------------------------------
// DCC monitor
// [DCC_monitor.ino]
// 2020 Ayanosuke(Maison de DCC)
//
// Building a DCC Monitor with an Arduino をベースに構文解析を追加しました
// http://www.mynabay.com/dcc_monitor/
//
// 大元のドライバ等は以下のサイトから使用しています
// https://github.com/MynaBay/DCC_Decoder
//
// http://maison-dcc.sblo.jp/ http://dcc.client.jp/ http://ayabu.blog.shinobi.jp/
// https://twitter.com/masashi_214
//
// DCC電子工作連合のメンバーです
// https://desktopstation.net/tmi/ https://desktopstation.net/bb/index.php
//
// 2022/2/9 アドレス64以降正常にデコードできなかったので修正
//  } else if((gPackets[i].data[0] & 0xc0)==0x40){  // 0100 0000 CV書き込み から
//  } else if((gPackets[i].data[0] & 0xf0)==0x70){  // 0111 0000 CV書き込み に変更
//--------------------------------------------------------------------------------

#include "DCC_Decoder.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Defines and structures
//
#define kDCC_INTERRUPT            0

typedef struct
{
    int count;
    byte validBytes;
    byte data[6];
} DCCPacket;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// The dcc decoder object and global data
//
int gPacketCount = 0;
int gIdlePacketCount = 0;
int gLongestPreamble = 0;

DCCPacket gPackets[25];

static unsigned long lastMillis = millis();
    
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Packet handlers
//

// ALL packets are sent to the RawPacket handler. Returning true indicates that packet was handled. DCC library starts watching for 
// next preamble. Returning false and library continue parsing packet and finds another handler to call.
boolean RawPacket_Handler(byte byteCount, byte* packetBytes)
{
        // Bump global packet count
    ++gPacketCount;
    
    int thisPreamble = DCC.LastPreambleBitCount();
    if( thisPreamble > gLongestPreamble )
    {
        gLongestPreamble = thisPreamble;
    }
    
        // Walk table and look for a matching packet
    for( int i=0; i<(int)(sizeof(gPackets)/sizeof(gPackets[0])); ++i )
    {
        if( gPackets[i].validBytes )
        {
                // Not an empty slot. Does this slot match this packet? If so, bump count.
            if( gPackets[i].validBytes==byteCount )
            {
                char isPacket = true;
                for( int j=0; j<byteCount; j++)
                {
                    if( gPackets[i].data[j] != packetBytes[j] )
                    {
                        isPacket = false;
                        break;
                    } 
                }
                if( isPacket )
                {
                   gPackets[i].count++;
                   return false;
                }
            }
        }else{
                // Empty slot, just copy over data
            gPackets[i].count++;
            gPackets[i].validBytes = byteCount;
            for( int j=0; j<byteCount; j++)
            {
                gPackets[i].data[j] = packetBytes[j];
            }
            return false;
        }
    }    
    
    return false;
}

// Idle packets are sent here (unless handled in rawpacket handler). 
void IdlePacket_Handler(byte byteCount, byte* packetBytes)
{
    ++gIdlePacketCount;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Setup
//
void setup() 
{ 
   Serial.begin(115200);
    
   DCC.SetRawPacketHandler(RawPacket_Handler);   
   DCC.SetIdlePacketHandler(IdlePacket_Handler);
            
   DCC.SetupMonitor( kDCC_INTERRUPT );   
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void DumpAndResetTable()
{
    char buffer60Bytes[60];
    char buf[10];
    unsigned int ad;
    
    Serial.print("Total Packet Count: ");
    Serial.println(gPacketCount, DEC);
    
    Serial.print("Idle Packet Count:  ");
    Serial.println(gIdlePacketCount, DEC);
        
    Serial.print("Longest Preamble:  ");
    Serial.println(gLongestPreamble, DEC);
    
    Serial.println("Count    Packet_Data");
    for( int i=0; i<(int)(sizeof(gPackets)/sizeof(gPackets[0])); ++i )
    {
        if( gPackets[i].validBytes > 0 )
        {
            Serial.print(gPackets[i].count, DEC);
            if( gPackets[i].count < 10 )
            {
                Serial.print("        ");
            }else{
                if( gPackets[i].count < 100 )
                {
                    Serial.print("       ");
                }else{
                    Serial.print("      ");
                }
            }
            Serial.print( DCC.MakePacketString(buffer60Bytes, gPackets[i].validBytes, &gPackets[i].data[0]) );
//              Serial.println(gPackets[i].validBytes);
            if(gPackets[i].validBytes==3)
              Serial.print("                  ");
            
            if(gPackets[i].validBytes==4)
              Serial.print("         ");

            Serial.print(" ");
            
            if(gPackets[i].data[0] == 0xff){ // idle
                Serial.println("idle");     
            } else if((gPackets[i].data[0] & 0xf0)==0x70){  // 0111 0000 CV書き込み
              ad = ((gPackets[i].data[0] & 0x03)*256)+gPackets[i].data[1]+1;
              sprintf(buf , "CV:%3d,%3d ",ad, gPackets[i].data[2]);
              Serial.println(buf);             
            } else if((gPackets[i].data[0] & 0xc0)==0xc0){  // 1100 0000
              ad = ((gPackets[i].data[0] & 0x3f)<<8)+gPackets[i].data[1];
              sprintf(buf , "Adr:%4d, ",ad); // adr 128-191
              Serial.print(buf);
              dcc_analysis(gPackets[i].data[2],gPackets[i].data[3]);
            } else if((gPackets[i].data[0] & 0xc0)==0x80){  // 1000 0000 アクセサリ
              dcc_acc_analysis(gPackets[i].data[0],gPackets[i].data[1]);           
            } else {
              sprintf(buf , "Adr:%4d, ",gPackets[i].data[0]);
              Serial.print(buf);
              dcc_analysis(gPackets[i].data[1],gPackets[i].data[2]);
            }

//            Serial.println(gPackets[i].data[1]);
        }
        gPackets[i].validBytes = 0;
        gPackets[i].count = 0;
    }
    Serial.println("============================================");
    
    gPacketCount = 0;
    gIdlePacketCount = 0;
    gLongestPreamble = 0;
}

void dcc_analysis(unsigned char pak1, unsigned char pak2){
  char temp[10];
  char buf[50];

//        sprintf(buf , "pak1:%2x ",pak1 & 0xe0);            
//        Serial.println(buf);
  if((pak1 & 0xe0 ) == 0x00) {     // 000:Decoder and Consist Control Instruction
        Serial.println("000:");
  
  } else if((pak1 & 0xe0 ) == 0x20){  // 001:Advanced Operation Instruction
    if(pak1 == 0x3f){     // CCCCC ~ 11111
        if(pak2 & 0x80){  // Forward
            strcpy( temp, "FWD");
        } else
            strcpy( temp, "REV");
        sprintf(buf , "128 Speed step,%3s,%d ",temp,pak2 & 0x7f);            
        Serial.println(buf);
    }
    
  } else if((pak1 & 0xe0) == 0x40){ // 010:Speed 
//        Serial.println("010:");    
        if(pak1 & 0x80){  // Forward
            strcpy( temp, "FWD");
        } else
            strcpy( temp, "REV");
        sprintf(buf , "128 Speed step,%3s,%d ",temp,pak1 & 0x7f);            
        Serial.println(buf);
    
  
  
  } else if((pak1 & 0xe0) == 0x60){ // 011:Speed
        Serial.println("011:");      
  } else if((pak1 & 0xe0) == 0x80){ // 100:Function Group One
    if(pak1 & 0x10)
      Serial.print("F 0 ON :");
    else
      Serial.print("F 0 OFF:");

    if(pak1 & 0x01)
      Serial.print("F 1 ON :");
    else
      Serial.print("F 1 OFF:");

    if(pak1 & 0x02)
      Serial.print("F 2 ON :");
    else
      Serial.print("F 2 OFF:");

    if(pak1 & 0x04)
      Serial.print("F 3 ON :");
    else
      Serial.print("F 3 OFF:");

    if(pak1 & 0x08)
      Serial.println("F 4 ON");
    else
      Serial.println("F 4 OFF");

    
  } else if((pak1 & 0xe0) == 0xa0){  // 101:Function Group Two
//        Serial.print("*");
    if(pak1 & 0x10){
      if(pak1 & 0x01)
        Serial.print("F 5 ON :");
      else
        Serial.print("F 5 OFF:");

      if(pak1 & 0x02)
        Serial.print("F 6 ON :");
      else
        Serial.print("F 6 OFF:");

      if(pak1 & 0x04)
        Serial.print("F 7 ON :");
      else
        Serial.print("F 7 OFF:");

      if(pak1 & 0x08)
        Serial.println("F 8 ON");
      else
        Serial.println("F 8 OFF");
    } else {
      if(pak1 & 0x01)
        Serial.print("F 9 ON :");
      else
        Serial.print("F 9 OFF:");

      if(pak1 & 0x02)
        Serial.print("F10 ON :");
      else
        Serial.print("F10 OFF:");

      if(pak1 & 0x04)
        Serial.print("F11 ON :");
      else
        Serial.print("F11 OFF:");

      if(pak1 & 0x08)
        Serial.println("F12 ON");
      else
        Serial.println("F12 OFF");
    }      

          
  } else if((pak1 & 0xe0) == 0xc0){  // 110:Future Expansison
    if(pak1 & 0x01){  // 110 11111 F21-F28
      if(pak2 & 0x01)
        Serial.print("F21 ON :");
      else
        Serial.print("F21 OFF:");

      if(pak2 & 0x02)
        Serial.print("F22 ON :");
      else
        Serial.print("F22 OFF:");

      if(pak2 & 0x04)
        Serial.print("F23 ON :");
      else
        Serial.print("F23 OFF:");

      if(pak2 & 0x08)
        Serial.print("F24 ON :");
      else
        Serial.print("F24 OFF:");

      if(pak2 & 0x10)
        Serial.print("F25 ON :");
      else
        Serial.print("F25 OFF:");

      if(pak2 & 0x20)
        Serial.print("F26 ON :");
      else
        Serial.print("F26 OFF:");

      if(pak2 & 0x40)
        Serial.print("F27 ON :");
      else
        Serial.print("F27 OFF:");

      if(pak2 & 0x80)
        Serial.println("F28 ON");
      else
        Serial.println("F28 OFF");
        
    } else {  // 110 11110 F13-F20
      if(pak2 & 0x01)
        Serial.print("F13 ON :");
      else
        Serial.print("F13 OFF:");

      if(pak2 & 0x02)
        Serial.print("F14 ON :");
      else
        Serial.print("F14 OFF:");

      if(pak2 & 0x04)
        Serial.print("F15 ON :");
      else
        Serial.print("F15 OFF:");

      if(pak2 & 0x08)
        Serial.print("F16 ON :");
      else
        Serial.print("F16 OFF:");

      if(pak2 & 0x10)
        Serial.print("F17 ON :");
      else
        Serial.print("F17 OFF:");

      if(pak2 & 0x20)
        Serial.print("F18 ON :");
      else
        Serial.print("F18 OFF:");

      if(pak2 & 0x40)
        Serial.print("F19 ON :");
      else
        Serial.print("F19 OFF:");

      if(pak2 & 0x80)
        Serial.println("F20 ON");
      else
        Serial.println("F20 OFF");
    }



  } else if((pak1 & 0xe0) ==  0xe0){  // 111:Configration Val
            Serial.println("***");
  }
}


void dcc_acc_analysis(unsigned char pak1, unsigned char pak2){
  char buf[50];
  unsigned int ad;

  ad = ((pak1 & 0x3f) -1 )*4 + ((pak2 & 0x06)>>1)+1;  // アドレスの計算　わかりにくい
  // adr 253 以上は計算めんどくさいから省こう。

  sprintf(buf , "Adr:%4d, %s",ad ,pak2 & 0x01 ?"C":"T"); //c:1 t:0 
  Serial.println(buf);
    
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Main loop
//
void loop()
{
    DCC.loop();
    
    if( millis()-lastMillis > 2000 )
    {
        DumpAndResetTable();
        lastMillis = millis();
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
