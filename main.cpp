#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>      //Unix�зǨ�Ʃw�q
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>       //��󱱨�w�q
#include <termios.h>     //POSIX���_����w�q
#include <errno.h>       //���~�T���w�q
#include <string.h>
#include <sys/wait.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <stdint.h>
#include <sstream>


// #define BAUDRATE B9600 
#define BAUDRATE B115200 
// #define MODEMDEVICE "/dev/ttyUSB0"
// #define MODEMDEVICE "/dev/ttyUSB1"
#define MODEMDEVICE "/dev/ttyUSB2"

#define _POSIX_SOURCE 1
#define STOP '@'

using namespace std;

#define polynomial 0xD5
struct HexCharStruct
{
  unsigned char c;
  HexCharStruct(unsigned char _c) : c(_c) {}
};

char getch() 
{
  char buf = 0;
  struct termios old = {0};
  
  if (tcgetattr(0, &old) < 0)
  perror("tcsetattr()");
  old.c_lflag &= ~ICANON;
  old.c_lflag &= ~ECHO;
  old.c_cc[VMIN] = 1;
  old.c_cc[VTIME] = 0;

  if (tcsetattr(0, TCSANOW, &old) < 0)
  perror("tcsetattr ICANON");
  
  if (read(0, &buf, 1) < 0)
  perror ("read()");
  old.c_lflag |= ICANON;
  old.c_lflag |= ECHO;
       
  if (tcsetattr(0, TCSADRAIN, &old) < 0)
  perror ("tcsetattr ~ICANON");
  return (buf);
}

unsigned char CalcCRCByte(unsigned char u8Byte, unsigned char u8CRC);
unsigned char CRCSpeed(int speed, unsigned char *outStr, int cmdSize);
unsigned char CRC(unsigned char cmdArray[], int cmdSize);

inline std::ostream &operator<<(std::ostream &o, const HexCharStruct &hs);
inline HexCharStruct hex(unsigned char _c);


//----------------------------cia402 cmd----------------------------------------------

unsigned char ini0[] = {0x53, 0x07, 0x01, 0x01, 0x18, 0x10, 0x04, 0xF4, 0x45};                //Serial number (Identity object)
unsigned char ini00[] = {0x53, 0x07, 0x02, 0x01, 0x18, 0x10, 0x04, 0xA2, 0x45};
unsigned char ini1[] = {0x53, 0x07, 0x00, 0x08, 0x0A, 0x10, 0x00, 0xEA, 0x45};                //Manufacturer software version
unsigned char ini2[] = {0x53, 0x07, 0x00, 0x01, 0x29, 0x23, 0x0B, 0x52, 0x45};                //Motor type (Motor and application data for motor control)
unsigned char ini3[] = {0x53, 0x07, 0x00, 0x01, 0x8F, 0x60, 0x01, 0xE8, 0x45};                //Encoder increments (Position enconder resolution)
unsigned char ini4[] = {0x53, 0x07, 0x00, 0x01, 0x91, 0x60, 0x01, 0xF6, 0x45};                //Motor shaft revolutions (Gear ratio)
unsigned char ini5[] = {0x53, 0x07, 0x00, 0x01, 0x91, 0x60, 0x02, 0x5F, 0x45};                //Driving shaft revolutions (Gear ratio)
unsigned char ini6[] = {0x53, 0x07, 0x00, 0x01, 0x92, 0x60, 0x01, 0xF5, 0x45};                //Feed (Feed constant)
unsigned char ini7[] = {0x53, 0x07, 0x00, 0x01, 0x92, 0x60, 0x02, 0x5C, 0x45};                //Shaft revolutions (Feed constant)
unsigned char ini8[] = {0x53, 0x07, 0x00, 0x01, 0x96, 0x60, 0x01, 0xF1, 0x45};                //Numerator (Velocity factor)
unsigned char ini9[] = {0x53, 0x07, 0x00, 0x01, 0x96, 0x60, 0x02, 0x58, 0x45};                //Divisor (Velocity factor)
unsigned char ini10[] = {0x53, 0x07, 0x00, 0x01, 0x28, 0x23, 0x01, 0xF3, 0x45};               //Controller type (Device data for thermal model)
unsigned char ini11[] = {0x53, 0x07, 0x00, 0x01, 0x07, 0x24, 0x00, 0x25, 0x45};               //SW revision number
unsigned char ini12[] = {0x53, 0x07, 0x00, 0x08, 0x09, 0x10, 0x00, 0xE9, 0x45};               //Manufacturer hardware version
unsigned char ini13[] = {0x53, 0x07, 0x00, 0x01, 0x02, 0x65, 0x00, 0x61, 0x45};               //Supported drive modes
unsigned char ini14[] = {0x53, 0x07, 0x00, 0x01, 0x60, 0x60, 0x00, 0xF9, 0x45};               //Modes of operation
unsigned char ini15[] = {0x53, 0x07, 0x00, 0x01, 0x01, 0x30, 0x03, 0x34, 0x45};               //Actual program position (Active program)
unsigned char ini16[] = {0x53, 0x07, 0x00, 0x01, 0x01, 0x30, 0x04, 0x33, 0x45};               //Actual program state (Active program)
unsigned char ini17[] = {0x53, 0x08, 0x00, 0x02, 0x60, 0x60, 0x00, 0x03, 0xA3, 0x45};         //Modes of operation
unsigned char ini18[] = {0x53, 0x07, 0x00, 0x01, 0x60, 0x60, 0x00, 0xF9, 0x45};               //Modes of operation
unsigned char ini19[] = {0x53, 0x07, 0x00, 0x01, 0x61, 0x60, 0x00, 0xF8, 0x45};               //Modes of operation display
unsigned char ini20[] = {0x53, 0x09, 0x00, 0x02, 0x40, 0x60, 0x00, 0x06, 0x00, 0xD2, 0x45};   //Controlword
unsigned char ini21[] = {0x53, 0x09, 0x00, 0x02, 0x40, 0x60, 0x00, 0x0F, 0x00, 0xDB, 0x45};   //Controlword
unsigned char ini22[] = {0x53, 0x07, 0x00, 0x01, 0x60, 0x60, 0x00, 0xF9, 0x45};               //Modes of operation
unsigned char ini23[] = {0x53, 0x07, 0x00, 0x01, 0x61, 0x60, 0x00, 0xF8, 0x45};               //Modes of operation display


unsigned char Speed[] = {0x53, 0x0B, 0x00, 0x02, 0xFF, 0x60, 0x00, 0x64, 0x00, 0x00, 0x00, 0x58, 0x45};     //All wheel speed
unsigned char Speed0[] = {0x53, 0x0B, 0x00, 0x02, 0xFF, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC3, 0x45};    //All wheel stop
unsigned char RSpeed[] = {0x53, 0x0B, 0x01, 0x02, 0xFF, 0x60, 0x00, 0x18, 0xFC, 0x00, 0x00, 0xFF, 0x45};    //L speed
unsigned char LSpeed[] = {0x53, 0x0B, 0x02, 0x02, 0xFF, 0x60, 0x00, 0x18, 0xFC, 0x00, 0x00, 0xFF, 0x45};    //R speed


int speed, rotateSpeed;


int main() 
{ 
  int fd, i, a;
  int res;
  struct termios oldtio, newtio;
  char ch;
  unsigned char outcome;

  printf("init...\n");
  
  fd = open(MODEMDEVICE, O_RDWR|O_NOCTTY);  //O_RDWR �iŪ�i�g,O_NOCTTY �p�G���|�W���V�׺ݳ]��,���n��o�ӳ]�ƥΧ@����׺�  
  if (fd < 0)                               //�p�G���]�w�o�ӺX�СA���ǿ�J(�Ҧp��L��abort)�H���i��v�T�{���C
  {                             
    perror(MODEMDEVICE);
    exit(1);
  }


  sleep(5);
  //res = write(fd, Speed0, sizeof(Speed0));

  tcgetattr(fd, &oldtio);    //�ΨӨ��o�ثe����C��ѼƭȡAtcgetattr()���o�ɮ״y�z�lfd��A�N��s�J&oldtio
  bzero(&newtio, sizeof(newtio));

  newtio.c_cflag = BAUDRATE|CS8|CLOCAL|CREAD;  //�j�v�]�w�U�r�Ū���:8bits�U�����ը�ѽվ��u�����A�U�ϥα�����
  newtio.c_iflag = IGNPAR;  //�����_�ե����~
  newtio.c_oflag = 0;
  newtio.c_lflag = ICANON;  //�ϥμзǿ�J�Ҧ�

  tcflush(fd, TCIFLUSH);  //�M���Ҧ���C�b��C�𪺿�J�M��X�A�䤤fd���Q�B�z����C��A�޼�TCIFLUSH���M����J 
  tcsetattr(fd, TCSANOW, &newtio);     //�Ψӳ]�w��C��ѼƭȡAtcsetattr()�����ϥ�newtio���V��termios��Ƶ��c�A���s�]�w�ɮ״y�z�lfd�A�䤤�޼�TCSANOW�O�ߧY�N�ȧ���

  //send ini param
  res = write(fd, ini0, sizeof(ini0));
  res = write(fd, ini00, sizeof(ini00));
  res = write(fd, ini1, sizeof(ini1));
  res = write(fd, ini2, sizeof(ini2));
  res = write(fd, ini3, sizeof(ini3));
  res = write(fd, ini4, sizeof(ini4));
  res = write(fd, ini5, sizeof(ini5));

  
  res = write(fd, ini6, sizeof(ini6));
  res = write(fd, ini7, sizeof(ini7));
  res = write(fd, ini8, sizeof(ini8));
  res = write(fd, ini9, sizeof(ini9));
  res = write(fd, ini10, sizeof(ini10));
  
  res = write(fd, ini11, sizeof(ini11));
  res = write(fd, ini12, sizeof(ini12));
  res = write(fd, ini13, sizeof(ini13));
  res = write(fd, ini14, sizeof(ini14));
  res = write(fd, ini15, sizeof(ini15));
  
  res = write(fd, ini16, sizeof(ini16));
  res = write(fd, ini17, sizeof(ini17));
  res = write(fd, ini18, sizeof(ini18));
  res = write(fd, ini19, sizeof(ini19));
  res = write(fd, ini20, sizeof(ini20));
  
  res = write(fd, ini21, sizeof(ini21));
  res = write(fd, ini22, sizeof(ini22));
  res = write(fd, ini23, sizeof(ini23));



  //while(1)
  do
  {
    cout<<"Enter cruise speed and press Enter"<<endl;
    cin >> speed;

    cout<<"Enter rotate speed and press Enter"<<endl;
    cin >> rotateSpeed;

    cout << "Start to remote control (w/a/s/d/space) & Press r to reset speed or Press q to Exit\n";
    do
    {
      ch = getch();

      switch (ch)
      {
      case 'w':
      case 'W':
        cout << "forward \n";
        outcome = CRCSpeed(0-speed, RSpeed, sizeof(RSpeed) - 3);
        res = write(fd, RSpeed, sizeof(RSpeed));
        outcome = CRCSpeed(speed, LSpeed, sizeof(LSpeed) - 3);
        res = write(fd, LSpeed, sizeof(LSpeed));
        break;
      case 'a':
      case 'A':
        cout << "CCW \n";
        outcome = CRCSpeed(rotateSpeed, Speed, sizeof(Speed) - 3);
        res = write(fd, Speed, sizeof(Speed));
        break;
      case 's':
      case 'S':
        cout << "backward \n";
        outcome = CRCSpeed(speed, RSpeed, sizeof(RSpeed) - 3);
        res = write(fd, RSpeed, sizeof(RSpeed));
        outcome = CRCSpeed(0-speed, LSpeed, sizeof(LSpeed) - 3);
        res = write(fd, LSpeed, sizeof(LSpeed));
        break;
      case 'd':
      case 'D':
        cout << "CW \n";
        outcome = CRCSpeed(0-rotateSpeed, Speed, sizeof(Speed) - 3);
        res = write(fd, Speed, sizeof(Speed));
        break;

      case 32:
        cout << "STOP \n";
        outcome = CRCSpeed(0, Speed, sizeof(Speed) - 3);
        res = write(fd, Speed, sizeof(Speed));
        break;

      }

    } while (ch != 'R' && ch != 'r' && ch != 'q' && ch != 'Q');

  } while (ch != 'q' && ch != 'Q');
  close(fd);
  return 0; 
}


//===============Function()===============================//
unsigned char CalcCRCByte(unsigned char u8Byte, unsigned char u8CRC)
{
  unsigned char i;
  u8CRC = u8CRC ^ u8Byte;
  for (i = 0; i < 8; i++)
  {
    if (u8CRC & 0x01)
    {
      u8CRC = (u8CRC >> 1) ^ polynomial;
    }
    else
    {
      u8CRC >>= 1;
    }
  }
  return u8CRC;
}

unsigned char CRCSpeed(int speed, unsigned char *outStr, int cmdSize)
{
  // Speed Dec to Hex
  char buffer[8];
  sprintf(buffer,"%X",speed);
  //itoa(speed, buffer, 16);
  //cout << "Dec:" << speed << " = Hex:" << buffer << endl;
  string str = buffer;
  int tempSize = str.size();
  for (int i = 0; i < (8 - tempSize); i++)
    str = "0" + str;

  const int numBytes = 4;
  for (int i = 0; i < numBytes; ++i)
  {
    // grab two characters from the string...
    std::string twoChars = str.substr(2 * i, 2);

    // convert them to an integer using a stringstream
    int byte;
    std::stringstream ss(twoChars);
    ss >> std::hex >> byte;

    // store the result in char array
    outStr[10 - i] = byte;
  }

  // calculate CRC
  unsigned char crc = 0xFF;

  for (int i = 1; i <= cmdSize; i++)
  {
    //cout<<hex(cmdArray[i]) <<endl;
    crc = CalcCRCByte(outStr[i], crc);
  }

  outStr[11] = crc;
  return crc;
}

unsigned char CRC(unsigned char cmdArray[], int cmdSize)
{

  unsigned char crc = 0xFF;

  for (int i = 1; i <= cmdSize; i++)
  {
    //cout << hex(cmdArray[i]) << endl;
    crc = CalcCRCByte(cmdArray[i], crc);
    //cout << hex(crc) << endl;
  }
  return crc;
}


inline std::ostream &operator<<(std::ostream &o, const HexCharStruct &hs)
{
  return (o << std::hex << (int)hs.c);
}

inline HexCharStruct hex(unsigned char _c)
{
  return HexCharStruct(_c);
}