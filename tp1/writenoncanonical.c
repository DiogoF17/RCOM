/*Non-Canonical Input Processing*/

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>

#define BAUDRATE B38400
#define MODEMDEVICE "/dev/ttyS1"
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define FALSE 0
#define TRUE 1

#define START 1
#define FLAG_RCV 2
#define A_RCV 3
#define C_RCV 4
#define BCC_OK 5
#define FINAL 6

#define TRANSMITTER 1

#define MAX_DATA_PACKET 512

struct termios oldtio,newtio;

volatile int STOP=FALSE;

int tentativas= 0, timeOut_var = 0;

int S = 0;

int nSeq = 0;

//-----------------------------
//    USADAS EM LLOPEN
//-----------------------------

void connect(int fd){
    unsigned char f = 0x7E, a = 0x03, c = 0x03, bcc = a ^ c;
    write(fd, &f, 1);
    write(fd, &a, 1);
    write(fd, &c, 1);
    write(fd, &bcc, 1);
    write(fd, &f, 1);

    alarm(3); // quantidade de tempo que espera pelo acknowledgment
}

int waitConnect(int fd){
  int estado = START;
  unsigned char buf;
  timeOut_var = 0;

  // State Machine For the Connection Request
  while(1){
    if(timeOut_var) return 0;

    if(read(fd, &buf, 1) != 0){ // caso tenha lido alguma coisa, ou seja, buf nao esta vazio
      if(estado == START){
        if(buf == 0x7E) estado = FLAG_RCV;
      }
      else if(estado == FLAG_RCV){
        if(buf == 0x01) estado = A_RCV;
        else if(buf != 0x7E) estado = START;
      }
      else if(estado == A_RCV){
        if(buf == 0x7E) estado = FLAG_RCV;
        else if(buf == 0x07) estado = C_RCV;
        else estado = START;
      }
      else if(estado == C_RCV){
        if(buf == 0x7E) estado = FLAG_RCV;
        else if(buf == (0x01 ^ 0x07)) estado = BCC_OK;
        else estado = START;
      }
      else if(estado == BCC_OK){
        if(buf == 0x7E){ 
          estado = FINAL;
          break;
        }
        else estado = START;
      }
    }
  }
  
  alarm(0); // cancela todos os alarmes pendentes*/
  return 1;
}

int llopen(int porta, int user){
  char buf[50];
  sprintf(buf, "/dev/ttyS%d", porta);
        
  int fd = open(buf, O_RDWR | O_NOCTTY );
  if (fd <0) {perror(buf); exit(-1); }

  if (tcgetattr(fd,&oldtio) == -1) { 
    perror("tcgetattr");
    exit(-1);
  }

  bzero(&newtio, sizeof(newtio));
  newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR;
  newtio.c_oflag = 0;
  newtio.c_lflag = 0;
  newtio.c_cc[VTIME]    = 0;   
  newtio.c_cc[VMIN]     = 0;   

  tcflush(fd, TCIOFLUSH);

  if ( tcsetattr(fd,TCSANOW,&newtio) == -1) {
    perror("tcsetattr");
    exit(-1);
  }

  while(tentativas != 3) {
        connect(fd);
        if(waitConnect(fd)) return fd; // ligacao estabelecida com sucesso, retorna descritor     
  }

  return -1; // numero de tentativas excedido, retorna valor negativo
}

//-------------------------------

void disconnect(int fd){
  unsigned char f = 0x7E, a = 0x03, c = 0x0B, bcc = a ^ c;

  write(fd, &f, 1);
  write(fd, &a, 1);
  write(fd, &c, 1);
  write(fd, &bcc, 1);
  write(fd, &f, 1);

  alarm(3); // quantidade de tempo que espera pelo acknowledgment
}

int waitDisconnect(int fd){
  int estado = START;
  unsigned char buf;
  timeOut_var = 0;

  // State Machine For the Disconnection Request
  while(1){    
    if(timeOut_var) return 0;

    if(read(fd, &buf, 1) != 0){ // caso tenha lido alguma coisa, ou seja, buf nao esta vazio
      if(estado == START){
        if(buf == 0x7E) estado = FLAG_RCV;
      }
      else if(estado == FLAG_RCV){
        if(buf == 0x01) estado = A_RCV;
        else if(buf != 0x7E) estado = START;
      }
      else if(estado == A_RCV){
        if(buf == 0x7E) estado = FLAG_RCV;
        else if(buf == 0x0B) estado = C_RCV;
        else estado = START;
      }
      else if(estado == C_RCV){
        if(buf == 0x7E) estado = FLAG_RCV;
        else if(buf == (0x01 ^ 0x0B)) estado = BCC_OK;
        else estado = START;
      }
      else if(estado == BCC_OK){
        if(buf == 0x7E){ 
          estado = FINAL;
          break;
        }
        else estado = START;
      }
      else  break;
    }
  }
  
  alarm(0); // cancela todos os alarmes pendentes*/
  return 1;
}

void sendAcknowledgement(int fd){
  unsigned char f = 0x7E, a = 0x03, c = 0x07, bcc = a ^ c;

  write(fd, &f, 1);
  write(fd, &a, 1);
  write(fd, &c, 1);
  write(fd, &bcc, 1);
  write(fd, &f, 1);
}

int llclose(int fd){
  while(tentativas != 3) {
        disconnect(fd);
        if(waitDisconnect(fd)){
          sendAcknowledgement(fd);
          
          if (tcsetattr(fd,TCSANOW,&oldtio) == -1) {
            perror("tcsetattr");
            exit(-1);
          }

          close(fd);

          return 1; // ligacao estabelecida com sucesso, retorna descritor     
        } 
  }

  if (tcsetattr(fd,TCSANOW,&oldtio) == -1) {
    perror("tcsetattr");
    exit(-1);
  }

  close(fd);

  return -1;
}

void timeOut(){
  timeOut_var = 1;
  tentativas++;/*
  if(fase == CONNECT) connect();
  else if(fase == DATA_TRANSMISSION){
      timeOut = 1;
      tentativas++;
  } // resend_data()
  else disconnect();*/
}

//-----------------------------
//    USADAS PARA ESCREVER INFO
//-----------------------------

void writeInfo(int fd, char *buffer, int length){

    unsigned char f = 0x7E, a = 0x03, c, bcc1, bcc2 = buffer[0];

    if(S) c = 0x40;
    else c = 0x00;

    bcc1 = a ^ c;

    write(fd, &f, 1);
    write(fd, &a, 1);
    write(fd, &c, 1);
    write(fd, &bcc1, 1);

    int cont = 1;
    while(cont < (length-1)){
        write(fd, &buffer[cont], 1);
        bcc2 = (bcc2 ^ buffer[cont]);
        cont++;
    }

    write(fd, &bcc2, 1);
    write(fd, &f, 1);

    alarm(3); // quantidade de tempo que espera pelo acknowledgment

}

int waitingAcknowledgmentInfo(int fd){
  int estado = START;
  unsigned char buf;
  int rec = 1;
  timeOut_var = 0;

  // State Machine For the Connection Request
  while(1){
    if(timeOut_var){
      rec = 0;
      break;
    }
    if(estado == START){
      read(fd, &buf, 1);
      if(buf == 0x7E) estado = FLAG_RCV;
    }
    else if(estado == FLAG_RCV){
      read(fd, &buf, 1);
      if(buf == 0x01) estado = A_RCV;
      else if(buf != 0x7E) estado = START;
    }
    else if(estado == A_RCV){
      read(fd, &buf, 1);
      if(buf == 0x7E) estado = FLAG_RCV;
      else if(buf == 0x07) estado = C_RCV;
      else estado = START;
    }
    else if(estado == C_RCV){
      read(fd, &buf, 1);
      if(buf == 0x7E) estado = FLAG_RCV;
      else if(buf == (0x01 ^ 0x07)) estado = BCC_OK;
      else estado = START;
    }
    else if(estado == BCC_OK){
      read(fd, &buf, 1);
      if(buf == 0x85 || buf == 0x81){
        estado = FINAL;
        S = 1;
      }
      else if(buf == 0x05 || buf == 0x01){
        estado = FINAL;
        S = 0;
      }
      else estado = START;
      if(buf == 0x81 || buf == 0x01) rec = 0; // info rejeitada
    }
    else  break;
  }
  
  alarm(0); // cancela todos os alarmes pendentes*/

  return rec;
}

//-----------------------------

void dataPacket(unsigned char *data, char *buf, int length){
  if(MAX_DATA_PACKET < (length + 4)){
    printf("Number of bytes of data packet exceded");
    exit(1);
  }

  data[0] = 0x01;
  data[1] = (unsigned char) nSeq;
  data[2] = (unsigned char) (length / 256);
  data[3] = (unsigned char) (length % 256);

  int aux = 0;
  while(aux < length){
    data[aux + 4] = (unsigned char) buf[length];
    aux++;
  } 
}

void controlPacket(unsigned char *data, int controll, int size){
  data[0] = (unsigned char) controll;
  data[1] = 0x00;
  data[2] = (unsigned char) (sizeof(int));
  data[3] = (unsigned char) size;
}

int llwrite(int fd, char *buf, int length) {
  
  while(tentativas != 3) {
      writeInfo(fd, buf, length + 4);
      if(waitingAcknowledgmentInfo(fd)) return length; // informacao enviada corretamente         
  }

  return -1; // numero de tentativas excedido
}

int fileSize(char *path){
  FILE* fp = fopen(path, "r");
  fseek(fp, 0, SEEK_END);
  int size = ftell(fp);
  fclose(fp);

  return size;
}

int main(int argc, char** argv){
  //char buf[MAX_DATA_PACKET - 4];

  if (argc < 3) {
    printf("Usage:\tnserial SerialPort\n\tex: nserial 11 path\n");
    exit(1);
  }

  /*int foto = open(argv[2], O_RDWR);
  if (foto < 0) {
      printf("An error has occured during the opening of the file");
      exit(1);
  }

  int size = fileSize(argv[2]); // size of file*/

  //=====================
  (void) signal(SIGALRM, timeOut);
  //=====================

  int fd = llopen(atoi(argv[1]), TRANSMITTER); // tries to connect with the receiver
  if(fd < 0){
    printf("An error has occured during the conncetion process!\n");
    exit(1);
  } 

  //--------------
/*
  tentativas = 0;

  unsigned char data[MAX_DATA_PACKET];

  controlPacket(data, 2, size); // cria o controllPacket de inicio
  if(llwrite(fd, data, strlen(data)) < 0){
    printf("an error has occured during the transfer of data");
    exit(1);
  }

  int nr;
  while(1){
    tentativas = 0;

    nr = read(foto, buf, MAX_DATA_PACKET - 4);
    
    if(nr == 0) break; // ja leu o ficheiro todo
    else if(nr < 0){
      printf("An error has occured during the reading");
      exit(1);
    }

    dataPacket(data, buf, strlen(buf));
    if(llwrite(fd, data, strlen(data))){
      printf("an error has occured during the transfer of data");
      exit(1);
    }
  }

  tentativas = 0;

  controlPacket(data, 3, size); // cria o controllPacket de fim
  if(llwrite(fd, data, strlen(data))){
    printf("an error has occured during the transfer of data");
    exit(1);
  }
  */

  //--------------

  tentativas = 0;

  int ret = llclose(fd); // tries to connect with the receiver
  if(ret < 0){
    printf("An error has occured during the disconnection process!\n");
    exit(1);
  } 


  //=================

  // Connecting
  /*connect(); // sends request to establish connection
  waitConnect(); // waits for the receiver acknowledgment

  // Sending Data
  tentativas = 0;
  fase = DATA_TRANSMISSION;
  //send_data()

  // Disconnecting
  tentativas = 0;
  fase = DISCONNECT;
  disconnect();
  waitingDisconnect();
  sendAcknowledgement();*/

  //=================

  return 0;
}
