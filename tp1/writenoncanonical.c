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

#define CONNECT 0
#define DATA_TRANSMISSION 1
#define DISCONNECT 2

int fase = CONNECT;

volatile int STOP=FALSE;

int tentativas= 0, fd, timeOut = 0;

int R = 0;

#define START 1
#define FLAG_RCV 2
#define A_RCV 3
#define C_RCV 4
#define BCC_OK 5
#define STOP 6


void connect(){
    unsigned char f = 0x7E, a = 0x03, c = 0x03, bcc = a ^ c;

    write(fd, &f, 1);
    write(fd, &a, 1);
    write(fd, &c, 1);
    write(fd, &bcc, 1);
    write(fd, &f, 1);

    alarm(3); // quantidade de tempo que espera pelo acknowledgment

}

void waitingAcknowledgment(){
  int estado = START;
  unsigned char buf;

  // State Machine For the Connection Request
  while(1){
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
      if(buf == 0x7E) estado = STOP;
      else estado = START;
    }
    else  break;
  }
  
  alarm(0); // cancela todos os alarmes pendentes*/
  printf("Confirmation Received!\n\n");
}

void disconnect(){
  /*unsigned char f = 0x7E, a = 0x03, c = 0x0B, bcc = a ^ c;

  write(fd, &f, 1);
  write(fd, &a, 1);
  write(fd, &c, 1);
  write(fd, &bcc, 1);
  write(fd, &f, 1);

  printf("Disconnection Request Sent!\n\n");*/
  if(tentativas != 3){
    unsigned char f = 0x7E, a = 0x03, c = 0x0B, bcc = a ^ c;

    write(fd, &f, 1);
    write(fd, &a, 1);
    write(fd, &c, 1);
    write(fd, &bcc, 1);
    write(fd, &f, 1);

    if(tentativas != 0)
      printf("\nDisconnection Request Resent!\n\nWaiting for Confirmation...\n");
    else
      printf("\nDisconnection Request Sent!\n\nWaiting for Confirmation...\n");
    alarm(3); // quantidade de tempo que espera pelo acknowledgment
  }
  else{
    printf("\nAborting Disconnection with the Receiver!\nReached the Limit of Resquests!\n\n");
    exit(-1); // acaba o programa
  }
  tentativas++;
  
}

void waitingDisconnect(){
  int estado = START;
  unsigned char buf;

  // State Machine For the Connection Request
  while(1){
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
      else if(buf == 0x0B) estado = C_RCV;
      else estado = START;
    }
    else if(estado == C_RCV){
      read(fd, &buf, 1);
      if(buf == 0x7E) estado = FLAG_RCV;
      else if(buf == (0x01 ^ 0x0B)) estado = BCC_OK;
      else estado = START;
    }
    else if(estado == BCC_OK){
      read(fd, &buf, 1);
      if(buf == 0x7E) estado = STOP;
      else estado = START;
    }
    else  break;
  }
  
  alarm(0); // cancela todos os alarmes pendentes*/
  printf("Disconnection Request Received!\n\n");
}

void sendAcknowledgement(){
  unsigned char f = 0x7E, a = 0x03, c = 0x07, bcc = a ^ c;

  write(fd, &f, 1);
  write(fd, &a, 1);
  write(fd, &c, 1);
  write(fd, &bcc, 1);
  write(fd, &f, 1);

  printf("Confirmation Sent!\n\n");
}

void timeOut(){
  if(fase == CONNECT) connect();
  else if(fase == DATA_TRANSMISSION){
      timeOut = 1;
      tentativas++;
  } // resend_data()
  else disconnect();
}

void writeInfo(int fd, char *buffer, int length){

    unsigned char f = 0x7E, a = 0x03, c, bcc1 = a ^ c, bcc2 = buffer[0];

    if(R) c = 0x40;
    else c = 0x00;

    write(fd, &f, 1);
    write(fd, &a, 1);
    write(fd, &c, 1);
    write(fd, &bcc1, 1);

    int cont = 1;
    while(cont < (length-1)){
        write(fd, buffer[cont], 1);
        bcc2 = (bcc2 ^ buffer[cont]);
        cont++;
    }

    write(fd, &bcc2, 1);
    write(fd, &f, 1);

    alarm(3); // quantidade de tempo que espera pelo acknowledgment

}

void waitingAcknowledgmentInfo(){
  int estado = START;
  unsigned char buf;
  int rec = 1;
  timeOut = 0;

  // State Machine For the Connection Request
  while(1){
    if(timeOut) break;
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
      else if(buf == 0x85 || buf == 0x81){
        estado = STOP;
        R = 1;
      }
      else if(buf == 0x05 || buf == 0x01){
        estado = STOP;
        R = 0;
      }
      else estado = START;
      if(buf == 0x81 || buf == 0x01) rec = 0; // info rejeitada
    }
    else  break;
  }
  
  alarm(0); // cancela todos os alarmes pendentes*/

  return rec;
}

int llwrite(int fd, char *buf, int length) {
    
    while(tentativas != 3) {
        writeInfo(fd, buf, length);
        if(waitingAcknowledgmentInfo()) return length; // informacao enviada corretamente         
    }

    return -1; // numero de tentativas excedido
}

int llopen(int porta, int user){
        
  int fd = open(argv[1], O_RDWR | O_NOCTTY );
  if (fd <0) {perror(argv[1]); exit(-1); }

  if ( tcgetattr(fd,&oldtio) == -1) { /* save current port settings */
    perror("tcgetattr");
    exit(-1);
  }

  /*
  Configuração da Porta Série
  */
  bzero(&newtio, sizeof(newtio));
  newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR;
  newtio.c_oflag = 0;

  /* set input mode (non-canonical, no echo,...) */
  newtio.c_lflag = 0;

  newtio.c_cc[VTIME]    = 0;   /* inter-character timer unused */
  newtio.c_cc[VMIN]     = 1;   /* blocking read until 5 chars received */

  /* 
    VTIME e VMIN devem ser alterados de forma a proteger com um temporizador a 
    leitura do(s) pr�ximo(s) caracter(es)
  */

  tcflush(fd, TCIOFLUSH);

  if ( tcsetattr(fd,TCSANOW,&newtio) == -1) {
    perror("tcsetattr");
    exit(-1);
  }

  return fd;
}

int main(int argc, char** argv)
{
  struct termios oldtio,newtio;
  
  if ( (argc < 2) || 
        ((strcmp("/dev/ttyS10", argv[1])!=0) && 
        (strcmp("/dev/ttyS11", argv[1])!=0) )) {
    printf("Usage:\tnserial SerialPort\n\tex: nserial /dev/ttyS11\n");
    exit(1);
  }

  /*
    Open serial port device for reading and writing and not as controlling tty
    because we don't want to get killed if linenoise sends CTRL-C.
  */

  fd = open(argv[1], O_RDWR | O_NOCTTY );
  if (fd <0) {perror(argv[1]); exit(-1); }

  if ( tcgetattr(fd,&oldtio) == -1) { /* save current port settings */
    perror("tcgetattr");
    exit(-1);
  }

  /*
  Configuração da Porta Série
  */
  bzero(&newtio, sizeof(newtio));
  newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR;
  newtio.c_oflag = 0;

  /* set input mode (non-canonical, no echo,...) */
  newtio.c_lflag = 0;

  newtio.c_cc[VTIME]    = 0;   /* inter-character timer unused */
  newtio.c_cc[VMIN]     = 1;   /* blocking read until 5 chars received */

  /* 
    VTIME e VMIN devem ser alterados de forma a proteger com um temporizador a 
    leitura do(s) pr�ximo(s) caracter(es)
  */

  tcflush(fd, TCIOFLUSH);

  if ( tcsetattr(fd,TCSANOW,&newtio) == -1) {
    perror("tcsetattr");
    exit(-1);
  }

  printf("New termios structure set\n");
  printf("\n==============\n");

  //=================

  (void) signal(SIGALRM, timeOut);

  // Connecting
  connect(); // sends request to establish connection
  waitingAcknowledgment(); // waits for the receiver acknowledgment

  // Sending Data
  tentativas = 0;
  fase = DATA_TRANSMISSION;
  //send_data()

  // Disconnecting
  tentativas = 0;
  fase = DISCONNECT;
  disconnect();
  waitingDisconnect();
  sendAcknowledgement();

  //=================

  printf("==============\n\n");

  if ( tcsetattr(fd,TCSANOW,&oldtio) == -1) {
    perror("tcsetattr");
    exit(-1);
  }

  close(fd);

  return 0;
}
