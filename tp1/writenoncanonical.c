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

volatile int STOP=FALSE;

int tentativas= 0, fd;

void establishConnection(){
  if(tentativas != 3){
    unsigned char f = 0x7E, a = 0x03, c = 0x03, bcc = a ^ c;

    write(fd, &f, 1);
    write(fd, &a, 1);
    write(fd, &c, 1);
    write(fd, &bcc, 1);
    write(fd, &f, 1);

    if(tentativas != 0)
      printf("\nReenviando Pedido de Conexao Enviado!\n\nEspera por Acknowledgment...\n");
    else
      printf("\nPedido de Conexao Enviado!\n\nEspera por Acknowledgment...\n");
    alarm(3); // quantidade de tempo que espera pelo acknowledgment
  }
  else{
    printf("\nAbortando Estabelecimento de Conexao com o Recetor!\nAtingiu o limite de tentativas!\n\n");
    exit(-1);
  }
  tentativas++;
}

void waitingAcknowledgment(){
  unsigned char buf;

  /*read(fd,&buf,1); // reads f
  if(buf == 0x7E){
    read(fd, &buf, 1); // reads a
    read(fd, &buf, 1); // reads c
    read(fd, &buf, 1); // reads bcc
    if(buf == (0x01 ^ 0x07)){
        read(fd, &buf, 1); // reads f
        if(buf == 0x7E) {
          printf("Acknowledgment Recebido!\n\n");
        }
    }
  }*/

  do{
    read(fd, &buf, 1);
  } while(buf != (0x01 ^ 0x07)); // verifica se a trama recebida e valida

  printf("Acknowledgment Recebido!\n\n");
  alarm(0); // cancela todos os alarmes pendentes
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

  /*printf("Insert string: ");
  
  if(fgets(buf, 255, stdin) == NULL){
    printf("Erro!\n");
    exit(1);
  }
  
  int num = 0;
  while(num < strlen(buf)){
    write(fd, &buf[num], 1);
    num++;
  }
  buf[strlen(buf)] = '\0';
  write(fd, &buf[strlen(buf)], 1);*/
  //printf("Mensagem Enviada!\nEspera por Confirmacao...\n\n");

  //=================

  (void) signal(SIGALRM, establishConnection);

  establishConnection(); // sends request to establish connection
  waitingAcknowledgment(); // waits for the receiver acknowledgment

  //=================

  /*num = 0;
  while(1){
    read(fd, &buf[num], 1);
    if(buf[num] == '\0') break;
    num++;
  }

  printf("Mensagem Recebida: %s\n", buf);*/

  /* 
    O ciclo FOR e as instru��es seguintes devem ser alterados de modo a respeitar 
    o indicado no gui�o 
  */

  printf("==============\n\n");

  if ( tcsetattr(fd,TCSANOW,&oldtio) == -1) {
    perror("tcsetattr");
    exit(-1);
  }

  close(fd);

  return 0;
}
