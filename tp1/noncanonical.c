/*Non-Canonical Input Processing*/

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>

#define BAUDRATE B38400
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define FALSE 0
#define TRUE 1

volatile int STOP=FALSE;

int main(int argc, char** argv)
{
    int fd,c, res;
    struct termios oldtio, newtio;
    unsigned char buf;

    if ( (argc < 2) || 
  	     ((strcmp("/dev/ttyS0", argv[1])!=0) && 
  	      (strcmp("/dev/ttyS1", argv[1])!=0) )) {
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

    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    /* set input mode (non-canonical, no echo,...) */
    newtio.c_lflag = 0;

    newtio.c_cc[VTIME]    = 1;   /* inter-character timer unused */
    newtio.c_cc[VMIN]     = 5;   /* blocking read until 5 chars received */



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

    int send = 0;
    
    read(fd,&buf,1); // reads f
    
    if(buf == 0x7E){
        printf("first f");
        read(fd,&buf,1); // reads a
        read(fd,&buf,1); // reads c
        read(fd,&buf,1); // reads bcc
        if(buf == 0x03 ^ 0x03){
            printf("bcc");
            read(fd,&buf,1); // reads f
            if(buf == 0x7E){
                printf("second f");
                send = 1;
                printf("Mensagem Recebida: %s\n\nEnviando       Confirmacao...\n", buf);
            }
        }
    }

    if(send){
        unsigned char f = 0x7E, a = 0x01, c = 0x07, bcc = a ^ c;
        write(fd, &f, 1);
        write(fd, &a, 1);
        write(fd, &c, 1);
        write(fd, &bcc, 1);
        write(fd, &f, 1);
    }


    /*num = 0;

    while(num < strlen(buf)){
      write(fd, &buf[num], 1);
      num++;
    }
    buf[strlen(buf)] = '\0';
    write(fd, &buf[strlen(buf)], 1);

    printf("Mensagem Enviada!\n");*/



  /* 
    O ciclo WHILE deve ser alterado de modo a respeitar o indicado no gui�o 
  */


    sleep(1);
    tcsetattr(fd,TCSANOW,&oldtio);
    close(fd);
    return 0;
}
