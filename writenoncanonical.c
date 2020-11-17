/*Non-Canonical Input Processing*/

#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include <sys/time.h>

#include "macros.h"


struct termios oldtio, newtio;

int tentativas = 0, timeOut_var = 0;
int S = 0;
int nSeq = 0;
int bcc2;

//-----------------------------
//    USADAS EM LLOPEN
//-----------------------------

void connect(int fd) {
  unsigned char f = FLAG, a = 0x03, c = 0x03, bcc = a ^ c;

  if(write(fd, &f, 1) <= 0) printf("Error on Write1!\n");
  if(write(fd, &a, 1) <= 0) printf("Error on Write2!\n");
  if(write(fd, &c, 1) <= 0) printf("Error on Write3!\n");
  if(write(fd, &bcc, 1) <= 0) printf("Error on Write4!\n");
  if(write(fd, &f, 1) <= 0) printf("Error on Write5!\n");

  alarm(TIMEOUT);  // quantidade de tempo que espera pelo acknowledgment
}

int waitConnect(int fd) {
  int state = START;
  unsigned char buf;
  timeOut_var = 0;

  // State Machine For the Connection Request
  while (1) {
    if (timeOut_var)
      return 0;

    if (read(fd, &buf, 1) !=
        0) {  // caso tenha lido alguma coisa, ou seja, buf nao esta vazio
      if (state == START) {
        if (buf == FLAG)
          state = FLAG_OK;
      } else if (state == FLAG_OK) {
        if (buf == 0x01)
          state = A_OK;
        else if (buf != FLAG)
          state = START;
      } else if (state == A_OK) {
        if (buf == FLAG)
          state = FLAG_OK;
        else if (buf == C_RCV)
          state = C_OK;
        else
          state = START;
      } else if (state == C_OK) {
        if (buf == FLAG)
          state = FLAG_OK;
        else if (buf == (0x01 ^ C_RCV))
          state = BCC_OK;
        else
          state = START;
      } else if (state == BCC_OK) {
        if (buf == FLAG) {
          state = STOP;
          break;
        } else
          state = START;
      }
    }
  }

  alarm(0);  // cancela todos os alarmes pendentes*/
  return 1;
}

int llopen(int porta, int user) {
  char buf[50];
  sprintf(buf, "/dev/ttyS%d", porta);

  int fd = open(buf, O_RDWR | O_NOCTTY);
  if (fd < 0) {
    perror(buf);
    exit(-1);
  }

  if (tcgetattr(fd, &oldtio) == -1) {
    perror("tcgetattr");
    exit(-1);
  }

  bzero(&newtio, sizeof(newtio));
  newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR;
  newtio.c_oflag = 0;
  newtio.c_lflag = 0;
  newtio.c_cc[VTIME] = 0;
  newtio.c_cc[VMIN] = 0;

  tcflush(fd, TCIOFLUSH);

  if (tcsetattr(fd, TCSANOW, &newtio) == -1) {
    perror("tcsetattr");
    exit(-1);
  }

  while (tentativas != MAX_TRIES) {
    connect(fd);
    printf("\nSENT: Connection Request\n");
    if (waitConnect(fd)) {
      printf("RECEIVED: Connection Acknowledgment\n\n");
      return fd;  // ligacao estabelecida com sucesso, retorna descritor
    }
  }

  return -1;  // numero de tentativas excedido, retorna valor negativo
}

//-----------------------------
//    USADAS EM LLCLOSE
//-----------------------------

void disconnect(int fd) {
  unsigned char f = FLAG, a = 0x03, c = C_DISC, bcc = a ^ c;

  write(fd, &f, 1);
  write(fd, &a, 1);
  write(fd, &c, 1);
  write(fd, &bcc, 1);
  write(fd, &f, 1);

  alarm(TIMEOUT);  // quantidade de tempo que espera pelo acknowledgment
}

int waitDisconnect(int fd) {
  int state = START;
  unsigned char buf;
  timeOut_var = 0;

  // State Machine For the Disconnection Request
  while (1) {
    if (timeOut_var)
      return 0;

    if (read(fd, &buf, 1) !=
        0) {  // caso tenha lido alguma coisa, ou seja, buf nao esta vazio
      if (state == START) {
        if (buf == FLAG)
          state = FLAG_OK;
      } else if (state == FLAG_OK) {
        if (buf == 0x01)
          state = A_OK;
        else if (buf != FLAG)
          state = START;
      } else if (state == A_OK) {
        if (buf == FLAG)
          state = FLAG_OK;
        else if (buf == C_DISC)
          state = C_OK;
        else
          state = START;
      } else if (state == C_OK) {
        if (buf == FLAG)
          state = FLAG_OK;
        else if (buf == (0x01 ^ C_DISC))
          state = BCC_OK;
        else
          state = START;
      } else if (state == BCC_OK) {
        if (buf == FLAG) {
          state = STOP;
          break;
        } else
          state = START;
      } else
        break;
    }
  }

  alarm(0);  // cancela todos os alarmes pendentes*/
  return 1;
}

void sendAcknowledgement(int fd) {
  unsigned char f = FLAG, a = 0x03, c = C_RCV, bcc = a ^ c;

  write(fd, &f, 1);
  write(fd, &a, 1);
  write(fd, &c, 1);
  write(fd, &bcc, 1);
  write(fd, &f, 1);
}

int llclose(int fd) {
  while (tentativas != MAX_TRIES) {
    disconnect(fd);
    printf("\nSENT: Disconnection Request\n");
    if (waitDisconnect(fd)) {
      printf("RECEIVED: Disconnection Request\n");
      sendAcknowledgement(fd);
      printf("SENT: Disconnection Acknowledgement\n");

      sleep(1);
      if (tcsetattr(fd, TCSANOW, &oldtio) == -1) {
        perror("tcsetattr");
        exit(-1);
      }

      close(fd);

      return 1;  // ligacao estabelecida com sucesso, retorna descritor
    }
  }

  if (tcsetattr(fd, TCSANOW, &oldtio) == -1) {
    perror("tcsetattr");
    exit(-1);
  }

  close(fd);

  return -1;
}

//-----------------------------
//    USADAS EM LLWRITE
//-----------------------------

int doStuffing(unsigned char* stuffed_data, char* buf, int length) {
  int indStuffing = 0, indBuf = 0;

  while (indBuf < length) {
    unsigned char data = (unsigned char)buf[indBuf];
    if (data == FLAG) {
      stuffed_data[indStuffing++] = (unsigned char)ESC;
      stuffed_data[indStuffing] = (unsigned char)0x5E;
    } else if (data == ESC) {
      stuffed_data[indStuffing++] = (unsigned char)ESC;
      stuffed_data[indStuffing] = (unsigned char)0x5D;
    } else {
      stuffed_data[indStuffing] = data;
    }

    indStuffing++;
    indBuf++;
  }

  return indStuffing;
}

int dataPacket(char* data, char* buf, int length) {
  if (MAX_DATA_PACKET < (length + 4)) {
    printf("Number of bytes of data packet exceded");
    exit(1);
  }

  data[0] = 0x01;
  data[1] = nSeq;
  data[2] = (length / 256);
  data[3] = (length % 256);

  int aux = 4, cont = 0;
  while (cont < length) {
    data[aux] = buf[cont];
    aux++;
    cont++;
  }

  return aux;
}

int controlPacket(char* packet, int controll, int size, char* filename) {
  int packet_size = 5;

  packet[0] = controll;
  packet[1] = 0;
  packet[2] = sizeof(size);
  memcpy(packet + 3, &size, sizeof(size));
  packet_size += sizeof(size);

  packet[3 + sizeof(size)] = 1;
  packet[4 + sizeof(size)] = strlen(filename);
  memcpy(packet + 5 + sizeof(size), filename, strlen(filename));
  packet_size += strlen(filename);

  return packet_size;
}

int fileSize(char* path) {
  FILE* fp = fopen(path, "r");
  fseek(fp, 0L, SEEK_END);
  int size = ftell(fp);
  fclose(fp);

  return size;
}

char getBcc2(char* buf, int length) {
  char bcc2 = buf[0];

  int cont = 1;
  while (cont < length) {
    char oct = buf[cont];
    bcc2 = (bcc2 ^ oct);
    cont++;
  }

  return bcc2;
}

void writeInfo(int fd, char* buffer, int length) {
  unsigned char f = FLAG, a = 0x03, c, bcc1;
  char bcc2 = getBcc2(buffer, length);

  if (S) c = C_I1;
  else c = C_I0;

  bcc1 = a ^ c;

  unsigned char stuffed_data[MAX_DATA_WITH_STUFFING], frame[MAX_FRAME];
  char to_stuff[MAX_DATA_WITHOUT_STUFFING];

  int cont = 0;
  while(cont < length) {
    to_stuff[cont] = buffer[cont];
    cont++;
  }
  to_stuff[cont] = bcc2;
  cont++;

  int size = doStuffing(stuffed_data, to_stuff, cont);
  
  frame[0] = f;
  frame[1] = a;
  frame[2] = c;
  frame[3] = bcc1;

  cont = 0;
  while (cont < size) {
    frame[cont + 4] = stuffed_data[cont];
    cont++;
  }

  frame[cont + 4] = f;

  write(fd, frame, cont + 5);  // write the stuffed frame to the receiver

  alarm(TIMEOUT);  // quantidade de tempo que espera pelo acknowledgment
}

int waitingAcknowledgmentInfo(int fd) {
  int state = START;
  unsigned char read_byte, c;
  int rec = 1;
  timeOut_var = 0;

  // State Machine For the Connection Request
  while (state != STOP) {
    if (timeOut_var) return 0;

    if (read(fd, &read_byte, 1) != 0) {
      switch (state) {
        case START:
          if (read_byte == FLAG)
            state = FLAG_OK;
          break;

        case FLAG_OK:
          if (read_byte == 0x01)
            state = A_OK;
          else if (read_byte != FLAG)
            state = START;
          break;

        case A_OK:
          if (read_byte == FLAG)
            state = FLAG_OK;
          else if (read_byte == C_RR0 || read_byte == C_RR1 || read_byte == C_REJ1 || read_byte == C_REJ0) {
            if (read_byte == C_RR1) S = 1;
            else if (read_byte == C_RR0) S = 0;
            else rec = 0;  // info rejeitada

            state = C_OK;
            c = read_byte;
          } 
          else
            state = START;
          break;

        case C_OK:
          if (read_byte == FLAG)
            state = FLAG_OK;
          else if (read_byte == (0x01 ^ c))
            state = BCC_OK;
          else
            state = START;
          break;

        case BCC_OK:
          if (read_byte == FLAG) {
            state = STOP;
            break;
          } else
            state = START;
          break;

        default:
          break;
      }
    }
  }

  alarm(0);  // cancela todos os alarmes pendentes*/
  return rec;
}

int llwrite(int fd, char* buf, int length) {
  int cont = 0;
  while (tentativas != MAX_TRIES) {
    writeInfo(fd, buf, length);
    if (waitingAcknowledgmentInfo(fd))
      return length;  // informacao enviada corretamente 
    printf("RESENDING DATA: %d\n", cont++);
    printf("-----------------------------------------\n");
  }
  return -1;  // numero de tentativas excedido
}

//-----------------------------

void timeOut() {
  tentativas++;
  timeOut_var = 1;
  printf("Connection timed out. Retrying...(failed attempt number %d) \n",
         tentativas);
}

//-----------------------------

int main(int argc, char** argv) {
  if (argc != 3) {
    printf(
        "Usage:\t ./send SerialPortNumber FilePath\nExample: ./send \t      11 "
        "pinguim.gif\n");
    exit(1);
  }

  int foto = open(argv[2], O_RDWR);
  if (foto < 0) {
    printf("An error has occured during the opening of the file\n");
    exit(1);
  }

  //=====================
  (void)signal(SIGALRM, timeOut);

  // Opening Connection
  //================================
  int fd = llopen(atoi(argv[1]), TRANSMITTER);
  if (fd < 0) {
    printf("An error has occured during the conncetion process\n");
    exit(1);
  }

  printf("Connection established SUCCESSFULLY! \n\n");

  // Sending controll packet (START)
  //================================
  tentativas = 0;

  int size = fileSize(argv[2]);
  char* filename = argv[2];  // size of file
  char control_packet[MAX_READ];

  int packet_size = controlPacket(control_packet, 2, size,
                                  filename);  // cria o controllPacket de fim
  printf("Start Packet Size: %d\n", packet_size);

  if (llwrite(fd, control_packet, packet_size) < 0) {
    printf("An error has occured during the transfer of the start packet\n");
    exit(1);
  }

  // Sending data packets
  //================================
  int nr;
  char buf[MAX_READ];
  char data[MAX_DATA_PACKET];
  int processed_bytes = 0;

  while (1) {
    tentativas = 0;

    nr = read(foto, buf, MAX_READ);

    if (nr == 0)
      break;  // ja leu o ficheiro todo
    else if (nr < 0) {
      printf("An error has occured during the reading\n");
      exit(1);
    }

    int numBytes = dataPacket(data, buf, nr);
    if (llwrite(fd, data, numBytes) == -1) {
      printf("An error has occured during the transfer of data\n");
      exit(1);
    }

    processed_bytes += nr;
    printf("Data sent: %d of %d\n", processed_bytes, size);
    printf("-----------------------------------------\n");
  }

  // Sending controll packet (END)
  //================================
  tentativas = 0;

  packet_size = controlPacket(control_packet, 3, size,
                              filename);  // cria o controllPacket de fim
  printf("End Packet Size: %d\n", packet_size);

  if (llwrite(fd, control_packet, packet_size) < 0) {
    printf("An error has occured during the transfer of the start packet\n");
    exit(1);
  }

  // Closing Connection
  //================================
  tentativas = 0;

  int ret = llclose(fd);  // tries to connect with the receiver
  if (ret < 0) {
    printf("An error has occured during the disconnection process!\n");
    exit(1);
  }

  printf("\nConnection was closed SUCCESSFULLY!\n");

  return 0;
}
