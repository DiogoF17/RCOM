/*Non-Canonical Input Processing*/

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include "macros.h"


int fd;
int R = 0;
int data_correct = 0, duplicated = 0;
struct termios oldtio, newtio;
char* filename = "";

int receiveConnectionRequest() {
  int state = START;
  unsigned char buf;

  // State Machine For the Connection Request
  while (1) {
    // printf("reading\n", buf);
    if (read(fd, &buf, 1) != 0) {
      if (state == START) {
        if (buf == FLAG)
          state = FLAG_OK;
      } else if (state == FLAG_OK) {
        if (buf == 0x03)
          state = A_OK;
        else if (buf != FLAG)
          state = START;
      } else if (state == A_OK) {
        if (buf == FLAG)
          state = FLAG_OK;
        else if (buf == 0x03)
          state = C_OK;
        else
          state = START;
      } else if (state == C_OK) {
        if (buf == FLAG)
          state = FLAG_OK;
        else if (buf == (0x03 ^ 0x03))
          state = BCC_OK;
        else
          state = START;
      } else if (state == BCC_OK) {
        if (buf == FLAG) {
          state = STOP;
          return 0;
        } else
          state = START;
      } else
        return -1;
    }
  }
}

int sendAcknowledgment() {
  unsigned char ua[5] = {FLAG, A_RCV, C_RCV, BCC_RCV, FLAG};

  tcflush(fd, TCIOFLUSH);
  if (write(fd, ua, 5) == -1)
    return -1;

  return 0;
}

int llopen(int porta, int user) {
  char buf[50];
  sprintf(buf, "/dev/ttyS%d", porta);

  /*
    Open serial port device for reading and writing and not as controlling tty
    because we don't want to get killed if linenoise sends CTRL-C.
  */

  fd = open(buf, O_RDWR | O_NOCTTY);
  if (fd < 0) {
    perror(buf);
    exit(-1);
  }

  if (tcgetattr(fd, &oldtio) == -1) { /* save current port settings */
    perror("tcgetattr");
    exit(-1);
  }

  bzero(&newtio, sizeof(newtio));
  newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR;
  newtio.c_oflag = 0;

  /* set input mode (non-canonical, no echo,...) */
  newtio.c_lflag = 0;

  newtio.c_cc[VTIME] = 0;
  newtio.c_cc[VMIN] = 0;

  tcflush(fd, TCIOFLUSH);

  if (tcsetattr(fd, TCSANOW, &newtio) == -1) {
    perror("tcsetattr");
    exit(-1);
  }

  if (receiveConnectionRequest() == -1) {
    printf("Error in connection request!");
    return -1;
  }
  printf("\nRECEIVED: Connection Request\n");

  if (sendAcknowledgment() == -1) {
    printf("Error in connection response!");
    return -1;
  }
  printf("SENT: Connection Acknowledgment\n\n");

  return fd;
}

int waitingDisconnectionRequest() {
  int state = START;
  unsigned char buf;

  // State Machine For the Connection Request
  while (1) {
    if (read(fd, &buf, 1) != 0) {
      if (state == START) {
        if (buf == FLAG)
          state = FLAG_OK;
      } else if (state == FLAG_OK) {
        if (buf == A_SND)
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
        else if (buf == (0x03 ^ C_DISC))
          state = BCC_OK;
        else
          state = START;
      } else if (state == BCC_OK) {
        if (buf == FLAG) {
          state = STOP;
          return 0;
        } else
          state = START;
      } else
        return -1;
    }
  }
}

int sendingDisconnectionRequest() {
  unsigned char disc[5] = {FLAG, A_RCV, C_DISC, (A_RCV ^ C_DISC), FLAG};

  tcflush(fd, TCIOFLUSH);
  if (write(fd, disc, 5) == -1) {
    return -1;
  }
  return 0;
}

void waintingAcknowledgment() {
  int state = START;
  unsigned char buf;

  // State Machine For the Connection Request
  while (1) {
    if (read(fd, &buf, 1) != 0) {
      if (state == START) {
        if (buf == FLAG)
          state = FLAG_OK;
      } else if (state == FLAG_OK) {
        if (buf == 0x03)
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
        else if (buf == (0x03 ^ C_RCV))
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

}

int llclose(int fd) {
  if(waitingDisconnectionRequest() == -1){

    return -1;
  }
  printf("\nRECEIVED: Disconnection Request\n");
  sendingDisconnectionRequest();
  printf("SENT: Disconnection Request\n");
  waintingAcknowledgment();
  printf("RECEIVED:Disconnection Acknowledgment\n");

  sleep(1);
  if (tcsetattr(fd, TCSANOW, &oldtio) == -1) {
    perror("tcsetattr");
    exit(-1);
  }

  close(fd);
  return 0;
}

int doDestuffing(unsigned char* stuffed_data, int length, char* buffer) {
  int indStuffed = 0, indBuffer = 0;

  while (indStuffed < length) {
    if (stuffed_data[indStuffed] == ESC) {
      indStuffed++;
      if (stuffed_data[indStuffed] == 0x5E)
        buffer[indBuffer] = (char)FLAG;
      else
        buffer[indBuffer] = (char)ESC;
    } else {
      buffer[indBuffer] = (char)stuffed_data[indStuffed];
    }
    indStuffed++;
    indBuffer++;
  }
  return indBuffer;
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

int readControllPacket(char* packet, int control_byte, int* filesize) {
  int index = 0;

  if (packet[index] != control_byte) {
    if (control_byte == 2) {
      printf("Error in start packet control\n");
      printf("packet index: %#x \n", packet[index]);
    } else if (control_byte == 3)
      printf("Error in end packet control\n");
    return -1;
  }

  printf("Controll packet is being processed...\n");

  index++;
  int typeLength;
  unsigned char type;

  for (int i = 0; i < 2; i++) {
    type = packet[index];
    index++;

    // checking tlv
    switch (type) {
      case 0:
        typeLength = (int)packet[index];
        index++;
        *filesize = *((off_t*)(packet + index));
        index += typeLength;
        printf("File size: %d\n", *filesize);
        break;

      case 1:
        typeLength = (int)packet[index];
        printf("File name length: %d\n", typeLength);
        index++;
        filename = malloc(typeLength + 1);
        memset(filename, '\0', typeLength + 1);
        memcpy(filename, (char*)packet + index, typeLength);
        printf("Filename: %s\n", filename);
        break;
      default:
        printf("Error in control packet processing...\n");
        break;
    }
  }

  return control_byte;
}

int readDataPacket(int fd, char* data_packet, char* buffer) {
  if (data_packet[0] != 0x01){
    printf("erro: %X\n", data_packet[0]);
    return -1;  // error
  }
  // unsigned char N = data_packet[1];
  unsigned char L2 = data_packet[2];
  unsigned char L1 = data_packet[3];
  int length = ((int)L2) * 256 + ((int)L1);

  int cont = 0;
  while (cont < length) {
    buffer[cont] = data_packet[cont + 4];
    cont++;
  }

  return cont;
}

int readInfo(int fd, char* data_packet) {
  int state = START, ret, cont = 0;
  unsigned char stuffed_data[MAX_DATA_WITH_STUFFING], current_byte, c;
  char deStuffed_data[MAX_DATA_WITHOUT_STUFFING];

  // State Machine For the Connection Request
  while (state != STOP) {
    if(read(fd, &current_byte, 1) != 0){
      switch (state) {
        case START:
          if (current_byte == FLAG){
            state = FLAG_OK;
          }
          break;

        case FLAG_OK:
          if (current_byte == 0x03)
            state = A_OK;
          else if (current_byte != FLAG)
            state = START;
          break;

        case A_OK:
          if (current_byte == FLAG)
            state = FLAG_OK;
          else if (current_byte == C_I0 || current_byte == C_I1){
            // verifies if it's duplicated info
            if((R == 0 && current_byte == C_I1) || (R == 1 && current_byte == C_I0))
              duplicated = 1;
            else //if it's not duplicated changes the r for next frame number
              duplicated = 0;

            c = current_byte;
            state = C_OK;
          }
          else
            state = START;
          break;

        case C_OK:
          if (current_byte == FLAG)
            state = FLAG_OK;
          else if (current_byte == (0x03 ^ c))
            state = BCC_OK;
          else
            state = START;
          break;

        case BCC_OK:
          if(cont >= MAX_DATA_WITH_STUFFING + 1){
            printf("MAXIMUM CAPACITY EXCEDED IN DATA!\n\n");
            exit(1);
          }
          else if(current_byte != FLAG){
            stuffed_data[cont] = current_byte;
            cont++;
          }
          else{
            state = STOP;
            break;
          }
          break;

        default:
          break;
      }
    }
  }

  cont = doDestuffing(stuffed_data, cont, deStuffed_data);

  int aux = 0;
  while(aux < (cont - 1)){
    data_packet[aux] = deStuffed_data[aux];
    aux++;
  }

  ret = aux;
  char bcc2Frame = deStuffed_data[aux], bcc2Data;

  bcc2Data = getBcc2(data_packet, aux);

  if (bcc2Data == bcc2Frame) data_correct = 1;
  else data_correct = 0;

  return ret;
}

void writeAnswer() {
  unsigned char f = FLAG, a = 0x01, c, bcc;

  if(R == 0 && data_correct == 1) c = C_RR0;
  else if(R == 1 && data_correct == 1) c = C_RR1;
  else if(R == 0 && data_correct == 0 && duplicated == 0) c = C_REJ0;
  else if(R == 1 && data_correct == 0 && duplicated == 0) c = C_REJ1;
  else if(R == 0 && data_correct == 0 && duplicated == 1) c = C_RR0;
  else if(R == 1 && data_correct == 0 && duplicated == 1) c = C_RR1;

  if(c == C_REJ0 || c == C_REJ1) {
    printf("REJECTING DATA\n");
    printf("-----------------------------------------\n");
  }

  bcc = (a ^ c);

  write(fd, &f, 1);
  write(fd, &a, 1);
  write(fd, &c, 1);
  write(fd, &bcc, 1);
  write(fd, &f, 1);
}

void changeSequenceNumber(){
  // data correct and new info
  if(duplicated == 0 && data_correct == 1){
    if (R == 0) R = 1;
    else R = 0;
  }
}

int llread(int fd, char* data_packet) {
  int length = readInfo(fd, data_packet);
  changeSequenceNumber();
  writeAnswer();

  return length;
}

int main(int argc, char** argv) {
  
  if (argc != 2) {
    printf(
        "Usage:\t ./receive SerialPortNumber \nExample: ./receive \t      11 "
        "\n");
    exit(1);
  }

  // Starting Connection
  //================================
  int fd = llopen(atoi(argv[1]), RECEIVER);
  if (fd < 0) {
    printf("An error has occured during the conncetion process\n");
    exit(1);
  }

  printf("Connection established SUCCESSFULLY! \n\n");

  //================================
  char data_packet[MAX_DATA_PACKET];
  char buffer[MAX_READ];

  int filesize = 0;
  char control_packet[MAX_READ];

  int received_control_packet = FALSE;
  while (!received_control_packet) {
    if (llread(fd, control_packet) < 0) {
      printf("Failed to read Start Packet...\n");
      return -1;
    }
    if(duplicated == 0 && data_correct == 1){
      if (readControllPacket(control_packet, 2, &filesize) != 2)
        printf("Did not receive the Start Packet correctly...\n");
      else
        received_control_packet = TRUE;
    }
  }
  printf("Start Packet was processed correctly.\n");

  // DATA PACKETS
  int imagem = open(filename, O_CREAT | O_RDWR, 0777);
  if(imagem == -1) {
    perror("open");
    exit(1);
  }
  int processed_bytes = 0;

  while (processed_bytes < filesize) {
    int length = llread(fd, data_packet);

    // new and correct info
    if(duplicated == 0 && data_correct == 1){
      length = readDataPacket(fd, data_packet, buffer);

      int aux = 0;
      while (aux < length) {
        write(imagem, &buffer[aux], 1);
        aux++;
      }

      processed_bytes += length;
      printf("Data Received: %d of %d\n", processed_bytes, filesize);
      printf("-----------------------------------------\n");
    }
  }


  close(imagem);

  // END PACKET
  received_control_packet = FALSE;
  while (!received_control_packet) {
    if (llread(fd, control_packet) < 0) {
      printf("Failed to read End Packet...\n");
      return -1;
    }

    if(duplicated == 0 && data_correct == 1){
      if (readControllPacket(control_packet, 3, &filesize) != 3)
        printf("Did not receive the End Packet correctly...\n");
      else
        received_control_packet = TRUE;
    }
  }

  printf("End Packet was processed correctly.\n");


  // Closing Connection
  //================================
  if (llclose(fd) < 0) {
    printf("An error has occured during the disconnection process!\n");
    exit(1);
  }

  printf("\nConnection was closed SUCCESSFULLY!\n");

  return 0;
}
