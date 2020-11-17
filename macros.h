// Application Layer
#define BAUDRATE B38400
#define FALSE 0
#define TRUE 1

#define RECEIVER 0
#define TRANSMITTER 1

#define MAX_DATA_PACKET 514
#define MAX_DATA_WITHOUT_STUFFING 515
#define MAX_DATA_WITH_STUFFING 1030
#define MAX_READ 510 
#define MAX_FRAME 1035

#define MAX_TRIES 3
#define TIMEOUT 3

// Data Link Layer

#define FLAG            0x7E

#define A_RCV           0x01
#define A_SND           0x03

#define C_RCV           0x07
#define C_SND           0x03
#define C_I0            0x00
#define C_I1            0x40
#define C_RR1           0x85
#define C_RR0           0x05
#define C_REJ0          0x01
#define C_REJ1          0x81
#define C_DISC          0x0B


#define BCC_RCV         (A_RCV ^ C_RCV)

#define ESC             0x7D
#define MASK            0x20


typedef enum {
    START = 0,
    FLAG_OK = 1,
    A_OK = 2,
    C_OK = 3,
    BCC_OK = 4,
    STOP = 5
}states;
