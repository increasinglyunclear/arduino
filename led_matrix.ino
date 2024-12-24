// For testing Adafruit LED Matrix
// for Respiratrees project https://respiratrees.tumblr.com/
// Kevin Walker 18 Jan 2022

// WIRING: 
// MOSI to 11
// CS to 10
// SCLK to 13

//Define the "Normal" Colors
#define BLACK   0x00
#define RED     0xE0
#define GREEN   0x1C
#define BLUE    0x03
#define YELLOW  RED|GREEN
#define ORANGE  0xFC
#define MAGENTA RED|BLUE
#define TEAL    BLUE|GREEN
#define WHITE   (RED|GREEN|BLUE)-0xA0

//Define the SPI Pin Numbers
#define SLAVESELECT 10//ss
#define DATAOUT     11//MOSI
#define DATAIN      12//MISO 
#define SPICLOCK    13//sck

//Define final array to output to LED Matrices (maximum of 8 matrices daisy chained)
//Ex.) if you have 1 matrix, you have an array of 64=64*1
//     if you have 2 matrices, you have an array of 128=64*2
//     if you have 3 matrices, you have an array of 194=64*3
const int num_matrix = 1; //define number of daisy chained matrices
char color_buffer[64 * num_matrix]; //final array

//array to clear 1 matrix
char blank[] = {
  BLACK, BLACK, BLACK, BLACK, BLACK, BLACK, BLACK, BLACK,
  BLACK, BLACK, BLACK, BLACK, BLACK, BLACK, BLACK, BLACK,
  BLACK, BLACK, BLACK, BLACK, BLACK, BLACK, BLACK, BLACK,
  BLACK, BLACK, BLACK, BLACK, BLACK, BLACK, BLACK, BLACK,
  BLACK, BLACK, BLACK, BLACK, BLACK, BLACK, BLACK, BLACK,
  BLACK, BLACK, BLACK, BLACK, BLACK, BLACK, BLACK, BLACK,
  BLACK, BLACK, BLACK, BLACK, BLACK, BLACK, BLACK, BLACK,
  BLACK, BLACK, BLACK, BLACK, BLACK, BLACK, BLACK, BLACK
};

//set LED Matrix Value for daisy chained LED Matrices
//maximum of 8 daisy chained LED Matrices
void set_matrix() {
  digitalWrite(SLAVESELECT, LOW); //Activate the RGB Matrix
  spi_transfer(0x25); //command character % == 0x25
  spi_transfer(0x02); //set LED matrix to position in daisy chain, must be between 0x01-0x08
  digitalWrite(SLAVESELECT, HIGH); //Deactivate the RGB Matrix
}

int p = 0; //dot position
int xMAX = 8;//max number column in 1 subset
int yMAX = 8;//max number row in 1 subset

void clear_buffer() {
  //start clearing array moving down rows
  for (int y = 0; y < yMAX; y++) {
    //clear array from the highest subset in daisy chained array
    for (int i = num_matrix; i > 0; i--) {
      //clear array from right to left
      for (int x = 0; x < xMAX; x++) {
        //current position that is being cleared
        p = (yMAX * y) + (x + (i - 1) * (xMAX * yMAX));
        //clear by saving what is in the blank sub-array to your final array
        color_buffer[p] = blank[x];
      }
    }
  }
  matrix_write();//write array of colors to matrix
  delay(10);//allow some time for the final matrix to be seen
}

void color_test() {
  for (int i = 0; i < 64; i++) {
    color_buffer[i] = WHITE;
  }
  //  color_buffer[1] = BLACK;
  //  color_buffer[2] = BLACK;
  //  color_buffer[3] = BLACK;
  //  color_buffer[4] = BLACK;
  //  color_buffer[5] = BLACK;
  //  color_buffer[6] = BLACK;
  //  color_buffer[7] = BLACK;

  matrix_write();//write array of colors to matrix
  delay(100);  //allow some time for the final matrix to be seen
}

void setup() {
  //----------SPI BUS SETUP----------
  SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR1); //Enable SPI HW, Master Mode, divide clock by 16

  //Set the pin modes for the RGB matrix
  pinMode(DATAOUT, OUTPUT);
  pinMode(DATAIN, INPUT);
  pinMode(SPICLOCK, OUTPUT);
  pinMode(SLAVESELECT, OUTPUT);

  //Make sure the RGB matrix is deactivated
  digitalWrite(SLAVESELECT, HIGH);
  //----------END SPI BUS SETUP----------

  //use this to set a LED Matrix to position in daisy chain
  //set_matrix();

  //make sure buffer is cleared before writing to matrices
  clear_buffer();

  //test colors
  //color_test();

}

void loop() {
  color_test();
}

//display array on the LED Matrix/daisy chained LED Matrices
void matrix_write() {
  digitalWrite(SLAVESELECT, LOW); //Activate the RGB Matrix
  spi_transfer(0x26); //command char to reset frame index
  digitalWrite(SLAVESELECT, HIGH); //Deactivate the RGB Matrix

  digitalWrite(SLAVESELECT, LOW); //Activate the RGB Matrix
  //Send the color buffer to the RGB Matrix
  delayMicroseconds(500);

  for (int LED = 0; LED < (64 * num_matrix); LED++) {
    spi_transfer(color_buffer[LED]);//send what is in buffer to the matrices
  }
  delayMicroseconds(500);//allow some time for the serial data to be sent
  digitalWrite(SLAVESELECT, HIGH); //Deactivate the RGB Matrix
}

//Use this command to send a single color value to the RGB matrix.
//NOTE: You must send x number of color values to the RGB matrix
//      before it displays an image!
char spi_transfer(volatile char data) {
  SPDR = data;                    // Start the transmission
  while (!(SPSR & (1 << SPIF))) {
    // Wait for the end of the transmission
  };
  return SPDR;                    // return the received byte
}
