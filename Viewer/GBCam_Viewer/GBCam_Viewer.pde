import processing.serial.*;

Serial port;

int cnt;

boolean update = false;
boolean frameReady = false;

int[] image = new int[16128];

void setup() {
  size(128, 126);
  
  String portName = "/dev/ttyUSB0";
  port = new Serial(this, portName, 115200);
  
  cnt = 0;
}

void serialEvent(Serial port) {
  int read = port.read();
  
  if (read == 0x00) {
    cnt = 0;
    frameReady = true;
  } else {
    image[cnt] = read;
    cnt++;
    
    if (cnt >= 16128) {
      cnt = 0;
      frameReady = true;
    }
  }
}

void draw() {
  int pix;
  
  if (frameReady == true) {
    
    
    for (int i = 0; i < 16128; i++) {
      loadPixels();
      pixels[i] = color(image[i]);
      updatePixels();
    }
    
    
    
    frameReady = false;
  }
}

void exit() {
  port.stop();
}
