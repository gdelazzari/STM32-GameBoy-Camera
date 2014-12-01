#define VOUT     A0

#define XCK      2
#define RESET    3
#define START    4
#define READ     5
#define LOAD     6
#define SIN      7

#define XCK_H    digitalWrite(XCK, HIGH);
#define XCK_L    digitalWrite(XCK, LOW);

// #define DEBUG

void CAM_reset(void)
{
  XCK_H; // Dummy clock cycle
  XCK_L;
  
  digitalWrite(RESET, LOW); // Reset command
  XCK_H; // Clock it in
  digitalWrite(RESET, HIGH);
  XCK_L;
  
  XCK_H; // Dummy clock cycle
  XCK_L;
}

void CAM_loadRegister(byte address, byte data)
{
  for (int i = 2; i >= 0; i--) {
    digitalWrite(SIN, address & (1 << i));
    XCK_H;
    XCK_L;
  }
  
  for (int i = 7; i >= 1; i--) {
    digitalWrite(SIN, data & (1 << i));
    XCK_H;
    XCK_L;
  }
  
  digitalWrite(SIN, data & 0x01);
  XCK_H;
  digitalWrite(LOAD, HIGH);
  digitalWrite(SIN, LOW);
  XCK_L;
  digitalWrite(LOAD, LOW);
}

unsigned int CAM_capture(boolean first)
{
  unsigned int count = 0; // Pixel count variable
  
  if (first) {
    XCK_H; // Dummy clock cycle
    XCK_L;
    
    digitalWrite(START, HIGH); // Issue start command
    XCK_H; // Dummy clock cycle
    XCK_L;
    digitalWrite(START, LOW);
  }
  
  while (!digitalRead(READ)) { // Wait for READ signal
    XCK_H;
    XCK_L;
  }
  
  Serial.write(0x00);
  
  while (digitalRead(READ)) { // While READ is high
    XCK_H; // Clock cycle
    XCK_L;
    
    Serial.write(analogRead(A0) >> 2);
    
    count++;
  }
  
  return count;
}

void setup()
{
  Serial.begin(115200);
  
  /*
    Configure pins
  */
#ifdef DEBUG
  Serial.println("Configuring interface...");
#endif
  
  pinMode(VOUT, INPUT);
  
  pinMode(XCK, OUTPUT);
  pinMode(RESET, OUTPUT);
  pinMode(START, OUTPUT);
  pinMode(READ, INPUT);
  pinMode(LOAD, OUTPUT);
  pinMode(SIN, OUTPUT);
  
  /*
    Set pins to their initial state
  */
  digitalWrite(XCK, LOW);
  digitalWrite(RESET, HIGH);
  digitalWrite(START, LOW);
  digitalWrite(LOAD, LOW);
  digitalWrite(SIN, LOW);
  
  /*
    Initialization sequence
  */
#ifdef DEBUG
  Serial.println("Initializing camera...");
  
  Serial.println("[RESET]");
#endif

  CAM_reset();

#ifdef DEBUG
  Serial.println("[REGs SETUP]");
#endif

  CAM_loadRegister(0x00, 0x80);
  CAM_loadRegister(0x01, 0xD6);
  CAM_loadRegister(0x02, 0x00);
  CAM_loadRegister(0x03, 0xF0);
  CAM_loadRegister(0x04, 0x01);
  CAM_loadRegister(0x05, 0x00);
  CAM_loadRegister(0x06, 0x01);
  CAM_loadRegister(0x07, 0x07);
  
  /*
    Test
  */
#ifdef DEBUG
  Serial.println("Acquiring a sample image...");
#endif

  unsigned int c = CAM_capture(true);
  
#ifdef DEBUG
  Serial.print("Captured ");
  Serial.print(c);
  Serial.println(" pixels");
#endif
}

void loop()
{
  unsigned int c = CAM_capture(false);
#ifdef DEBUG
  Serial.print("Captured ");
  Serial.print(c);
  Serial.println(" pixels");
#endif
}

