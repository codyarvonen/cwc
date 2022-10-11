volatile unsigned long  count = 0;
unsigned long copyCount = 0;

unsigned long lastRead = 0;
unsigned long interval = 5000;

void setup() {
 Serial.begin(9600);
 Serial.println("start...");
 pinMode(2,INPUT_PULLUP);
 attachInterrupt(digitalPinToInterrupt(2), isrCount, FALLING); //interrupt signal to pin 2
}

void loop() {
 if (millis() - lastRead >= interval) { //read interrupt count every second
   lastRead  += interval;
   // disable interrupts,make copy of count, reenable interrupts
   noInterrupts();
   copyCount = count;
   count = 0;
   interrupts();

   Serial.println(copyCount*12); 

 }

}

void isrCount() {
 count++;
}
