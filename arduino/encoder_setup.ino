#define encodPinA1      3     // encoder A pin
#define encodPinB1      8     // encoder B pin
#define encodPinA2      2
#define encodPinB2      7

volatile long count1 = 0;          // rev counter
volatile long count2 = 0;
long countAnt1 = 0;
long countAnt2 = 0;

void setup() {
  Serial.begin(9600);
    pinMode(encodPinA1, INPUT);
  pinMode(encodPinB1, INPUT);
  digitalWrite(encodPinA1, HIGH);                // turn on pullup resistor
  digitalWrite(encodPinB1, HIGH);
  attachInterrupt(1, encoder1, RISING);

  pinMode(encodPinA2, INPUT);
  pinMode(encodPinB2, INPUT);
  digitalWrite(encodPinA2, HIGH);                // turn on pullup resistor
  digitalWrite(encodPinB2, HIGH);
  attachInterrupt(0, encoder2, RISING);// put your setup code here, to run once:

}

void loop() {
  Serial.print(count1);
  Serial.print("    ");
  Serial.println(count2);
 
}

void encoder1() {
  if (digitalRead(encodPinA1) == digitalRead(encodPinB1)) count1--;
  else count1++;
}
void encoder2() {
  if (digitalRead(encodPinA2) == digitalRead(encodPinB2)) count2++;
  else count2--;
}
