void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
  TCCR3A = 0;
  TCCR3B = 0;
  TCNT3 = 0;
  OCR3A = 62499; //Laver 1Hz interrupt
  TCCR3A |= (1 << WGM32);
  TCCR3B |= (1 << CS32);
  TIMSK3 |= (1 << OCIE3A);

}


ISR(TIMER3_COMPA_vect)
{
  Serial.println("Timer");
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("No timer");
  delay(100);

}
