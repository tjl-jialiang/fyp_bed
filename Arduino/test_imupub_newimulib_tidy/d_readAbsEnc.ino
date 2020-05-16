//=========
float readAbsEncoder(int absenc_wheelx[]){
  //absenc_wheelx = {Csn, CLK, DO}
  unsigned int dataOut = 0;
  float anglex = 0;

  digitalWrite(absenc_wheelx[0], LOW);
  delayMicroseconds(1); //Waiting for Tclkfe

  //Passing 10 times, from 0 to 9
  for(int x=0; x<10; x++){
    digitalWrite(absenc_wheelx[1], LOW);
    delayMicroseconds(1); //Tclk/2
    digitalWrite(absenc_wheelx[1], HIGH);
    delayMicroseconds(1); //Tdo valid, like Tclk/2
    dataOut = (dataOut << 1) | digitalRead(absenc_wheelx[2]); //shift all the entering data to the left and past the pin state to it. 1e bit is MSB
  }
  
  digitalWrite(absenc_wheelx[0], HIGH); //
  anglex = (dataOut*enc_res);
  return anglex;
}

