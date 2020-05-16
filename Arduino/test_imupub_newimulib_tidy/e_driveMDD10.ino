//=========
void drivemotor_dirchange(float wheelx_pwm_signed[], int mdd_wheelx_right[], int mdd_wheelx_left[]){
  //Direction of rotation
  if (wheelx_pwm_signed[0] < 0){
    digitalWrite(mdd_wheelx_right[1], LOW);
  }
  else{
    digitalWrite(mdd_wheelx_right[1], HIGH);
  }
    
  if (wheelx_pwm_signed[1] < 0){
    digitalWrite(mdd_wheelx_left[1], LOW);
  }
  else{
    digitalWrite(mdd_wheelx_left[1], HIGH);
  }
  
  //PWM Value
  analogWrite(mdd_wheelx_right[0], sqrt(wheelx_pwm_signed[0]*wheelx_pwm_signed[0]));
  analogWrite(mdd_wheelx_left[0], sqrt(wheelx_pwm_signed[1]*wheelx_pwm_signed[1]));
}

