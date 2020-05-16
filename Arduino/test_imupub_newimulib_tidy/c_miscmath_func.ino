//===========
void matrixAbsolute(BLA::Matrix<2,1> wheelx_vel, float wheelx_abs[]){
  for (int i=0;i<=1;i++){
    wheelx_abs[i] = sqrt(wheelx_vel(i)*wheelx_vel(i));
  }
}

//==============
float maximum(float wheel1_abs[], float wheel2_abs[]){
  float maxv = wheel1_abs[0];
  if (wheel1_abs[1] > maxv) { 
    maxv = wheel1_abs[1];
  } 

  if (wheel2_abs[0] > maxv) {
    maxv = wheel2_abs[0];
  } 

  if (wheel2_abs[1] > maxv) {
    maxv = wheel2_abs[1];
  }
  return maxv; 
}

//Used in mappwm function==========
float map_f(float val, float i_min, float i_max, float t_min, float t_max){
  return (val-i_min)*(t_max-t_min)/(i_max-i_min) + t_min;
}

//============
void mappwm(float wheelx_abs[], float wheelx_pwm[]){
  wheelx_pwm[0] = map_f(wheelx_abs[0],0,max_norm,70,255); //4th argument is PWM signal that gives zero speed
  wheelx_pwm[1] = map_f(wheelx_abs[1],0,max_norm,70,255);
}

//============
void signedpwm(float wheelx_pwm[],float wheelx_pwm_signed[], BLA::Matrix<2,1> wheelx_vel){
  for (int i=0;i<=1;i++){
    if (wheelx_vel(i)<0){
      wheelx_pwm_signed[i] = - wheelx_pwm[i];
    }
    else{
      wheelx_pwm_signed[i] = wheelx_pwm[i];
    }
  }
}

//===============
void prop_pwm(float wheelx_pwm_signed[],float prev_wheelx_pwm_signed[],float out_wheelx_pwm_signed[]){
  for (int i=0;i<=1;i++){
    if (isnan(wheelx_pwm_signed[i])){
      prev_wheelx_pwm_signed[i] = 0;
      out_wheelx_pwm_signed[i] = 0;
      continue;
    }
    float error = prev_wheelx_pwm_signed[i]-wheelx_pwm_signed[i];
    if (error<50 && error>-50){
      out_wheelx_pwm_signed[i] = wheelx_pwm_signed[i];
    } else {
      out_wheelx_pwm_signed[i] = prev_wheelx_pwm_signed[i] - Kp*error;
    }
    prev_wheelx_pwm_signed[i] = out_wheelx_pwm_signed[i];
  }
}