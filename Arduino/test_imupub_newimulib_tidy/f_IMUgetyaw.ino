//===============
void getpsi(){
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
      #ifdef OUTPUT_READABLE_YAWPITCHROLL
          // display Euler angles in degrees
          mpu.dmpGetQuaternion(&q, fifoBuffer);
          mpu.dmpGetGravity(&gravity, &q);
          mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
          //Serial.print("ypr\t");
          //Serial.print(ypr[0] * 180/M_PI);
          //Serial.print("\t");
          //Serial.print(ypr[1] * 180/M_PI);
          //Serial.print("\t");
          //Serial.println(ypr[2] * 180/M_PI);
          yaw_angle.data = -ypr[0];
      #endif
  }
}