/*

//============
void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = 's';
    char endMarker = 'e';
    char rc;

    while (Serial3.available() > 0 && newData == false) {
        rc = Serial3.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                //if (ndx >= numChars) {
                    //ndx = numChars - 1;
                //}
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}


//============
void parseData() {      // split the data into its parts

    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars,",");      // get the first part - the string
    xvel_app = atoi(strtokIndx); // copy it to messageFromPC
 
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    yvel_app = atoi(strtokIndx);     // convert this part to an integer

    strtokIndx = strtok(NULL, ",");
    rotvel_app = atoi(strtokIndx);     // convert this part to a float

}

//============

void showParsedData() {
    Serial.print(F("xvel_app: "));
    Serial.print(xvel_app);
    Serial.print(F(" yvel_app: "));
    Serial.print(yvel_app);
    Serial.print(F(" rotvel_app: "));
    Serial.println(rotvel_app);
}

*/
