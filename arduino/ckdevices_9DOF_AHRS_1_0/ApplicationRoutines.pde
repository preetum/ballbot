



//=================================
// Fill in the calibration values
// this could be expanded to allow the values to determined 
// using a calibration mode for example
void mongooseCalibrate(void)
{
 
    // See the calibration guide for more details on what each 
    // of these are for. These values are unique for each Mongoose
    // Magnetometer calibration values can also be dependent on
    // the sensor platform
    
    sen_offset.accel_offset[0]     = 12; 
    sen_offset.accel_offset[1]     = 5; 
    sen_offset.accel_offset[2]     = -25;
    
    sen_offset.gyro_offset[0]      = -4;
    sen_offset.gyro_offset[1]      = 1;
    sen_offset.gyro_offset[2]      = 3;
    
    sen_offset.magnetom_offset[0]  = -159; //39;
    sen_offset.magnetom_offset[1]  = -85; //-365;
    sen_offset.magnetom_offset[2]  = 0; //-138;
    
    sen_offset.magnetom_XY_Theta   = ToRad(0);
    sen_offset.magnetom_XY_Scale   = 1;
    
    sen_offset.magnetom_YZ_Theta   = ToRad(0);
    sen_offset.magnetom_YZ_Scale   = 1;
    
}



void StatusLEDToggle()
{
  static unsigned int counter = 0;
  static char state = 0;
  
  counter++;
  if (counter > 20)
  {
    counter=0;
    if(state)
    {
      digitalWrite(STATUS_LED,LOW);
      state = FALSE;
    }
    else
    {
      digitalWrite(STATUS_LED,HIGH);
      state = TRUE;
    }
 
  }
}



//EEPROM writing and reading Data structure
int EEPROM_WriteByteArray(int ee, int count, byte * p)
{
  int i = 0;
        
  for(i=0;i<count;i++)
    EEPROM.write(ee++, *p++);
          
  return(i);
}
    
int EEPROM_ReadByteArray(int ee, int count, byte * p)
{
  int i = 0;
        
  for(i=0;i<count;i++)
    *p++ = EEPROM.read(ee++);
          
  return(i);
}






