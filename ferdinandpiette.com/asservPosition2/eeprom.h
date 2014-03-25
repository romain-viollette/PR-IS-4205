

#define EEPROM_L_KP    0      
#define EEPROM_L_KI    EEPROM_L_KP+4  
#define EEPROM_L_KD    EEPROM_L_KI+4 
#define EEPROM_R_KP    EEPROM_L_KD+4      
#define EEPROM_R_KI    EEPROM_R_KP+4  
#define EEPROM_R_KD    EEPROM_R_KI+4 

float eepromReadFloat(int address)
{
  union u_tag {
    byte b[4];
    float dval;
  } 
  u;   
  u.b[0] = EEPROM.read(address);
  u.b[1] = EEPROM.read(address+1);
  u.b[2] = EEPROM.read(address+2);
  u.b[3] = EEPROM.read(address+3);
  return u.dval;
}

void eepromWriteFloat(int address, float value)
{
  union u_tag {
    byte b[4];
    float dval;
  } 
  u;
  u.dval=value;

  EEPROM.write(address  , u.b[0]);
  EEPROM.write(address+1, u.b[1]);
  EEPROM.write(address+2, u.b[2]);
  EEPROM.write(address+3, u.b[3]);
}

void eepromUpdateFloat(int address, float value)
{
  if(eepromReadFloat(address) != value)
    eepromWriteFloat(address, value);
}

float eepromReadLong(int address)
{
  union u_tag {
    byte b[4];
    long dval;
  } 
  u;   
  u.b[0] = EEPROM.read(address);
  u.b[1] = EEPROM.read(address+1);
  u.b[2] = EEPROM.read(address+2);
  u.b[3] = EEPROM.read(address+3);
  return u.dval;
}

void eepromWriteLong(int address, long value)
{
  union u_tag {
    byte b[4];
    long dval;
  } 
  u;
  u.dval=value;

  EEPROM.write(address  , u.b[0]);
  EEPROM.write(address+1, u.b[1]);
  EEPROM.write(address+2, u.b[2]);
  EEPROM.write(address+3, u.b[3]);
}

void eepromUpdateLong(int address, long value)
{
  if(eepromReadLong(address) != value)
    eepromWriteLong(address, value);
}
