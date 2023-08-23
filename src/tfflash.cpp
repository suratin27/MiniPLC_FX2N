#include "tftype_def.h"
#ifdef ESP32
  #include "SPIFFS.h"
#else
  #include #include <FS.h>
#endif

#define FORMAT_SPIFFS_IF_FAILED true


/*-------------------------------------------------------------------
                          SPIFFS File system
-------------------------------------------------------------------*/
void listDir(fs::FS &fs, const char * dirname, uint8_t levels){
  Serial.printf("Listing directory: %s\r\n", dirname);

  File root = fs.open(dirname);
  if(!root){
    Serial.println("- failed to open directory");
    return;
  }
  if(!root.isDirectory()){
    Serial.println(" - not a directory");
    return;
  }

  File file = root.openNextFile();
  while(file){
    if(file.isDirectory()){
      Serial.print("DIR : ");
      Serial.println(file.name());
      if(levels){
        listDir(fs, file.path(), levels -1);
      }
    } else {
      Serial.print("File: ");
      Serial.println(file.name());
      Serial.print("Size: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

void readFile(fs::FS &fs, const char * path){
  Serial.printf("Reading file: %s\r\n", path);

  File file = fs.open(path);
  if(!file || file.isDirectory()){
    Serial.println("- failed to open file for reading");
    return;
  }

  Serial.println("- read from file:");
  while(file.available()){
    Serial.write(file.read());
  }
  file.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Writing file: %s\r\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file){
    Serial.println("- failed to open file for writing");
    return;
  }
  if(file.print(message)){
    Serial.println("- file written");
  } else {
    Serial.println("- write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
  Serial.printf("Appending to file: %s\r\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file){
    Serial.println("- failed to open file for appending");
    return;
  }
  if(file.print(message)){
    Serial.println("- message appended");
  } else {
    Serial.println("- append failed");
  }
  file.close();
}

void renameFile(fs::FS &fs, const char * path1, const char * path2){
  Serial.printf("Renaming file %s to %s\r\n", path1, path2);
  if (fs.rename(path1, path2)) {
    Serial.println("- file renamed");
  } else {
    Serial.println("- rename failed");
  }
}

void deleteFile(fs::FS &fs, const char * path){
  Serial.printf("Deleting file: %s\r\n", path);
  if(fs.remove(path)){
    Serial.println("- file deleted");
  } else {
    Serial.println("- delete failed");
  }
}

void testFileIO(fs::FS &fs, const char * path){
  Serial.printf("Testing file I/O with %s\r\n", path);

  static uint8_t buf[512];
  size_t len = 0;
  File file = fs.open(path, FILE_WRITE);
  if(!file){
    Serial.println("- failed to open file for writing");
    return;
  }

  size_t i;
  Serial.print("- writing" );
  uint32_t start = millis();
  for(i=0; i<2048; i++){
    if ((i & 0x001F) == 0x001F){
      Serial.print(".");
    }
    file.write(buf, 512);
  }
  Serial.println("");
  uint32_t end = millis() - start;
  Serial.printf(" - %u bytes written in %u ms\r\n", 2048 * 512, end);
  file.close();

  file = fs.open(path);
  start = millis();
  end = start;
  i = 0;
  if(file && !file.isDirectory()){
    len = file.size();
    size_t flen = len;
    start = millis();
    Serial.print("- reading" );
    while(len){
      size_t toRead = len;
      if(toRead > 512){
          toRead = 512;
      }
      file.read(buf, toRead);
      if ((i++ & 0x001F) == 0x001F){
        Serial.print(".");
      }
      len -= toRead;
    }
    Serial.println("");
    end = millis() - start;
    Serial.printf("- %u bytes read in %u ms\r\n", flen, end);
    file.close();
  } else {
    Serial.println("- failed to open file for reading");
  }
}

void createDir(fs::FS &fs, const char * path){
  Serial.printf("Creating Dir: %s\n", path);
  if(fs.mkdir(path)){
      Serial.println("Dir created");
  } else {
      Serial.println("mkdir failed");
  }
}

void removeDir(fs::FS &fs, const char * path){
  Serial.printf("Removing Dir: %s\n", path);
  if(fs.rmdir(path)){
      Serial.println("Dir removed");
  } else {
      Serial.println("rmdir failed");
  }
}

bool checkFile(fs::FS &fs, const char * path){
  if(SPIFFS.exists(path)){
    return true;
  }else{
    return false;
  }
}

void storeInSPIFFS(fs::FS &fs, byte* payload, uint16_t* des,unsigned int length){
  File _firmware = fs.open("/firmware.lx", "wb");
  if (_firmware)
  {
    Serial.println(F("Going to write Firmware to SPIFFS"));
    Serial.printf("File size: %d words\n",length);                //- Print data length in words 
    unsigned int byteLength;
    byteLength = length*2;        
    Serial.printf("File size: %d bytes\n",byteLength);
    
    for (int i = 0; i < byteLength; i++) {                           //  for debug purposes check the content of the payload
      Serial.printf(" %X ",(byte)payload[i]);                          //  DEBUG
      if(i<(byteLength-1))
        Serial.print(":");                                       //  DEBUG
    }                                                            //  DEBUG

    _firmware.write(payload, byteLength); 
    _firmware.close();
    Serial.printf("\n%d bytes,written.\n",byteLength);
    Serial.println();                                            //  DEBUG
  }
  

  File readStorage = fs.open("/firmware.lx", "rb");
  if (readStorage) {
  byte buffer[readStorage.size()];
  Serial.printf("Read: %d bytes\n",readStorage.size());
  size_t bufferSize = readStorage.readBytes((char*)buffer,readStorage.size());

  for (int i = 0; i < bufferSize; i++) {
    Serial.printf(" %X ",(byte)buffer[i]);                     //  Print loaded data
    if(i<(bufferSize-1))
      Serial.print(":");                                       //  DEBUG
  }
  readStorage.close();

  unsigned int wordSize;
  if(readStorage.size() >0){
    wordSize= readStorage.size()/2;
    
    for(int i=0;i<wordSize;i++){
      des[i] = buffer[i]+buffer[i+1]*256;
      Serial.printf(" %X ",des[i]);                     //  Print loaded data
      if(i<(wordSize-1))
        Serial.print(":"); 
    }
  }
  //-Print last PLC Prog data
  Serial.println();
 } 
}

