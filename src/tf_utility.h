#ifndef TF_UTILITY_H
#define TF_UTILITY_H

int32_t hex2int(char *hex,uint8_t lenght) {
  uint8_t _count;
  uint32_t val = *hex > 56 ? 0xFFFFFFFF : 0;
  while (_count < lenght) {
    // get current character then increment
    uint8_t byte = *hex++; 
    // transform hex character to the 4bit equivalent number, using the ascii table indexes
    if (byte >= '0' && byte <= '9') byte = byte - '0';
    else if (byte >= 'a' && byte <='f') byte = byte - 'a' + 10;
    else if (byte >= 'A' && byte <='F') byte = byte - 'A' + 10;    
    // shift 4 to make space for new digit, and add the 4 bits of the new digit 
    val = (val << 4) | (byte & 0xF);
    _count++;
  }
  return val;
}

#endif
