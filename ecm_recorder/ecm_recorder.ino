/* 
 * Digitise audio from microphone and save to SD card.
 * Up to 10 .wav files can be saved.
 * Replay latest recorded file 2 seconds after save.
 * Update file name after replay.
 * 
 * Play one of the recorded files if button not pressed for 10 seconds
 * Play one of the music files if nothing recorded for 1 minute
 * If file not found dont do anything.
 * 
 * Modified by James Grigg 27 November 2018
 * from 
 *    APC magazine  - Arduino Masterclass
 *    Project #18 - Digital Audio Recorder v2.0
 *    Darren Yates - 11 April 2014
 *    Includes the SdFat library - https://code.google.com/p/sdfatlib/downloads/list
 *    
 *    Installed SdFat Library by Bill Greiman Version 1.0.7
 *    ***** excellent library ******
 *    
 * 29 November 2018   
 * Add file to record current filename, ( filename.txt )
 *    check for valid file
 *    read at start to get name of file to save
 *    when record save to this filename,
 *    when record end, increment name, 
 *    save in filename.txt for next record.
 *    
 *    Standard audio sampling rates 44100, 48000, 32000, 22050, 11025, 8000
 *    Use these in wave file definitions
 *    
 * 02 December 2018   
 *    Record and playback working ok.
 *    Audio playback quality very poor.
 *    
 * 06 Jamuary 2019
 *    Seemed to have fixed audio distortion
 *    Carrier and rate discrepancy, last carrier cycle ended too early
 *    Introducing frequencies not in the original.
 *    But at higher volume level audio distortion still present.
 *    Not really fixed.
 *    
 *    Changed sample rate from 8KHz to 15K625Hz, seems to have fixed problem.
 *    Audio hiss present, reduces if mic amp gain set lower. ie 60dB to 40dB.
 *    LM386 board used to amplify audio, PAM8302A did not want to work. 
 *    680ohm from pin to 100uF capacitor to LM386 input. 
 * 
 * 09 Jan 2019
 *    Single switch rather than two.
 *    Changed to ADC5, better layout for PCB.
 *    MIC closer to ADC5 than ADC0 pin on ATMega328P
 *    
 * 16 Jan 2019   
 *    Single switch now working ok, circuit built.
 *    Added LM385 power switch, reduces current drain on battery. 
*/
//==================================================================================================
#include <SdFat.h>

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

// digital pin definitions
#define SD_CS           10  // chip select line for SD card
#define RedLED           2  // Red LED
#define GreenLED         4  // Green LED
#define btnStart         5  // push button start/stop record
#define LM386pwr         8  // power up LM386 board only when replay.

// audio output pin = 9. Timer 1A. Direct register access.

#define BUF_SIZE         256  // 
#define BUF_SIZEx2       512  // 
#define WAIT_REPLAY     2000  // Delay in milliseconds between file saved and file replay
#define WAIT_RANDOM    10000  // Delay in milliseconds to wait after replay, to play random file
#define WAIT_MUSIC     60000  // Delay in milliseconds to wait after any file output, before playing random music file
#define WAIT_DEBOUNCE    500  // Delay in milliseconds to wait after button pressed

unsigned long fileSize = 0L;
unsigned long waveChunk = 16;           // Size of 'chunk', waveType + numChannels + sampleRate + bytesPerSec + blockAlign + bitsPerSample
unsigned int waveType = 1;              // 1 = PCM
unsigned int numChannels = 1;           // 1 = mono, 2 = stereo
unsigned long sampleRate = 15625;
unsigned long bytesPerSec = 15625;     // sampleRate * numChannels * bitsPerSample / 8
unsigned int blockAlign = 1;           // numChannels * bitsPerSample / 8
unsigned int bitsPerSample = 8;
unsigned long dataSize = 0L;

unsigned long millisCount = 0L;
unsigned long millisWaitReplay = 0L;
unsigned long millisWaitRandom = 0L;
unsigned long millisWaitMusic = 0L;
unsigned long millisDebounce = 0L;

bool RecordActive = false;              // audio recording in progress
bool FileSaved = false;                 // indicates a .wav file has been saved
bool Replay = false;                    // indicates a .wav file has been saved and has been replayed
bool SendByte = false ;                 // indicates an interrupt has occured and time to output another byte
bool Buffer = false;                    // select buffer 0 at start, buffer 0 = false, buffer 1 = true
bool ButtonRelease = false;             // button debounce, must release button before it can be used again

byte buf00[BUF_SIZE]; 
byte buf01[BUF_SIZE]; 
byte byte1, byte2, byte3, byte4;

unsigned long ByteCount = 0L;
unsigned long ByteSaved = 0L;
unsigned int bufByteCount;

char Wav_File[13] = "rec00000.wav";  // starting filename
char Wav_Name[13] = "wav_name.txt";  // has the next Wav_File.wav name to use

SdFat sd;
SdFile SD_file;

//==================================================================================================
void setup()
{
  Serial.begin(115200);
  Serial.println("Audio Recorder Playback PWM Output 003 ");
  Serial.println(' ');

  pinMode(RedLED, OUTPUT);
  pinMode(GreenLED, OUTPUT);
  pinMode(btnStart, INPUT_PULLUP);
  pinMode(LM386pwr, OUTPUT);
  digitalWrite(LM386pwr,HIGH);  // power off

  //@INITIALISE SD CARD
  Serial.println(F("====== Initializing SD card ..."));
  if (sd.begin(SD_CS, SPI_FULL_SPEED))
  {
    // 8MHz SPI bus speed
    for (int i = 0; i < 8; i++)
    {
      digitalWrite(RedLED, !digitalRead(RedLED));
      delay(100);
    }
// indicate recording stopped, this is the start up condition
    digitalWrite(RedLED,LOW);  
    digitalWrite(GreenLED,HIGH);
  } else { 
// no SD card or card error, flash LED twice per second, until reset
//Serial.println(F("Card failed, or not present"));
    SD_Error();             // never returns from here
  }
  GetWavFileName(false);          // get current name, dont change it
  millisDelay(1000);

// Timer 1, 2 and ADC setup, direct register access
  Setup_Timer_1_2_and_ADC();
}

//==================================================================================================
void loop()
{
// Button must be released for at least 500 milliseconds before it can be actioned again.
// Also check the button has been released before actioning to avoid the situation where
// holding down the button switches from start record to stop record to start record etc.
  if (millis() > millisDebounce) {
    if (digitalRead(btnStart) == LOW) {
      if (ButtonRelease) {
        if (!RecordActive) { StartRec(); } else { StopRec(); }
        ButtonRelease = false;
      }    
      millisDebounce = millis() + WAIT_DEBOUNCE;
    } else { ButtonRelease = true; }
  }
// buffer 0 full save to card
  if (ByteCount % BUF_SIZEx2 == BUF_SIZE && RecordActive) { 
    SD_file.write(buf00,BUF_SIZE); 
    ByteSaved+= BUF_SIZE; 
  }
// buffer 1 full save to card
  if (ByteCount % BUF_SIZEx2 == 0 && RecordActive) { 
    SD_file.write(buf01,BUF_SIZE); 
    ByteSaved+= BUF_SIZE; 
  }
// check for file recorded and saved, if so wait replay time then output  
  if (FileSaved && !RecordActive) {
    if (millisWaitReplay > 0) {
      if (millis() > millisWaitReplay) {
        OutputWav_File(Wav_File);
        GetWavFileName(true);               // update file name after file has been output.
      }
    }
  }
// check for inactive time expired then output a random file
  if (Replay && !FileSaved && !RecordActive) {
    if (millisWaitRandom > 0) {
      if (millis() > millisWaitRandom) {
        OutputRandom_File(false);
      }
    }
    if (millisWaitMusic > 0) {
      if (millis() > millisWaitMusic) {
        OutputRandom_File(true);
        millisWaitMusic = millis() + WAIT_MUSIC;
      }
    }
  }
}

//==================================================================================================
//==================================================================================================
//==================================================================================================
void OutputRandom_File(bool music)
{
  char f[13] = "rec00000.wav";    
  char m[13] = "play0000.wav";    
  int num = 0;
  num = random(10);
  
  if (music)
  {
    OCR2A = 255;             // music files 8000Hz sample rate
    m[7] = char(num + 48);   // convert to ASCII 
    OutputWav_File(m);
    OCR2A = 127;              // set rate back to 15625Hz rate
  } else {
    f[7] = char(num + 48);  // convert to ASCII 
    OutputWav_File(f);
  }
  return;
}
//==================================================================================================
void OutputWav_File(char* _File) {

//  unsigned long i = 0L;
  
  digitalWrite(RedLED,HIGH);
  digitalWrite(GreenLED,LOW);

  ByteCount = 0;
  ByteSaved = 0;
  Buffer = false;     // point to buffer 0

  if (SD_file.open(_File, O_READ)) {      // if file does not exist do nothing
    SD_file.seekSet(40);

// get data size of file
    byte1 = (byte)(SD_file.read());
    byte2 = (byte)(SD_file.read());
    byte3 = (byte)(SD_file.read());
    byte4 = (byte)(SD_file.read());
    dataSize = byte4; 
    dataSize = dataSize << 8;
    dataSize = dataSize + byte3; 
    dataSize = dataSize << 8;
    dataSize = dataSize + byte2; 
    dataSize = dataSize << 8;
    dataSize = dataSize + byte1;

    SD_file.read(buf00,BUF_SIZE);         // fill buffers before output start
    SD_file.read(buf01,BUF_SIZE);

    digitalWrite(LM386pwr,LOW);  // power on
// enable interrupts    
    sbi (TIMSK2, OCIE2A); 
// output the file data, stay in this loop till all of the file sent
// interrupt routine sends byte, this gets new buffer data when necessary
    while (ByteSaved < dataSize) {
      if (ByteCount % BUF_SIZEx2 == BUF_SIZE) { 
        SD_file.read(buf00,BUF_SIZE); 
        ByteSaved+= BUF_SIZE; 
      }
      if (ByteCount % BUF_SIZEx2 == 0) { 
        SD_file.read(buf01,BUF_SIZE); 
        ByteSaved+= BUF_SIZE; 
      }
    }
// disable interrupts    
    cbi (TIMSK2, OCIE2A); 
    OCR1A = 1;
    digitalWrite(LM386pwr,HIGH);  // power off
  }
  SD_file.close();
  millisWaitReplay = 0L;
  millisWaitRandom = millis() + WAIT_RANDOM;
  FileSaved = false;
  Replay = true;
  digitalWrite(RedLED,LOW);
  digitalWrite(GreenLED,HIGH);
  return;
}

void GetWavFileName(bool update_name)
{
  bool fileok = true;
  
// check SD card files
// check if Wav_Name exists on SD card
// check if it has a valid file size
// check if the name stored in the file is valid "rec00000.txt"

  if (sd.exists(Wav_Name))
  {                    
    if (SD_file.open(Wav_Name, O_RDWR)) {
      SD_file.rewind();
      if (SD_file.fileSize() == 12) {            
        SD_file.read(Wav_File, 12);
        if ((Wav_File[0] == 'r') && (Wav_File[1] == 'e') && (Wav_File[2] == 'c')) {
          for (int i = 3; i < 8; i++) {
            if ((Wav_File[i] < 48) || (Wav_File[i] > 57)) { fileok = false; }
          }
          if (fileok && update_name) {
            int n0 = (int)(Wav_File[7]);
            if (n0++ == 57) { n0 = 48; }                // note: n0 is incremented after the compare
            Wav_File[7] = char(n0);
            SD_file.rewind();
            SD_file.write(Wav_File);
          }
        } else { fileok = false; }
      } else { fileok = false; }
      SD_file.close();
    } else {
//Serial.println(F("Cannot open wav_name.txt"));
       SD_Error();                              // never returns from here
    }
  } else { fileok = false; }
// no record of valid files or the Wav_Name file is corrupt, then start at beginning 
  if (! fileok) {
//Serial.println(F("Creating new wav_name.txt ....."));
    SD_file.open(Wav_Name, O_CREAT | O_TRUNC | O_RDWR);
    SD_file.write(Wav_File);                              
  }
// close Wav_Name after we have next Wav_File name 
  SD_file.close();
  millisDelay(500);
  return;
}

//==================================================================================================
void StartRec() {

// try and get a random seed, ie it is not known when record started.
  randomSeed(millis());

// indicate recording started
  digitalWrite(RedLED,HIGH);    
  digitalWrite(GreenLED,LOW);

  ByteCount = 0;
  ByteSaved = 0;
  Buffer = false;     // point to buffer 0
  Replay = false;
  FileSaved = false;
  RecordActive = true;
  millisWaitReplay = 0L;
  millisWaitRandom = 0L;
  millisWaitMusic = 0L;
  FileSaved = false;

  SD_file.open(Wav_File, O_CREAT | O_TRUNC | O_RDWR);

// write the initial WAV header to file
  SD_file.write("RIFF");

  byte1 = fileSize & 0xff;
  byte2 = (fileSize >> 8) & 0xff;
  byte3 = (fileSize >> 16) & 0xff;
  byte4 = (fileSize >> 24) & 0xff;  
  SD_file.write(byte1);  SD_file.write(byte2);  SD_file.write(byte3);  SD_file.write(byte4);

  SD_file.write("WAVE");
  SD_file.write("fmt ");

  byte1 = waveChunk & 0xff;
  byte2 = (waveChunk >> 8) & 0xff;
  byte3 = (waveChunk >> 16) & 0xff;
  byte4 = (waveChunk >> 24) & 0xff;  
  SD_file.write(byte1);  SD_file.write(byte2);  SD_file.write(byte3);  SD_file.write(byte4);

  byte1 = waveType & 0xff;
  byte2 = (waveType >> 8) & 0xff;
  SD_file.write(byte1);  SD_file.write(byte2); 

  byte1 = numChannels & 0xff;
  byte2 = (numChannels >> 8) & 0xff;
  SD_file.write(byte1);  SD_file.write(byte2); 

  byte1 = sampleRate & 0xff;
  byte2 = (sampleRate >> 8) & 0xff;
  byte3 = (sampleRate >> 16) & 0xff;
  byte4 = (sampleRate >> 24) & 0xff;  
  SD_file.write(byte1);  SD_file.write(byte2);  SD_file.write(byte3);  SD_file.write(byte4);

  byte1 = bytesPerSec & 0xff;
  byte2 = (bytesPerSec >> 8) & 0xff;
  byte3 = (bytesPerSec >> 16) & 0xff;
  byte4 = (bytesPerSec >> 24) & 0xff;  
  SD_file.write(byte1);  SD_file.write(byte2);  SD_file.write(byte3);  SD_file.write(byte4);
  
  byte1 = blockAlign & 0xff;
  byte2 = (blockAlign >> 8) & 0xff;
  SD_file.write(byte1);  SD_file.write(byte2); 
  
  byte1 = bitsPerSample & 0xff;
  byte2 = (bitsPerSample >> 8) & 0xff;
  SD_file.write(byte1);  SD_file.write(byte2); 
  
  SD_file.write("data");
  
  byte1 = dataSize & 0xff;
  byte2 = (dataSize >> 8) & 0xff;
  byte3 = (dataSize >> 16) & 0xff;
  byte4 = (dataSize >> 24) & 0xff;  
  SD_file.write(byte1);  SD_file.write(byte2);  SD_file.write(byte3);  SD_file.write(byte4);

// enable timer interrupt, start recording audio
// interrupt routine gets the data
  sbi (TIMSK2, OCIE2A); 

  return;  
}
//==================================================================================================
void StopRec() {
    
// disable timer interrupt
  cbi (TIMSK2, OCIE2A); 

// Update the WAV file with its file size and data size in bytes  
// values saved in file LS byte first. ie 129,024 bytes = 0001f800h. 00 f8 01 00 stored in file 

// FileSize is 36 byte longer than DataSize due to header information but doesnt include first 8 bytes.
//      "RIFF" - 4 bytes, FileSize - 4 bytes.
// So actual file size will be Datasize + 36 + 4 + 4 bytes
  byte1 = (ByteSaved + 36) & 0xff;
  byte2 = ((ByteSaved + 36) >> 8) & 0xff;
  byte3 = ((ByteSaved + 36) >> 16) & 0xff;
  byte4 = ((ByteSaved + 36) >> 24) & 0xff;  
  SD_file.seekSet(4);
  SD_file.write(byte1);  SD_file.write(byte2);  SD_file.write(byte3);  SD_file.write(byte4);

// Update DataSize, number of bytes in audio wave form
  byte1 = ByteSaved & 0xff;
  byte2 = (ByteSaved >> 8) & 0xff;
  byte3 = (ByteSaved >> 16) & 0xff;
  byte4 = (ByteSaved >> 24) & 0xff;  
  SD_file.seekSet(40);
  SD_file.write(byte1);  SD_file.write(byte2);  SD_file.write(byte3);  SD_file.write(byte4);

  SD_file.close();

  millisWaitReplay = millis() + WAIT_REPLAY;
  millisWaitMusic = millis() + WAIT_MUSIC;
  RecordActive = false;
  Replay = false;
  FileSaved = true;

// indicate recording stopped
  digitalWrite(RedLED,LOW);
  digitalWrite(GreenLED,HIGH);
  return;  
}
//==================================================================================================
ISR(TIMER2_COMPA_vect) {

  if (RecordActive) {
    sbi(ADCSRA, ADSC);                  // start ADC sample
    while(bit_is_set(ADCSRA, ADSC));    // wait until ADSC bit goes low = new sample ready, 
                                        //           about 6.5uSecs at 2MHz ADC clock
    ByteCount++;                        
    bufByteCount++;
    if (bufByteCount == BUF_SIZE) {
      if (! Buffer) { Buffer = true; } else { Buffer = false; }
      bufByteCount = 0;
    }
    if (! Buffer) { buf00[bufByteCount] = ADCH; } else { buf01[bufByteCount] = ADCH; }
  }
  if (FileSaved || Replay) {
    ByteCount++;                        
    bufByteCount++;
    if (bufByteCount == BUF_SIZE) {
      if (! Buffer) { Buffer = true; } else { Buffer = false; }
      bufByteCount = 0;
    }
    if (! Buffer) { OCR1A = buf00[bufByteCount]; } else { OCR1A = buf01[bufByteCount]; }
  }
}
//==================================================================================================
void Setup_Timer_1_2_and_ADC() {

// Timer 2 setup, Use compare match to interrupt to read ADC value, also to output bytes on Timer 1 OC1A pin.
//    (note: SD card module has 256 times interrupt rate to write buffer before next buffer ready.)
// WGM22,21,20 = 010. CTC mode - clear timer on compare match. OCR2A sets top of counter.
// CS22,21,20 = 010. Prescaler 8.
// Interupt frequency  = CPU clock (16MHz) / (Prescale * (OCR2A + 1))
//  16MHz/(8*(250+1)) = 7968.127 
//  For a 7812.5Hz rate OCR2A = 16MHz/(7812.5*8) - 1 =  255. (62,500 / 8 = 7812.5)
//  For a 8KHz rate OCR2A = 16MHz/(8KHz*8) - 1 =  249. 
//  For a 15,625Hz rate OCR2A = 16MHz/(15625*8) - 1 =  127. (62,500 / 4 = 15625)
  
  TCCR2A &= ~_BV(WGM22);  // 
  TCCR2A |= _BV(WGM21);   // 
  TCCR2A &= ~_BV(WGM20);  // 
                          
  TCCR2B &= ~_BV(CS22);   // Timer2 Clock Prescaler to : 8
  TCCR2B |= _BV(CS21);   
  TCCR2B &= ~_BV(CS20);  

  OCR2A = 127;            // Compare Match register

// other interrupt frequencies
/*  Hard to get exact rate correctly. 16MHz should be 16M384Hz for exact rates.
 *  OCR2A = 89;           // 16MHz/(8*(89+1)) = 22,222.22222Hz 
 *  OCR2A = 90;           // 16MHz/(8*(90+1)) = 21,978.02197Hz
 *  OCR2A = 91;           // 16MHz/(8*(91+1)) = 21,730.13043Hz 
 *   
 *  The following evenly divide into 62,500 carrier rate. 
 *  OCR2A = 255;           // 16MHz/(8*(X+1)) = 7,812.5   128uSecs
 *                            16MHz = 7,812.5 * (8*(X+1)) 
 *                            8*(X+1)  = 2048
 *                            X+1 = 256
 *                            X = 255
 *                            Time to write buffer 256 * 128uS = 32.768mS
 *                            
 *  OCR2A = 127;           // 16MHz/(8*(X+1)) = 15,625    64usecs
 *                            16MHz = 15,625 * (8*(X+1)) 
 *                            8*(X+1)  = 1024
 *                            X+1 = 128
 *                            X = 127
 *                            Time to write buffer 256 * 64uS = 16.384mS
 *                            
 *  OCR2A = 63;           // 16MHz/(8*(X+1)) = 31,250     32uSecs
 *                            16MHz = 31,250 * (8*(X+1)) 
 *                            8*(X+1)  = 512
 *                            X+1 = 64
 *                            Time to write buffer 256 * 128uS = 8.192mS
 *                            X = 63
*/
//........................
// Timer 1 setup. Outputs audio on pin 9 OCR1A. 
// PWM clock = 62,500Hz. Pulse width = size of the byte.  

// Counts from BOTTOM (0x00) to TOP (0xff) then restarts from the BOTTOM, 256 cycles.
// With no prescaler, highest frequency is F_CPU (16MHz) / 256 = 62,500Hz. 16uSecs.
// OC1A connected to the output, pin 9.
// On a compare match OC1A is cleared. Timer count value = OCR1A value.
// At BOTTOM OC1A is set. This generates the variable pulse width representing the audio signal.
// Higher values will result in a longer pulse width.
// When timer 2 interrupts, OCR1A changed to the new value. 
// If rate is 8000Hz, OCR1A will be the same for 62,500/8000 = 7.8125 cycles. 
// Is this a problem, not integer boundary ??? YES YES YES
//          8th pulse will be distorted.
// 62,500/2 = 31,250; 31,250/2 = 15,625; 15,625/2 = 7812.5; Exactly 8 times.
// Set Timer 2 to 7812.5Hz interrupt.

// Fast PWM 8-bit  WGM13,12,11,10 = 0101
  TCCR1B &= ~_BV(WGM13);
  TCCR1B |= _BV(WGM12);
  TCCR1A &= ~_BV(WGM11);
  TCCR1A |= _BV(WGM10);

// Clear OC1A on Compare Match (Set output to low level) COM1A1,0 = 10
  TCCR1A |= _BV(COM1A1);
  TCCR1A &= ~_BV(COM1A0); 

// I/O Clock / 1 (No prescaling)
  TCCR1B &= ~_BV(CS12);
  TCCR1B &= ~_BV(CS11); 
  TCCR1B |= _BV(CS10);

// values placed here will alter the duty cycle
  OCR1A = 1;
// set data direction on PB1, D9, pin15, OCR1A as output.  
  DDRB |= _BV(0x01);      

//........................
// ADC setup, select analog pin, left adjust, voltage reference, set prescaler for ADC clock
  ADMUX = 0x25;         // REFS1,0 = 00. External AREF, internal Vref turned off
                        //               2.5V reference connected to ARef pin 21.
                        // Input signal from MIC amplifier is 2Vpp 1.25V DC level. 
                        // Signal range is 0.25V to 2.25V.
                        // ADC will produce (1024/2.5)*2  = 819 samples.
                        //    2.5V/1024 = 0.00244140625V per sample.  
                        //    At 0.25V sample = 102.4,  at 2.25V sample = 921.6
                        //    921.6 - 102.4 = 819.2
                        // 
                        // ADLAR = 1. Result is left adjusted, ADCH has upper 8 bits of conversion.
                        //            8 bit resolution. 102 to 921 becomes 5 to 204. 
                        //            Note: never 0 or 255. Avoids timing issues.
                        // MUX3,2,1,0 = 0101. ADC5 pin assigned. Connect audio input to A5.
                        // PCB board design placed MIC output closer to pin 28 (A5) than pin 23 (A0)

  cbi(ADCSRA,ADPS2);    // ADPS2,1,0 bits = 011. Prescaler set to 8 
  sbi(ADCSRA,ADPS1);    //    ADC clock = CPU Clock / 8 = 2MHz,  0.5uSecs per clock cycle.
  sbi(ADCSRA,ADPS0);    //    13 ADC clocks for each conversion, 0.5 * 13  =  6.5uSecs to convert. 

  return;
}
// other settings for ADMUX
/*  
  ADMUX = 0x20;         // REFS1,0 = 00.      External AREF, internal Vref turned off
                        //                    2.5V reference connected to ARef pin 21.
                        // ADLAR = 1.         Result is left adjusted, ADCH has upper 8 bits of conversion.
                        // MUX3,2,1,0 = 0000. ADC0 pin assigned. Connect audio input to A0.

  ADMUX = 0x65;         // REFS1,0 = 01.      AVcc with external capacitor at AREF pin
                        // ADLAR = 1.         Result is left adjusted, ADCH has upper 8 bits of conversion.
                        // MUX3,2,1,0 = 0101. ADC5 pin assigned. Connect audio input to A5.

  ADMUX = 0x60;         // REFS1,0 = 01.      AVcc with external capacitor at AREF pin
                        // ADLAR = 1.         Result is left adjusted, ADCH has upper 8 bits of conversion.
                        // MUX3,2,1,0 = 0101. ADC0 pin assigned. Connect audio input to A0.
*/
//==================================================================================================
void SD_Error (void) {
// flash LED twice per second indicating an error, most likely in SD card
  while(1) {
    digitalWrite(RedLED,!digitalRead(RedLED));
    millisDelay(500);
  }
}
//==================================================================================================
void millisDelay (int d) {
  millisCount = millis() + d;
  while (millis() < millisCount) {}
  return;
}
//==================================================================================================
/*void printSDfileList(void) {

  File root = sd.open("/");
  if (root) {
    root.rewind();
    root.ls(LS_SIZE);
//    root.ls(LS_DATE | LS_SIZE);
  }
  return;
}*/
