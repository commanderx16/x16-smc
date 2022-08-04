#define SCANCODE_TIMEOUT_MS 50

// Should clock and data pins be part of the template instead of constructor args?
// Doing so might lead to code bloat if the compiler builds two nearly identical functions
// that differ only by the constants used.

/// @brief PS/2 IO Port handler
/// @tparam size Circular buffer size for incoming data, must be a power of 2 and not more than 256
template<int clk, int dat, uint8_t size = 16> // Single keycodes can be 4 bytes long. We want a little bit of margin here.
class PS2Port
{
    static_assert(size <= 256, "Buffer size may not exceed 256");                // Hard limit on buffer size
    static_assert((size & (size - 1)) == 0, "Buffer size must be a power of 2");  // size must be a power of 2
    static_assert(digitalPinToInterrupt(clk) != NOT_AN_INTERRUPT);

  private:
    uint8_t clkPin;
    uint8_t datPin;

    // We keep a record of the point in the buffer that contains the last completed keycodes.
    // Below are the possible keycode-patters. "lastCompletedKeycodeIndex" now points to the last byte
    // of the last completed keycode. In the table below this is always the point of 'yy'.

    // KeyDown  |   KeyUp
    // ------------------------
    //     yy   |      F0yy
    //   E0yy   |    E0F0yy
    // E1xxyy   |  E1F0xxyy

    volatile uint8_t head;
    volatile uint8_t tail;
    volatile uint8_t buffer[size];

    uint8_t curCode;
    byte parity;
    byte rxBitCount;
    uint32_t lastBitMillis;

    volatile uint8_t ps2ddr;
    volatile uint8_t outputBuffer[2];
    volatile uint8_t outputSize = 0;
    volatile uint8_t timerCountdown;
    volatile uint8_t commandStatus = 0;

    void resetReceiver() {
      pinMode(datPin, INPUT);
      pinMode(clkPin, INPUT);
      curCode = 0;
      parity = 0;
      rxBitCount = 0;
      ps2ddr = 0;
      outputSize = 0;
      timerCountdown = 0;
      flush();
    };

  public:
    PS2Port() :
      clkPin(clk), datPin(dat),
      head(0), tail(0), curCode(0), parity(0), lastBitMillis(0), rxBitCount(0), ps2ddr(0), timerCountdown(0)
      {
        resetReceiver();
      };

    /// @brief Begin processing PS/2 traffic
    void begin(void(*irqFunc)()) {
      attachInterrupt(digitalPinToInterrupt(clkPin), irqFunc, FALLING);
    }

    /// @brief Process data on falling clock edge
    /// @attention This is interrupt code
    void onFallingClock() {
      if (ps2ddr == 0)
        onFallingClock_DeviceToHost();
      else
        onFallingClock_HostToDevice();
    }

    void onFallingClock_DeviceToHost()
    {
      uint32_t curMillis = millis();
      if (curMillis >= (lastBitMillis + SCANCODE_TIMEOUT_MS))
      {
        // Haven't heard from device in a while, assume this is a new keycode
        resetReceiver();
      }
      lastBitMillis = curMillis;

      byte curBit = digitalRead(datPin);
      switch (rxBitCount)
      {
        case 0:
          // Start bit
          if (curBit == 0)
          {
            rxBitCount++;
          } // else Protocol error - no start bit
          break;

        case 1: case 2: case 3: case 4: case 5: case 6: case 7: case 8:
          // Data bit, LSb first
          if (curBit) curCode |= 1 << (rxBitCount - 1);
          parity += curBit;
          rxBitCount++;
          break;

        case 9:
          // parity bit
          parity += curBit;
          // Parity bit will be checked after stop bit
          rxBitCount++;
          break;

        case 10:
          // stop bit
          if (curBit != 1) {
            // Protocol Error - no stop bit
          }
          else if ((parity & 0x1) != 1) {
            // Protocol Error - parity mismatch
          }

          //Hhost to device command response handler
          if (commandStatus==1){
            if (curCode==0xFE){
              //Command error - Resend
              commandStatus=0xFE;
            }
            else if (curCode==0xFA){
              if (outputSize==2){
                //Send second byte
                sendPS2Command(1, outputBuffer[1]);
              }
              else{
                //Command ACK
                commandStatus=0xFA;
              }
            }
          }

          //Update input buffer
          byte headNext = (head + 1) & (size - 1);
          if (headNext != tail){
            buffer[head] = (byte)(curCode);
            head = headNext;
          }
          //Else Ring buffer overrun, drop the incoming code :(
          DBG_PRINT("keycode: ");
          DBG_PRINTLN((byte)(curCode), HEX);
          rxBitCount = 0;
          parity = 0;
          curCode = 0;
          break;
      }
    }

    /**
       Interrupt handler used when sending data
       to the PS/2 device
    */
    void onFallingClock_HostToDevice() {
      if (timerCountdown > 0) {
        //Ignore clock transitions during the request-to-send
        return;
      }


      switch (rxBitCount)
      {
        case 0:
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
          //Output data bits 0-7
          if (outputBuffer[0] & 1) {
            pinMode(datPin, INPUT);
            digitalWrite(datPin, HIGH);
          }
          else {
            digitalWrite(datPin, LOW);
            pinMode(datPin, OUTPUT);
          }

          //Update parity
          parity = parity + outputBuffer[0];

          //Right shift output value - always sending the rightmost bit
          outputBuffer[0] = (outputBuffer[0] >> 1);

          //Prepare for next clock cycle
          rxBitCount++;
          break;

        case 8:
          //Send odd parity bit
          if ((parity & 1) == 1) {
            digitalWrite(datPin, LOW);
            pinMode(datPin, OUTPUT);
          }
          else {
            pinMode(datPin, INPUT);
            digitalWrite(datPin, HIGH);
          }

          //Prepare for stop bit
          rxBitCount++;
          break;

        case 9:
          //Stop bit
          pinMode(datPin, INPUT);
          digitalWrite(datPin, HIGH);
          rxBitCount++;
          break;

        case 10:
          //ACK
          resetReceiver();    //Prepare host to receive device ACK or Resend (error) code
          break;
      }
    }

    /// @brief Returns true if at least one byte is available from the PS/2 port
    inline bool available() {
      return head != tail;
    };

    /// @brief Returns the next available byte from the PS/2 port
    uint8_t next() {
      if (available()) {
        uint8_t value = buffer[tail];
        tail = (tail + 1) & (size - 1);
        return value;
      }
      else {
        return 0;
      }
    };

    void flush() {
      head = tail = 0;
      //lastBitMillis = 0;
    }

    void reset() {
      resetReceiver();
      commandStatus = 0;
    }

    /**
       Sends a command to the PS/2 device
       A command may consist of one or two bytes

       argc  - Number of arguments = bytes to send
       cmd   - First byte, typically a command
       data  - Second byte, typically a command parameter
    */
    void sendPS2Command(uint8_t argc, uint8_t cmd = 0, uint8_t data = 0) {
      if (argc < 1 || argc > 2) {
        //Invalid input
        outputSize = 0;
        return;
      }

      //Prepare sending data to the device
      commandStatus = 1;        //Command pending
      outputBuffer[0] = cmd;    //Fill output buffer
      outputBuffer[1] = data;
      outputSize = argc;        //Output buffer size

      timerCountdown = 3;       //Will determine clock hold time for the request-to-send initiated in the timer 1 interrupt handler
    }

    uint8_t getCommandStatus() {
      return commandStatus;
    }

    /*
       The timerInterrupt is made to be called by the Arduino
       timer interrupt handler once every 100 us. It
       controls the timing requirements when writing to
       a PS/2 device
    */
    void timerInterrupt() {
      if (timerCountdown == 0x00) {
        //The host is currently sending or receiving, no operation required
        return;
      }

      else if (timerCountdown == 3) {
        //Initiate request-to-send sequence
        digitalWrite(clkPin, LOW);
        pinMode(clkPin, OUTPUT);

        digitalWrite(datPin, LOW);
        pinMode(datPin, OUTPUT);

        ps2ddr = 1;
        timerCountdown--;
      }

      else if (timerCountdown == 1) {
        //We are at end of the request-to-send clock hold time of minimum 100 us
        pinMode(clkPin, INPUT);

        timerCountdown = 0;
        rxBitCount = 0;
        parity = 0;
      }

      else{
        timerCountdown--;
      }
    }

    uint8_t count(){
      return (size+head-tail)&(size-1);
    }

};
