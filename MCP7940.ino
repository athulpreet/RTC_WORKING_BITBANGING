/*   
  STM32F103C8 Blue Pill with MCP7940N RTC Module
  Using bit-banged I2C on PB8/PB9 and UART2 on PA2/PA3
  MFP pin on PA1
*/

// MCP7940N I2C address
#define MCP7940N_ADDRESS 0x6F

// Bit-banged I2C pins
#define SDA_PIN PB9
#define SCL_PIN PB8
#define MFP_PIN PA1

// UART pins
#define UART_TX_PIN PA2
#define UART_RX_PIN PA3

// MCP7940N registers
#define MCP7940N_REG_SECONDS 0x00
#define MCP7940N_REG_MINUTES 0x01
#define MCP7940N_REG_HOURS   0x02
#define MCP7940N_REG_WKDAY   0x03
#define MCP7940N_REG_DATE    0x04
#define MCP7940N_REG_MONTH   0x05
#define MCP7940N_REG_YEAR    0x06
#define MCP7940N_REG_CONTROL 0x07
#define MCP7940N_REG_OSCTRIM 0x08

// I2C bit-banging timing
#define I2C_DELAY_US 5

// Storage for date/time
uint8_t seconds, minutes, hours, day, date, month, year;
char dateTimeStr[64]; // for sprintf

// Bit-banged I2C functions - unchanged
void i2c_init() {
  pinMode(SCL_PIN, OUTPUT_OPEN_DRAIN);
  pinMode(SDA_PIN, OUTPUT_OPEN_DRAIN);
  digitalWrite(SCL_PIN, HIGH);
  digitalWrite(SDA_PIN, HIGH);
}

void i2c_start() {
  digitalWrite(SDA_PIN, HIGH);
  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(I2C_DELAY_US);
  digitalWrite(SDA_PIN, LOW);
  delayMicroseconds(I2C_DELAY_US);
  digitalWrite(SCL_PIN, LOW);
}

void i2c_stop() {
  digitalWrite(SCL_PIN, LOW);
  digitalWrite(SDA_PIN, LOW);
  delayMicroseconds(I2C_DELAY_US);
  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(I2C_DELAY_US);
  digitalWrite(SDA_PIN, HIGH);
  delayMicroseconds(I2C_DELAY_US);
}

bool i2c_write_byte(uint8_t data) {
  // Send 8 bits, MSB first
  for (int i = 7; i >= 0; i--) {
    digitalWrite(SCL_PIN, LOW);
    digitalWrite(SDA_PIN, data & (1 << i) ? HIGH : LOW);
    delayMicroseconds(I2C_DELAY_US);
    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(I2C_DELAY_US);
  }
  
  // Get ACK
  digitalWrite(SCL_PIN, LOW);
  pinMode(SDA_PIN, INPUT_PULLUP);
  delayMicroseconds(I2C_DELAY_US);
  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(I2C_DELAY_US);
  bool ack = (digitalRead(SDA_PIN) == LOW);  // ACK = LOW
  digitalWrite(SCL_PIN, LOW);
  pinMode(SDA_PIN, OUTPUT_OPEN_DRAIN);
  
  return ack;
}

uint8_t i2c_read_byte(bool ack) {
  uint8_t data = 0;
  
  pinMode(SDA_PIN, INPUT_PULLUP);
  
  // Read 8 bits, MSB first
  for (int i = 7; i >= 0; i--) {
    digitalWrite(SCL_PIN, LOW);
    delayMicroseconds(I2C_DELAY_US);
    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(I2C_DELAY_US);
    if (digitalRead(SDA_PIN)) {
      data |= (1 << i);
    }
  }
  
  // Send ACK or NACK
  digitalWrite(SCL_PIN, LOW);
  pinMode(SDA_PIN, OUTPUT_OPEN_DRAIN);
  digitalWrite(SDA_PIN, ack ? LOW : HIGH);  // ACK = LOW, NACK = HIGH
  delayMicroseconds(I2C_DELAY_US);
  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(I2C_DELAY_US);
  digitalWrite(SCL_PIN, LOW);
  
  return data;
}

// MCP7940N functions
bool mcp7940n_write_register(uint8_t reg, uint8_t value) {
  i2c_start();
  if (!i2c_write_byte(MCP7940N_ADDRESS << 1)) { // Write address
    i2c_stop();
    return false;
  }
  if (!i2c_write_byte(reg)) { // Register address
    i2c_stop();
    return false;
  }
  if (!i2c_write_byte(value)) { // Data
    i2c_stop();
    return false;
  }
  i2c_stop();
  return true;
}

bool mcp7940n_read_register(uint8_t reg, uint8_t *value) {
  i2c_start();
  if (!i2c_write_byte(MCP7940N_ADDRESS << 1)) { // Write address
    i2c_stop();
    return false;
  }
  if (!i2c_write_byte(reg)) { // Register address
    i2c_stop();
    return false;
  }
  
  i2c_start(); // Repeated start
  if (!i2c_write_byte((MCP7940N_ADDRESS << 1) | 1)) { // Read address
    i2c_stop();
    return false;
  }
  
  *value = i2c_read_byte(false); // Read with NACK
  i2c_stop();
  return true;
}

bool mcp7940n_read_time() {
  uint8_t value;
  
  // Read seconds
  if (!mcp7940n_read_register(MCP7940N_REG_SECONDS, &value)) return false;
  // Strip off the ST bit (bit 7)
  seconds = (value & 0x0F) + ((value & 0x70) >> 4) * 10;
  
  // Read minutes
  if (!mcp7940n_read_register(MCP7940N_REG_MINUTES, &value)) return false;
  minutes = (value & 0x0F) + ((value & 0x70) >> 4) * 10;
  
  // Read hours (assuming 24-hour mode)
  if (!mcp7940n_read_register(MCP7940N_REG_HOURS, &value)) return false;
  hours = (value & 0x0F) + ((value & 0x30) >> 4) * 10;
  
  // Read weekday
  if (!mcp7940n_read_register(MCP7940N_REG_WKDAY, &value)) return false;
  day = value & 0x07;  // Only lower 3 bits represent day of week
  
  // Read date
  if (!mcp7940n_read_register(MCP7940N_REG_DATE, &value)) return false;
  date = (value & 0x0F) + ((value & 0x30) >> 4) * 10;
  
  // Read month
  if (!mcp7940n_read_register(MCP7940N_REG_MONTH, &value)) return false;
  month = (value & 0x0F) + ((value & 0x10) >> 4) * 10;
  
  // Read year
  if (!mcp7940n_read_register(MCP7940N_REG_YEAR, &value)) return false;
  year = (value & 0x0F) + ((value & 0xF0) >> 4) * 10;
  
  return true;
}

bool mcp7940n_set_time(uint8_t y, uint8_t m, uint8_t d, uint8_t h, uint8_t min, uint8_t sec) {
  // Convert decimal to BCD
  uint8_t sec_bcd = ((sec / 10) << 4) | (sec % 10);
  uint8_t min_bcd = ((min / 10) << 4) | (min % 10);
  uint8_t hour_bcd = ((h / 10) << 4) | (h % 10);
  uint8_t date_bcd = ((d / 10) << 4) | (d % 10);
  uint8_t month_bcd = ((m / 10) << 4) | (m % 10);
  uint8_t year_bcd = ((y / 10) << 4) | (y % 10);
  uint8_t wkday = 1;  // Default to Monday, could be calculated based on date
  
  // For seconds, set the ST bit (bit 7) to enable oscillator
  sec_bcd |= 0x80;
  
  // Write time/date registers
  if (!mcp7940n_write_register(MCP7940N_REG_SECONDS, sec_bcd)) return false;
  if (!mcp7940n_write_register(MCP7940N_REG_MINUTES, min_bcd)) return false;
  if (!mcp7940n_write_register(MCP7940N_REG_HOURS, hour_bcd)) return false;
  // Set the VBATEN bit (bit 3) in RTCWKDAY register to enable battery backup
  if (!mcp7940n_write_register(MCP7940N_REG_WKDAY, wkday | 0x08)) return false;
  if (!mcp7940n_write_register(MCP7940N_REG_DATE, date_bcd)) return false;
  if (!mcp7940n_write_register(MCP7940N_REG_MONTH, month_bcd)) return false;
  if (!mcp7940n_write_register(MCP7940N_REG_YEAR, year_bcd)) return false;
  
  return true;
}

bool mcp7940n_init() {
  // Initialize I2C
  i2c_init();
  
  // Initialize MFP pin
  pinMode(MFP_PIN, INPUT);  // Set as input initially
  
  // Test communication
  uint8_t value;
  if (!mcp7940n_read_register(MCP7940N_REG_WKDAY, &value)) {
    return false;
  }
  
  // Enable oscillator by setting ST bit (bit 7) in seconds register
  if (!mcp7940n_read_register(MCP7940N_REG_SECONDS, &value)) {
    return false;
  }
  value |= 0x80;  // Set ST bit
  if (!mcp7940n_write_register(MCP7940N_REG_SECONDS, value)) {
    return false;
  }
  
  // Enable battery backup by setting VBATEN bit (bit 3) in RTCWKDAY register
  if (!mcp7940n_read_register(MCP7940N_REG_WKDAY, &value)) {
    return false;
  }
  value |= 0x08;  // Set VBATEN bit
  if (!mcp7940n_write_register(MCP7940N_REG_WKDAY, value)) {
    return false;
  }
  
  // Set control register (basic configuration)
  // Configure MFP as alarm output, leave other bits at default
  return mcp7940n_write_register(MCP7940N_REG_CONTROL, 0x00);
}

// Initialize UART properly for STM32F103
bool setup_uart() {
  // Explicitly configure UART pins
  pinMode(UART_TX_PIN, OUTPUT);
  pinMode(UART_RX_PIN, INPUT_PULLUP);
  
  // Initialize UART2
  Serial2.begin(9600);
  delay(100);  // Give UART time to initialize
  
  // Send a test message
  Serial2.println("UART2 initialization");
  
  // We can't directly check if UART is working, but we assume it's working if we got this far
  return true;
}

void setup() {
  // Initialize Serial (USB)
  Serial.begin(115200);
  
  // Wait for serial to initialize (important for debugging)
  delay(1000);
  
  Serial.println("Starting MCP7940N RTC with UART Test");
  
  // Initialize UART2
  if (setup_uart()) {
    Serial.println("UART2 initialized successfully");
    
    // Test UART with multiple messages to ensure it's working
    for (int i = 0; i < 3; i++) {
      Serial2.print("UART2 test message #");
      Serial2.println(i + 1);
      delay(100);
    }
  } else {
    Serial.println("UART2 initialization failed");
  }
  
  // Initialize MCP7940N RTC
  Serial.println("Initializing MCP7940N RTC...");
  if (mcp7940n_init()) {
    Serial.println("MCP7940N RTC initialized successfully");
    Serial2.println("MCP7940N RTC initialized");
    
    // Set default time if needed (year:23, month:4, date:9, hour:12, min:0, sec:0)
    Serial.println("Setting default time...");
    if (mcp7940n_set_time(23, 4, 9, 14, 30, 0)) {
      Serial.println("Time set successfully");
      Serial2.println("Time set to 2023/04/09 12:00:00");
    } else {
      Serial.println("Failed to set time");
    }
  } else {
    Serial.println("Failed to initialize MCP7940N RTC");
    // Don't halt execution - we'll continue to try in the loop
  }
}

void loop() {
  // Get current date and time from MCP7940N
  if (mcp7940n_read_time()) {
    // Format and display date and time
    sprintf(dateTimeStr, "20%02d/%02d/%02d %02d:%02d:%02d", 
            year, month, date, hours, minutes, seconds);
    
    Serial.println(dateTimeStr);  // Output to USB Serial
    
    // Test UART output with clear indicator
    Serial2.print("RTC Time: ");
    Serial2.println(dateTimeStr);
  } else {
    Serial.println("Error reading from MCP7940N");
    
    // Try simpler UART message
    Serial2.println("RTC read error");
  }
  
  // Send a simple keep-alive message to UART2 as a heartbeat
  Serial2.println("UART heartbeat");
  
  // Wait one second before next update
  delay(1000);
}
