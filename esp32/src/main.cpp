/* Includes *****************************************************************/

#include <Arduino.h>
#include <../lib/arduino-mcp2515/mcp2515.h>
#include <../lib/nimble-arduino/src/NimBLEDevice.h>

/* Private Definitions ******************************************************/

/* BLE service UUIDs. */

#define IMG_SVC_UUID  "12345678-1234-1234-1234-123456789abc"
#define SS_SVC_UUID   "56789427-5749-2413-5748-087439816abc"
#define CTRL_SVC_UUID "AABBCCDD-1122-3344-5566-778899AABBCC"

/* BLE characteristic UUIDs. */

#define IMG_CHR_UUID  "B2C3D4E5-6789-2345-6789-ABCDEF123456"
#define SS_CHR_UUID   "B2G3D4R5-4749-2345-0759-ABCDEF103256"
#define CTRL_CHR_UUID "C1D2E3F4-5678-9876-5432-ABCDEF654321"

/* MCP2515 CS and INT pin. */

#define MCP_CS  10
#define MCP_INT 2

/* Private Variables ********************************************************/

/* MCP2515. */

MCP2515       mcp2515(MCP_CS, 500000);
volatile bool can_msg_rcv = false;

/* NimBLE characteristics. */

NimBLECharacteristic *img_chr;
NimBLECharacteristic *ss_chr;
NimBLECharacteristic *ctrl_chr;

/* Internal data variables. */

uint8_t ss_dat[8];
uint8_t ctrl_dat[6];
bool    start_read = false;
bool    stop_read  = false;

/* Private Functions ********************************************************/

void IRAM_ATTR CAN_ISR()
{
  can_msg_rcv = true;
}

class BLESvrCb final : public NimBLEServerCallbacks
{
public:
  static void onConnect(NimBLEServer *pServer)
  {
    Serial.println("Device connected");
  }

  static void onDisconnect(NimBLEServer *pServer)
  {
    Serial.println("Device disconnected");
    NimBLEDevice::startAdvertising();
  }
};

class CtrlCb final : public NimBLECharacteristicCallbacks
{
  static void onWrite(const NimBLECharacteristic *pCharacteristic)
  {
    const std::string value = pCharacteristic->getValue();
    if (value.length() == 6)
    {
      memcpy(ctrl_dat, value.data(), 6);

      Serial.print("Received Control Data: ");
      for (const unsigned char i : ctrl_dat)
      {
        Serial.print(i);
        Serial.print(" ");
      }
      Serial.println();
    }
  }
};

/* Main Functions ***********************************************************/

void setup()
{
  /* Initialize serial. */

  delay(2000);
  Serial.begin(115200);
  while (!Serial) { }

  /* Initialzie MCP2515. */

  Serial.println("Initializing MCP2515...");

  if (mcp2515.reset() != MCP2515::ERROR_OK)
  {
    Serial.println("Error resetting MCP2515");
    while (true) { }
  }

  if (mcp2515.setBitrate(CAN_125KBPS, MCP_8MHZ) != MCP2515::ERROR_OK)
  {
    Serial.println("Error setting CAN bitrate.");
    while (true) { }
  }

  if (mcp2515.setNormalMode() != MCP2515::ERROR_OK)
  {
    Serial.println("Error setting normal mode.");
    while (true) { }
  }

  /* Initialize CAN data interrupt pin. */

  pinMode(MCP_INT, INPUT);
  attachInterrupt(digitalPinToInterrupt(MCP_INT), CAN_ISR, FALLING);

  Serial.println("MCP2515 initialized.");

  /* Initialzie BLE. */

  Serial.println("Initializing BLE Server...");

  NimBLEDevice::init("ESP32-ImageServer");
  NimBLEServer *server = NimBLEDevice::createServer();
  server->setCallbacks(new BLESvrCb());

  /* Image service/characteristic: send image to computer. */

  NimBLEService *img_svc = server->createService(IMG_SVC_UUID);

  img_chr = img_svc->createCharacteristic(IMG_CHR_UUID, NOTIFY);

  /* Sensor service/characteristic: send sensor data to computer. */

  NimBLEService *ss_svc = server->createService(SS_SVC_UUID);

  ss_chr = ss_svc->createCharacteristic(SS_CHR_UUID, NOTIFY);

  /* Control service/characteristic: receive control data from computer. */

  NimBLEService *ctrl_svc = server->createService(CTRL_SVC_UUID);

  ctrl_chr = ctrl_svc->createCharacteristic(CTRL_CHR_UUID, WRITE);

  /* Add callback for control characteristic data. */

  ctrl_chr->setCallbacks(new CtrlCb());

  /* Start NimBLE services. */

  img_svc->start();
  ss_svc->start();
  ctrl_svc->start();

  Serial.println("BLE Server Initialized.");
  Serial.println("Advertising BLE Services...");

  /* Start BLE advertising. */

  NimBLEAdvertising *adv = server->getAdvertising();
  adv->addServiceUUID(img_svc->getUUID());
  adv->addServiceUUID(ss_svc->getUUID());
  adv->addServiceUUID(ctrl_svc->getUUID());
  adv->start();

  Serial.println("BLE Services Advertised.");
}

void loop()
{
  if (start_read)
  {
    can_frame canMsg1{};
    canMsg1.can_id  = 0x0F6;
    canMsg1.can_dlc = 1;
    canMsg1.data[0] = 0x8E;

    start_read = false;
  }

  if (stop_read)
  {
    can_frame cmd_msg{};
    cmd_msg.can_id  = 0x0F6;
    cmd_msg.can_dlc = 1;
    cmd_msg.data[0] = 0xE8;

    stop_read = false;
  }

  if (can_msg_rcv)
  {
    can_frame rcv_msg{};

    if (mcp2515.readMessage(&rcv_msg) == MCP2515::ERROR_OK)
    {
      Serial.print(rcv_msg.can_id, HEX);
      Serial.print(" ");
      Serial.print(rcv_msg.can_dlc, HEX);
      Serial.print(" ");

      for (int i = 0; i < rcv_msg.can_dlc; i++)
      {
        Serial.print(rcv_msg.data[i],HEX);
        Serial.print(" ");
      }

      Serial.println();

      /* TODO: Make if clause to detect data type, and send data to BLE. */
    }

    can_msg_rcv = false;
  }
}
