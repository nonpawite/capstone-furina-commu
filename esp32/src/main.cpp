#include <Arduino.h>
#include <SPI.h>
#include <../lib/arduino-mcp2515/mcp2515.h>
#include <../lib/nimble-arduino/src/NimBLEDevice.h>

/* MCP2515 variables. */

MCP2515   mcp2515(10, 500000);
can_frame can_msg;

/* NimBLE variables. */

NimBLECharacteristic *input_chr;
NimBLECharacteristic *ss_chr;
std::string           last_input_val = "";

/* NimBLE callbacks. */

class nimble_callbacks final : public NimBLEServerCallbacks
{
public:
  void on_conn(NimBLEServer *pServer)
  {
    Serial.println("Device connected");
  }

  void on_disc(NimBLEServer *pServer)
  {
    Serial.println("Device disconnected");
    NimBLEDevice::startAdvertising();
  }
};

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

  Serial.println("MCP2515 initialized.");

  /* Initialzie BLE. */

  Serial.println("Initializing BLE Server...");

  NimBLEDevice::init("Furina");
  NimBLEServer *server = NimBLEDevice::createServer();
  server->setCallbacks(new nimble_callbacks());

  NimBLEService *service = server->
    createService("12345678-1234-1234-1234-123456789abc");

  input_chr = service->
    createCharacteristic("A1B2C3D4-5678-1234-5678-9ABCDEF01234", WRITE);
  input_chr->setValue("Ready to receive input");

  ss_chr = service->
    createCharacteristic("B2C3D4E5-6789-2345-6789-ABCDEF123456",
                         READ | WRITE | NOTIFY);
  ss_chr->setValue("Feedback initialized");

  service->start();

  NimBLEAdvertising *advertising = NimBLEDevice::getAdvertising();
  advertising->addServiceUUID(service->getUUID());
  advertising->start();

  Serial.println("BLE initialzed.");
}

void loop()
{
  const std::string newInputValue = input_chr->getValue();

  if (newInputValue != last_input_val)
  {
    Serial.print("Received value from Unity: ");
    Serial.println(newInputValue.c_str());
    last_input_val = newInputValue;

    if (newInputValue == "Start")
    {
      ss_chr->setValue("Process started");
    }
    else if (newInputValue == "Stop")
    {
      ss_chr->setValue("Process stopped");
    }
    else
    {
      ss_chr->setValue("Invalid input received");
    }

    ss_chr->notify();
  }

  delay(100);
}
