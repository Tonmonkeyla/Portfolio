import asyncio
import logging
import os
from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic

logger = logging.getLogger(__name__)

# Hardcoded configuration
DEVICE_ADDRESS = "64:e8:33:82:03:72"
CHARACTERISTIC_UUID = "ff01"
MACOS_USE_BDADDR = False

#previous first byte
previous_first_byte = None

async def write_uuid_ff01(client):
    """Write the UUID ff01 to the specified characteristic."""
    data_to_send = bytes.fromhex('ff01')
    await client.write_gatt_char(CHARACTERISTIC_UUID, data_to_send)
    logger.info("Sent UUID: ff01")

def notification_handler(characteristic: BleakGATTCharacteristic, data: bytearray):
    """Notification handler which checks for changes in the first byte."""
    global previous_first_byte
    
    logger.info("Received data: %r", data)
    
    if len(data) > 0:
        current_first_byte = data[0]
        
        if previous_first_byte is None:
            previous_first_byte = current_first_byte
            logger.info("Initial first byte: %02x", current_first_byte)
        elif current_first_byte != previous_first_byte:
            logger.info("First byte changed from %02x to %02x", previous_first_byte, current_first_byte)
            previous_first_byte = current_first_byte
            
            execute_custom_code(data)

def execute_custom_code(data: bytearray):
    print(data[0])
    if data[0] == 4:
        os.popen(r'"C:\display64.exe" /rotate:0')
    elif data[0] == 1:
        os.popen(r'"C:\display64.exe" /rotate:90')
    elif data[0] == 2:
        os.popen(r'"C:\display64.exe" /rotate:270')
    else:
        os.popen(r'"C:\display64.exe" /rotate:180')
    #os.popen(r'"C:\Users\tonmo\Desktop\display64.exe" /rotate:90')
async def main():
    logger.info("Searching for device %s...", DEVICE_ADDRESS)

    device = await BleakScanner.find_device_by_address(
        DEVICE_ADDRESS, cb=dict(use_bdaddr=MACOS_USE_BDADDR)
    )
    if device is None:
        logger.error("Device not found")
        return

    logger.info("Connecting to device...")

    async with BleakClient(device) as client:
        logger.info("Connected successfully")
        
        # First, write the UUID ff01
        await write_uuid_ff01(client)
        
        # Then start notifications
        await client.start_notify(CHARACTERISTIC_UUID, notification_handler)
        logger.info("Notification handler set up - waiting for data...")
        
        try:
            while True:
                await asyncio.sleep(1)  # Run indefinitely
        except KeyboardInterrupt:
            logger.info("Stopping notifications...")
            await client.stop_notify(CHARACTERISTIC_UUID)

if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    )

    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("Program stopped by user")
    except Exception as e:
        logger.error(f"Error: {e}")