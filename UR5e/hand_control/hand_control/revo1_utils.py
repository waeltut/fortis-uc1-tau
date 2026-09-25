"""Modbus connection support used by the Revo1 ROS node."""

import logging

try:
    from bc_stark_sdk import main_mod as libstark
except ImportError as exc:
    raise ImportError(
        "hand_control requires bc_stark_sdk in the ROS Python environment. "
        "Install the same SDK version used by your working robot setup."
    ) from exc

logger = logging.getLogger(__name__)


async def open_modbus_revo1(port_name=None, quick=True):
    """
    Automatically detect and open Modbus connection for Revo1 dexterous hand

    Revo1 dexterous hand supports both Modbus and Protobuf communication protocols, Modbus is recommended.
    This function automatically detects port, baud rate and device ID, and establishes connection.

    Args:
        port_name (str, optional): Serial port name, None by default means auto-detect the first available port.
            Can specify a specific port when multiple ports exist, e.g.: "/dev/ttyUSB0"
        quick (bool, optional): Quick detection mode configuration, True by default.
            True: Only detect common baud rates and default device ID, faster
            False: Detect device ID range 1~247, more comprehensive but takes longer

    Returns:
        tuple: (client, slave_id) - Modbus client instance and device slave ID

    Raises:
        RuntimeError: When detected protocol is not Modbus

    Example:
        client, slave_id = await open_modbus_revo1()
        # Or manually specify parameters:
        # client: libstark.DeviceContext = await libstark.modbus_open(port_name, baudrate)
    """
    # Auto-detect the first available slave device
    (
        protocol,
        detected_port_name,
        baudrate,
        slave_id,
    ) = await libstark.auto_detect_modbus_revo1(port_name, quick)

    # Verify detected protocol type
    if protocol != libstark.StarkProtocolType.Modbus:
        raise RuntimeError("Only Modbus protocol is supported for Revo1")

    # Establish Modbus connection
    client: libstark.DeviceContext = await libstark.modbus_open(
        detected_port_name, baudrate
    )

    # Get device information
    device_info: libstark.DeviceInfo = await client.get_device_info(slave_id)
    logger.info(f"Device info: {device_info.description}")

    if device_info.uses_revo1_motor_api():
        if device_info.uses_revo1_touch_api():
            logger.info("Touch hand")
        else:
            logger.info("Standard version")

    return (client, slave_id)
