"""Read gs_usb adapter capabilities only; never start CAN or send CAN frames.

Run with .venv-can-diagnostics/bin/python canable_inspect.py.
Dependencies: pyusb, libusb-package. No motor power is needed.
Protocol reference: Linux drivers/net/can/usb/gs_usb.c.
"""

import json
import struct
import sys

import libusb_package
import usb.backend.libusb1
import usb.core
import usb.util


def main():
    backend = usb.backend.libusb1.get_backend(
        find_library=libusb_package.find_library
    )
    if backend is None:
        raise RuntimeError("Bundled libusb could not be loaded")
    devices = list(usb.core.find(
        find_all=True, backend=backend,
        custom_match=lambda d: (d.idVendor, d.idProduct)
        in {(0x1D50, 0x606F), (0x1209, 0x2323)},
    ))
    results = []
    for device in devices:
        result = {"usb_id": f"{device.idVendor:04x}:{device.idProduct:04x}"}
        try:
            try:
                result["usb_serial"] = device.serial_number
                if not result["usb_serial"]:
                    result["serial_error"] = "Adapter has no USB serial descriptor"
            except (usb.core.USBError, ValueError) as exc:
                result["usb_serial"] = None
                result["serial_error"] = str(exc)
            result["product"] = device.product
            # Only device-to-host vendor reads on endpoint 0. No reset,
            # configuration change, interface start, or bulk OUT transfer.
            info = bytes(device.ctrl_transfer(0xC1, 5, 0, 0, 12, timeout=1500))
            _, _, _, last_channel, sw, hw = struct.unpack("<4B2I", info)
            result.update(channels=last_channel + 1,
                          firmware_version_raw=sw, hardware_version_raw=hw)
            caps = bytes(device.ctrl_transfer(0xC1, 4, 0, 0, 40, timeout=1500))
            values = struct.unpack("<10I", caps)
            features, clock = values[:2]
            result.update(features_hex=hex(features), can_clock_hz=clock,
                          can_fd_advertised=bool(features & (1 << 8)),
                          listen_only_supported=bool(features & 1))
            result["nominal_timing_limits"] = dict(zip(
                ["tseg1_min", "tseg1_max", "tseg2_min", "tseg2_max",
                 "sjw_max", "brp_min", "brp_max", "brp_inc"], values[2:]
            ))
        except (usb.core.USBError, ValueError, struct.error) as exc:
            result["error"] = str(exc)
        finally:
            usb.util.dispose_resources(device)
        results.append(result)
    print(json.dumps({"mode": "USB capability reads only; no CAN frames sent",
                      "adapters": results}, indent=2))
    return 0 if results and all("error" not in r for r in results) else 1


if __name__ == "__main__":
    sys.exit(main())
