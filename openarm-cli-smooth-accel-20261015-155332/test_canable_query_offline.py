"""USB is mocked throughout; these tests never access hardware."""
import contextlib
import io
import itertools
import json
import struct
import unittest
from unittest.mock import Mock, PropertyMock, patch

import canable_inspect as inspect
import canable_query as query
from openarm_motor_test import Link


class MultiAdapterChecks(unittest.TestCase):
    def device(self, serial, address):
        dev = Mock(bus=1, address=address, serial_number=serial)
        caps = struct.pack('<10I', 0xf3, 170000000, 1, 256, 1, 128, 128, 1, 512, 1)
        dev.ctrl_transfer.return_value = caps
        dev.read.side_effect = query.usb.core.USBTimeoutError('mock timeout')
        return dev

    def run_main(self, devices, argv):
        output = io.StringIO()
        with patch.object(query.usb.backend.libusb1, 'get_backend', return_value=object()), \
                patch.object(query.usb.core, 'find', return_value=devices), \
                patch.object(query.usb.util, 'dispose_resources'), \
                patch.object(query.time, 'monotonic', side_effect=itertools.count(0, .2)), \
                contextlib.redirect_stdout(output):
            code = query.main(argv)
        return code, json.loads(output.getvalue())

    def test_each_adapter_gets_only_parameter_reads_and_is_stopped(self):
        devices = [self.device('arm-a', 2), self.device('arm-b', 3)]
        code, result = self.run_main(devices, ['--query', '--all-adapters'])
        self.assertEqual(code, 0)
        self.assertEqual(result['adapter_count'], 2)
        self.assertEqual([r['usb_serial'] for r in result['adapters']], ['arm-a', 'arm-b'])
        for dev, report in zip(devices, result['adapters']):
            self.assertTrue(report['adapter_stopped'])
            self.assertEqual(report['online_motor_ids'], [])
            self.assertEqual(dev.write.call_count, 16)
            for motor, call in enumerate(dev.write.call_args_list, 1):
                endpoint, raw = call.args
                self.assertEqual(endpoint, 2)
                self.assertEqual(struct.unpack('<IIBBBB', raw[:12]), (0, 0x7ff, 8, 0, 0, 0))
                self.assertEqual(struct.unpack('<HBBI', raw[12:]), (motor, 0x33, 7, 0))
            self.assertEqual(dev.ctrl_transfer.call_args.args,
                             (0x41, 2, 0, 0, struct.pack('<II', 0, 0)))

    def test_requires_explicit_all_flag(self):
        devices = [self.device('a', 2), self.device('b', 3)]
        with self.assertRaisesRegex(RuntimeError, '--all-adapters'):
            self.run_main(devices, ['--query'])
        for dev in devices:
            dev.ctrl_transfer.assert_not_called()
            dev.write.assert_not_called()

    def test_usb_probe_reports_serials_without_can_writes(self):
        devices = [self.device('arm-a', 2), self.device('arm-b', 3)]
        for dev in devices:
            dev.idVendor, dev.idProduct, dev.product = 0x1d50, 0x606f, 'CANable'
            dev.ctrl_transfer.side_effect = [bytes(12), dev.ctrl_transfer.return_value]
        type(devices[1]).serial_number = PropertyMock(side_effect=ValueError('descriptor unavailable'))
        output = io.StringIO()
        with patch.object(inspect.usb.backend.libusb1, 'get_backend', return_value=object()), \
                patch.object(inspect.usb.core, 'find', return_value=devices), \
                patch.object(inspect.usb.util, 'dispose_resources'), contextlib.redirect_stdout(output):
            self.assertEqual(inspect.main(), 0)
        first, second = json.loads(output.getvalue())['adapters']
        self.assertEqual(first['usb_serial'], 'arm-a')
        self.assertIsNone(second['usb_serial'])
        self.assertIn('descriptor', second['serial_error'])
        for dev in devices:
            self.assertTrue(all(call.args[0] == 0xc1 for call in dev.ctrl_transfer.call_args_list))
            dev.write.assert_not_called()

    def test_busy_robot_interface_released_without_reset_or_can_write(self):
        device = self.device('selected', 2)
        with patch.object(query.usb.backend.libusb1, 'get_backend', return_value=object()), \
                patch.object(query.usb.core, 'find', return_value=[device]), \
                patch.object(query.usb.util, 'claim_interface',
                             side_effect=query.usb.core.USBError('busy')) as claim, \
                patch.object(query.usb.util, 'dispose_resources') as dispose:
            with self.assertRaisesRegex(RuntimeError, 'Cannot claim CANable selected'):
                Link('selected')
        claim.assert_called_once_with(device, 0)
        dispose.assert_called_once_with(device)
        device.ctrl_transfer.assert_not_called()
        device.write.assert_not_called()

    def test_robot_link_claims_selected_interface_and_stops_it_on_close(self):
        other, selected = self.device('other', 2), self.device('selected', 3)
        with patch.object(query.usb.backend.libusb1, 'get_backend', return_value=object()), \
                patch.object(query.usb.core, 'find', return_value=[other, selected]), \
                patch.object(query.usb.util, 'claim_interface') as claim, \
                patch.object(query.usb.util, 'dispose_resources') as dispose:
            link = Link('selected')
            self.assertEqual(selected.ctrl_transfer.call_args.args,
                             (0x41, 2, 0, 0, struct.pack('<II', 1, 0)))
            link.close()
            claim.assert_called_once_with(selected, 0)
            self.assertEqual(dispose.call_count, 2)
            self.assertEqual(selected.ctrl_transfer.call_args.args,
                             (0x41, 2, 0, 0, struct.pack('<II', 0, 0)))
            other.ctrl_transfer.assert_not_called()

    def test_one_adapter_failure_does_not_hide_second(self):
        devices = [self.device('bad', 2), self.device('good', 3)]
        devices[0].ctrl_transfer.side_effect = query.usb.core.USBError('mock disconnect')
        code, result = self.run_main(devices, ['--query', '--all-adapters'])
        self.assertEqual(code, 1)
        self.assertIn('failure', result['adapters'][0])
        self.assertNotIn('failure', result['adapters'][1])
        self.assertEqual(devices[1].write.call_count, 16)

    def test_reports_keep_motor_replies_separate(self):
        devices = [self.device('a', 2), self.device('b', 3)]
        reports = [{'online_motor_ids': list(range(1, 9))},
                   {'online_motor_ids': list(range(1, 8))}]
        with patch.object(query, 'diagnose', side_effect=reports) as diagnose:
            code, result = self.run_main(devices, ['--query', '--all-adapters'])
        self.assertEqual(code, 0)
        self.assertEqual(diagnose.call_count, 2)
        self.assertEqual(result['adapters'][0]['online_motor_ids'], list(range(1, 9)))
        self.assertEqual(result['adapters'][1]['online_motor_ids'], list(range(1, 8)))


if __name__ == '__main__':
    unittest.main()
