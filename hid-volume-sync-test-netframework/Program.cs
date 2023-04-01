﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using HidLibrary;


namespace hid_volume_sync_test_netframework
{
    internal class Program
    {

        private const int VendorID = 0x303A;
        private const int ProductID = 0x1001;

        private static HidDevice _device;

        static void Main(string[] args)
        {
            Console.WriteLine("Hello, World!");

            _device = HidDevices.Enumerate(VendorID, ProductID).FirstOrDefault();

            if (_device != null)
            {
                _device.OpenDevice();
                _device.Inserted += DeviceAttachedHandler;
                _device.Removed += DeviceRemovedHandler;
                _device.MonitorDeviceEvents = true;
                _device.ReadReport(OnReport);
                Console.ReadLine();
                _device.CloseDevice();
            }
            else
            {
                Console.WriteLine("Could not find device");
            }
        }

        private static void OnReport(HidReport report)
        {
            if (!_device.IsConnected)
                return;

            var reportData = report.Data;

            Console.WriteLine($"Report data: {reportData[0]:X2}");

            WriteData(0x45);

            _device.ReadReport(OnReport);
        }

        private static void DeviceAttachedHandler()
        {
            Console.WriteLine("Device attached");
            _device.ReadReport(OnReport);
        }

        private static void DeviceRemovedHandler()
        {
            Console.WriteLine("Device removed");
        }

        private static void WriteData(byte data)
        {
            var report = _device.CreateReport();
            report.ReportId = 1;
            report.Data = new byte[] { data };
            _device.WriteReport(report);
        }
    }
}
