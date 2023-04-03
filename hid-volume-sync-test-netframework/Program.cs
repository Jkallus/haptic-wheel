using System;
using System.Linq;
using HidLibrary;
using NAudio;
using NAudio.CoreAudioApi;

namespace hid_volume_sync_test_netframework
{
    internal class Program
    {

        private const int VendorID = 0x303A;
        private const int ProductID = 0x1001;

        private static HidDevice _device;

        private static MMDeviceEnumerator enumerator = new MMDeviceEnumerator();
        
        private static MMDevice dev;

        static void Main(string[] args)
        {
            Console.WriteLine("Hello, World!");

            dev = enumerator.GetDefaultAudioEndpoint(DataFlow.Render, Role.Multimedia);
            dev.AudioEndpointVolume.OnVolumeNotification += AudioEndpointVolume_OnVolumeNotification;

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
                Console.ReadLine();
            }
        }

        private static void AudioEndpointVolume_OnVolumeNotification(AudioVolumeNotificationData data)
        {
            byte percent = Convert.ToByte(data.MasterVolume * 100);
            Console.WriteLine($"Volume updated to {data.MasterVolume}, Percent value: {percent}");
            WriteData(percent);
        }

        private static byte GetCurrentVolume()
        {
            float volume = dev.AudioEndpointVolume.MasterVolumeLevelScalar;
            if (volume > 1.0 || volume < 0.0)
                throw new Exception("Volume out of range");
            byte percent = Convert.ToByte(volume * 100);
            Console.WriteLine($"Got current volume: {percent}");
            return percent;
        }

        private static void OnReport(HidReport report)
        {
            if (!_device.IsConnected)
                return;

            var reportData = report.Data;

            Console.WriteLine($"Report data: {reportData[0]:X2}");
            _device.ReadReport(OnReport);
        }

        private static void DeviceAttachedHandler()
        {
            Console.WriteLine("Device attached");
            WriteData(GetCurrentVolume()); // Write initial volume when connected
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
            Console.WriteLine($"Writing {data} to device");
            _device.WriteReport(report);

        }

        private static void WriteData(byte[] data)
        {
            var report = _device.CreateReport();
            report.ReportId = 1;
            report.Data = data;
            _device.WriteReport(report);
        }
    }
}
