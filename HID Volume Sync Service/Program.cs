using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.ServiceProcess;
using System.IO.Ports;
using HidLibrary;
using NAudio;
using NAudio.CoreAudioApi;
using System.Threading;
using Serilog;

namespace HID_Volume_Sync_Service
{
    public static class Program
    {
        public const string ServiceName = "HIDVolumeSync";

        private const int VendorID = 0x303A;
        private const int ProductID = 0x1001;

        private static HidDevice _device;

        private static MMDeviceEnumerator enumerator = new MMDeviceEnumerator();

        private static MMDevice dev;

        public class Service : ServiceBase
        {
            public Service()
            {
                ServiceName = Program.ServiceName;
            }

            protected override void OnStart(string[] args)
            {
                Program.Start(args);
            }

            protected override void OnStop()
            {
                Program.Stop();
            }
        }

        static void Main(string[] args)
        {
            Log.Logger = new LoggerConfiguration()
                .WriteTo.File("C:\\temp\\log.txt")
                .WriteTo.Console()
                .CreateLogger();

            Log.Information("Service starting...");

            if(!Environment.UserInteractive)
            {
                using(var service = new Service())
                {
                    Log.Information("Running as service");
                    ServiceBase.Run(service);
                }
            }
            else
            {
                Start(args);
                Log.Information("Running as console app, press any key to stop");
                Console.ReadKey();
                Stop();
            }
        }

        private static void Start(string[] args)
        {
            dev = enumerator.GetDefaultAudioEndpoint(DataFlow.Render, Role.Multimedia);
            dev.AudioEndpointVolume.OnVolumeNotification += AudioEndpointVolume_OnVolumeNotification;

            while (_device == null)
            {
                _device = HidDevices.Enumerate(VendorID, ProductID).FirstOrDefault();

                if (_device != null)
                {
                    Log.Information("Found device");

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
                    Log.Information("Could not find device");
                    Log.Information("Trying again in 5s");
                    Thread.Sleep(5000);
                }
            }
        }

        private static void Stop()
        {
            Log.Information("Got stop command, closing device");
            _device.CloseDevice();
        }

        private static void AudioEndpointVolume_OnVolumeNotification(AudioVolumeNotificationData data)
        {
            byte percent = Convert.ToByte(data.MasterVolume * 100);
            Log.Information("Volume updated to {volume}, Percent value: {percent}", data.MasterVolume, percent);
            WriteData(percent);
        }

        private static byte GetCurrentVolume()
        {
            float volume = dev.AudioEndpointVolume.MasterVolumeLevelScalar;
            if (volume > 1.0 || volume < 0.0)
                throw new Exception("Volume out of range");
            byte percent = Convert.ToByte(volume * 100);
            Log.Information("Got current volume: {percent}", percent);
            return percent;
        }

        private static void OnReport(HidReport report)
        {
            if (!_device.IsConnected)
                return;

            var reportData = report.Data;
            Log.Information("Report data: {reportData}", reportData[0]);
            _device.ReadReport(OnReport);
        }

        private static void DeviceAttachedHandler()
        {
            Log.Information("Device attached");
            WriteData(GetCurrentVolume()); // Write initial volume when connected
            _device.ReadReport(OnReport);
        }

        private static void DeviceRemovedHandler()
        {
            Log.Information("Device removed");
        }

        private static void WriteData(byte data)
        {
            var report = _device.CreateReport();
            report.ReportId = 1;
            report.Data = new byte[] { data };
            Log.Information("Writing {data} to device", data);
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
