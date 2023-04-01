using HidLibrary;

namespace hid_volume_sync_test
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
                //_device.MonitorDeviceEvents = true;
                _device.ReadReport(OnReport);
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
            {
                return;
            }
            var reportData = report.Data;

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
    }
}