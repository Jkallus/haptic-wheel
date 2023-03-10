namespace SaleaeParser
{
    internal class Program
    {
        static void Main(string[] args)
        {
            List<Double> angles = new List<double>();

            string path = "C:\\Users\\joshk\\source\\repos\\haptic-wheel\\spi_capture_5.csv";
            string outFile = "C:\\Users\\joshk\\source\\repos\\haptic-wheel\\spi_capture_processed.txt";
            IEnumerable<string> lines = File.ReadLines(path);

            Byte high_bits = 0;
            Byte low_bits = 0;

            UInt16 max_counts = 2 << 14;

            bool high_set = false;
            bool low_set = false;
            
            foreach(string line in lines)
            {
                string[] parts = line.Split(',');
                if (parts[0].Trim('"') == "SPI" && parts[1].Trim('"') == "result")
                {
                    string miso = parts[5];
                    if(!low_set)
                    {
                        low_bits = Convert.ToByte(miso, 16);
                        low_set = true;
                    }
                    else if(low_set && !high_set)
                    {
                        high_bits = Convert.ToByte(miso, 16);
                        high_set = true;
                    }

                    if(high_set && low_set)
                    {
                        UInt16 reg_value = BitConverter.ToUInt16(new byte[] { high_bits, low_bits });
                        UInt16 lower_14_bits = (ushort)(reg_value & 0b0011111111111111);

                        double angle = 360.0f * ((float)lower_14_bits / (float)max_counts);
                        if (angle != 0)
                        {
                            angles.Add(angle);
                        }

                        high_set = false;
                        low_set = false;
                    }
                }
            }

            string output = "";

            foreach(double angle in angles)
            {
                output += angle.ToString() + "\n";
            }

            File.WriteAllText(outFile, output);

        }
    }
}