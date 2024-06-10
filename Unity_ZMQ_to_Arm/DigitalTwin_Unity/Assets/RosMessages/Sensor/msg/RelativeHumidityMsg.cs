//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Sensor
{
    [Serializable]
    public class RelativeHumidityMsg : Message
    {
        public const string k_RosMessageName = "sensor_msgs/RelativeHumidity";
        public override string RosMessageName => k_RosMessageName;

        //  Single reading from a relative humidity sensor.
        //  Defines the ratio of partial pressure of water vapor to the saturated vapor
        //  pressure at a temperature.
        public Std.HeaderMsg header;
        //  timestamp of the measurement
        //  frame_id is the location of the humidity sensor
        public double relative_humidity;
        //  Expression of the relative humidity
        //  from 0.0 to 1.0.
        //  0.0 is no partial pressure of water vapor
        //  1.0 represents partial pressure of saturation
        public double variance;
        //  0 is interpreted as variance unknown

        public RelativeHumidityMsg()
        {
            this.header = new Std.HeaderMsg();
            this.relative_humidity = 0.0;
            this.variance = 0.0;
        }

        public RelativeHumidityMsg(Std.HeaderMsg header, double relative_humidity, double variance)
        {
            this.header = header;
            this.relative_humidity = relative_humidity;
            this.variance = variance;
        }

        public static RelativeHumidityMsg Deserialize(MessageDeserializer deserializer) => new RelativeHumidityMsg(deserializer);

        private RelativeHumidityMsg(MessageDeserializer deserializer)
        {
            this.header = Std.HeaderMsg.Deserialize(deserializer);
            deserializer.Read(out this.relative_humidity);
            deserializer.Read(out this.variance);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.relative_humidity);
            serializer.Write(this.variance);
        }

        public override string ToString()
        {
            return "RelativeHumidityMsg: " +
            "\nheader: " + header.ToString() +
            "\nrelative_humidity: " + relative_humidity.ToString() +
            "\nvariance: " + variance.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}
