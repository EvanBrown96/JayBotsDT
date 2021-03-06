/* 
 * This message is auto generated by ROS#. Please DO NOT modify.
 * Note:
 * - Comments from the original code will be written in their own line 
 * - Variable sized arrays will be initialized to array of size 0 
 * Please report any issues at 
 * <https://github.com/siemens/ros-sharp> 
 */



namespace RosSharp.RosBridgeClient.MessageTypes.RemoteApp
{
    public class LaunchRoverRequest : Message
    {
        public const string RosMessageName = "remote_app/LaunchRover";

        public string machine_name { get; set; }
        public string ip_addr { get; set; }

        public LaunchRoverRequest()
        {
            this.machine_name = "";
            this.ip_addr = "";
        }

        public LaunchRoverRequest(string machine_name, string ip_addr)
        {
            this.machine_name = machine_name;
            this.ip_addr = ip_addr;
        }
    }
}
