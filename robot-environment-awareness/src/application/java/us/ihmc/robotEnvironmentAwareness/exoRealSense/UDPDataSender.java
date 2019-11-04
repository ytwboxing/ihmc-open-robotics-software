package us.ihmc.robotEnvironmentAwareness.exoRealSense;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;

public class UDPDataSender
{   
   //variables
   DatagramSocket socket; 
   InetAddress IPAddress;  
   int port;   
   
   public UDPDataSender(String IPAddressString, int port) {
      try {
         socket = new DatagramSocket(port+1);   
         IPAddress = InetAddress.getByName(IPAddressString);
         this.port = port;
      }
      catch (Exception e) {
         e.printStackTrace();
      }
   }
   
   public void sendDistance(String distance) {
      try
      {
         byte[] sendData = distance.getBytes();
         DatagramPacket sendPacket = new DatagramPacket(sendData, sendData.length, IPAddress, port);
         socket.send(sendPacket);   
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }
}
