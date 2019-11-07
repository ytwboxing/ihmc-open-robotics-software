package us.ihmc.valkyrie;

import static us.ihmc.valkyrie.ValkyrieROS2LoadTestEndPoint.linkEndPoint;
import static us.ihmc.valkyrie.ValkyrieROS2LoadTestEndPoint.zeldaEndPoint;

import java.io.IOException;

public class ValkyrieROS2LoadTestLink
{
   public static void main(String[] args) throws IOException
   {
      new ValkyrieROS2LoadTestEndPoint(linkEndPoint(), zeldaEndPoint());
   }
}
