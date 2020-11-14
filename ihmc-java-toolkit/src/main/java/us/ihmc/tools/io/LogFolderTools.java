package us.ihmc.tools.io;

import java.nio.file.Path;
import java.nio.file.Paths;

public class LogFolderTools
{
   private static final String BAMBOO_PLAN_KEY = System.getenv("bamboo_planKey");
   private static final String BAMBOO_BUILD_KEY = System.getenv("bamboo_buildResultKey");

   private static final String DEFAULT_LOG_FOLDER = System.getProperty("user.home") + "/.ihmc/logs";

   private static final Path LOG_FOLDER = decideLogFolderPath();

   private static Path decideLogFolderPath()
   {
      Path defaultPath = Paths.get(DEFAULT_LOG_FOLDER);
      if (BAMBOO_PLAN_KEY == null || BAMBOO_BUILD_KEY == null)
      {
         return defaultPath;
      }
      else
      {
         return defaultPath.resolve(BAMBOO_PLAN_KEY).resolve(BAMBOO_BUILD_KEY);
      }
   }

   public static String getLogFolder()
   {
      return LOG_FOLDER.toAbsolutePath().normalize().toString();
   }

   public static Path getLogFolderPath()
   {
      return LOG_FOLDER;
   }
}
