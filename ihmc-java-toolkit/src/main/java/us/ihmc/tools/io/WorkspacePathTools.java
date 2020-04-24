package us.ihmc.tools.io;

import us.ihmc.commons.nio.BasicPathVisitor;
import us.ihmc.commons.nio.PathTools;
import us.ihmc.log.LogTools;

import java.nio.file.FileVisitResult;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class WorkspacePathTools
{
   /**
    * Use this method when applications are being run from source and need to access a project file.
    *
    * This method searches for a directory in the path parts of the current working directory and
    * will also find it if it is a child of the current working directory.
    *
    * For example, if directoryNameToFind is ihmc-open-robotics-software and the current working directory is
    * ihmc-open-robotics-software/ihmc-java-toolkit/src, this will return the absolute path to ihmc-open-robotics-software.
    *
    * @param directoryNameToFind
    * @return absolute path to that directory, or null if fails
    */
   public static Path handleWorkingDirectoryFuzziness(String directoryNameToFind)
   {
      Path absoluteWorkingDirectory = Paths.get(".").toAbsolutePath().normalize();
      Path pathBuiltFromSystemRoot = Paths.get("/").toAbsolutePath().normalize(); // start with system root

      AtomicBoolean directoryFound = new AtomicBoolean(false);
      AtomicReference<Path> foundDirectory = new AtomicReference<>();

      for (Path path : absoluteWorkingDirectory)
      {
         pathBuiltFromSystemRoot = pathBuiltFromSystemRoot.resolve(path); // building up the path

         if (path.toString().equals(directoryNameToFind))
         {
            directoryFound.set(true);
            foundDirectory.set(pathBuiltFromSystemRoot);
            break;
         }
      }

      if (!directoryFound.get() && Files.exists(absoluteWorkingDirectory.resolve(directoryNameToFind))) // working directory contains directory to find
      {
         directoryFound.set(true);
         foundDirectory.set(absoluteWorkingDirectory.resolve(directoryNameToFind));
      }

      if (!directoryFound.get()) // check with greater depth into children
      {
         PathTools.walkBreadthFirst(pathBuiltFromSystemRoot, 5, (path, pathType) ->
         {
            if (pathType == BasicPathVisitor.PathType.DIRECTORY && path.getFileName().toString().equals(directoryNameToFind))
            {
               directoryFound.set(true);
               foundDirectory.set(path.toAbsolutePath().normalize());
               return FileVisitResult.TERMINATE;
            }

            return FileVisitResult.CONTINUE;
         });
      }

      if (!directoryFound.get())
      {
         LogTools.warn("{} directory could not be found. Working directory: {} Search stopped at: {}",
                       directoryNameToFind,
                       absoluteWorkingDirectory,
                       pathBuiltFromSystemRoot);
         return null;
      }

      return foundDirectory.get();
   }
}
